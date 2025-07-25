#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pi3net.h"
#include "pi3net.pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "timing.h"

// bits/word and clocks/bit are set in pi3net.pio
// and thus come in via pi3net.pio.h
#if (p3n_bits_per_word == 32)
#define DMA_SIZE DMA_SIZE_32
#elif (p3n_bits_per_word == 16)
#define DMA_SIZE DMA_SIZE_16
#elif (p3n_bits_per_word == 8)
#define DMA_SIZE DMA_SIZE_8
#else
#error "bad bits-per-word"
#endif

#define BYTES_PER_WORD  (p3n_bits_per_word/8u)

#define COMMAND_SHIFT           (p3n_silence_delay_bits)
#define POST_CMD_DELAY_SHIFT    (p3n_silence_delay_bits+p3n_command_bits)

#define MAX_SILENCE_DELAY       ((1<<(p3n_silence_delay_bits))-1)
#define MAX_POST_CMD_DELAY      ((1<<(p3n_post_cmd_delay_bits))-1)

#define SILENCE_DELAY_RX        ((p3n_clocks_per_bit*((p3n_bits_per_word)+3))/2)
#define SILENCE_DELAY_TX        ((p3n_clocks_per_bit*((p3n_bits_per_word)+5))/2)

static_assert(MAX_SILENCE_DELAY > SILENCE_DELAY_RX);
static_assert(MAX_SILENCE_DELAY > SILENCE_DELAY_TX);

#define PIO_CMD_RX              ((1<<(COMMAND_SHIFT))|(SILENCE_DELAY_RX))
#define PIO_CMD_TX              ((0<<(COMMAND_SHIFT))|(SILENCE_DELAY_TX))
#define PIO_CMD_MASK            ((1<<(COMMAND_SHIFT+1))-1)

/**********************************************************
 * This structure defines a pi3net command.
 */
typedef struct p3n_cmd_s {
    p3n_cmd_state_t      state;
    uint32_t             pio_cmd;
    p3n_buffer_t        *buf;
} p3n_cmd_t;

static_assert(sizeof(p3n_cmd_t) == 12 );


/**********************************************************
 * This structure defines a pi3net channel.
 */
typedef struct p3n_chan_s {
    uint8_t                   sm_index;
    uint8_t                   dma_index;
    uint8_t                   rx_pin;
    uint8_t                   tx_pin;
    uint8_t                   tx_ena_pin;
    uint8_t                   queue_len;
    uint8_t                   queue_in;
    uint8_t                   queue_out;
    dma_channel_config        rx_dma_config;
    dma_channel_config        tx_dma_config;
    p3n_cmd_t                *active_cmd;
    p3n_cmd_t                *cmd_queue;
} p3n_chan_t;

/*************************************************
 * Global data - initialized by p3n_init()
 */

static PIO p3n_pio = NULL;
static uint p3n_code_offset = PIO_INSTRUCTION_COUNT;
static irq_num_t p3n_irq_num = IRQ_COUNT;
static uint p3n_pio_irq_num = NUM_PIO_IRQS;
static uint p3n_crc_dma_index = NUM_DMA_CHANNELS;
static uint32_t crc_dummy;
static p3n_chan_t p3n_chans[P3N_NUM_CHAN];

static void chan_start_next_cmd(p3n_chan_t *ch);
static bool try_pio(uint index);


p3n_buffer_t *p3n_buffer_new(uint16_t max_msg_len_bytes)
{
    p3n_buffer_t *b;
    b = malloc(sizeof(p3n_buffer_t));
    assert(b != NULL);
    // round size up to a multiple of 4 bytes
    b->max_len = ( max_msg_len_bytes + 3 ) & ~3;
    // allocate space including 4 bytes for CRC
    b->data = malloc(b->max_len + 4);
    assert(b->data != NULL);
    b->data_len = 0;
    return b;
}

void p3n_buffer_free(p3n_buffer_t *buf)
{
    if ( buf != NULL ) {
        if ( buf->data != NULL ) {
            free(buf->data);
        }
        free(buf);
    }
}

void print_buffer(p3n_buffer_t *buf)
{
    int len;

    printf("buf: %p max_len: %d  data_len: %d  data: %p", buf, buf->max_len, buf->data_len, buf->data);
    len = buf->data_len;
    if ( len > buf->max_len ) {
        len = buf->max_len;
    }
    len += 4;  // print past the end of the data (to include the CRC)
    for ( int n = 0 ; n < len ; n ++ ) {
        if ( ( n % 16 ) == 0 ) {
            printf("\n%3d: ", n);
        }
        printf(" %02x", buf->data[n]);
    }
    printf("\n");
}

void print_buffer_as_string(p3n_buffer_t *buf)
{
    int len;
    char *cp;

    printf("buf: %p max_len: %d  data_len: %d  data: '", buf, buf->max_len, buf->data_len);
    // print the data
    len = buf->data_len;
    if ( len > buf->max_len ) {
        len = buf->max_len;
    }
    cp = buf->data;
    while ( ( *cp != '\0' ) && ( len-- > 0 ) ) {
        printf("%c", *(cp++));
    }
    printf("'\n");
}

void copy_string_to_buffer(p3n_buffer_t *buf, const char *str)
{
    char const *s;
    char *d;
    int len;

    s = str;
    d = buf->data;
    len = buf->max_len;
    while ( ( *s != '\0' ) && ( len-- > 0 ) ) {
        *(d++) = *(s++);
    }
    // buffer full
    buf->data_len = (uint16_t)(d - buf->data);
}

void erase_buffer(p3n_buffer_t *buf)
{
    uint32_t *p;
    int len;

    p = (uint32_t *)buf->data;
    // max_len is a multiple of 4 and has 4 extra bytes for CRC
    len = ( buf->max_len >> 2 ) + 1;
    while ( len-- > 0 ) {
        *(p++) = 0xDEADBEEF;
    }
    buf->data_len = 0;
}

static void crc32_start(uint32_t *data, uint num_words)
{
    // configure the sniffer
    dma_sniffer_enable(p3n_crc_dma_index, 0, true);
    dma_sniffer_set_data_accumulator(0xFFFFFFFF);
    // start the transfer
    dma_channel_transfer_from_buffer_now(p3n_crc_dma_index, data, num_words);
}

static uint32_t crc32_result(void)
{
    dma_channel_wait_for_finish_blocking(p3n_crc_dma_index);
    return dma_sniffer_get_data_accumulator();
}

void p3n_buffer_add_crc32(p3n_buffer_t *buf)
{
    // if data is not a multiple of 4 bytes, pad it with zeros
    while ( buf->data_len & ~3u ) {
        buf->data[buf->data_len++] = 0;
    }
    crc32_start((uint32_t *)buf->data, (buf->data_len >> 2));
    // append CRC to data
    *(uint32_t *)&(buf->data[buf->data_len]) = crc32_result();
}

bool p3n_buffer_check_crc32(p3n_buffer_t *buf)
{
    // data must be a multiple of 4 bytes
    assert((buf->data_len & ~3) == 0 );
    // and must be at least 4 (the CRC itself)
    if ( buf->data_len < 4 ) {
        return false;
    }
    crc32_start((uint32_t *)buf->data, (buf->data_len >> 2) - 1);
    if ( crc32_result() == *(uint32_t *)&(buf->data[buf->data_len]) ) {
        buf->data_len -= 4;
        return true;
    }
    return false;
}

#define SCOPE_CHAN_1  (10)


void __time_critical_func(p3n_chan_irq_handler)(p3n_chan_t *ch)
{
    p3n_cmd_t *cmd;

    gpio_put(SCOPE_CHAN_1, 1);
    cmd = ch->active_cmd;
    if ( cmd != NULL ) {
        if (( cmd->pio_cmd & PIO_CMD_MASK ) == PIO_CMD_RX ) {
            // previous command was receive
            dma_channel_abort(ch->dma_index);
            // capture last DMA address and compute message length
            char *addr = (char *) dma_channel_hw_addr(ch->dma_index)->write_addr;
            cmd->buf->data_len = (uint16_t)(addr - cmd->buf->data);
        } else {
            // previous command was transmit
            dma_channel_abort(ch->dma_index);
        }
        cmd->state = P3N_CMD_ST_DONE;
        ch->active_cmd = NULL;
    }
    chan_start_next_cmd(ch);
    gpio_put(SCOPE_CHAN_1, 0);
}

/*********************************************************
 * This IRQ handler is a dispatcher; it calls a channel
 * handler for any channel(s) that need serviced.
  */
void __time_critical_func(p3n_pio_irq_handler)(void)
{
    p3n_chan_t *ch;

    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        if ( pio_interrupt_get(p3n_pio, ch->sm_index) ) {
            p3n_chan_irq_handler(ch);
            pio_interrupt_clear(p3n_pio, ch->sm_index);
        }
    }
}


bool p3n_init(void)
{
    p3n_chan_t *ch;
    dma_channel_config dma_config;
    int rv;

    // make sure we have a clean slate
    p3n_uninit();
    // find a PIO with enough memory and free state machines
    for ( uint n = 0; n < NUM_PIOS ; n++ ) {
        if ( try_pio(n) ) {
            break;
        }
    }
    if ( p3n_pio == NULL ) {
        // couldn't find a PIO with enough resources
        p3n_uninit();
        printf("insufficient PIO resources\n");
        return false;
    }
    // claim DMA channel for each state machine
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        rv = dma_claim_unused_channel(false);
        if ( rv < 0 ) {
            p3n_uninit();
            printf("insufficient DMA resources\n");
            return false;
        }
        ch->dma_index = (uint8_t)rv;
    }
    // we do CRC calculations by dma'ing the data to
    // a dummy address with the sniffer enabled
    // claim a DMA channel for CRC calculations
    rv = dma_claim_unused_channel(false);
    if ( rv < 0 ) {
        p3n_uninit();
        printf("insufficient DMA resources\n");
        return false;
    }
    p3n_crc_dma_index = (uint)rv;
    dma_config = dma_channel_get_default_config(p3n_crc_dma_index);
    // default config is already almost perfect:
    //  incr read addr, don't incr write addr, 32-bit transfers
    //  DREQ_FORCE (run at max speed), no byte swap, low priority
    // make the only change needed
    channel_config_set_sniff_enable(&dma_config, true);
    // send the config to the channel
    dma_channel_set_config(p3n_crc_dma_index, &dma_config, false);
    // point at our dummy address
    dma_channel_set_write_addr(p3n_crc_dma_index, &crc_dummy, false);

    // enable interrupts
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        pio_set_irqn_source_enabled(p3n_pio, p3n_pio_irq_num, pis_interrupt0+ch->sm_index, true);
    }
    irq_set_enabled(p3n_irq_num, true);

#if 1
    // print results - this will be deleted later
    printf("Using PIO # %d, sm/dma ", pio_get_index(p3n_pio));
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        printf(" %d/%d", ch->sm_index, ch->dma_index);
    }
    printf("\nPIO code @: %d, CRC DMA: %d\n", p3n_code_offset, p3n_crc_dma_index);
    printf("MCU IRQ: %d, PIO IRQ: %d\n", p3n_irq_num, p3n_pio_irq_num);
#endif

    return true;
}


static bool try_pio(uint index)
{
    int rv;

    // attempt to load the program
    p3n_pio = pio_get_instance(index);
    rv = pio_add_program(p3n_pio, &p3n_rxtx_program);
    if ( rv < 0 ) {
        p3n_uninit();
        printf("couldn't load program\n");
        return false;
    }
    p3n_code_offset = (uint)rv;
    // attempt to find an unused interrupt vector for this PIO
    for ( uint n = 0 ; n < NUM_PIO_IRQS ; n++ ) {
        if ( ! irq_has_handler((uint)pio_get_irq_num(p3n_pio, n))) {
            p3n_pio_irq_num = n;
            p3n_irq_num = pio_get_irq_num(p3n_pio, p3n_pio_irq_num);
            irq_set_exclusive_handler(p3n_irq_num, p3n_pio_irq_handler);
            break;
        }
    }
    if ( p3n_irq_num == IRQ_COUNT ) {
        p3n_uninit();
        printf("couldn't find interrupt\n");
        return false;
    }
    // attempt to claim the state machines
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        rv = pio_claim_unused_sm(p3n_pio, false);
        if ( rv < 0 ) {
            p3n_uninit();
            printf("couldn't reserve state machines\n");
            return false;
        }
        p3n_chans[n].sm_index = (uint8_t)rv;
    }
    return true;
}

static void chan_reset(p3n_chan_t *ch)
{
    ch->sm_index = NUM_PIO_STATE_MACHINES;
    ch->dma_index = NUM_DMA_CHANNELS;
    ch->rx_pin = P3N_NO_PIN;
    ch->tx_pin = P3N_NO_PIN;
    ch->tx_ena_pin = P3N_NO_PIN;
    ch->queue_len = 0;
    ch->queue_in = 0;
    ch->queue_out = 0;
    ch->cmd_queue = NULL;
    ch->active_cmd = NULL;
}

void p3n_uninit(void)
{
    p3n_chan_t *ch;

    if ( p3n_pio == NULL ) {
        // nothing initialized
        // make sure channel data is cleared
        for ( uint n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
            chan_reset(&(p3n_chans[n]));
        }
        return;
    }
    // release per-channel allocated resources first
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        if ( ch->sm_index != NUM_PIO_STATE_MACHINES ) {
            pio_sm_set_enabled(p3n_pio, ch->sm_index, false);
            pio_set_irqn_source_enabled(p3n_pio, p3n_pio_irq_num, pis_interrupt0 + ch->sm_index, false);
            pio_sm_unclaim(p3n_pio, ch->sm_index);
            ch->sm_index = NUM_PIO_STATE_MACHINES;
        }
        if ( ch->dma_index != NUM_DMA_CHANNELS ) {
            dma_channel_abort(ch->dma_index);
            dma_channel_unclaim(ch->dma_index);
            ch->dma_index = NUM_DMA_CHANNELS;
        }
        if ( ch->rx_pin != P3N_NO_PIN ) {
            gpio_deinit(ch->rx_pin);
        }
        if ( ch->tx_pin != P3N_NO_PIN ) {
            gpio_deinit(ch->tx_pin);
        }
        if ( ch->tx_ena_pin != P3N_NO_PIN ) {
            gpio_deinit(ch->tx_ena_pin);
        }
        if ( ch->cmd_queue != NULL ) {
            free(ch->cmd_queue);
        }
        chan_reset(ch);
    }
    // now release global resources
    irq_set_enabled(p3n_irq_num, false);
    irq_remove_handler(p3n_irq_num, p3n_pio_irq_handler);
    if ( p3n_code_offset != PIO_INSTRUCTION_COUNT ) {
        pio_remove_program(p3n_pio, &p3n_rxtx_program, p3n_code_offset);
        p3n_code_offset = PIO_INSTRUCTION_COUNT;
    }
    if ( p3n_crc_dma_index != NUM_DMA_CHANNELS ) {
        dma_channel_abort(p3n_crc_dma_index);
        dma_channel_unclaim(p3n_crc_dma_index);
        p3n_crc_dma_index = NUM_DMA_CHANNELS;
    }
    // mark system as un-initialized
    p3n_pio = NULL;
}

static void __time_critical_func(chan_start_next_cmd)(p3n_chan_t *ch)
{
    uint8_t index;
    p3n_cmd_t *cmd;

    index = ch->queue_out;
    cmd = &(ch->cmd_queue[index]);
    if ( cmd->state != P3N_CMD_ST_QUEUED ) {
        // nothing to do
        ch->active_cmd = NULL;
        return;
    }
    if ( ( cmd->pio_cmd & PIO_CMD_MASK ) == PIO_CMD_RX ) {
        // receive command
        // start the DMA transfer
        dma_channel_configure(ch->dma_index, &ch->rx_dma_config, cmd->buf->data,
                                ((char *)&(p3n_pio->rxf[ch->sm_index]))+(4-BYTES_PER_WORD),
                                cmd->buf->max_len / BYTES_PER_WORD, true);
        // send command to the PIO
        pio_sm_put(p3n_pio, ch->sm_index, cmd->pio_cmd);
    } else {
        // transmit command
        // send command to the PIO
        pio_sm_put(p3n_pio, ch->sm_index, cmd->pio_cmd);
        // start the DMA transfer (data must follow command)
        dma_channel_configure(ch->dma_index, &ch->tx_dma_config,
                                &(p3n_pio->txf[ch->sm_index]), cmd->buf->data,
                                (cmd->buf->data_len+(BYTES_PER_WORD-1))/BYTES_PER_WORD, true);
    }
    ch->active_cmd = cmd;
    cmd->state = P3N_CMD_ST_ACTIVE;
    // decrement queue pointer; point to next entry
    if ( index > 0 ) {
        ch->queue_out = index - 1;
    } else {
        ch->queue_out = ch->queue_len - 1;
    }
}

p3n_retval_t p3n_receive(uint ch_num, p3n_buffer_t *buf, uint32_t timeout_us)
{
    p3n_chan_t *ch;
    int index;
    uint32_t start;

    // validate inputs
    if ( timeout_us > TM_MAX_TIME_US ) return P3N_ERR_BAD_TIMEOUT;
    if ( ch_num >= P3N_NUM_CHAN) return P3N_ERR_BAD_CHAN;
    ch = &(p3n_chans[ch_num]);
    if ( ch->active_cmd != NULL ) return P3N_ERR_CHAN_BUSY;
    index = p3n_queue_receive_cmd(ch_num, buf, 0u);
    if ( index < P3N_SUCCESS ) {
        return index;
    }
    start = tm_get_timestamp();
    while ( p3n_get_cmd_state(ch_num, index) != P3N_CMD_ST_DONE ) {
        if ( tm_us_since(start) > timeout_us ) {
            p3n_channel_rx_abort(ch_num);
        }        
    }
    p3n_cmd_release(ch_num, index);
    return P3N_SUCCESS;
}

p3n_retval_t p3n_transmit(uint ch_num, p3n_buffer_t *buf)
{
    p3n_chan_t *ch;
    int index;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return P3N_ERR_BAD_CHAN;
    ch = &(p3n_chans[ch_num]);
    if ( ch->active_cmd != NULL ) return P3N_ERR_CHAN_BUSY;
    index = p3n_queue_transmit_cmd(ch_num, buf, 0u);
    if ( index < P3N_SUCCESS ) {
        return index;
    }
    while ( p3n_get_cmd_state(ch_num, index) != P3N_CMD_ST_DONE );
    p3n_cmd_release(ch_num, index);
    return P3N_SUCCESS;
}

static int queue_cmd(p3n_chan_t *ch, p3n_buffer_t *buf, uint32_t delay, uint32_t pio_cmd)
{
    uint32_t save;
    uint8_t index;
    p3n_cmd_t *cmd;

    // validate inputs
    if ( delay > MAX_POST_CMD_DELAY ) return P3N_ERR_BAD_DELAY;
    if ( ch->cmd_queue == NULL ) return P3N_ERR_CHAN_UNINIT;
    // start atomic region
    save = save_and_disable_interrupts();
    index = ch->queue_in;
    cmd = &(ch->cmd_queue[index]);
    if ( cmd->state != P3N_CMD_ST_AVAIL ) {
        restore_interrupts_from_disabled(save);
        return P3N_ERR_QUEUE_FULL;
    }
    cmd->state = P3N_CMD_ST_QUEUED;
    cmd->buf = buf;
    cmd->pio_cmd = (delay << (POST_CMD_DELAY_SHIFT)) | pio_cmd;
    // decrement queue pointer
    if ( index > 0 ) {
        ch->queue_in = index - 1;
    } else {
        ch->queue_in = ch->queue_len - 1;
    }
    restore_interrupts_from_disabled(save);
    if ( ch->active_cmd == NULL ) {
        chan_start_next_cmd(ch);
    }
    return index;
}

int p3n_queue_receive_cmd(uint ch_num, p3n_buffer_t *buf, uint32_t delay)
{
    p3n_chan_t *ch;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return P3N_ERR_BAD_CHAN;
    ch = &(p3n_chans[ch_num]);
    if ( ch->rx_pin == P3N_NO_PIN ) return P3N_ERR_BAD_PIN;
    if ( buf == NULL ) return P3N_ERR_BAD_BUFFER;
    if ( buf->data_len > 0 ) return P3N_ERR_BAD_BUFFER;
    if ( buf->max_len < 4 ) return P3N_ERR_BAD_BUFFER;
    return queue_cmd(ch, buf, delay, PIO_CMD_RX);
}

int p3n_queue_transmit_cmd(uint ch_num, p3n_buffer_t *buf, uint32_t delay)
{
    p3n_chan_t *ch;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return P3N_ERR_BAD_CHAN;
    ch = &(p3n_chans[ch_num]);
    if ( ch->tx_pin == P3N_NO_PIN ) return P3N_ERR_BAD_PIN;
    if ( buf == NULL ) return P3N_ERR_BAD_BUFFER;
    if ( buf->data_len == 0 ) return P3N_ERR_BAD_BUFFER;
    return queue_cmd(ch, buf, delay, PIO_CMD_TX);
}

p3n_cmd_state_t p3n_get_cmd_state(uint ch_num, int cmd_index)
{
    p3n_chan_t *ch;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return P3N_CMD_ST_ERROR;
    if ( cmd_index < 0 ) return P3N_CMD_ST_ERROR;
    ch = &(p3n_chans[ch_num]);
    if ( cmd_index >= ch->queue_len ) return P3N_CMD_ST_ERROR;
    return ch->cmd_queue[cmd_index].state;
}

p3n_buffer_t *p3n_get_cmd_buffer(uint ch_num, int cmd_index)
{
    p3n_chan_t *ch;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return NULL;
    if ( cmd_index < 0 ) return NULL;
    ch = &(p3n_chans[ch_num]);
    if ( cmd_index >= ch->queue_len ) return NULL;
    return ch->cmd_queue[cmd_index].buf;
}

bool p3n_cmd_release(uint ch_num, int cmd_index)
{
    p3n_chan_t *ch;
    p3n_cmd_t *cmd;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return false;
    if ( cmd_index < 0 ) return false;
    ch = &(p3n_chans[ch_num]);
    if ( cmd_index >= ch->queue_len ) return false;
    cmd = &(ch->cmd_queue[cmd_index]);
    if ( cmd->state != P3N_CMD_ST_DONE ) return false;
    cmd->state = P3N_CMD_ST_AVAIL;
    return true;
}

void p3n_channel_rx_abort(uint ch_num)
{
    p3n_chan_t *ch;
    uint pc;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return;
    ch = &(p3n_chans[ch_num]);
    pc = pio_sm_get_pc(p3n_pio, ch->sm_index);
    if ( (pc - p3n_code_offset) == p3n_rxtx_offset_rx_wait_4_low ) {
        // generate the end-of-message interrupt
        pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_irq_set(true, 0));
        // then skip the delay and go right to the entry point for next command
        pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_jmp(p3n_code_offset+p3n_rxtx_offset_entry));
    }
}

void p3n_channel_delay_abort(uint ch_num)
{
    p3n_chan_t *ch;
    uint pc;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return;
    ch = &(p3n_chans[ch_num]);
    pc = pio_sm_get_pc(p3n_pio, ch->sm_index);
    if ( (pc - p3n_code_offset) == p3n_rxtx_offset_post_cmd_delay ) {
        pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_jmp(p3n_code_offset+p3n_rxtx_offset_entry));
    }
}

void p3n_channel_full_abort(uint ch_num)
{
    p3n_chan_t *ch;
    p3n_cmd_t *cmd;

    // validate inputs
    if ( ch_num >= P3N_NUM_CHAN) return;
    ch = &(p3n_chans[ch_num]);
    if ( ch->cmd_queue == NULL ) return;
    // ensure that an IRQ won't start another command
    ch->cmd_queue[0].state = P3N_CMD_ST_AVAIL;
    ch->queue_out = 0;
    // stop the PIO state machine and the DMA
    pio_sm_set_enabled(p3n_pio, ch->sm_index, false);
    dma_channel_abort(ch->dma_index);
    // set initial state and direction of the tx and tx_ena pins
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_mov(pio_osr, pio_null));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_out(pio_pindirs, 1));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_mov_not(pio_osr, pio_null));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_out(pio_pins, 1));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_set(pio_pins, 0));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_set(pio_pindirs, 1));
    // flush fifos
    pio_sm_drain_tx_fifo(p3n_pio, ch->sm_index);
    while( ! pio_sm_is_rx_fifo_empty(p3n_pio, ch->sm_index) ) {
        pio_sm_get(p3n_pio, ch->sm_index);
    }
    // flush command queue
    for ( int n = 0 ; n < ch->queue_len ; n++ ) {
        cmd = &(ch->cmd_queue[n]);
        cmd->state = P3N_CMD_ST_AVAIL;
        cmd->buf = NULL;
    }
    ch->queue_in = 0;
    ch->queue_out = 0;
    // mark channel idle
    ch->active_cmd = NULL;
    // restart the state machine
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_jmp(p3n_code_offset+p3n_rxtx_offset_entry));
    pio_sm_set_enabled(p3n_pio, ch->sm_index, true);
}

bool p3n_configure_chan(uint ch_num, p3n_port_t const *port, uint bitrate, uint8_t queue_len)
{
    p3n_chan_t *ch;
    p3n_cmd_t *cmd;
    pio_sm_config c;

    assert(ch_num < P3N_NUM_CHAN);
    assert(queue_len > 1 );
    ch = &(p3n_chans[ch_num]);
    if ( ch->active_cmd != NULL ) {
        // channel busy
        return false;
    }
    if ( ( port->rx_pin == P3N_NO_PIN ) && ( port->tx_pin == P3N_NO_PIN ) ) {
        // whats the point if you can neither send nor receive?
        return false;
    }
    // stop the state machine and any DMA
    pio_sm_set_enabled(p3n_pio, ch->sm_index, false);
    dma_channel_abort(ch->dma_index);
    // unmap any previously used pins
    if ( ch->rx_pin != P3N_NO_PIN ) {
        gpio_deinit(ch->rx_pin);
        ch->rx_pin = P3N_NO_PIN;
    }
    if ( ch->tx_pin != P3N_NO_PIN ) {
        gpio_deinit(ch->tx_pin);
        ch->tx_pin = P3N_NO_PIN;
    }
    if ( ch->tx_ena_pin != P3N_NO_PIN ) {
        gpio_deinit(ch->tx_ena_pin);
        ch->tx_ena_pin = P3N_NO_PIN;
    }
    // allocate space for command queue
    if ( ch->queue_len != queue_len ) {
        if ( ch->cmd_queue != NULL ) {
            free(ch->cmd_queue);
            ch->queue_len = 0;
        }
        ch->cmd_queue = malloc(queue_len*sizeof(p3n_cmd_t));
        if ( ch->cmd_queue == NULL ) {
            // no memory for queue
            return false;
        }
        ch->queue_len = queue_len;
    }
    // intialize command queue
    ch->queue_in = 0;
    ch->queue_out = 0;
    for ( uint n = 0 ; n < ch->queue_len ; n++ ) {
        cmd = &(ch->cmd_queue[n]);
        cmd->state = P3N_CMD_ST_AVAIL;
        cmd->buf = NULL;
    }
    // store new pin mapping
    ch->rx_pin = port->rx_pin;
    ch->tx_pin = port->tx_pin;
    ch->tx_ena_pin = port->tx_ena_pin;
    // prepare DMA configurations for RX and TX
    ch->rx_dma_config = dma_channel_get_default_config(ch->dma_index);
    channel_config_set_read_increment(&ch->rx_dma_config, false);
    channel_config_set_write_increment(&ch->rx_dma_config, true);
    channel_config_set_dreq(&ch->rx_dma_config, pio_get_dreq(p3n_pio, ch->sm_index, false));
    channel_config_set_transfer_data_size(&ch->rx_dma_config, DMA_SIZE);
    ch->tx_dma_config = dma_channel_get_default_config(ch->dma_index);
    channel_config_set_read_increment(&ch->tx_dma_config, true);
    channel_config_set_write_increment(&ch->tx_dma_config, false);
    channel_config_set_dreq(&ch->tx_dma_config, pio_get_dreq(p3n_pio, ch->sm_index, true));
    channel_config_set_transfer_data_size(&ch->tx_dma_config, DMA_SIZE);
    // prepare PIO configuration
    c = p3n_rxtx_program_get_default_config(p3n_code_offset);
    if ( ch->rx_pin != P3N_NO_PIN ) {
        // enable pull up on the receive pin (it might be bidirectional)
        gpio_pull_up(ch->rx_pin);
        // set pad mux so PIO gets its value from the pin
        pio_gpio_init(p3n_pio, ch->rx_pin);
        // set up the "in" and "jmp" pin mapping
        sm_config_set_in_pins(&c, ch->rx_pin);
        sm_config_set_jmp_pin(&c, ch->rx_pin);
    } else {
        // no receive pin, map to transmit pin
        sm_config_set_in_pins(&c, ch->tx_pin);
        sm_config_set_jmp_pin(&c, ch->tx_pin);
    }
    if ( ch->tx_pin != P3N_NO_PIN ) {
        // enable pull up on the transmit pin (it might be bidirectional)
        gpio_pull_up(ch->tx_pin);
        // set pad mux so pin gets its value from PIO
        pio_gpio_init(p3n_pio, ch->tx_pin);
        // set up the "out" pin mapping
        sm_config_set_out_pins(&c, ch->tx_pin, 1);
    } else {
        // if no transmit pin, "out" mapping has zero pins
        sm_config_set_out_pins(&c, 0, 0);
    }
    // tx_ena uses the "set" pin mapping
    if ( ch->tx_ena_pin != P3N_NO_PIN ) {
        // enable pull down on the transmit enable pin (don't transmit)
        gpio_pull_down(ch->tx_ena_pin);
        // set pad mux so pin gets its value from PIO
        pio_gpio_init(p3n_pio, ch->tx_ena_pin);
        // set up the "set" pin mapping
        sm_config_set_set_pins(&c, ch->tx_ena_pin, 1);
    } else {
        // if no transmit enable pin, "set" mapping has zero pins
        sm_config_set_set_pins(&c, 0, 0);
    }
#if 1
    // map the sideset pin (this is temporary for debug only)
    sm_config_set_sideset_pins(&c, ch_num + 6);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(p3n_pio, ch_num + 6);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(p3n_pio, ch->sm_index, ch_num + 6, 1, true);
#endif
    // set the bit rate
    uint32_t pio_clk = bitrate * p3n_clocks_per_bit;
    double div = 125000000.0 / pio_clk;
    sm_config_set_clkdiv(&c, (float)div);

    // Load our configuration, and jump to the start of the program (but don't run)
    pio_sm_init(p3n_pio, ch->sm_index, p3n_code_offset+p3n_rxtx_offset_entry, &c);
    // set initial state and direction of the tx and tx_ena pins
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_mov(pio_osr, pio_null));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_out(pio_pindirs, 1));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_mov_not(pio_osr, pio_null));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_out(pio_pins, 1));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_set(pio_pins, 0));
    pio_sm_exec(p3n_pio, ch->sm_index, pio_encode_set(pio_pindirs, 1));
    // Set the state machine running
    pio_sm_set_enabled(p3n_pio, ch->sm_index, true);
    return true;
}


/*************************************************
 * Port data - normally provided by application
 */

const p3n_port_t port_bus1  = { 16, 16, 17 };
const p3n_port_t port_bus2  = { 22, 22, 28 };
const p3n_port_t port_up = { 5, 4, P3N_NO_PIN };
const p3n_port_t port_dn = { 3, 2, P3N_NO_PIN };
const p3n_port_t port_rx = { 9, P3N_NO_PIN, P3N_NO_PIN };
const p3n_port_t port_tx = { P3N_NO_PIN, 9, P3N_NO_PIN };

// testing stuff

#define SCOPE_CHAN_1  (10)
#define SCOPE_CHAN_2  (14)
#define SCOPE_CHAN_3  (6)
#define SCOPE_CHAN_4  (8)
#define SCOPE_CHAN_5  (12)

#define BITRATE 5000000

#define TX 1
#define RX 2
#define NONE 0
#define TESTMODE NONE

#define NUM_BUFFERS 6


#include "timing.h"

p3n_buffer_t *buffers[NUM_BUFFERS];

void p3n_test(void)
{
//    uint old_count;
    bool rb;
//    uint32_t start, now, dt;

    printf("Hello from pi3net\n");
    for ( int n = 0 ; n < NUM_BUFFERS ; n++ ) {
        buffers[n] = p3n_buffer_new(100);
        assert(buffers[n] != NULL);
    }

    
    gpio_init(SCOPE_CHAN_1);
    gpio_put(SCOPE_CHAN_1, 0);
    gpio_set_dir(SCOPE_CHAN_1, GPIO_OUT);
    pio_sm_claim(pio0, 1);
    rb = p3n_init();
    assert(rb);
    // transmit channel
    rb = p3n_configure_chan(0, &port_tx, BITRATE, 10);
    assert(rb);
    // receive channel (on same pin, I'm looping back)
    rb = p3n_configure_chan(1, &port_rx, BITRATE, 10);
    assert(rb);

    while(1) {
        int index[4];

        copy_string_to_buffer(buffers[0], "msg1");
        copy_string_to_buffer(buffers[1], "and2");
        erase_buffer(buffers[2]);
        erase_buffer(buffers[3]);
//        print_buffer(buffers[0]);
//        print_buffer(buffers[1]);
//        print_buffer(buffers[2]);
//        print_buffer(buffers[3]);
        printf("start receive\n");
        gpio_put(SCOPE_CHAN_1, 1);
        index[0] = p3n_queue_receive_cmd(1, buffers[2], 10);
        index[1] = p3n_queue_receive_cmd(1, buffers[3], 10);
//        printf("receive state = %d\n", p3n_chans[1].state);
//        printf("start transmit\n");
        index[2] = p3n_queue_transmit_cmd(0, buffers[0], 15);
        index[3] = p3n_queue_transmit_cmd(0, buffers[1], 20);
        gpio_put(SCOPE_CHAN_1, 0);
    
//        while ( p3n_get_cmd_state(0, index[3]) != P3N_CMD_ST_DONE ) {
//            printf("%d %d %d %d\n", p3n_get_cmd_state(1, index[0]),
//                                    p3n_get_cmd_state(1, index[1]),
//                                    p3n_get_cmd_state(0, index[2]),
//                                    p3n_get_cmd_state(0, index[3]) );
//        }
//        printf("transmit state = %d\n", p3n_chans[0].state);
//        printf("transmitter state is now idle\n");
//            printf("%d:%d %d:%d %d:%d %d:%d\n", index[0], p3n_get_cmd_state(1, index[0]),
//                                                index[1], p3n_get_cmd_state(1, index[1]),
//                                                index[2], p3n_get_cmd_state(0, index[2]),
//                                                index[3], p3n_get_cmd_state(0, index[3]) );
        tm_delay_ms(5);
        if ( p3n_get_cmd_state(1, index[1]) != P3N_CMD_ST_DONE ) {
            printf("aborting\n");
            p3n_channel_rx_abort(1);
        }
        tm_delay_ms(5);
            printf("%d:%d %d:%d %d:%d %d:%d\n", index[0], p3n_get_cmd_state(1, index[0]),
                                                index[1], p3n_get_cmd_state(1, index[1]),
                                                index[2], p3n_get_cmd_state(0, index[2]),
                                                index[3], p3n_get_cmd_state(0, index[3]) );
        printf("receive active_cmd = %p\n", p3n_chans[1].active_cmd);
//        print_buffer(buffers[0]);
//        print_buffer(buffers[1]);
//        print_buffer(buffers[2]);
//        print_buffer(buffers[3]);
if ( index[1] < 0 ) {
    while(1);
}
        p3n_cmd_release(1, index[0]);
        p3n_cmd_release(1, index[1]);
        p3n_cmd_release(0, index[2]);
        p3n_cmd_release(0, index[3]);
            printf("%d:%d %d:%d %d:%d %d:%d\n", index[0], p3n_get_cmd_state(1, index[0]),
                                                index[1], p3n_get_cmd_state(1, index[1]),
                                                index[2], p3n_get_cmd_state(0, index[2]),
                                                index[3], p3n_get_cmd_state(0, index[3]) );
        tm_delay_ms(5000);
        //while(1);
    
    };


#if (TESTMODE == TX )
    // prepare the transmit links
    rb = p3n_assign_sm_to_link(&links[0]);
    assert(rb);
    rb = p3n_assign_sm_to_link(&links[1]);
    assert(rb);
    rb = p3n_assign_sm_to_link(&links[2]);
    assert(rb);
    rb = p3n_assign_sm_to_link(&links[4]);
    assert(rb);
    copy_string_to_buffer(buffers[0], "bus1, using buffers[0]");
    copy_string_to_buffer(buffers[1], "bus2, using buffers[1]");
    copy_string_to_buffer(buffers[2], "neighbor up, using buffers[2]");
    copy_string_to_buffer(buffers[4], "neighbor down, using buffers[4]");
printf("Buffer 0 before CRC\n");
print_buffer(buffers[0]);
    buffer_crc_set(buffers[0]);
    buffer_crc_set(buffers[1]);
    buffer_crc_set(buffers[2]);
    buffer_crc_set(buffers[4]);
printf("Buffer 0 after CRC\n");
print_buffer(buffers[0]);
#endif

#if (TESTMODE == RX )
    // prepare the receive links
    rb = p3n_assign_sm_to_link(&links[0]);
    assert(rb);
    rb = p3n_assign_sm_to_link(&links[1]);
    assert(rb);
    rb = p3n_assign_sm_to_link(&links[3]);
    assert(rb);
    rb = p3n_assign_sm_to_link(&links[5]);
    assert(rb);
    erase_buffer(buffers[0]);
    erase_buffer(buffers[1]);
    erase_buffer(buffers[3]);
    erase_buffer(buffers[5]);
#endif
#if (TESTMODE == TX)

    gpio_put(SCOPE_CHAN_1, 1);
    rb = p3n_start_link(&links[0], BITRATE);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 0);
    rb = p3n_start_link(&links[1], BITRATE);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 1);
    rb = p3n_start_link(&links[2], BITRATE);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 0);
    rb = p3n_start_link(&links[4], BITRATE);
    assert(rb);
    delay_ms(1);
    while ( 1 ) {
        gpio_put(SCOPE_CHAN_1, 1);
        rb = p3n_transmit(&links[0], buffers[0]);
        assert(rb);
        gpio_put(SCOPE_CHAN_1, 0);
        delay_ms(1000);

        gpio_put(SCOPE_CHAN_1, 1);
        rb = p3n_transmit(&links[1], buffers[1]);
        assert(rb);
        gpio_put(SCOPE_CHAN_1, 0);
        delay_ms(1000);

        gpio_put(SCOPE_CHAN_1, 1);
        rb = p3n_transmit(&links[2], buffers[2]);
        assert(rb);
        gpio_put(SCOPE_CHAN_1, 0);
        delay_ms(1000);

        gpio_put(SCOPE_CHAN_1, 1);
        rb = p3n_transmit(&links[4], buffers[4]);
        assert(rb);
        gpio_put(SCOPE_CHAN_1, 0);
        delay_ms(1000);
    }
#endif

#if (TESTMODE == RX)
    gpio_put(SCOPE_CHAN_1, 1);
    rb = p3n_start_link(&links[0], BITRATE);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 0);
    rb = p3n_start_link(&links[1], BITRATE);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 1);
    rb = p3n_start_link(&links[3], BITRATE);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 0);
    rb = p3n_start_link(&links[5], BITRATE);
    assert(rb);
    delay_ms(1);
    gpio_put(SCOPE_CHAN_1, 1);
    rb = p3n_receive(&links[0], buffers[0]);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 0);
    rb = p3n_receive(&links[1], buffers[1]);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 1);
    rb = p3n_receive(&links[3], buffers[3]);
    assert(rb);
    gpio_put(SCOPE_CHAN_1, 0);
    rb = p3n_receive(&links[5], buffers[5]);
    assert(rb);

    while ( 1 ) {
        for ( int n = 0 ; n < 6 ; n++ ) {
            if ( n == 2 ) continue;
            if ( n == 4 ) continue;
            if ( links[n].status != P3N_LINK_LISTEN ) {
                gpio_put(SCOPE_CHAN_1, 1);
                printf("\nLink %d:\n", n);
                print_buffer(links[n].buffer);
                if ( buffer_crc_check(links[n].buffer) ) {
                    printf("CRC pass\n");
                } else {
                    printf("ERROR: CRC FAILED!\n");
                }
                print_buffer_as_string(links[n].buffer);
                erase_buffer(links[n].buffer);
                rb = p3n_receive(&links[n], buffers[n]);
                assert(rb);
                gpio_put(SCOPE_CHAN_1, 0);
            }
        }
    }
#endif
}


