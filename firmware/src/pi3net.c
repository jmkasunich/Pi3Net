#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pi3net.h"
#include "pi3net.pio.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/pio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"


#if (p3n_bits_per_word == 32)
#define DMA_SIZE DMA_SIZE_32
#elif (p3n_bits_per_word == 16)
#define DMA_SIZE DMA_SIZE_16
#elif (p3n_bits_per_word == 8)
#define DMA_SIZE DMA_SIZE_8
#else
#error "bad bits-per-word"
#endif

#define BYTES_PER_WORD  (p3n_bits_per_word/8)



#define SCOPE_CHAN_1  (10)
#define SCOPE_CHAN_2  (14)
#define SCOPE_CHAN_3  (6)
#define SCOPE_CHAN_4  (8)
#define SCOPE_CHAN_5  (12)

#define BITRATE 10000000

#define TX 1
#define RX 2
#define NONE 0
#define TESTMODE NONE



/*************************************************
 * Global data - initialized by p3n_init()
 */

static PIO p3n_pio = NULL;
static int p3n_crc_dma_index = -1;
static int p3n_code_offset = -1;
static p3n_chan_t p3n_chans[P3N_NUM_CHAN];
static uint32_t crc_dummy;

/*************************************************
 * Port data - normally provided by application
 */

const p3n_port_t p3n_bus1  = { 16, 16, 17 };
const p3n_port_t p3n_bus2  = { 22, 22, 28 };
const p3n_port_t p3n_tx_up = { NO_PIN, 4, NO_PIN };
const p3n_port_t p3n_rx_up = { 5, NO_PIN, NO_PIN };
const p3n_port_t p3n_tx_dn = { NO_PIN, 2, NO_PIN };
const p3n_port_t p3n_rx_dn = { 3, NO_PIN, NO_PIN };

const p3n_port_t null_port = { NO_PIN, NO_PIN, NO_PIN };


// FIXME - this code should live somewhere else
// I'm astonished that the SDK doesn't have a proper busywait

void delay_clks(uint32_t clks)
{
    uint32_t start, now, dt;

    // systick rolls over at 2^24 = 134ms
    // only an idiot busywaits that long
    // clamp it at 100ms
    if ( clks > 12500000 ) {
        clks = 12500000;
    }
    // it takes about 25 clocks to run this code when clks = 0
    if ( clks > 25 ) {
        clks -= 25;
    } else {
        clks = 0;
    }
    start = systick_hw->cvr;
    do {
        now = systick_hw->cvr;
        dt = (start - now) & 0x00FFFFFF;
    } while ( dt < clks );
}

void delay_us(uint32_t us)
{
    // systick rolls over at 2^24 = 134ms
    // only an idiot busywaits that long
    // clamp it at 100ms
    if ( us > 100000 ) {
        us = 100000;
    }
    if ( us == 0 ) {
        return;
    }
    // subtract 26 clocks for call overhead
    delay_clks(us*125-26);
}

void delay_ms(uint32_t ms)
{
    // sometimes I'm an idiot and do long busywaits
    while ( ms > 0 ) {
        delay_us(1000);
        ms--;
    }
}


p3n_buffer_t *p3n_buffer_new(uint32_t max_msg_len_bytes)
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

    printf("buf: %p max_len: %d  data_len: %d  data: %p\n", buf, buf->max_len, buf->data_len, buf->data);
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
    buf->data_len = d - buf->data;
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

static void crc32_start(uint32_t *data, int num_words)
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
    while ( buf->data_len & ~3 ) {
        buf->data[buf->data_len++] = 0;
    }
    crc32_start((uint32_t *)buf->data, buf->data_len >>2);
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


void p3n_reset(void)
{
    p3n_chan_t *ch;

    // initialize to "nothing claimed"
    p3n_pio = NULL;
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        ch->sm_index = -1;
        ch->dma_index = -1;
        ch->state = P3N_CHAN_ST_UNINIT;
        //ch->cmd = NULL;
        ch->buf = NULL;
    }
    p3n_code_offset = -1;
    p3n_crc_dma_index = -1;
}

static bool try_pio(PIO pio);

bool p3n_init(void)
{
    p3n_chan_t *ch;
    dma_channel_config dma_config;

    // find a PIO with enough memory and free state machines
    if ( ! try_pio(pio0) ) {
        p3n_uninit();
        if ( ! try_pio(pio1) ) {
            p3n_uninit();
            return false;
        }
    }
    // we do CRC calculations by dma'ing the data to a
    // dummy address with the sniffer enabled
    // claim a DMA channel for CRC calculations
    p3n_crc_dma_index = dma_claim_unused_channel(false);
    if ( p3n_crc_dma_index < 0 ) {
        p3n_uninit();
        return false;
    }
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
    // claim DMA channel for each state machine
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        ch->dma_index = dma_claim_unused_channel(false);
        if ( ch->dma_index < 0 ) {
            p3n_uninit();
            return false;
        }
        ch->state = P3N_CHAN_ST_IDLE;
        ch->port = &null_port;
    }
#if 1
    // print results - this will be deleted later
    printf("Using PIO # %d, sm/dma ", pio_get_index(p3n_pio));
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        printf(" %d/%d", ch->sm_index, ch->dma_index);
    }
    printf("\nPIO code @: %d, CRC DMA: %d\n", p3n_code_offset, p3n_crc_dma_index);
#endif
    return true;
}

void p3n_uninit(void)
{
    p3n_chan_t *ch;
    if ( p3n_pio == NULL ) {
        // either initial boot, or already un-initialized
        // nothing to do
        return;
    }
    // release allocated resources
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        ch = &(p3n_chans[n]);
        if ( ch->sm_index >= 0 ) {
            pio_sm_unclaim(p3n_pio, ch->sm_index);
            ch->sm_index = -1;
        }
        if ( ch->dma_index >= 0 ) {
            dma_channel_unclaim(ch->dma_index);
            ch->dma_index = -1;
        }
        //ch->cmd = NULL;
        ch->buf = NULL;
        ch->state = P3N_CHAN_ST_UNINIT;
    }
    // unload the PIO program
    if ( p3n_code_offset >= 0 ) {
        pio_remove_program(p3n_pio, &p3n_rxtx_program, p3n_code_offset);
        p3n_code_offset = -1;
    }
    // free up the CRC calculation DMA channel
    if ( p3n_crc_dma_index >= 0 ) {
        dma_channel_unclaim(p3n_crc_dma_index);
        p3n_crc_dma_index = -1;
    }
    // mark system as un-initialized
    p3n_pio = NULL;
}


static bool try_pio(PIO pio)
{
    // mark that we might have allocated things
    // (in case we fail and someone needs to clean up)
    p3n_pio = pio;
    // attempt to load the program
    p3n_code_offset = pio_add_program(pio, &p3n_rxtx_program);
    if ( p3n_code_offset < 0 ) {
        return false;
    }
    // program loaded, claim the state machines
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        p3n_chans[n].sm_index = pio_claim_unused_sm(pio, false);
        if ( p3n_chans[n].sm_index < 0 ) {
            return false;
        }
    }
    return true;
}

bool p3n_is_chan_avail(int chan_num)
{
    if ( ( chan_num < 0 ) || ( chan_num > (P3N_NUM_CHAN-1) ) ) {
        return false;
    }
    return ( p3n_chans[chan_num].state == P3N_CHAN_ST_IDLE );
}

int p3n_find_avail_chan(void)
{
    for ( int n = 0 ; n < P3N_NUM_CHAN ; n++ ) {
        if ( p3n_is_chan_avail(n) ) {
            return n;
        }
    }
    return -1;
}

void map_pins(p3n_chan_t *ch, p3n_port_t *port)
{
    if ( ch->port != port ) {
        if ( port->rx_pin == NO_PIN ) {
            pio_sm_set_in_pins(p3n_pio, ch->sm_index, 0);
            pio_sm_set_jmp_pin(p3n_pio, ch->sm_index, 0);
        } else {
            pio_sm_set_in_pins(p3n_pio, ch->sm_index, port->rx_pin);
            pio_sm_set_jmp_pin(p3n_pio, ch->sm_index, port->rx_pin);
        }
        if ( port->tx_pin == NO_PIN ) {
            pio_sm_set_out_pins(p3n_pio, ch->sm_index, 0, 0);
        } else {
            pio_sm_set_out_pins(p3n_pio, ch->sm_index, port->tx_pin, 1);
        }
        if ( port->tx_ena_pin == NO_PIN ) {
            pio_sm_set_set_pins(p3n_pio, ch->sm_index, 0, 0);
        } else {
            pio_sm_set_set_pins(p3n_pio, ch->sm_index, port->tx_ena_pin, 1);
        }
        ch->port = port;
    }
}



#if 0

void p3n_link_irq_handler(p3n_link_t *link)
{
    printf("-%d", link-links);
    switch ( link->status ) {
        case P3N_LINK_LISTEN:
            // stop the DMA
            dma_channel_abort(link->sm->dma_index);
            // capture last DMA address
            char *addr = (char *) dma_channel_hw_addr(link->sm->dma_index)->write_addr;
            // received a message, figure out how long it is
            link->buffer->data_len = addr - link->buffer->data;
            link->status = P3N_LINK_STANDBY;
            break;
        case P3N_LINK_SEND:
            // stop the DMA
            dma_channel_abort(link->sm->dma_index);
            // disable the line driver (if present)
            if ( link->hardware->has_tx_ena ) {
                gpio_put(link->hardware->tx_ena_pin, 0);
            }
            link->status = P3N_LINK_STANDBY;
            break;
        case P3N_LINK_STANDBY:
            // should never get an interrupt while in standby
            break;
        case P3N_LINK_IDLE:
            // should never get an interrupt while in idle
            break;
        default:
            break;
    }
}


uint32_t irq_count = 0;

void p3n_pio_irq_handler(void)
{
    irq_count++;
    printf("i%d", irq_count);
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        if ( pio_interrupt_get(p3n_pio, p3n_sms[n].sm_index) ) {
            p3n_link_irq_handler(p3n_sms[n].link);
            pio_interrupt_clear(p3n_pio, p3n_sms[n].sm_index);
        }
    }
}
#endif

#define MAX_SILENCE_DELAY       ((1<<(p3n_silence_delay_bits))-1)
#define MAX_POST_CMD_DELAY      ((1<<(p3n_post_cmd_delay_bits))-1)
#define COMMAND_SHIFT           (p3n_silence_delay_bits)
#define POST_CMD_DELAY_SHIFT    (COMMAND_SHIFT+(p3n_command_bits))
#define PIO_CMD_RX              (1<<(COMMAND_SHIFT))
#define PIO_CMD_TX              (0<<(COMMAND_SHIFT))
#define SILENCE_DELAY_RX        ((p3n_clocks_per_bit*((p3n_bits_per_word)+3))/2)
#define SILENCE_DELAY_TX        ((p3n_clocks_per_bit*((p3n_bits_per_word)+5))/2)

static_assert(MAX_SILENCE_DELAY > SILENCE_DELAY_RX);
static_assert(MAX_SILENCE_DELAY > SILENCE_DELAY_TX);


bool p3n_receive(p3n_chan_t *chan, p3n_port_t *port, p3n_buffer_t *buf, uint32_t delay)
{
    dma_channel_config c;
    dreq_num_t dreq;
    uint32_t pio_cmd;

    if ( port->rx_pin == NO_PIN ) {
        printf("can't receive\n");
        return false;
    }
    if ( ( buf == NULL ) || ( buf->max_len == 0 ) ) {
        printf("bad buf\n");
        return false;
    }
    // FIXME - probably want some form of atomic operation here....
    if ( chan->state != P3N_CHAN_ST_IDLE ) {
        printf("status\n");
        return false;
    }
    // swap pins around if needed
    map_pins(chan, port);
    // build the DMA controller configuration
    c = dma_channel_get_default_config(chan->dma_index);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    if ( pio_get_index(p3n_pio) == 0 ) {
        dreq = DREQ_PIO0_RX0 + chan->sm_index;
    } else {
        dreq = DREQ_PIO1_RX0 + chan->sm_index;
    }
    channel_config_set_dreq(&c, dreq);
    channel_config_set_transfer_data_size(&c, DMA_SIZE);
#if 0
    // is the PIO busy?
    pio_pc = pio_sm_get_pc(p3n_pio, chan->sm_index) - p3n_code_offset;
    if ( ( pio_pc != p3n_rxtx_offset_cmd_start ) &&
         ( pio_pc != p3n_rxtx_offset_post_cmd_delay ) ) {
        printf("bad PC = %d\n", pio_pc);
        return false;
    }
#endif
    // save the buffer
    chan->buf = buf;
    // start the DMA transfer
    dma_channel_configure(chan->dma_index, &c, chan->buf->data,
                            ((char *)&(p3n_pio->rxf[chan->sm_index]))+(4-BYTES_PER_WORD),
                             chan->buf->max_len / BYTES_PER_WORD, true);
    chan->state = P3N_CHAN_ST_RX;
    // send command to the PIO
    if ( delay > MAX_POST_CMD_DELAY ) {
        delay = MAX_POST_CMD_DELAY;
    }
    pio_cmd = (delay << (p3n_silence_delay_bits+p3n_command_bits)) | PIO_CMD_RX | SILENCE_DELAY_RX;
    pio_sm_put(p3n_pio, chan->sm_index, pio_cmd);
    chan->state = P3N_CHAN_ST_RX;
    return true;
}


bool p3n_transmit(p3n_chan_t *chan, p3n_port_t *port, p3n_buffer_t *buf, uint32_t delay)
{
    dma_channel_config c;
    dreq_num_t dreq;
    uint32_t pio_cmd;

    if ( port->tx_pin == NO_PIN ) {
        printf("can't transmit\n");
        return false;
    }
    if ( ( buf == NULL ) || ( buf->data_len == 0 ) ) {
        printf("bad buf\n");
        return false;
    }
    // FIXME - probably want some form of atomic operation here....
    if ( chan->state != P3N_CHAN_ST_IDLE ) {
        printf("status\n");
        return false;
    }
    // swap pins around if needed
    map_pins(chan, port);
    // build the DMA controller configuration
    c = dma_channel_get_default_config(chan->dma_index);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    if ( pio_get_index(p3n_pio) == 0 ) {
        dreq = DREQ_PIO0_TX0 + chan->sm_index;
    } else {
        dreq = DREQ_PIO1_TX0 + chan->sm_index;
    }
    channel_config_set_dreq(&c, dreq);
    channel_config_set_transfer_data_size(&c, DMA_SIZE);
    // save the buffer
    chan->buf = buf;

    // send command to the PIO
    if ( delay > MAX_POST_CMD_DELAY ) {
        delay = MAX_POST_CMD_DELAY;
    }
    pio_cmd = (delay << (p3n_silence_delay_bits+p3n_command_bits)) | PIO_CMD_TX | SILENCE_DELAY_TX;
    pio_sm_put(p3n_pio, chan->sm_index, pio_cmd);
    // start the DMA transfer (data must follow command)
    dma_channel_configure(chan->dma_index, &c, &(p3n_pio->txf[chan->sm_index]),
                             chan->buf->data, (chan->buf->data_len+(BYTES_PER_WORD-1))/BYTES_PER_WORD, true);
    chan->state = P3N_CHAN_ST_TX;
    return true;
}

#if 0



bool p3n_start_link(p3n_link_t *link, uint bitrate)
{
    float pio_clk, div;

    if ( link->status != P3N_LINK_IDLE ) {
        return false;
    }
    if ( link->hardware == NULL ) return false;
    if ( link->sm == NULL ) return false;
    // set the bit rate
    pio_sm_config c = p3n_rxtx_program_get_default_config(p3n_code_offset);
    pio_clk = 125000000.0 / p3n_clocks_per_bit;
    div = pio_clk / bitrate;
    sm_config_set_clkdiv(&c, div);
    // enable pullup on the data pin (it might be bidirectional)
    gpio_pull_up(link->hardware->data_pin);
    // Map the state machine's OUT and SET pin groups
    // as well as the IN pin group and the JMP pin
    // to the data pin
    sm_config_set_out_pins(&c, link->hardware->data_pin, 1);
    sm_config_set_set_pins(&c, link->hardware->data_pin, 1);
    sm_config_set_in_pins(&c, link->hardware->data_pin);
    sm_config_set_jmp_pin(&c, link->hardware->data_pin);
    // Set the data pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(p3n_pio, link->hardware->data_pin);
    if ( link->hardware->has_tx_ena ) {
        // the PIO doeesn't manipulate this pin, it ns normal GPIO
        gpio_init(link->hardware->tx_ena_pin);
        // disable transmitter
        gpio_put(link->hardware->tx_ena_pin, 0);
        gpio_set_dir(link->hardware->tx_ena_pin, 1);
    }
    // map the sideset pin (this is temporary for debug only)
    sm_config_set_sideset_pins(&c, link->hardware->sideset_pin);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(p3n_pio, link->hardware->sideset_pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(p3n_pio, link->sm->sm_index, link->hardware->sideset_pin, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(p3n_pio, link->sm->sm_index, p3n_code_offset+p3n_rxtx_offset_start_wait, &c);
    // make the data pin an input for now
    pio_sm_exec(p3n_pio, link->sm->sm_index, pio_encode_set(pio_pindirs, 0));
    // but if/when it becomes an output, it should be high
    pio_sm_exec(p3n_pio, link->sm->sm_index, pio_encode_set(pio_pins, 1));

    // Set the state machine running
    pio_sm_set_enabled(p3n_pio, link->sm->sm_index, true);
    link->status = P3N_LINK_STANDBY;
    return true;
}



#endif



#define NUM_BUFFERS 6

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
    p3n_reset();
    rb = p3n_init();
    assert(rb);
    while(1);

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

#if 0
    // FIXME - move to p3n_init()?
    irq_add_shared_handler(PIO0_IRQ_0, p3n_pio_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);
    // FIXME - make dependent on number of state machines
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt0, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt1, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt2, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt3, true);
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


