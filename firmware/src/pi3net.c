#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pi3net.h"
#include "pi3net.pio.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/pio.h"


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


#define TX 1
#define RX 2
#define TESTMODE RX

p3n_buffer_t *buffers[6] = { NULL, NULL, NULL, NULL };
p3n_link_t links[6] = { P3N_LINK_IDLE, NULL, NULL, NULL };

#define BITRATE 100000

const p3n_link_hw_t p3n_bus1  = { 1, 1, 1, 16, 17, 18 };
const p3n_link_hw_t p3n_bus2  = { 1, 1, 1, 22, 28, 26 };
const p3n_link_hw_t p3n_tx_up = { 1, 0, 0, 4, 0, 8 };
const p3n_link_hw_t p3n_rx_up = { 0, 1, 0, 5, 0, 9 };
const p3n_link_hw_t p3n_tx_dn = { 1, 0, 0, 2, 0, 6 };
const p3n_link_hw_t p3n_rx_dn = { 0, 1, 0, 3, 0, 7 };

const p3n_link_hw_t p3n_test_tx = { 1, 0, 0, SCOPE_CHAN_2, 0, SCOPE_CHAN_3 };
const p3n_link_hw_t p3n_test_rx = { 0, 1, 0, SCOPE_CHAN_2, 0, SCOPE_CHAN_4 };


p3n_buffer_t *new_buffer(uint max_size_bytes)
{
    p3n_buffer_t *b;
    b = malloc(sizeof(p3n_buffer_t));
    if ( b == NULL ) {
        return NULL;
    }
    // round size up to a multiple of 4 bytes
    max_size_bytes = ( max_size_bytes + 3 ) & ~3;
    b->data = malloc(max_size_bytes);
    if ( b->data == NULL ) {
        free(b);
        return NULL;
    }
    b->max_len = max_size_bytes;
    b->data_len = 0;
    return b;
}


static PIO p3n_pio = NULL;
static p3n_sm_t p3n_sms[P3N_NUM_STATE_MACHS];
static int p3n_code_offset;


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

void print_buffer(p3n_buffer_t *buf)
{
    int l;

    printf("buf: %08x max_len: %d  data_len: %d  data: %08x\n", buf, buf->max_len, buf->data_len, buf->data);
    // print a few bytes past the end
    l = buf->data_len + 7;
    if ( l > buf->max_len ) {
        l = buf->max_len;
    }
    for ( int n = 0 ; n < l ; n ++ ) {
        if ( ( n % 16 ) == 0 ) {
            printf("\n%3d: ", n);
        }
        printf(" %02x", buf->data[n]);
    }
    printf("\n");
}

void print_buffer_as_string(p3n_buffer_t *buf)
{
    int l;

    printf("buf: %08x max_len: %d  data_len: %d  data: '", buf, buf->max_len, buf->data_len);
    // print the data
    l = buf->data_len;
    if ( l > buf->max_len ) {
        l = buf->max_len;
    }
    for ( int n = 0 ; n < l ; n ++ ) {
        printf("%c", buf->data[n]);
    }
    printf("'\n");
}

void copy_string_to_buffer(p3n_buffer_t *buf, char *str)
{
    char *s, *d, c;
    int n;

    s = str;
    d = buf->data;
    n = 0;
    while ( n < buf->max_len ) { 
        c = *(s++);
        *(d++) = c;
        n++;
        if ( c == '\0' ) {
            // done
            buf->data_len = n;
            return;
        }
    }
    // buffer full
    buf->data_len = n;
    return;
}

void erase_buffer(p3n_buffer_t *buf)
{
    for ( int n = 0 ; n < buf->max_len ; n += 4 ) {
        buf->data[n] = 0xDE;
        buf->data[n+1] = 0xAD;
        buf->data[n+2] = 0xBE;
        buf->data[n+3] = 0xEF;
    }
    buf->data_len = 0;
}


void delay_clks(uint32_t clks)
{
    uint32_t start, now, dt;

    // systick rolls over at 2^24 = 134ms
    // only an idiot busywaits that long
    // clamp it at 100ms
    if ( clks > 12500000 ) {
        clks = 12500000;
    }
    // it takes about 25 clocks to run
    // this code when clks = 0
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
    // sometimes I'm an idiot
    while ( ms > 0 ) {
        delay_us(1000);
        ms--;
    }
}



void p3n_test(void)
{
    uint old_count;
    bool rb;
    uint32_t start, now, dt;

    printf("Hello from pi3net\n");
    for ( int n = 0 ; n < 6 ; n++ ) {
        buffers[n] = new_buffer(100);
        assert(buffers[n]);
    }
    for ( int n = 0 ; n < 6 ; n++ ) {
        links[n].buffer = NULL;
        links[n].hardware = NULL;
        links[n].sm = NULL;
        links[n].status = P3N_LINK_IDLE;
    }
    links[0].hardware = (p3n_link_hw_t *)&(p3n_bus1);
    links[1].hardware = (p3n_link_hw_t *)&(p3n_bus2);
    links[2].hardware = (p3n_link_hw_t *)&(p3n_tx_up);
    links[3].hardware = (p3n_link_hw_t *)&(p3n_rx_up);
    links[4].hardware = (p3n_link_hw_t *)&(p3n_tx_dn);
    links[5].hardware = (p3n_link_hw_t *)&(p3n_rx_dn);
    gpio_init(SCOPE_CHAN_1);
    gpio_put(SCOPE_CHAN_1, 0);
    gpio_set_dir(SCOPE_CHAN_1, GPIO_OUT);
    p3n_reset();
    rb = p3n_init();
    assert(rb);

#if (TESTMODE == TX )
    // prepare the transmit links
    rb = p3n_assign_sm_to_link(&links[0]);
    assert(rb);
    copy_string_to_buffer(buffers[0], "bus1, using buffers[0]");
    rb = p3n_assign_sm_to_link(&links[1]);
    assert(rb);
    copy_string_to_buffer(buffers[1], "bus2, using buffers[1]");
    rb = p3n_assign_sm_to_link(&links[2]);
    assert(rb);
    copy_string_to_buffer(buffers[2], "neighbor up, using buffers[2]");
    rb = p3n_assign_sm_to_link(&links[4]);
    assert(rb);
    copy_string_to_buffer(buffers[4], "neighbor down, using buffers[4]");
#endif

#if (TESTMODE == RX )
    // prepare the receive links
    rb = p3n_assign_sm_to_link(&links[0]);
    assert(rb);
    erase_buffer(buffers[0]);
    rb = p3n_assign_sm_to_link(&links[1]);
    assert(rb);
    erase_buffer(buffers[1]);
    rb = p3n_assign_sm_to_link(&links[3]);
    assert(rb);
    erase_buffer(buffers[3]);
    rb = p3n_assign_sm_to_link(&links[5]);
    assert(rb);
    erase_buffer(buffers[5]);
#endif

    // FIXME - move to p3n_init()?
    irq_add_shared_handler(PIO0_IRQ_0, p3n_pio_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);
    // FIXME - make dependent on number of state machines
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt0, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt1, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt2, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt3, true);

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

bool p3n_receive(p3n_link_t *link, p3n_buffer_t *buf)
{
    dma_channel_config c;
    dreq_num_t dreq;

    if ( link->status != P3N_LINK_STANDBY ) {
        printf("status\n");
        return false;
    }
    if ( ! link->hardware->can_receive ) {
        printf("can't receive\n");
        return false;
    }
    if ( ( buf == NULL ) || ( buf->max_len == 0 ) ) {
        printf("bad buf\n");
        return false;
    }
    // build the DMA controller configuration
    c = dma_channel_get_default_config(link->sm->dma_index);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    if ( pio_get_index(p3n_pio) == 0 ) {
        dreq = DREQ_PIO0_RX0 + link->sm->sm_index;
    } else {
        dreq = DREQ_PIO1_RX0 + link->sm->sm_index;
    }
    channel_config_set_dreq(&c, dreq);
    channel_config_set_transfer_data_size(&c, DMA_SIZE);
    // is the PIO ready to receive?
    if ( pio_sm_get_pc(p3n_pio, link->sm->sm_index) != (p3n_rxtx_offset_idle_wait + p3n_code_offset) ) {
        printf("bad PC = %d\n", pio_sm_get_pc(p3n_pio, link->sm->sm_index));
        return false;
    }
    // save the buffer
    link->buffer = buf;
    // start the DMA transfer
    dma_channel_configure(link->sm->dma_index, &c, link->buffer->data, 
                            ((char *)&(p3n_pio->rxf[link->sm->sm_index]))+(4-BYTES_PER_WORD),
                             link->buffer->max_len / BYTES_PER_WORD, true);
    link->status = P3N_LINK_LISTEN;
    return true;
}

bool p3n_transmit(p3n_link_t *link, p3n_buffer_t *buf)
{
    dma_channel_config c;
    dreq_num_t dreq;

    if ( link->status != P3N_LINK_STANDBY ) {
        printf("status\n");
        return false;
    }
    if ( ! link->hardware->can_transmit ) {
        printf("can't transmit\n");
        return false;
    }
    if ( ( buf == NULL ) || ( buf->data_len == 0 ) ) {
        printf("bad buf\n");
        return false;
    }
    // build the DMA controller configuration
    c = dma_channel_get_default_config(link->sm->dma_index);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    if ( pio_get_index(p3n_pio) == 0 ) {
        dreq = DREQ_PIO0_TX0 + link->sm->sm_index;
    } else {
        dreq = DREQ_PIO1_TX0 + link->sm->sm_index;
    }
    channel_config_set_dreq(&c, dreq);
    channel_config_set_transfer_data_size(&c, DMA_SIZE);
    // is the PIO ready to send?
    if ( pio_sm_get_pc(p3n_pio, link->sm->sm_index) != (p3n_rxtx_offset_idle_wait + p3n_code_offset) ) {
        printf("bad PC = %d\n", pio_sm_get_pc(p3n_pio, link->sm->sm_index));
        return false;
    }
    // save the buffer
    link->buffer = buf;
    // enable the line driver, if present
    if ( link->hardware->has_tx_ena ) {
        gpio_put(link->hardware->tx_ena_pin, 1);
    }
    // make the PIO jump to the transmit routine
    pio_sm_exec(p3n_pio, link->sm->sm_index, pio_encode_jmp(p3n_rxtx_offset_txbegin + p3n_code_offset));
    // start the DMA transfer
    dma_channel_configure(link->sm->dma_index, &c, &(p3n_pio->txf[link->sm->sm_index]),
                             link->buffer->data, (link->buffer->data_len+(BYTES_PER_WORD-1))/BYTES_PER_WORD, true);
    link->status = P3N_LINK_SEND;
    return true;
}




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
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        p3n_sms[n].sm_index = pio_claim_unused_sm(pio, false);
        if ( p3n_sms[n].sm_index < 0 ) {
            return false;
        }
    }
    return true;
}

// this resets all of the global data without checking or freeing anything
// should only be done after MCU reset
void p3n_reset(void)
{
    p3n_sm_t *sm;
    // initialize to "nothing claimed"
    p3n_pio = NULL;
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        sm = &(p3n_sms[n]);
        sm->sm_index = -1;
        sm->dma_index = -1;
        sm->link = NULL;
    }
    p3n_code_offset = -1;
}


void p3n_uninit(void)
{
    p3n_sm_t *sm;
    if ( p3n_pio == NULL ) {
        // either initial boot, or already un-initialized
        // nothing to do
        return;
    }
    // release allocated resources
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        sm = &(p3n_sms[n]);
        if ( sm->sm_index >= 0 ) {
            pio_sm_unclaim(p3n_pio, sm->sm_index);
            sm->sm_index = -1;
        }
        if ( sm->dma_index >= 0 ) {
            dma_channel_unclaim(sm->dma_index);
            sm->dma_index = -1;
        }
        if ( sm->link != NULL ) {
            sm->link->sm = NULL;
        }
        sm->link = NULL;
    }
    // unload the PIO program
    if ( p3n_code_offset >= 0 ) {
        pio_remove_program(p3n_pio, &p3n_rxtx_program, p3n_code_offset);
        p3n_code_offset = -1;
    }
    // mark system as un-initialized
    p3n_pio = NULL;
}


bool p3n_init(void)
{
    p3n_sm_t *sm;
    if ( ! try_pio(pio0) ) {
        p3n_uninit();
        if ( ! try_pio(pio1) ) {
            p3n_uninit();
            return false;
        }
    }
    // state machines claimed, claim DMA channel for each
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        sm = &(p3n_sms[n]);
        sm->dma_index = dma_claim_unused_channel(false);
        if ( sm->dma_index < 0 ) {
            p3n_uninit();
            return false;
        }
    }
    printf("Using PIO # %d, sm/dma ", pio_get_index(p3n_pio));
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        sm = &(p3n_sms[n]);
        printf(" %d/%d", sm->sm_index, sm->dma_index);
    }
    printf("\ncode offset: %d\n", p3n_code_offset);
}


bool p3n_assign_sm_to_link(p3n_link_t *link)
{
    p3n_sm_t *sm;
    if ( link->sm != NULL ) {
        // state machine already assigned, nothing to do
        return true;
    }
    // find free state machine (and dma channel)
    for ( int n = 0 ; n < P3N_NUM_STATE_MACHS ; n++ ) {
        sm = &(p3n_sms[n]);
        if ( sm->link == NULL ) {
            // it's free, use it
            link->sm = sm;
            sm->link = link;
            return true;
        }
    }
    // no free state machine found
    return false;
}

void p3n_release_sm_from_link(p3n_link_t *link)
{
    if ( link->sm != NULL ) {
        link->sm->link = NULL;
        link->sm = NULL;
    }
}

