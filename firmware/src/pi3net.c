#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pi3net.h"
#include "pi3net.pio.h"

#define P3N_PIO_NUM 0




const p3n_link_hw_t p3n_bus1  = { 1, 1, 1, 16, 16, 17 };
const p3n_link_hw_t p3n_bus2  = { 1, 1, 1, 22, 22, 28 };
const p3n_link_hw_t p3n_tx_up = { 1, 0, 0, 4, 0, 0 };
const p3n_link_hw_t p3n_rx_up = { 0, 1, 0, 0, 5, 0 };
const p3n_link_hw_t p3n_tx_dn = { 1, 0, 0, 2, 0, 0 };
const p3n_link_hw_t p3n_rx_dn = { 0, 1, 0, 0, 3, 0 };


#define SCOPE_CHAN_1  (10)
#define SCOPE_CHAN_2  (14)
#define SCOPE_CHAN_3  (6)
#define SCOPE_CHAN_4  (8)

#define NUM_BUF 4

p3n_buffer_t buffers[NUM_BUF];

p3n_buffer_t *new_buffer(uint max_size)
{
    p3n_buffer_t *b;
    b = malloc(sizeof(p3n_buffer_t));
    if ( b == NULL ) {
        return NULL;
    }
    b->data = malloc(max_size);
    if ( b->data == NULL ) {
        free(b);
        return NULL;
    }
    b->max_len = max_size;
    b->data_len = 0;
    return b;
}


p3n_buffer_t *buf1 = NULL;
p3n_buffer_t *buf2 = NULL;


static inline void p3n_tx_program_init(PIO pio, uint sm, uint offset, uint tx_pin, uint side_pin) {
    pio_sm_config c = p3n_tx_program_get_default_config(offset);

    // Map the state machine's OUT and SET pin groups to one pin, namely the `tx_pin`
    // parameter to this function.
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_set_pins(&c, tx_pin, 1);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, tx_pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, tx_pin, 1, true);

    // map the sideset pin (this is temporary for debug only)
    sm_config_set_sideset_pins(&c, side_pin);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, side_pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, side_pin, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

static inline void p3n_rx_program_init(PIO pio, uint sm, uint offset, uint rx_pin, uint side_pin) {
    pio_sm_config c = p3n_rx_program_get_default_config(offset);

    // Map the state machine's IN pin group to one pin, namely the `rx_pin`
    // parameter to this function.
    sm_config_set_in_pins(&c, rx_pin);
    sm_config_set_jmp_pin(&c, rx_pin);

    // map the sideset pin (this is temporary for debug only)
    sm_config_set_sideset_pins(&c, side_pin);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, side_pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, side_pin, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, (offset + p3n_rx_offset_rxbegin) & 0x1F, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

#define P3N_MAX_SM 4

PIO p3n_pio = NULL;
uint p3n_sms[P3N_MAX_SM] = { -1, -1, -1, -1 };
uint p3n_tx_code_offset = -1;
uint p3n_rx_code_offset = -1;


bool p3n_try_pio(PIO pio, uint num_sm)
{
    // attempt to load first program
    p3n_tx_code_offset = pio_add_program(pio, &p3n_tx_program);
    if ( p3n_tx_code_offset < 0 ) {
        goto error;
    }
    // attempt to load second program
    p3n_rx_code_offset = pio_add_program(pio, &p3n_rx_program);
    if ( p3n_rx_code_offset < 0 ) {
        goto error;
    }
    // programs loaded, claim the state machines
    for ( int n = 0 ; n < num_sm ; n++ ) {
        p3n_sms[n] = pio_claim_unused_sm(pio, false);
        if ( p3n_sms[n] < 0 ) {
            goto error;
        }
    }
    p3n_pio = pio;
    return true;

    // error handling; free resources in the reverse order of obtaining them
    error:
    // unclaim any successfully allocated state machines
    for ( int n = 0 ; n < num_sm ; n++ ) {
        if ( p3n_sms[n] >= 0 ) {
            pio_sm_unclaim(pio, p3n_sms[n] );
            p3n_sms[n] = -1;
        }
    }
    // unload any successfully loaded programs
    if ( p3n_rx_code_offset >= 0 ) {
        pio_remove_program(pio, &p3n_rx_program, p3n_rx_code_offset);
        p3n_rx_code_offset = -1;
    }
    if ( p3n_tx_code_offset >= 0 ) {
        pio_remove_program(pio, &p3n_tx_program, p3n_tx_code_offset);
        p3n_tx_code_offset = -1;
    }
    p3n_pio = NULL;
    return false;
}

bool load_programs_claim_sms(uint num_sm)
{
    if ( ! p3n_try_pio(pio0, num_sm) ) {
        if ( ! p3n_try_pio(pio1, num_sm) ) {
            return false;
        }
    }
    printf("Using PIO # %d, state machines ", pio_get_index(p3n_pio));
    for ( int n = 0 ; n < P3N_MAX_SM ; n++ ) {
        printf(" %d", p3n_sms[n]);
    }
    printf("\nTX code: %d\nRX code: %d\n", p3n_tx_code_offset, p3n_rx_code_offset);
}

uint32_t irq_count = 0;

void p3n_pio_irq_handler(void)
{
    irq_count++;
    pio_interrupt_clear(p3n_pio, 0);
    printf("i");
}

void print_irq_globals(uint stamp)
{
    printf("%d: IRQ: %08x  INTR: %08x  RXF: %d CNT: %10d\n", stamp,  p3n_pio->irq, p3n_pio->intr, 
                                pio_sm_get_rx_fifo_level(p3n_pio, p3n_sms[1]), irq_count);
}

void p3n_test(void)
{
    uint old_count;

    printf("Hello from pi3net\n");
    buf1 = new_buffer(100);
    assert(buf1);
    buf2 = new_buffer(500);
    assert(buf2);

    gpio_init(SCOPE_CHAN_1);
    gpio_set_dir(SCOPE_CHAN_1, GPIO_OUT);
    load_programs_claim_sms(3);
    p3n_tx_program_init(p3n_pio, p3n_sms[0], p3n_tx_code_offset, SCOPE_CHAN_2, SCOPE_CHAN_4);
    pio_sm_set_clkdiv(p3n_pio, p3n_sms[0], 15625.0);
    p3n_rx_program_init(p3n_pio, p3n_sms[1], p3n_rx_code_offset, SCOPE_CHAN_2, SCOPE_CHAN_3);
    pio_sm_set_clkdiv(p3n_pio, p3n_sms[1], 15625.0);
    irq_add_shared_handler(PIO0_IRQ_0, p3n_pio_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(p3n_pio, pis_interrupt0, true);
    printf("init done\n");
    while(1) {
        old_count = irq_count;
        print_irq_globals(1);
        //pio_interrupt_clear(p3n_pio, 0);
        gpio_put(SCOPE_CHAN_1, 1);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xF0F0F0F0);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0x550FF0AA);
//        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xDEADBE00);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xDEADBEFF);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xDEADBE3C);
        while ( irq_count == old_count ) {
            print_irq_globals(2);
        }
        print_irq_globals(3);
        print_irq_globals(4);
        print_irq_globals(5);
        print_irq_globals(6);
        printf("\nfifo: %d  ", pio_sm_get_rx_fifo_level(p3n_pio, p3n_sms[1]));
        while ( pio_sm_get_rx_fifo_level(p3n_pio, p3n_sms[1]) > 0 ) {
            printf("%08x ", pio_sm_get(p3n_pio, p3n_sms[1]));
        }
        printf("\n");
        print_irq_globals(7);
        sleep_ms(5000);
        print_irq_globals(8);
        gpio_put(SCOPE_CHAN_1, 0);
    }

}


