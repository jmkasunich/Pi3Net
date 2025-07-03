#include <stdio.h>
#include "pico/stdlib.h"
#include "pi3net.h"
#include "pi3net.pio.h"

#define P3N_PIO_NUM 0




const p3n_hwdef_t p3n_bus1 = { 1, 1, 1, 16, 16, 17 };
const p3n_hwdef_t p3n_bus2 = { 1, 1, 1, 16, 16, 28 };
const p3n_hwdef_t p3n_tx_up = { 1, 0, 0, 4, 0, 0 };
const p3n_hwdef_t p3n_rx_up = { 0, 1, 0, 0, 5, 0 };
const p3n_hwdef_t p3n_tx_dn = { 1, 0, 0, 2, 0, 0 };
const p3n_hwdef_t p3n_rx_dn = { 0, 1, 0, 0, 2, 0 };


#define SCOPE_CHAN_1  (10)
#define SCOPE_CHAN_2  (14)
#define SCOPE_CHAN_3  (6)


static inline void pi3net_tx_program_init(PIO pio, uint sm, uint offset, uint tx_pin, uint rx_pin) {
    pio_sm_config c = pi3net_program_get_default_config(offset);

    // Map the state machine's OUT pin group to one pin, namely the `tx_pin`
    // parameter to this function.
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_set_pins(&c, tx_pin, 1);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, tx_pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, tx_pin, 1, true);

    // Map the state machine's IN pin group to one pin, namely the `rx_pin`
    // parameter to this function.
    sm_config_set_in_pins(&c, rx_pin);

    sm_config_set_out_shift(&c, true, false, 8);
    sm_config_set_mov_status(&c, 0, 1);

    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

static inline void pi3net_rx_program_init(PIO pio, uint sm, uint offset, uint tx_pin, uint rx_pin) {
    pio_sm_config c = pi3net_program_get_default_config(offset);

    // Map the state machine's OUT pin group to one pin, namely the `tx_pin`
    // parameter to this function.
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_set_pins(&c, tx_pin, 1);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, tx_pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, tx_pin, 1, true);

    // Map the state machine's IN pin group to one pin, namely the `rx_pin`
    // parameter to this function.
    sm_config_set_in_pins(&c, rx_pin);
    sm_config_set_jmp_pin(&c, rx_pin);

    sm_config_set_out_shift(&c, true, false, 8);
    sm_config_set_mov_status(&c, 0, 1);

    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, (offset + pi3net_offset_rxbegin) & 0x1F, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

#define P3N_NUM_CHANS 2

static_assert(P3N_NUM_CHANS > 0 );
static_assert(P3N_NUM_CHANS <= 4 );

PIO p3n_pio;
uint p3n_sms[P3N_NUM_CHANS];
uint p3n_code_offset;


bool is_pio_available(PIO pio)
{

    // will the program fit?
    if ( ! pio_can_add_program(pio, &pi3net_program) ) {
        return false;
    }
    // are there enough free state machines?
    uint sm_free = 0;
    for ( int n = 0 ; n < 4 ; n++ ) {
        if ( ! pio_sm_is_claimed(pio, n) ) {
            sm_free++;
        }
    }
    if ( sm_free < P3N_NUM_CHANS ) {
        return false;
    }
    return true;
}



bool load_program_claim_sms(void)
{
    if ( is_pio_available(pio0) ) {
        p3n_pio = pio0;
    } else if ( is_pio_available(pio1) ) {
        p3n_pio = pio1;
    } else return false;
    // load the program
    p3n_code_offset = pio_add_program(p3n_pio, &pi3net_program);
    hard_assert(p3n_code_offset >= 0 );
    // claim the state machines
    for ( int n = 0 ; n < P3N_NUM_CHANS ; n++ ) {
        p3n_sms[n] = pio_claim_unused_sm(p3n_pio, 1);
    }

    printf("Using PIO # %d, state machines ", pio_get_index(p3n_pio));
    for ( int n = 0 ; n < P3N_NUM_CHANS ; n++ ) {
        printf(" %d", p3n_sms[n]);
    }
    printf("\n");
}

void p3n_test(void)
{
    printf("Hello from pi3net\n");
    gpio_init(SCOPE_CHAN_1);
    gpio_set_dir(SCOPE_CHAN_1, GPIO_OUT);
    load_program_claim_sms();
    pi3net_tx_program_init(p3n_pio, p3n_sms[0], p3n_code_offset, SCOPE_CHAN_2, SCOPE_CHAN_3);
    pio_sm_set_clkdiv(p3n_pio, p3n_sms[0], 15.625);
    pi3net_rx_program_init(p3n_pio, p3n_sms[1], p3n_code_offset, SCOPE_CHAN_3, SCOPE_CHAN_2);
    pio_sm_set_clkdiv(p3n_pio, p3n_sms[1], 15.625);
    printf("init done\n");
    while(1) {
        sleep_ms(2);
        gpio_put(SCOPE_CHAN_1, 1);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xF0F0F0F0);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0x550FF0AA);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xDEADBE00);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xDEADBEFF);
        pio_sm_put_blocking(p3n_pio, p3n_sms[0], 0xDEADBE3C);
        printf("\nfifo: %d  ", pio_sm_get_rx_fifo_level(p3n_pio, p3n_sms[1]));
        while ( pio_sm_get_rx_fifo_level(p3n_pio, p3n_sms[1]) > 0 ) {
            printf("%08x ", pio_sm_get(p3n_pio, p3n_sms[1]));
        }
        sleep_ms(2);
        gpio_put(SCOPE_CHAN_1, 0);
    }

}


