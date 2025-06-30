/***************************************************************
 * 
 * pi3net.h - header for Pi Pico PIO Networking
 * 
 * 
 * 
 * 
 **************************************************************/
#ifndef PI3NET_H
#define PI3NET_H

#include <stdint.h>    // int32_t, uint32_t

#define PIN_ID_BITS  5

typedef struct p3n_hwdef_s {
    uint32_t can_transmit       : 1;
    uint32_t can_recieve        : 1;
    uint32_t has_tx_ena         : 1;
    uint32_t transmit_pin       : PIN_ID_BITS;
    uint32_t receive_pin        : PIN_ID_BITS;
    uint32_t tx_ena_pin         : PIN_ID_BITS;
} p3n_hwdef_t;

extern const p3n_hwdef_t p3n_bus1;
extern const p3n_hwdef_t p3n_bus2;
extern const p3n_hwdef_t p3n_tx_up;
extern const p3n_hwdef_t p3n_rx_up;
extern const p3n_hwdef_t p3n_tx_dn;
extern const p3n_hwdef_t p3n_rx_dn;

void p3n_test(void);


#endif // PI3NET_H