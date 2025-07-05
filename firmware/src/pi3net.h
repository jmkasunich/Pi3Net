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
#include "hardware/structs/pio.h"



/**********************************************************
 * This structure defines a link in terms of the available 
 * hardware; is it transmit only, recieve only, or both?
 * Does it use a pin to enable an external line driver?
 * Typically there will be one of these structures in flash
 * for each hardware link; when calling the API functions
 * you pass a pointer to the p3n_link_hw_t structure for
 * the link you want to talk or listen on.
 */
#define PIN_ID_BITS  5

typedef struct p3n_link_hw_s {
    uint32_t can_transmit       : 1;
    uint32_t can_recieve        : 1;
    uint32_t has_tx_ena         : 1;
    uint32_t transmit_pin       : PIN_ID_BITS;
    uint32_t receive_pin        : PIN_ID_BITS;
    uint32_t tx_ena_pin         : PIN_ID_BITS;
} p3n_link_hw_t;

/**********************************************************
 * This structure defines a buffer for data transfer 
 */

 typedef struct p3n_buffer_s {
    uint16_t    max_len;
    uint16_t    data_len;
    uint32_t    *data;
 } p3n_buffer_t;



typedef enum {
    P3N_LINK_IDLE = 0,
    P3N_LINK_SEND,
    P3N_LINK_LISTEN
} p3n_link_status_t;


/**********************************************************
 * This structure defines the status of a hardware link.
 * Since the link status is dynamic, these are stored in 
 * RAM.
 */
typedef struct p3n_link_s {
    p3n_link_status_t   status;
    uint32_t            sm_index;
    p3n_link_hw_t       *hardware;
    p3n_buffer_t        *buffer;

} p3n_link_t;




void p3n_test(void);


#endif // PI3NET_H
