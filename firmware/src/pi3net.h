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
#include "hardware/pio.h"
#include "hardware/dma.h"


/**********************************************************
 * The number of PIO state machines that should be claimed,
 * from 1 to 4.
 */ 
#define P3N_NUM_STATE_MACHS     4

/* sanity check: a single PIO has only four state machines */
static_assert(P3N_NUM_STATE_MACHS <= 4);



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
    uint32_t can_receive        : 1;
    uint32_t has_tx_ena         : 1;
    uint32_t data_pin           : PIN_ID_BITS;
    uint32_t tx_ena_pin         : PIN_ID_BITS;
    uint32_t sideset_pin        : PIN_ID_BITS;
} p3n_link_hw_t;

/**********************************************************
 * This structure defines a buffer for data transfer 
 */

 typedef struct p3n_buffer_s {
    uint16_t    max_len;
    uint16_t    data_len;
    char        *data;
 } p3n_buffer_t;



typedef enum {
    P3N_LINK_IDLE = 0,
    P3N_LINK_STANDBY,
    P3N_LINK_SEND,
    P3N_LINK_LISTEN
} p3n_link_status_t;

typedef enum {
    P3N_SM_FREE = 0,
    P3N_SM_IDLE,
    P3N_SM_INUSE
} p3n_sm_status_t;


/**********************************************************
 * This structure defines the status of a state machine and
 * its associated DMA channel.  Any state machine may be
 * used to send/receive data over any link; this is needed
 * because some hardware platforms have more links than
 * available state machines and need to manage which links
 * are active at any given time.
 */
typedef struct p3n_sm_s {
    int                 sm_index;
    int                 dma_index;
    struct p3n_link_s   *link;
} p3n_sm_t;


/**********************************************************
 * This structure defines the status of a hardware link.
 * Since the link status is dynamic, these are stored in 
 * RAM.
 */
typedef struct p3n_link_s {
    p3n_link_status_t   status;
    struct p3n_sm_s     *sm;
    p3n_link_hw_t       *hardware;
    p3n_buffer_t        *buffer;
} p3n_link_t;


/**********************************************************
 * Resets all global pi3net data without checking or
 * freeing anything.  Should only be done after MCU reset
 * to ensure good initial conditions for p3n_init()
 */
void p3n_reset(void);

/**********************************************************
 * Cleans up and frees global pi3net data.  Call any time
 * except immediately after MCU reset. 
 */
void p3n_uninit(void);

/**********************************************************
 * Initializes global pi3net data including acquiring PIO
 * and DMA resources.  It will attempt to reserve 
 * P3N_NUM_STATE_MACH state machines and DMA channels.
 */
bool p3n_init(void);

/**********************************************************
 * Assigns one of the state machines (and DMA channels)
 * reserved by p3n_init() to the specified link.  Fails if
 * all state machines are already assigned to links; try 
 * p3n_release_sm_from_link() on a link that you don't
 * immediately need, then call again.
 */
bool p3n_assign_sm_to_link(p3n_link_t *link);

/**********************************************************
 * Disassociates a state machine and DMA channel from a
 * link so that it can be used elsewhere.  As currently
 * implemented it just acts without checking anything, 
 * and if a message is in progress things are likely to
 * get ugly.
 * 
 * I need to decide if it should terminate and clean up
 * any transfer in progress, or if it should fail and
 * return false if the link is busy.
 */
void p3n_release_sm_from_link(p3n_link_t *link);

/**********************************************************
 * Configures a link based on the p3n_link_hw_t data 
 * associated with the link, and the specified bit rate,
 * then starts the PIO.
 */
bool p3n_start_link(p3n_link_t *link, uint bitrate);

/**********************************************************
 * Starts listening for messages on 'link', capturing
 * incoming data to 'bufr'.
 */
bool p3n_receive(p3n_link_t *link, p3n_buffer_t *buf);

/**********************************************************
 * Starts a message transmission on 'link' from 'buf'.
 */
bool p3n_transmit(p3n_link_t *link, p3n_buffer_t *buf);

void p3n_test(void);


#endif // PI3NET_H
