/***************************************************************
 * 
 * pi3net.h - header for Pi Pico PIO Networking
 * 
 * 
 * a couple of key concepts for this library:
 * 
 * 
 *   A "port" is a hardware pin or pins over which messages
 *   can travel.  There can be any number of ports, this
 *   depends on your hardware.  For example, the first Pi3Net
 *   PC board contains six if fully populated:
 *      B1 - one pair, half-duplex bus, transmit or receive
 *      B2 - one pair, half-duplex bus, transmit or receive
 *      TXUP - one pair, transmit to upstream's RXDN
 *      TXDN - one pair, transmit to downstream's RXUP
 *      RXUP - one pair, receive from upstream's TXDN
 *      RXDN - one pair, recieve from downstream's TXUP
 *
 *   A "channel" is a combination of a PIO state machine
 *   and a DMA channel, which cdoes the actual work of sending
 *   or receiving data.  The number of channels determines
 *   how many messages can be in progress at any given time.
 *   The number of channels is set at compile time by
 *   P3N_NUM_CHAN.  There can be no more than four, since
 *   a PIO has only four state machines.  A channel can
 *   transfer data over any port; when there are more ports
 *   than channels, the port used by a channel can be changed
 *   for each message.
 *
 *
 *
 **************************************************************/
#ifndef PI3NET_H
#define PI3NET_H

#include <stdint.h>    // int32_t, uint32_t

#define BITS2STORE(n) (32-(__builtin_clz((n))))

/**********************************************************
 * The number of channels desired, from 1 to 4.
 */ 
#ifndef P3N_NUM_CHAN
#define P3N_NUM_CHAN     4
#endif

/* sanity check: a PIO has only four state machines */
static_assert(P3N_NUM_CHAN <= 4);

/**********************************************************
 * This structure defines a port in terms of the available
 * hardware; is it transmit only, recieve only, or both?
 * Does it use a pin to enable an external line driver?
 */
#define PIN_ID_BITS  6
#define NO_PIN 0x3F

typedef struct p3n_port_s {
    uint32_t rx_pin             : PIN_ID_BITS;
    uint32_t tx_pin             : PIN_ID_BITS;
    uint32_t tx_ena_pin         : PIN_ID_BITS;
} p3n_port_t;

static_assert((PIN_ID_BITS * 3 + 3) <= 32);
static_assert(sizeof(p3n_port_t) == 4 );

/**********************************************************
 * This structure defines a buffer for message data.
 *
 * Note that the actual buffer (at .data) is 4 bytes
 * bigger than .max_len to allow for appending a CRC.
 * This is handled transparently by p3n_buf_new().
 */
typedef struct p3n_buffer_s {
    uint16_t                max_len;
    uint16_t                data_len;
    char                   *data;
} p3n_buffer_t;

static_assert(sizeof(p3n_buffer_t) == 8 );

/**********************************************************
 * Create a buffer for message data.  Both the p3n_buf_t
 * structure and the actual data buffer are allocated on
 * the heap; it is recommended that all buffers needed be
 * allocated during init and never freed.
 */
p3n_buffer_t *p3n_buffer_new(uint32_t max_msg_len_bytes);

/**********************************************************
 * Free a message data buffer.  Provided for completeness,
 * it is recommended that this function not be used.
 */
void p3n_buffer_free(p3n_buffer_t *buf);

/**********************************************************
 * Appends a CRC to a message.  This should be called
 * before sending a message.
 *
 * NOTE: there is a good chance this will become private
 *
 */
void p3n_buffer_add_crc32(p3n_buffer_t *buf);

/**********************************************************
 * Checks to see if a message has a matching CRC at the
 * end.  If a match, data_len is reduced by the CRC length
 * and the function returns true, otherwise no change to
 * the buffer and returns false.  This should be called
 * after receiving a message.
 *
 * NOTE: there is a good chance this will become private
 *
 */
bool p3n_buffer_check_crc32(p3n_buffer_t *buf);

#if 0
/**********************************************************
 * This structure defines a command for a channel.
 * It determines whether the channel should transmit or
 * receive, what hardware port it should use, and how
 * long it should wait before processing the next
 * command.
 */

typedef enum {
    P3N_CMD_RECEIVE = 0,
    P3N_CMD_TRANSMIT,
    P3N_MAXCMDTYPE
} p3n_cmd_type_t;

typedef enum {
    P3N_CMD_WAITING = 0,
    P3N_CMD_ACTIVE,
    P3N_CMD_DONE,
    P3N_MAXCMDSTATE
} p3n_cmd_state_t;

#define CMD_TYPE_BITS   (BITS2STORE(P3N_MAXCMDTYPE))
#define CMD_STATE_BITS  (BITS2STORE(P3N_MAXCMDSTATE))

typedef struct p3n_cmd_s {
    p3n_cmd_type_t          cmd_type    : CMD_TYPE_BITS;
    p3n_cmd_state_t         cmd_state   : CMD_STATE_BITS;
    struct p3n_port_s      *port;
    struct p3n_buffer_s    *buf;
    uint32_t                delay;
    struct p3n_cmd_s       *next_cmd;
} p3n_cmd_t;

static_assert((CMD_TYPE_BITS + CMD_STATE_BITS) <= 32);
static_assert(sizeof(p3n_cmd_t) == 20 );
#endif


/**********************************************************
 * This structure defines the status of a channel.
 * Any channel may be used to send/receive data over
 * any port; this is needed because some hardware
 * platforms have more ports than available channels
 * and need to manage which ports are active at any
 * given time.
 */
typedef enum {
    P3N_CHAN_ST_UNINIT = 0,
    P3N_CHAN_ST_IDLE,
    P3N_CHAN_ST_TX,
    P3N_CHAN_ST_RX,
    P3N_MAXCHANST
} p3n_chan_state_t;

#define CHAN_STATE_BITS   (BITS2STORE(P3N_MAXCHANST))
#define SM_INDEX_BITS   2
#define DMA_INDEX_BITS  4

typedef struct p3n_chan_s {
    p3n_chan_state_t            state       : CHAN_STATE_BITS;
    uint32_t                    sm_index    : SM_INDEX_BITS;
    uint32_t                    dma_index   : DMA_INDEX_BITS;
    const struct p3n_port_s    *port;
    struct p3n_buffer_s        *buf;
//    struct p3n_cmd_s           *cmd;
} p3n_chan_t;

static_assert((CHAN_STATE_BITS + SM_INDEX_BITS + DMA_INDEX_BITS) <= 32);
static_assert(sizeof(p3n_chan_t) == 12 );


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
 * P3N_NUM_CHAN state machines and DMA channels
 * plus one more DMA channel for calculating CRCs
 */
bool p3n_init(void);

/**********************************************************
 * Check a channel to see if it is able to accept
 * commands.
 */
bool p3n_is_chan_avail(int chan_num);

/**********************************************************
 * Finds a channel that is able to accept commands.
 * Returns the channel number, or -1 if none available.
 */
int p3n_find_avail_chan(void);



/**********************************************************
 * use 'chan' to Start listening for messages on 'port',
 * capturing incoming data to 'buf', then wait 'delay'
 * bit times before starting next command.
 */
bool p3n_receive(p3n_chan_t *chan, p3n_port_t *port, p3n_buffer_t *buf, uint32_t delay);

/**********************************************************
 * use 'chan' to Start sending a messages from 'buf' to
 * 'port', then wait 'delay' bit times before starting
 * next command.
 */
bool p3n_transmit(p3n_chan_t *chan, p3n_port_t *port, p3n_buffer_t *buf, uint32_t delay);


#if 0

bool p3n_create_transmit_command(p3n_cmd_t *cmd, p3n_port_t port, uint32_t speed, uint32_t delay, uint32_t data_len, char *data);
bool p3n_create_receive_command(p3n_cmd_t *cmd, p3n_port_t port, uint32_t speed, uint32_t delay, uint32_t max_len, char *data);
#endif


void p3n_test(void);


#endif // PI3NET_H
