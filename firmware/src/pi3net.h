/***************************************************************
 * 
 * pi3net.h - header for Pi Pico PIO Networking
 * 
 * 
 * a couple of key concepts for this library:
 * 
 * 
 *   A "channel" is a combination of a PIO state machine
 *   and a DMA channel, which cdoes the actual work of sending
 *   or receiving data.  The number of channels determines
 *   how many messages can be in progress at any given time.
 *   The number of channels is set at compile time by
 *   P3N_NUM_CHAN.  There can be no more than four, since
 *   a PIO has only four state machines.  A channel can be
 *   configured to send data on any pins and at any bit rate.
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

static_assert(P3N_NUM_CHAN <= NUM_PIO_STATE_MACHINES);

/**********************************************************
 * This structure defines a port in terms of the available
 * hardware; is it transmit only, recieve only, or both?
 * Does it use a pin to enable an external line driver?
 */
#define P3N_NO_PIN 0x20
#define P3N_PIN_BITS BITS2STORE(P3N_NO_PIN)

typedef struct p3n_port_s {
    uint32_t rx_pin             : P3N_PIN_BITS;
    uint32_t tx_pin             : P3N_PIN_BITS;
    uint32_t tx_ena_pin         : P3N_PIN_BITS;
} p3n_port_t;

static_assert((P3N_PIN_BITS * 3) <= 32);
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
p3n_buffer_t *p3n_buffer_new(uint16_t max_msg_len_bytes);

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
 * Initializes global pi3net data including acquiring PIO
 * and DMA resources.  It will attempt to reserve 
 * P3N_NUM_CHAN state machines and DMA channels
 * plus one more DMA channel for calculating CRCs.
 */
bool p3n_init(void);

/**********************************************************
 * Cleans up and frees resources acquired by p3n_init().
 */
void p3n_uninit(void);

/**********************************************************
 * Configure the specified channel to send/receive data
 * using the specified pins and bit rate.
 * Fails if the channel is currently busy.
 */
bool p3n_configure_chan(uint ch_num, uint rx_pin, uint tx_pin, uint tx_ena_pin, uint bitrate);


/**********************************************************
 * Tell 'chan' to start listening for a message, capturing
 * incoming data to 'buf'.  Once a message is received,
 * wait 'delay' bit times before starting next command.
 */
bool p3n_receive(uint ch_num, p3n_buffer_t *buf, uint32_t delay);

/**********************************************************
 * Tell 'chan' to start sending a message from 'buf'.
 * Once message is sent, wait 'delay' bit times before
 * starting next command.
 */
bool p3n_transmit(uint ch_num, p3n_buffer_t *buf, uint32_t delay);





#if 0

bool p3n_create_transmit_command(p3n_cmd_t *cmd, p3n_port_t port, uint32_t speed, uint32_t delay, uint32_t data_len, char *data);
bool p3n_create_receive_command(p3n_cmd_t *cmd, p3n_port_t port, uint32_t speed, uint32_t delay, uint32_t max_len, char *data);
#endif


void p3n_test(void);


#endif // PI3NET_H
