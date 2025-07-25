/***************************************************************
 * 
 * pi3net.h - header for Pi Pico PIO Networking
 * 
 * Some key concepts for this library:
 * 
 *   A "channel" is a combination of a PIO state machine
 *   and a DMA channel, which does the actual work of sending
 *   or receiving data.  The number of channels determines
 *   how many messages can be in progress at any given time.
 *   The number of channels is set at compile time by
 *   P3N_NUM_CHAN.  There can be no more than four, since
 *   a PIO has only four state machines.  A channel can be
 *   configured to send data on any pins and at any bit rate.
 *
 *   A "port" is a set of pins that can be used to send
 *   and/or receive data.  Typically you would declare a
 *   port struct for each hardware Pi3Net link you want to
 *   use, then pass that struct to p3n_configure_chan().
 *
 *   A "buffer" is a data structure that holds either a
 *   message that you want to transmit, or has space for
 *   a message that you expect to receive.
 *
 **************************************************************/
#ifndef PI3NET_H
#define PI3NET_H

#include <stdint.h>    // int32_t, uint32_t

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
#define P3N_NO_PIN 0xFF

typedef struct p3n_port_s {
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint8_t tx_ena_pin;
} p3n_port_t;

/**********************************************************
 * This enum defines the possible states of a command
 */
typedef enum {
    P3N_CMD_ST_AVAIL = 0,
    P3N_CMD_ST_QUEUED,
    P3N_CMD_ST_ACTIVE,
    P3N_CMD_ST_DONE,
    P3N_CMD_ST_ERROR
} p3n_cmd_state_t;

/**********************************************************
 * This enum defines the possible error codes
 */
typedef enum { 
    P3N_SUCCESS             =  0,
    P3N_ERR_BAD_CHAN        = -1,
    P3N_ERR_BAD_BUFFER      = -2,
    P3N_ERR_BAD_DELAY       = -3,
    P3N_ERR_BAD_PIN         = -4,
    P3N_ERR_CHAN_UNINIT     = -5,
    P3N_ERR_CHAN_BUSY       = -6,
    P3N_ERR_QUEUE_FULL      = -7,
    P3N_ERR_BAD_INDEX       = -8,
    P3N_ERR_BAD_TIMEOUT     = -9
} p3n_retval_t;


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
 * before sending a message.  If buf->data_len is not a
 * multiple of 4, the data will be padded with zeros.
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
 * Configure the specified channel to send and/or receive
 * data using a specified port (set of pins) and bit rate.
 * Reserves memory for a command queue with 'queue_len'
 * entries.  Fails if the channel is currently busy or
 * if any of the inputs are out of range.
 */
bool p3n_channel_config(uint ch_num, p3n_port_t const *port, uint bitrate, uint8_t queue_len);

/**********************************************************
 * Blocking receive.  Tells channel 'ch_num' to listen for
 * an incoming message, storing the received data in 'buf'.
 * Returns immediately for bad arguments or if the channel
 * is busy.  Otherwise, returns when a message has been
 * received, or after 'timeout_us' microseconds, whichever
 * comes first.
 * Check 'buf->data_len' to see if a message was received.
 */
p3n_retval_t p3n_receive(uint ch_num, p3n_buffer_t *buf, uint32_t timeout_us);

/**********************************************************
 * Blocking transmit.  Tells channel 'ch_num' to send the
 * data in 'buf'.  Returns immediately for bad arguments
 * or of the channel is busy.  Otherwise, returns when
 * the message has been sent.
 */
p3n_retval_t p3n_transmit(uint ch_num, p3n_buffer_t *buf);

/**********************************************************
 * Queue a receive command for channel 'chan'.  Incoming
 * data will be captured to 'buf'.  Once a message is
 * received, the channel will wait 'delay' bit times
 * before starting the next command.
 *
 * Returns an index that can be used to check the command
 * status, or a negative error code on failure.
 */
int p3n_queue_receive_cmd(uint ch_num, p3n_buffer_t *buf, uint32_t delay);

/**********************************************************
 * Queue a transmit command for channel 'chan'.  Data will
 * be sent from 'buf'.  Once the message is transmitted,
 * the channel will wait 'delay' bit times before starting
 * the next command.
 *
 * Returns an index that can be used to check the command
 * status, or a negative error code on failure.
 */
int p3n_queue_transmit_cmd(uint ch_num, p3n_buffer_t *buf, uint32_t delay);

/**********************************************************
 * Returns the state of a command that was previously
 * placed in the command queue for 'ch_num'.  'cmd_index'
 * is the index returned when the command was queued.
 */
p3n_cmd_state_t p3n_get_cmd_state(uint ch_num, int cmd_index);

/**********************************************************
 * Returns the buffer associated with a command that was
 * previously placed in the command queue for 'ch_num'.
 * 'cmd_index' is the index returned when the command
 * was queued.
 */
p3n_buffer_t *p3n_get_cmd_buffer(uint ch_num, int cmd_index);

/**********************************************************
 * Switches a command queue entry from the "done" state
 * to the "available" state.  An application calls this
 * function once it has processed a received message, or
 * any time after a transmitted message has been sent.
 * It disassociates the message buffer from the commane
 * queue entry so that the buffer can be reused.
 * Returns false if the command is not done.
 */
bool p3n_cmd_release(uint ch_num, int cmd_index);

/**********************************************************
 * Cancels all queued commands for channel 'ch_num',
 * returning the command queue entries to the 'available'
 * state.
 */
void p3n_channel_full_abort(uint ch_num);

/**********************************************************
 * If 'ch_num' is currently waiting to receive a message,
 * this function skips the receive and its post-command
 * delay, allowing the next queued command (if any) to
 * run.  If not waiting for receive, does nothing.
 */
void p3n_channel_rx_abort(uint ch_num);

/**********************************************************
 * If 'ch_num' is currently in a post-command delay, this
 * function skips the delay, allowing the next queued
 * command (if any) to run.  If not in a post-command
 * delay, does nothing.
 */
void p3n_channel_delay_abort(uint ch_num);

void p3n_test(void);


#endif // PI3NET_H
