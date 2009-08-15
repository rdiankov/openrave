/*
 *  libplayerc : a Player client library
 *  Copyright (C) Andrew Howard 2002-2003
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

/***************************************************************************
 * Desc: Player client
 * Author: Andrew Howard
 * Date: 24 Aug 2001
 # CVS: $Id: playerc.h,v 1.185.2.6 2006/09/25 17:00:15 gerkey Exp $
 **************************************************************************/


/** @ingroup clientlibs
@defgroup player_clientlib_libplayerc libplayerc
@brief A C client library for the @ref util_player

libplayerc is a client library for the @ref util_player.  It
is written in C to maximize portability, and in the expectation that
users will write bindings for other languages (such as Python and
Java) against this library; @ref player_clientlib_libplayerc_py "Python bindings" are
already available.

Be sure to check out the @ref libplayerc_example "example".

The @ref libplayerc_datamodes "data modes" section is important reading for all
client writers.
*/
/** @{ */


#ifndef PLAYERC_H
#define PLAYERC_H

#include <stdio.h>
#include <sys/types.h>
#include <netinet/in.h>

// Get the message structures from Player
#include <libplayercore/player.h>
#include <libplayercore/playercommon.h>
#include <libplayercore/error.h>
#include <libplayerxdr/playerxdr.h>

#ifndef MIN
  #define MIN(a,b) ((a < b) ? a : b)
#endif
#ifndef MAX
  #define MAX(a,b) ((a > b) ? a : b)
#endif

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 * Useful constants (re-defined here so SWIG can pick them up easily)
 **************************************************************************/

/** The device access modes */
#define PLAYERC_OPEN_MODE     PLAYER_OPEN_MODE
#define PLAYERC_CLOSE_MODE    PLAYER_CLOSE_MODE
#define PLAYERC_ERROR_MODE    PLAYER_ERROR_MODE


/** The valid data delivery modes */
#define PLAYERC_DATAMODE_PUSH PLAYER_DATAMODE_PUSH
#define PLAYERC_DATAMODE_PULL PLAYER_DATAMODE_PULL

/** The valid transports */
#define PLAYERC_TRANSPORT_TCP 1
#define PLAYERC_TRANSPORT_UDP 2

/***************************************************************************
 * Array sizes
 **************************************************************************/

#define PLAYERC_MAX_DEVICES             PLAYER_MAX_DEVICES
#define PLAYERC_LASER_MAX_SAMPLES       PLAYER_LASER_MAX_SAMPLES
#define PLAYERC_FIDUCIAL_MAX_SAMPLES    PLAYER_FIDUCIAL_MAX_SAMPLES
#define PLAYERC_SONAR_MAX_SAMPLES       PLAYER_SONAR_MAX_SAMPLES
#define PLAYERC_BUMPER_MAX_SAMPLES      PLAYER_BUMPER_MAX_SAMPLES
#define PLAYERC_IR_MAX_SAMPLES          PLAYER_IR_MAX_SAMPLES
#define PLAYERC_BLOBFINDER_MAX_BLOBS    PLAYER_BLOBFINDER_MAX_BLOBS
#define PLAYERC_WIFI_MAX_LINKS          PLAYER_WIFI_MAX_LINKS
#define PLAYERC_RFID_MAX_TAGS           PLAYER_RFID_MAX_TAGS
#define PLAYERC_RFID_MAX_GUID           PLAYER_RFID_MAX_GUID

/** @} */

/**
@ingroup player_clientlib_libplayerc
@defgroup libplayerc_example libplayerc example
@brief An example

libplayerc is based on a device <i>proxy</i> model, in which the client
maintains a local proxy for each of the devices on the remote server.
Thus, for example, one can create local proxies for the
@ref interface_position2d
and @ref interface_laser devices.  There is also a special <tt>client</tt> proxy,
used to control the Player server itself.

Programs using libplayerc will generally have the following structure:

@include simpleclient.c

This example can be built using the command:
<PRE>
$ gcc -o simpleclient `pkg-config --cflags playerc` simpleclient.c `pkg-config --libs playerc`
</PRE>

Make sure that <tt>libplayerc</tt> is installed somewhere that pkg-config
can find it.

<P>
The above program can be broken into six steps, as follows.

<ul>

<li> Create and connect a client proxy.

<P>
<PRE>
client = playerc_client_create(NULL, "localhost", 6665);
playerc_client_connect(client);
</PRE>
The <TT>create</TT> function creates a new client proxy and returns a
pointer to be used in future function calls (<TT>localhost</TT> should be
replaced with the network host name of the robot).  The <TT>connect</TT>
function notifies the Player server that a new client wishes to
recieve data.


<li> Create and subscribe a device proxy.

<P>
<PRE>
position2d = playerc_position2d_create(client, 0);
playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE);
</PRE>
The <TT>create</TT> function creates a new position2d device proxy and
returns a pointer to be used in future function calls.  The <TT>subscribe</TT> function notifies the Player server that the client is
using the position2d device, and that the client expects to both send
commands and recieve data (<TT>PLAYERC_OPEN_MODE</TT>).

<li> Configure the device, send commands.

<P>
<PRE>
playerc_position2d_enable(position2d, 1);
playerc_position2d_set_speed(position2d, 0, 0, 0.1);
</PRE>
The <TT>enable</TT> function sends a configuration request to the server,
changing the robot's motor state from <TT>off</TT> to <TT>on</TT>, thereby
allowing the robot to move.  The <TT>setspeed</TT> function sends a new
motor speed, in this case commanding the robot to turn on the spot.


<P>
Note that most Player devices will accept both asynchronous
<EM>command</EM> and synchronous <EM>configuration</EM> requests.
Sending commands is analogous using the standard Unix <TT>write</TT>
device interface, while sending configuration requests is analogous to
using the <TT>ioctl</TT> interface.  For the most part,
<TT>libplayerc</TT> hides the distinction between these two
interfaces.  Users should be aware, however, that while commands are
always handled promptly by the server, configuration requests may take
significant time to complete.  If possible, configuration requests
should therefore be restricted to the initialization phase of the
program.


<li> Read data from the device.

<P>
<PRE>
playerc_client_read(client);
printf("position : %f %f %f\n", position2d-&gt;px, ... );
</PRE>
The <TT>read</TT> function blocks until new data arrives from the Player
server.  This data may be from one of the subscribed devices, or it
may be from the server itself (which sends regular synchronization
messages to all of its clients).  The <TT>read</TT> function inspects the
incoming data and automatically updates the elements in the
appropriate device proxy.  This function also returns a pointer to the
proxy that was updated, so that user programs may, if desired, trigger
appropriate events on the arrival of different kinds of data.


<li> Unsubscribe and destroy the device proxy.

<P>
<PRE>
playerc_position2d_unsubscribe(position2d);
playerc_position2d_destroy(position2d);
</PRE>
The <TT>unsubscribe</TT> function tells the Player server that the client
is no longer using this device.  The <TT>destroy</TT> function then frees
the memory associated with the device proxy; the <TT>device</TT> pointer
is now invalid and should be not be re-used.


<li> Disconnect and destroy the client proxy.

<P>
<PRE>
playerc_client_disconnect(client);
playerc_client_destroy(client);
</PRE>
The <TT>disconnect</TT> function tells the server that the client is
shutting down.  The <TT>destroy</TT> function then frees the memory
associated with the client proxy; the <TT>client</TT> pointer is now
invalid and should be not be re-used.

</ul>

@defgroup libplayerc_datamodes Client Data Modes
@brief Describes the data modes used for delivering data from the server to the client.

The server may send data to the client in one of two modes: PUSH and PULL. The
current data mode is changed using the @ref playerc_client_datamode function.

The default mode is PUSH.

@section pushmode PUSH Mode

PUSH mode is more reliable for ensuring that all data generated by the server
will reach the client, although there is still a risk of message queues
overflowing in this mode.

When in PUSH mode the server will send all messages directed at the client as
soon as they become available. For example, if a message carrying range
readings from a laser scanner is generated by a laser data, it will be added
to the client's queue and sent out as soon as possible by the server. When
a client read is performed by calling @ref playerc_client_read the next packet
in the client's queue is read.

When using PUSH mode be aware that the client must keep up with the speed of
generation of DATA messages by the server. If the client cannot read messages
as fast as the server is producing them, the data received by the client will
become increasingly out of date. This may be avoidable by setting appropriate
add-replace rules, but read the next paragraph first.

If you are using the TCP protocol for the connection between the client and the
server, and are in PUSH mode, then add-replace rules for the client's queue
may not function correctly. This is because messages will leave the client's
queue on the server as soon as there is space for them in the operating
system's TCP buffer for the connection. As a result, messages will not be in
the actual queue long enough for add-replace rules to function correctly until
this buffer becomes full (it is typically many megabytes large). To avoid this
problem, use PULL mode.

@section pullmode PULL Mode

PULL mode is the preferred mode for most clients. When used together with
suitable add-replace rules, it guarentees that data received by the client is
the most up-to-date data available from the server at the beginning of the read
function.

When in PULL mode the server will send all messages directed at the client, but
only when requested by the client. This request is performed at the beginning
of every call to @ref playerc_client_read, so client programs do not need to
perform it themselves. The client will then read all data that is waiting on
the server in one go. In this mode, client programs do not need to continually
call read until there are no messages waiting.

When using PULL mode, beware of the client's queue on the server overflowing,
particularly if using TCP. While PUSH mode can take advantage of the OS's TCP
buffer, in PULL mode this buffer is not used until the client requests data.
The standard length of the client's message queue on the server is 32 messages.
To avoid the queue overflowing, use add-replace rules to automatically replace
older messages with more up-to-date messages for the same message signature.

@section datamodeexample Data Mode Example

The following code will change a client's data mode to PULL mode and set an
add-replace rule to replace all DATA messages generated by the server,
preventing the message queue from overflowing when they are produced faster
than the client reads and ensuring that the client receives the most up-to-date
data.

<pre>
if (playerc_client_datamode (client, PLAYERC_DATAMODE_PULL) != 0)
{
  fprintf(stderr, "error: %s\n", playerc_error_str());
  return -1;
}
if (playerc_client_set_replace_rule (client, -1, -1, PLAYER_MSGTYPE_DATA, -1, 1) != 0)
{
  fprintf(stderr, "error: %s\n", playerc_error_str());
  return -1;
}
</pre>

The add-replace rule specifies that any message of type PLAYER_MSGTYPE_DATA
will be replaced by newer versions. So, if a driver produces a data packet and
there is already a data packet from that driver with the same signature on the
queue, it will be removed from the queue (and not sent to the client), with
the new packet pushed onto the end of the queue.
*/

/***************************************************************************/
/** @ingroup player_clientlib_libplayerc
 * @defgroup playerc_utility Utility and error-handling
 * @brief Some helper functions
@{
*/
/***************************************************************************/

/** Retrieve the last error (as a descriptive string).  Most functions
    in will return 0 on success and non-zero value on error; a
    descriptive error message can be obtained by calling this
    function. */
const char *playerc_error_str(void);

/** Get the name for a given interface code. */
const char *playerc_lookup_name(int code);

/** Get the interface code for a given name. */
int playerc_lookup_code(const char *name);

/** @}*/
/***************************************************************************/


// Forward declare types
struct _playerc_client_t;
struct _playerc_device_t;


// forward declaration to avoid including <sys/poll.h>, which may not be
// available when people are building clients against this lib
struct pollfd;


/***************************************************************************/
/** @ingroup player_clientlib_libplayerc
 * @defgroup multiclient Multi-Client object
 * @brief The multi-client object manages connections to multiple server in
 * parallel.

@todo Document mutliclient

@{
*/

// Items in incoming data queue.
typedef struct
{
  player_msghdr_t header;
  void *data;
} playerc_client_item_t;


// Multi-client data
typedef struct
{
  // List of clients being managed
  int client_count;
  struct _playerc_client_t *client[128];

  // Poll info
  struct pollfd* pollfd;

  // Latest time received from any server
  double time;

} playerc_mclient_t;

// Create a multi-client object
playerc_mclient_t *playerc_mclient_create(void);

// Destroy a multi-client object
void playerc_mclient_destroy(playerc_mclient_t *mclient);

// Add a client to the multi-client (private).
int playerc_mclient_addclient(playerc_mclient_t *mclient, struct _playerc_client_t *client);

// Test to see if there is pending data.
// Returns -1 on error, 0 or 1 otherwise.
int playerc_mclient_peek(playerc_mclient_t *mclient, int timeout);

// Read incoming data.  The timeout is in ms.  Set timeout to a
// negative value to wait indefinitely.
int playerc_mclient_read(playerc_mclient_t *mclient, int timeout);

/** @} */
/***************************************************************************/


/***************************************************************************/
/** @ingroup player_clientlib_libplayerc
 * @defgroup playerc_client Client API

The client object manages the connection with the Player server; it is
responsible for reading new data, setting data transmission modes and
so on.  The client object must be created and connected before device
proxies are initialized.

@{
*/

/** @brief Typedef for proxy callback function */
typedef void (*playerc_putmsg_fn_t) (void *device, char *header, char *data);

/** @brief Typedef for proxy callback function */
typedef void (*playerc_callback_fn_t) (void *data);


/** @brief Info about an available (but not necessarily subscribed)
    device.
 */
typedef struct
{
  /** Player id of the device. */
  player_devaddr_t addr;

  /** The driver name. */
  char drivername[PLAYER_MAX_DRIVER_STRING_LEN];

} playerc_device_info_t;


/** @brief Client object data. */
typedef struct _playerc_client_t
{
  /** A useful ID for identifying devices; mostly used by other
      language bindings. */
  void *id;

  /** Server address. */
  char *host;
  int port;
  int transport;
  struct sockaddr_in server;

  /** How many times we'll try to reconnect after a socket error.  Use @ref
   * playerc_client_set_retry_limit() to set this value. Set to -1 for
   * infinite retry. */
  int retry_limit;

  /** How long to sleep, in seconds, to sleep between reconnect attempts.
   * Use @ref playerc_client_set_retry_time() to set this value. */
  double retry_time;

  /** @internal Socket descriptor */
  int sock;

  /** @internal Data delivery mode */
  uint8_t mode;

  /** List of available (but not necessarily subscribed) devices.
      This list is filled in by playerc_client_get_devlist(). */
  playerc_device_info_t devinfos[PLAYERC_MAX_DEVICES];
  int devinfo_count;

  /** List of subscribed devices */
  struct _playerc_device_t *device[32];
  int device_count;

  /** @internal A circular queue used to buffer incoming data packets. */
  playerc_client_item_t qitems[512];
  int qfirst, qlen, qsize;

  /** @internal Temp buffers for incoming / outgoing packets. */
  char *data;
  char *write_xdrdata;
  char *read_xdrdata;
  size_t read_xdrdata_len;


  /** Server time stamp on the last packet. */
  double datatime;
  /** Server time stamp on the previous packet. */
  double lasttime;

  double request_timeout;

} playerc_client_t;


/** @brief Create a client object.

@param mclient Multiclient object; set this NULL if this is a
stand-alone client.

@param host Player server host name (i.e., name of the machine
with the Player server).

@param port Player server port (typically 6665, but depends on the
server configuration).

@returns Returns a newly allocated pointer to the client object; use
playerc_client_destroy() to delete the object.

*/
playerc_client_t *playerc_client_create(playerc_mclient_t *mclient,
                                        const char *host, int port);

/** @brief Destroy a client object.

@param client Pointer to client object.

*/
void playerc_client_destroy(playerc_client_t *client);

/** @brief Set the transport type.

@param transport Either PLAYERC_TRANSPORT_UDP or PLAYERC_TRANSPORT_TCP
*/
void playerc_client_set_transport(playerc_client_t* client,
                                  unsigned int transport);

/** @brief Connect to the server.

@param client Pointer to client object.

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_client_connect(playerc_client_t *client);

/** @brief Disconnect from the server.

@param client Pointer to client object.

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_client_disconnect(playerc_client_t *client);

/** @brief Change the server's data delivery mode.

Be sure to @ref libplayerc_datamodes "read about data modes" before using this function.

@param client Pointer to client object.

@param mode Data delivery mode; must be one of
PLAYERC_DATAMODE_PUSH, PLAYERC_DATAMODE_PULL; the defalt mode is
PLAYERC_DATAMODE_PUSH.

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_client_datamode(playerc_client_t *client, uint8_t mode);

/** @brief Request a round of data.

@param client Pointer to client object.

Request a round of data; only valid when in a request/reply (aka PULL)
data delivery mode.  But you don't need to call this function, because @ref
playerc_client_read will do it for you if the client is in a PULL mode.

Use @ref playerc_client_datamode to change modes.

*/
int playerc_client_requestdata(playerc_client_t* client);

/** @brief Set a replace rule for the client queue on the server

If a rule with the same pattern already exists, it will be replaced with
the new rule (i.e., its setting to replace will be updated).

@param client Pointer to client object.

@param interf Interface to set replace rule for (-1 for wildcard)

@param index Index to set replace rule for (-1 for wildcard)

@param type Type to set replace rule for (-1 for wildcard),
i.e. PLAYER_MSGTYPE_DATA

@param subtype Message subtype to set replace rule for (-1 for wildcard)

@param replace Should we replace these messages

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_client_set_replace_rule(playerc_client_t *client, int interf, int index, int type, int subtype, int replace);


/** @brief Add a device proxy. @internal
 */
int playerc_client_adddevice(playerc_client_t *client, struct _playerc_device_t *device);


/** @brief Remove a device proxy. @internal
 */
int playerc_client_deldevice(playerc_client_t *client, struct _playerc_device_t *device);

/** @brief Add user callbacks (called when new data arrives). @internal
 */
int  playerc_client_addcallback(playerc_client_t *client, struct _playerc_device_t *device,
                                playerc_callback_fn_t callback, void *data);

/** @brief Remove user callbacks (called when new data arrives). @internal
 */
int  playerc_client_delcallback(playerc_client_t *client, struct _playerc_device_t *device,
                                playerc_callback_fn_t callback, void *data);

/** @brief Get the list of available device ids.

This function queries the server for the list of available devices,
and write result to the devinfos list in the client object.

@param client Pointer to client object.

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_client_get_devlist(playerc_client_t *client);

/** @brief Subscribe a device. @internal
 */
int playerc_client_subscribe(playerc_client_t *client, int code, int index,
                             int access, char *drivername, size_t len);

/** @brief Unsubscribe a device. @internal
 */
int playerc_client_unsubscribe(playerc_client_t *client, int code, int index);

/** @brief Issue a request to the server and await a reply (blocking). @internal

@returns Returns -1 on error and -2 on NACK.

*/
int playerc_client_request(playerc_client_t *client,
                           struct _playerc_device_t *device, uint8_t reqtype,
                           void *req_data, void *rep_data, int rep_len);

/** @brief Wait for response from server (blocking).

@param client Pointer to client object.
@param device
@param index
@param sequence

@returns Will return data size for ack, -1 for nack and -2 for failure

*/
/*int playerc_client_getresponse(playerc_client_t *client, uint16_t device,
    uint16_t index, uint16_t sequence, uint8_t * resptype, uint8_t * resp_data, int resp_len);
*/
/** @brief Test to see if there is pending data.

@param client Pointer to client object.

@param timeout Timeout value (ms).  Set timeout to 0 to check for
currently queued data.

@returns Returns -1 on error, 0 or 1 otherwise.

*/
int playerc_client_peek(playerc_client_t *client, int timeout);

/** @brief Read data from the server (blocking).

@param client Pointer to client object.

@returns PUSH mode: For data packets, will return the ID of the proxy that got
the data; for synch packets, will return the ID of the client itself;
on error, will return NULL.
PULL mode: Will return NULL on error, the ID of the client on success. Will
never return the ID of a proxy other than the client.

*/
void *playerc_client_read(playerc_client_t *client);


/** @brief Set the timeout for client requests.

@param client Pointer to client object.
@param seconds Seconds to wait for a reply.

*/
void playerc_client_set_request_timeout(playerc_client_t* client, uint seconds);

/** @brief Set the connection retry limit.

@param client Pointer to the client object
@param limit The number of times to attempt to reconnect to the server.  Give -1 for 
       infinite retry.
*/
void playerc_client_set_retry_limit(playerc_client_t* client, int limit);

/** @brief Set the connection retry sleep time.

@param client Pointer to the client object
@param time The amount of time, in seconds, to sleep between reconnection attempts.
*/
void playerc_client_set_retry_time(playerc_client_t* client, double time);

/** @brief Write data to the server.  @internal
*/
int playerc_client_write(playerc_client_t *client,
                         struct _playerc_device_t *device,
                         uint8_t subtype,
                         void *cmd, double* timestamp);


/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup player_clientlib_libplayerc
 * @defgroup playerc_device Device API

The device object provides a common interface to the functionality
that is shared by all device proxies (in OOP parlance, it is a base
class).  In general, this object should not be instantiated or
accessed directly: use the device proxies instead.

@{
*/

/** @brief Common device info. */
typedef struct _playerc_device_t
{
  /** A useful ID for identifying devices; mostly used by other
      language bindings.  For backwards-compatibility, this is passed
      as void pointer. */
  void *id;

  /** Pointer to the client proxy. */
  playerc_client_t *client;

  /** Device address */
  player_devaddr_t addr;

  /** The driver name. */
  char drivername[PLAYER_MAX_DRIVER_STRING_LEN];

  /** The subscribe flag is non-zero if the device has been
      successfully subscribed (read-only). */
  int subscribed;

  /** Data timestamp, i.e., the time at which the data was generated (s). */
  double datatime;

  /** Data timestamp from the previous data. */
  double lasttime;

  /** Freshness flag.  Set to 1 whenever data is dispatched to this
      proxy.  Useful with the multi-client, but the user must manually
      set it to 0 after using the data. */
  int fresh;
  /** Freshness flag.  Set to 1 whenever data is dispatched to this
      proxy.  Useful with the multi-client, but the user must manually
      set it to 0 after using the data. */
  int freshgeom;
  /** Freshness flag.  Set to 1 whenever data is dispatched to this
      proxy.  Useful with the multi-client, but the user must manually
      set it to 0 after using the data. */
  int freshconfig;

  /**  Standard message callback for this device.  @internal */
  playerc_putmsg_fn_t putmsg;

  /** Extra user data for this device. @internal */
  void *user_data;

  /** Extra callbacks for this device. @internal */
  int callback_count;
  playerc_callback_fn_t callback[4];
  void *callback_data[4];

} playerc_device_t;


/** @brief Initialise the device. Additional callbacks for geom and config @internal */
void playerc_device_init(playerc_device_t *device, playerc_client_t *client,
                         int code, int index, playerc_putmsg_fn_t putmsg);

/** @brief Finalize the device. @internal */
void playerc_device_term(playerc_device_t *device);

/** @brief Subscribe the device. @internal */
int playerc_device_subscribe(playerc_device_t *device, int access);

/** @brief Unsubscribe the device. @internal */
int playerc_device_unsubscribe(playerc_device_t *device);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup player_clientlib_libplayerc
 * @defgroup playerc_proxies Device proxies
 * @brief Each interface has a corresponding proxy
*/
/***************************************************************************/

/**************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_aio aio

The aio proxy provides an interface to the analog input/output sensors.

@{
*/

/** Aio proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /// The number of valid analog inputs.
  uint8_t voltages_count;

  /// A bitfield of the current digital inputs.
  float voltages[PLAYER_AIO_MAX_INPUTS];

} playerc_aio_t;


/** Create a aio proxy. */
playerc_aio_t *playerc_aio_create(playerc_client_t *client, int index);

/** Destroy a aio proxy. */
void playerc_aio_destroy(playerc_aio_t *device);

/** Subscribe to the aio device. */
int playerc_aio_subscribe(playerc_aio_t *device, int access);

/** Un-subscribe from the aio device. */
int playerc_aio_unsubscribe(playerc_aio_t *device);

/** Set the output for the aio device. */
int playerc_aio_set_output(playerc_aio_t *device, uint8_t id, float volt);

/** @} */
/***************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_actarray actarray

The actarray proxy provides an interface to actuator arrays
such as the ActivMedia Pioneer Arm. See the Player User Manual for a
complete description of the drivers that support this interface.

@{
*/

/** @brief Actarray device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** The number of actuators in the array. */
  uint32_t actuators_count;
  /** The actuator data and geometry. */
  player_actarray_actuator_t actuators_data[PLAYER_ACTARRAY_NUM_ACTUATORS];
  player_actarray_actuatorgeom_t actuators_geom[PLAYER_ACTARRAY_NUM_ACTUATORS];

  /** current trajectory robot is executing. If  */
  uint32_t trajectory_id;
  /** how far the robot is along the current trajectory */
  float trajectory_time;
} playerc_actarray_t;

/** @brief Create an actarray proxy. */
playerc_actarray_t *playerc_actarray_create(playerc_client_t *client, int index);

/** @brief Destroy an actarray proxy. */
void playerc_actarray_destroy(playerc_actarray_t *device);

/** @brief Subscribe to the actarray device. */
int playerc_actarray_subscribe(playerc_actarray_t *device, int access);

/** @brief Un-subscribe from the actarray device. */
int playerc_actarray_unsubscribe(playerc_actarray_t *device);

/** @brief Get the actarray geometry.  The writes the result into the proxy
    rather than returning it to the caller. */
int playerc_actarray_get_geom(playerc_actarray_t *device);

/** @brief Command a joint in the array to move to a specified position. */
int playerc_actarray_position_cmd(playerc_actarray_t *device, int joint, float position);

/** @brief Command a joint in the array to move at a specified speed. */
int playerc_actarray_speed_cmd(playerc_actarray_t *device, int joint, float speed);

/** @brief Command a joint (or, if joint is -1, the whole array) to go to its home position. */
int playerc_actarray_home_cmd(playerc_actarray_t *device, int joint);

/** @brief Give a trajectory to all the actarray joints. */
int playerc_actarray_trajectory_cmd(playerc_actarray_t *device, int numpoints, int trajformat, float time_of_divergence, uint32_t trajectory_id, const float* data);

/** @brief Signals the driver to start the currently queued trajectory. Only valid
    if PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCEXECUTION was specified as a trajectory flag format. */
int playerc_actarray_start_cmd(playerc_actarray_t *device, int64_t timestamp);

/** @brief Turn the power to the array on or off. Be careful
when turning power on that the array is not obstructed from its home
position in case it moves to it (common behaviour). */
int playerc_actarray_power(playerc_actarray_t *device, uint8_t enable);

/** @brief Turn the brakes of all actuators in the array that have them on or off. */
int playerc_actarray_brakes(playerc_actarray_t *device, uint8_t enable);

/** @brief Set the speed of a joint (-1 for all joints) for all subsequent movement commands. */
int playerc_actarray_speed_config(playerc_actarray_t *device, int joint, float speed);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_blobfinder blobfinder

The blobfinder proxy provides an interface to color blob detectors
such as the ACTS vision system.  See the Player User Manual for a
complete description of the drivers that support this interface.

@{
*/

typedef player_blobfinder_blob_t playerc_blobfinder_blob_t;

/** @brief Blobfinder device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Image dimensions (pixels). */
  unsigned int width, height;

  /** A list of detected blobs. */
  unsigned int blobs_count;
  playerc_blobfinder_blob_t blobs[PLAYER_BLOBFINDER_MAX_BLOBS];

} playerc_blobfinder_t;


/** @brief Create a blobfinder proxy. */
playerc_blobfinder_t *playerc_blobfinder_create(playerc_client_t *client, int index);

/** @brief Destroy a blobfinder proxy. */
void playerc_blobfinder_destroy(playerc_blobfinder_t *device);

/** @brief Subscribe to the blobfinder device. */
int playerc_blobfinder_subscribe(playerc_blobfinder_t *device, int access);

/** @brief Un-subscribe from the blobfinder device. */
int playerc_blobfinder_unsubscribe(playerc_blobfinder_t *device);


/** @} */
/**************************************************************************/


/**************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_bumper bumper

The bumper proxy provides an interface to the bumper sensors built
into robots such as the RWI B21R.

@{
*/

/** @brief Bumper proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Number of pose values. */
  int pose_count;

  /** Pose of each bumper relative to robot (mm, mm, deg, mm, mm).
      This structure is filled by calling playerc_bumper_get_geom().
      values are x,y (of center) ,normal,length,curvature */
  player_bumper_define_t poses[PLAYERC_BUMPER_MAX_SAMPLES];

  /** Number of points in the scan. */
  int bumper_count;

  /** Bump data: unsigned char, either boolean or code indicating corner. */
  uint8_t bumpers[PLAYERC_BUMPER_MAX_SAMPLES];

} playerc_bumper_t;


/** @brief Create a bumper proxy. */
playerc_bumper_t *playerc_bumper_create(playerc_client_t *client, int index);

/** @brief Destroy a bumper proxy. */
void playerc_bumper_destroy(playerc_bumper_t *device);

/** @brief Subscribe to the bumper device. */
int playerc_bumper_subscribe(playerc_bumper_t *device, int access);

/** @brief Un-subscribe from the bumper device. */
int playerc_bumper_unsubscribe(playerc_bumper_t *device);

/** @brief Get the bumper geometry.

The writes the result into the proxy rather than returning it to the
caller.

*/
int playerc_bumper_get_geom(playerc_bumper_t *device);


/** @} */
/***************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_camera camera

The camera proxy can be used to get images from a camera.

@{
*/

/** @brief Camera proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Image dimensions (pixels). */
  int width, height;

  /** Image bits-per-pixel (8, 16, 24). */
  int bpp;

  /** Image format (e.g., RGB888). */
  int format;

  /** Some images (such as disparity maps) use scaled pixel values;
      for these images, fdiv specifies the scale divisor (i.e., divide
      the integer pixel value by fdiv to recover the real pixel value). */
  int fdiv;

  /** Image compression method. */
  int compression;

  /** Size of image data (bytes) */
  int image_count;

  /** Image data (byte aligned, row major order).  Multi-byte image
      formats (such as MONO16) are automatically converted to the
      correct host byte ordering.
  */
  uint8_t image[PLAYER_CAMERA_IMAGE_SIZE];

} playerc_camera_t;


/** @brief Create a camera proxy. */
playerc_camera_t *playerc_camera_create(playerc_client_t *client, int index);

/** @brief Destroy a camera proxy. */
void playerc_camera_destroy(playerc_camera_t *device);

/** @brief Subscribe to the camera device. */
int playerc_camera_subscribe(playerc_camera_t *device, int access);

/** @brief Un-subscribe from the camera device. */
int playerc_camera_unsubscribe(playerc_camera_t *device);

/** @brief Decompress the image (modifies the current proxy data). */
void playerc_camera_decompress(playerc_camera_t *device);

/** @brief Saves the image to disk as a .ppm */
void playerc_camera_save(playerc_camera_t *device, const char *filename);



/** @} */
/**************************************************************************/


/**************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_dio dio

The dio proxy provides an interface to the digital input/output sensors.

@{
*/

/** Dio proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

    /// The number of valid digital inputs.
    uint8_t count;

    /// A bitfield of the current digital inputs.
    uint32_t digin;

} playerc_dio_t;


/** Create a dio proxy. */
playerc_dio_t *playerc_dio_create(playerc_client_t *client, int index);

/** Destroy a dio proxy. */
void playerc_dio_destroy(playerc_dio_t *device);

/** Subscribe to the dio device. */
int playerc_dio_subscribe(playerc_dio_t *device, int access);

/** Un-subscribe from the dio device. */
int playerc_dio_unsubscribe(playerc_dio_t *device);

/** Set the output for the dio device. */
int playerc_dio_set_output(playerc_dio_t *device, uint8_t output_count, uint32_t digout);


/** @} */
/***************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_fiducial fiducial

The fiducial proxy provides an interface to a fiducial detector.  This
device looks for fiducials (markers or beacons) in the laser scan, and
determines their identity, range, bearing and orientation.  See the
Player User Manual for a complete description of the various drivers
that support the fiducial interface.

@{
*/


/** @brief Fiducial finder data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Geometry in robot cs.  These values are filled in by
      playerc_fiducial_get_geom().  [pose] is the detector pose in the
      robot cs, [size] is the detector size, [fiducial_size] is the
      fiducial size. */
  player_fiducial_geom_t fiducial_geom;

  /** List of detected beacons. */
  int fiducials_count;
  player_fiducial_item_t fiducials[PLAYERC_FIDUCIAL_MAX_SAMPLES];

} playerc_fiducial_t;


/** @brief Create a fiducial proxy. */
playerc_fiducial_t *playerc_fiducial_create(playerc_client_t *client, int index);

/** @brief Destroy a fiducial proxy. */
void playerc_fiducial_destroy(playerc_fiducial_t *device);

/** @brief Subscribe to the fiducial device. */
int playerc_fiducial_subscribe(playerc_fiducial_t *device, int access);

/** @brief Un-subscribe from the fiducial device. */
int playerc_fiducial_unsubscribe(playerc_fiducial_t *device);

/** @brief Get the fiducial geometry.

Ths writes the result into the proxy rather than returning it to the
caller.

*/
int playerc_fiducial_get_geom(playerc_fiducial_t *device);


/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_visionserver visionserver

This device looks for objects in a scene, and returns their identities.
The ids can later be used as visionboject interfaces to query pose data.

@{
*/

#define PLAYER_MAX_MODEL_STRING_LENGTH 128

/** @brief visionserver finder data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Geometry in robot cs.  These values are filled in by
      playerc_visionserver_get_geom().  [pose] is the object pose in the
      opneRAVE coordinate space.
  */
  player_visionserver_geom_t _geom;

  /** visionserver pose and status */
  player_visionserver_data_t _data;

} playerc_visionserver_t;


/** @brief Create a visionserver proxy. */
playerc_visionserver_t *playerc_visionserver_create(playerc_client_t *client, int index);

/** @brief Destroy a visionserver proxy. */
void playerc_visionserver_destroy(playerc_visionserver_t *device);

/** @brief Subscribe to the fiducial device. */
int playerc_visionserver_subscribe(playerc_visionserver_t *device, int access);

/** @brief Un-subscribe from the fiducial device. */
int playerc_visionserver_unsubscribe(playerc_visionserver_t *device);

/** @brief Get the fiducial geometry.

Ths writes the result into the proxy rather than returning it to the
caller.

*/
int playerc_visionserver_get_geom(playerc_visionserver_t *device);


/** @} */
/**************************************************************************/

/***************************************************************************/
/** @defgroup playerc_proxy_gps gps

The gps proxy provides an interface to a GPS-receiver.

@{
*/

/** @brief GPS proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** UTC time (seconds since the epoch) */
  double utc_time;

  /** Latitude and logitude (degrees).  Latitudes are positive for
      north, negative for south.  Logitudes are positive for east,
      negative for west. */
  double lat, lon;

  /** Altitude (meters).  Positive is above sea-level, negative is
      below. */
  double alt;

  /** UTM easting and northing (meters). */
  double utm_e, utm_n;

  /** Horizontal dilution of precision. */
  double hdop;

  /** Vertical dilution of precision. */
  double vdop;

  /** Horizontal and vertical error (meters). */
  double err_horz, err_vert;

  /** Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix */
  int quality;

  /** Number of satellites in view. */
  int sat_count;

} playerc_gps_t;


/** @brief Create a gps proxy. */
playerc_gps_t *playerc_gps_create(playerc_client_t *client, int index);

/** @brief Destroy a gps proxy. */
void playerc_gps_destroy(playerc_gps_t *device);

/** @brief Subscribe to the gps device. */
int playerc_gps_subscribe(playerc_gps_t *device, int access);

/** @brief Un-subscribe from the gps device. */
int playerc_gps_unsubscribe(playerc_gps_t *device);


/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_graphics2d graphics2d

The graphics2d proxy provides an interface to the graphics2d

@{
*/

/** @brief Graphics2d device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** current drawing color */
  player_color_t color;

} playerc_graphics2d_t;


/** @brief Create a graphics2d device proxy. */
playerc_graphics2d_t *playerc_graphics2d_create(playerc_client_t *client, int index);

/** @brief Destroy a graphics2d device proxy. */
void playerc_graphics2d_destroy(playerc_graphics2d_t *device);

/** @brief Subscribe to the graphics2d device */
int playerc_graphics2d_subscribe(playerc_graphics2d_t *device, int access);

/** @brief Un-subscribe from the graphics2d device */
int playerc_graphics2d_unsubscribe(playerc_graphics2d_t *device);

/** @brief Set the current drawing color */
int playerc_graphics2d_setcolor(playerc_graphics2d_t *device,
                                player_color_t col );

/** @brief Draw some points */
int playerc_graphics2d_draw_points(playerc_graphics2d_t *device,
           player_point_2d_t pts[],
           int count );

/** @brief Draw a polyline that connects an array of points */
int playerc_graphics2d_draw_polyline(playerc_graphics2d_t *device,
             player_point_2d_t pts[],
             int count );

/** @brief Draw a polygon */
int playerc_graphics2d_draw_polygon(playerc_graphics2d_t *device,
            player_point_2d_t pts[],
            int count,
            int filled,
            player_color_t fill_color );

/** @brief Clear the canvas */
int playerc_graphics2d_clear(playerc_graphics2d_t *device );


/** @} */

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_graphics3d graphics3d

The graphics3d proxy provides an interface to the graphics3d

@{
*/

/** @brief Graphics3d device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** current drawing color */
  player_color_t color;

} playerc_graphics3d_t;


/** @brief Create a graphics3d device proxy. */
playerc_graphics3d_t *playerc_graphics3d_create(playerc_client_t *client, int index);

/** @brief Destroy a graphics3d device proxy. */
void playerc_graphics3d_destroy(playerc_graphics3d_t *device);

/** @brief Subscribe to the graphics3d device */
int playerc_graphics3d_subscribe(playerc_graphics3d_t *device, int access);

/** @brief Un-subscribe from the graphics3d device */
int playerc_graphics3d_unsubscribe(playerc_graphics3d_t *device);

/** @brief Set the current drawing color */
int playerc_graphics3d_setcolor(playerc_graphics3d_t *device,
                                player_color_t col );

/** @brief Draw some points in the given mode */
int playerc_graphics3d_draw(playerc_graphics3d_t *device,
           player_graphics3d_draw_mode_t mode,
           player_point_3d_t pts[],
           int count );

/** @brief Clear the canvas */
int playerc_graphics3d_clear(playerc_graphics3d_t *device );


/** @} */

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_gripper gripper

The gripper proxy provides an interface to the gripper

@{
*/

/** @brief Gripper device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Gripper geometry in the robot cs: pose gives the position and
      orientation, size gives the extent.  These values are initially
      zero, but can be filled in by calling
      playerc_gripper_get_geom(). */
  double pose[3];
  double size[2];

  unsigned char state;
  unsigned char beams;
  int outer_break_beam;
  int inner_break_beam;
  int paddles_open;
  int paddles_closed;
  int paddles_moving;
  int gripper_error;
  int lift_up;
  int lift_down;
  int lift_moving;
  int lift_error;

} playerc_gripper_t;


/** @brief Create a gripper device proxy. */
playerc_gripper_t *playerc_gripper_create(playerc_client_t *client, int index);

/** @brief Destroy a gripper device proxy. */
void playerc_gripper_destroy(playerc_gripper_t *device);

/** @brief Subscribe to the gripper device */
int playerc_gripper_subscribe(playerc_gripper_t *device, int access);

/** @brief Un-subscribe from the gripper device */
int playerc_gripper_unsubscribe(playerc_gripper_t *device);

/** @brief Send the gripper a command */
int playerc_gripper_set_cmd(playerc_gripper_t *device, uint8_t cmd, uint8_t arg);

/** @brief Print a human-readable version of the gripper state. If
    set, the string &lt;prefix&gt; is printed before the state string. */
void playerc_gripper_printout( playerc_gripper_t *device, const char* prefix );


/** @} */
/**************************************************************************/



/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_ir ir

The ir proxy provides an interface to the ir sensors built into robots
such as the RWI B21R.

@{
*/

/** @brief Ir proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  // data
  player_ir_data_t ranges;

  // config
  player_ir_pose_t poses;

} playerc_ir_t;


/** @brief Create a ir proxy. */
playerc_ir_t *playerc_ir_create(playerc_client_t *client, int index);

/** @brief Destroy a ir proxy. */
void playerc_ir_destroy(playerc_ir_t *device);

/** @brief Subscribe to the ir device. */
int playerc_ir_subscribe(playerc_ir_t *device, int access);

/** @brief Un-subscribe from the ir device. */
int playerc_ir_unsubscribe(playerc_ir_t *device);

/** @brief Get the ir geometry.

This writes the result into the proxy rather than returning it to the
caller.

*/
int playerc_ir_get_geom(playerc_ir_t *device);


/** @} */
/***************************************************************************/



/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_laser laser

The laser proxy provides an interface to scanning laser range finders
such as the @ref driver_sicklms200.  Data is returned in the
playerc_laser_t structure.

This proxy wraps the low-level @ref interface_laser interface.

@{
*/

/** @brief Laser proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Laser geometry in the robot cs: pose gives the position and
      orientation, size gives the extent.  These values are filled in by
      playerc_laser_get_geom(). */
  double pose[3];
  double size[2];

  /** Robot pose (m,m,rad), filled in if the scan came with a pose attached */
  double robot_pose[3];

  /** Is intesity data returned. */
  int intensity_on;

  /** Number of points in the scan. */
  int scan_count;

  /** Start bearing of the scan (radians). */
  double scan_start;

  /** Angular resolution in radians. */
  double scan_res;

  /** Range resolution, in m. */
  double range_res;

  /** Maximum range of sensor, in m. */
  double max_range;

  /** Raw range data; range (m). */
  double ranges[PLAYERC_LASER_MAX_SAMPLES];

  /** Scan data; range (m) and bearing (radians). */
  double scan[PLAYERC_LASER_MAX_SAMPLES][2];

  /** Scan data; x, y position (m). */
  player_point_2d_t point[PLAYERC_LASER_MAX_SAMPLES];

  /** Scan reflection intensity values (0-3).  Note that the intensity
      values will only be filled if intensity information is enabled
      (using the set_config function). */
  int intensity[PLAYERC_LASER_MAX_SAMPLES];

  /** ID for this scan */
  int scan_id;

} playerc_laser_t;


/** @brief Create a laser proxy. */
playerc_laser_t *playerc_laser_create(playerc_client_t *client, int index);

/** @brief Destroy a laser proxy. */
void playerc_laser_destroy(playerc_laser_t *device);

/** @brief Subscribe to the laser device. */
int playerc_laser_subscribe(playerc_laser_t *device, int access);

/** @brief Un-subscribe from the laser device. */
int playerc_laser_unsubscribe(playerc_laser_t *device);

/** @brief Configure the laser.

@param device Pointer to proxy object.

@param min_angle, max_angle Start and end angles for the scan
(radians).

@param resolution Angular resolution in radians. Valid values depend on the
underlyling driver.

@param range_res Range resolution in m.  Valid values depend on the
underlyling driver.

@param intensity Intensity flag; set to 1 to enable reflection intensity data.

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_laser_set_config(playerc_laser_t *device,
                             double min_angle, double max_angle,
                             double resolution,
                             double range_res,
                             unsigned char intensity);

/** @brief Get the laser configuration.

@param device Pointer to proxy object.

@param min_angle, max_angle Start and end angles for the scan
(radians).

@param resolution Angular resolution in radians. Valid values depend on the
underlyling driver.

@param range_res Range resolution in m.  Valid values depend on the
underlyling driver.

@param intensity Intensity flag; set to 1 to enable reflection intensity data.

@returns Returns 0 on success, non-zero otherwise.  Use
playerc_error_str() to get a descriptive error message.

*/
int playerc_laser_get_config(playerc_laser_t *device,
                             double *min_angle,
                             double *max_angle,
                             double *resolution,
                             double *range_res,
                             unsigned char *intensity);

/** @brief Get the laser geometry.

This writes the result into the proxy rather than returning it to the
caller.

*/
int playerc_laser_get_geom(playerc_laser_t *device);

/** @brief Print a human-readable summary of the laser state on
    stdout. */
void playerc_laser_printout( playerc_laser_t * device,
        const char* prefix );

/** @} */
/**************************************************************************/


/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_limb limb

The limb proxy provides an interface to limbs using forward/inverse
kinematics, such as the ActivMedia Pioneer Arm. See the Player User Manual for a
complete description of the drivers that support this interface.

@{
*/

/** @brief Limb device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  player_limb_data_t data;
  player_limb_geom_req_t geom;
} playerc_limb_t;

/** @brief Create a limb proxy. */
playerc_limb_t *playerc_limb_create(playerc_client_t *client, int index);

/** @brief Destroy a limb proxy. */
void playerc_limb_destroy(playerc_limb_t *device);

/** @brief Subscribe to the limb device. */
int playerc_limb_subscribe(playerc_limb_t *device, int access);

/** @brief Un-subscribe from the limb device. */
int playerc_limb_unsubscribe(playerc_limb_t *device);

/** @brief Get the limb geometry.  The writes the result into the proxy
    rather than returning it to the caller. */
int playerc_limb_get_geom(playerc_limb_t *device);

/** @brief Command the end effector to move home. */
int playerc_limb_home_cmd(playerc_limb_t *device);

/** @brief Command the end effector to stop immediatly. */
int playerc_limb_stop_cmd(playerc_limb_t *device);

/** @brief Command the end effector to move to a specified pose. */
int playerc_limb_setpose_cmd(playerc_limb_t *device, float pX, float pY, float pZ, float aX, float aY, float aZ, float oX, float oY, float oZ);

/** @brief Command the end effector to move to a specified position
(ignoring approach and orientation vectors). */
int playerc_limb_setposition_cmd(playerc_limb_t *device, float pX, float pY, float pZ);

/** @brief Command the end effector to move along the provided vector from
its current position for the provided distance. */
int playerc_limb_vecmove_cmd(playerc_limb_t *device, float x, float y, float z, float length);

/** @brief Turn the power to the limb on or off. Be careful
when turning power on that the limb is not obstructed from its home
position in case it moves to it (common behaviour). */
int playerc_limb_power(playerc_limb_t *device, uint enable);

/** @brief Turn the brakes of all actuators in the limb that have them on or off. */
int playerc_limb_brakes(playerc_limb_t *device, uint enable);

/** @brief Set the speed of the end effector (m/s) for all subsequent movement commands. */
int playerc_limb_speed_config(playerc_limb_t *device, float speed);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_localize localize

The localize proxy provides an interface to localization drivers.
Generally speaking, these are abstract drivers that attempt to
localize the robot by matching sensor observations (odometry, laser
and/or sonar) against a prior map of the environment (such as an
occupancy grid).  Since the pose may be ambiguous, multiple hypotheses
may returned; each hypothesis specifies one possible pose estimate for
the robot.  See the Player manual for details of the localize
interface, and and drivers that support it (such as the amcl driver).

@{
*/

/** @brief Hypothesis data. */
typedef struct playerc_localize_particle
{
  double pose[3];
  double weight;
} playerc_localize_particle_t;


/** @brief Localization device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Map dimensions (cells). */
  int map_size_x, map_size_y;

  /** Map scale (m/cell). */
  double map_scale;

  /** Next map tile to read. */
  int map_tile_x, map_tile_y;

  /** Map data (empty = -1, unknown = 0, occupied = +1). */
  int8_t *map_cells;

  /** The number of pending (unprocessed) sensor readings. */
  int pending_count;

  /** The timestamp on the last reading processed. */
  double pending_time;

  /** List of possible poses. */
  int hypoth_count;
  player_localize_hypoth_t hypoths[PLAYER_LOCALIZE_MAX_HYPOTHS];

  double mean[3];
  double variance;
  int num_particles;
  playerc_localize_particle_t particles[PLAYER_LOCALIZE_PARTICLES_MAX];

} playerc_localize_t;


/** @brief Create a localize proxy. */
playerc_localize_t *playerc_localize_create(playerc_client_t *client, int index);

/** @brief Destroy a localize proxy. */
void playerc_localize_destroy(playerc_localize_t *device);

/** @brief Subscribe to the localize device. */
int playerc_localize_subscribe(playerc_localize_t *device, int access);

/** @brief Un-subscribe from the localize device. */
int playerc_localize_unsubscribe(playerc_localize_t *device);

/** @brief Set the the robot pose (mean and covariance). */
int playerc_localize_set_pose(playerc_localize_t *device, double pose[3], double cov[3]);

/** @brief Request the particle set.  Caller must supply sufficient storage for
   the result. */
int playerc_localize_get_particles(playerc_localize_t *device);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_log log

The log proxy provides start/stop control of data logging

@{
*/

/** @brief Log proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** What kind of log device is this? Either PLAYER_LOG_TYPE_READ or
      PLAYER_LOG_TYPE_WRITE. Call playerc_log_get_state() to fill it. */
  int type;

  /** Is logging/playback enabled? Call playerc_log_get_state() to
      fill it. */
  int state;
} playerc_log_t;


/** @brief Create a log proxy. */
playerc_log_t *playerc_log_create(playerc_client_t *client, int index);

/** @brief Destroy a log proxy. */
void playerc_log_destroy(playerc_log_t *device);

/** @brief Subscribe to the log device. */
int playerc_log_subscribe(playerc_log_t *device, int access);

/** @brief Un-subscribe from the log device. */
int playerc_log_unsubscribe(playerc_log_t *device);

/** @brief Start/stop logging */
int playerc_log_set_write_state(playerc_log_t* device, int state);

/** @brief Start/stop playback */
int playerc_log_set_read_state(playerc_log_t* device, int state);

/** @brief Rewind playback */
int playerc_log_set_read_rewind(playerc_log_t* device);

/** @brief Get logging/playback state.

The result is written into the proxy.

*/
int playerc_log_get_state(playerc_log_t* device);

/** @brief Change name of log file to write to. */
int playerc_log_set_filename(playerc_log_t* device, const char* fname);


/** @} */


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_map map

The map proxy provides an interface to a map.

@{
*/

/** @brief Map proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Map resolution, m/cell */
  double resolution;

  /** Map size, in cells */
  int width, height;

  /** Map origin, in meters (i.e., the real-world coordinates of cell 0,0)*/
  double origin[2];

  /** Occupancy for each cell (empty = -1, unknown = 0, occupied = +1) */
  char* cells;

  /** Vector-based version of the map (call playerc_map_get_vector() to
   * fill this in). */
  double vminx, vminy, vmaxx, vmaxy;
  int num_segments;
  player_segment_t* segments;
} playerc_map_t;


/** @brief Convert from a cell position to a map index.
 */
#define PLAYERC_MAP_INDEX(dev, i, j) ((dev->width) * (j) + (i))

/** @brief Create a map proxy. */
playerc_map_t *playerc_map_create(playerc_client_t *client, int index);

/** @brief Destroy a map proxy. */
void playerc_map_destroy(playerc_map_t *device);

/** @brief Subscribe to the map device. */
int playerc_map_subscribe(playerc_map_t *device, int access);

/** @brief Un-subscribe from the map device. */
int playerc_map_unsubscribe(playerc_map_t *device);

/** @brief Get the map, which is stored in the proxy. */
int playerc_map_get_map(playerc_map_t* device);

/** @brief Get the vector map, which is stored in the proxy. */
int playerc_map_get_vector(playerc_map_t* device);

/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_opaque opaque

The opaque proxy provides an interface for generic messages to drivers.
See examples/plugins/opaquedriver for an example of using this interface in
combination with a custom plugin.
@{
*/

/** @brief Opaque device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Size of data (bytes) */
  int data_count;

  /** Data  */
  uint8_t data[PLAYER_OPAQUE_MAX_SIZE];
} playerc_opaque_t;

/** @brief Create an opaque device proxy. */
playerc_opaque_t *playerc_opaque_create(playerc_client_t *client, int index);

/** @brief Destroy an opaque device proxy. */
void playerc_opaque_destroy(playerc_opaque_t *device);

/** @brief Subscribe to the opaque device */
int playerc_opaque_subscribe(playerc_opaque_t *device, int access);

/** @brief Un-subscribe from the opaque device */
int playerc_opaque_unsubscribe(playerc_opaque_t *device);

/** @brief Send a generic command */
int playerc_opaque_cmd(playerc_opaque_t *device, player_opaque_data_t *data);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_planner planner

The planner proxy provides an interface to a 2D motion planner.

@{
*/

/** @brief Planner device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Did the planner find a valid path? */
  int path_valid;

  /** Have we arrived at the goal? */
  int path_done;

  /** Current pose (m, m, radians). */
  double px, py, pa;

  /** Goal location (m, m, radians) */
  double gx, gy, ga;

  /** Current waypoint location (m, m, radians) */
  double wx, wy, wa;

  /** Current waypoint index (handy if you already have the list
      of waypoints). May be negative if there's no plan, or if
      the plan is done */
  int curr_waypoint;

  /** Number of waypoints in the plan */
  int waypoint_count;

  /** List of waypoints in the current plan (m,m,radians).  Call
      playerc_planner_get_waypoints() to fill this in. */
  double waypoints[PLAYER_PLANNER_MAX_WAYPOINTS][3];

} playerc_planner_t;

/** @brief Create a planner device proxy. */
playerc_planner_t *playerc_planner_create(playerc_client_t *client, int index);

/** @brief Destroy a planner device proxy. */
void playerc_planner_destroy(playerc_planner_t *device);

/** @brief Subscribe to the planner device */
int playerc_planner_subscribe(playerc_planner_t *device, int access);

/** @brief Un-subscribe from the planner device */
int playerc_planner_unsubscribe(playerc_planner_t *device);

/** @brief Set the goal pose (gx, gy, ga) */
int playerc_planner_set_cmd_pose(playerc_planner_t *device,
                                  double gx, double gy, double ga);

/** @brief Get the list of waypoints.

Writes the result into the proxy rather than returning it to the
caller.

*/
int playerc_planner_get_waypoints(playerc_planner_t *device);

/** @brief Enable / disable the robot's motion

Set state to 1 to enable, 0 to disable.

*/
int playerc_planner_enable(playerc_planner_t *device, int state);

/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_position1d position1d

The position1d proxy provides an interface to 1 DOF actuator such as a
linear or rotational actuator.

@{
*/

/** Position1d device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Robot geometry in robot cs: pose gives the position1d and
      orientation, size gives the extent.  These values are filled in
      by playerc_position1d_get_geom(). */
  double pose[3];
  double size[2];

  /** Odometric pose [m] or [rad]. */
  double pos;

  /** Odometric velocity [m/s] or [rad/s]. */
  double vel;

  /** Stall flag [0, 1]. */
  int stall;

  /** Status bitfield of extra data in the following order:
      - status (unsigned byte)
        - bit 0: limit min
        - bit 1: limit center
        - bit 2: limit max
        - bit 3: over current
        - bit 4: trajectory complete
        - bit 5: is enabled
        - bit 6:
        - bit 7:
    */
  int status;

} playerc_position1d_t;

/** Create a position1d device proxy. */
playerc_position1d_t *playerc_position1d_create(playerc_client_t *client,
                                                int index);

/** Destroy a position1d device proxy. */
void playerc_position1d_destroy(playerc_position1d_t *device);

/** Subscribe to the position1d device */
int playerc_position1d_subscribe(playerc_position1d_t *device, int access);

/** Un-subscribe from the position1d device */
int playerc_position1d_unsubscribe(playerc_position1d_t *device);

/** Enable/disable the motors */
int playerc_position1d_enable(playerc_position1d_t *device, int enable);

/** Get the position1d geometry.  The writes the result into the proxy
    rather than returning it to the caller. */
int playerc_position1d_get_geom(playerc_position1d_t *device);

/** Set the target speed. */
int playerc_position1d_set_cmd_vel(playerc_position1d_t *device,
                                   double vel, int state);

/** @brief Set the target position.
    -@arg pos The position to move to
 */
int playerc_position1d_set_cmd_pos(playerc_position1d_t *device,
                                   double pos, int state);

/** @brief Set the target position with movement velocity
    -@arg pos The position to move to
    -@arg vel The speed at which to move to the position
 */
int playerc_position1d_set_cmd_pos_with_vel(playerc_position1d_t *device,
                                            double pos, double vel, int state);

/** Set the odometry offset */
int playerc_position1d_set_odom(playerc_position1d_t *device,
                                double odom);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_position2d position2d

The position2d proxy provides an interface to a mobile robot base,
such as the ActiveMedia Pioneer series.  The proxy supports both
differential drive robots (which are capable of forward motion and
rotation) and omni-drive robots (which are capable of forward,
sideways and rotational motion).

@{
*/

/** Position2d device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Robot geometry in robot cs: pose gives the position2d and
      orientation, size gives the extent.  These values are filled in
      by playerc_position2d_get_geom(). */
  double pose[3];
  double size[2];

  /** Odometric pose (m, m, rad). */
  double px, py, pa;

  /** Odometric velocity (m/s, m/s, rad/s). */
  double vx, vy, va;

  /** Stall flag [0, 1]. */
  int stall;

} playerc_position2d_t;

/** Create a position2d device proxy. */
playerc_position2d_t *playerc_position2d_create(playerc_client_t *client,
                                                int index);

/** Destroy a position2d device proxy. */
void playerc_position2d_destroy(playerc_position2d_t *device);

/** Subscribe to the position2d device */
int playerc_position2d_subscribe(playerc_position2d_t *device, int access);

/** Un-subscribe from the position2d device */
int playerc_position2d_unsubscribe(playerc_position2d_t *device);

/** Enable/disable the motors */
int playerc_position2d_enable(playerc_position2d_t *device, int enable);

/** Get the position2d geometry.  The writes the result into the proxy
    rather than returning it to the caller. */
int playerc_position2d_get_geom(playerc_position2d_t *device);

/** Set the target speed.  vx : forward speed (m/s).  vy : sideways
    speed (m/s); this field is used by omni-drive robots only.  va :
    rotational speed (rad/s).  All speeds are defined in the robot
    coordinate system. */
int playerc_position2d_set_cmd_vel(playerc_position2d_t *device,
                                   double vx, double vy, double va, int state);

/** Set the target pose with given motion vel */
int playerc_position2d_set_cmd_pose_with_vel(playerc_position2d_t *device,
                                             player_pose_t pos,
                                             player_pose_t vel,
                                             int state);

/** Set the target pose (gx, gy, ga) is the target pose in the
    odometric coordinate system. */
int playerc_position2d_set_cmd_pose(playerc_position2d_t *device,
                                    double gx, double gy, double ga, int state);

/** Set the target cmd for car like position */
int playerc_position2d_set_cmd_car(playerc_position2d_t *device,
                                    double vx, double a);

/** Set the odometry offset */
int playerc_position2d_set_odom(playerc_position2d_t *device,
                                double ox, double oy, double oa);

/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_position3d position3d

The position3d proxy provides an interface to a mobile robot base,
such as the Segway RMP series.  The proxy supports both differential
drive robots (which are capable of forward motion and rotation) and
omni-drive robots (which are capable of forward, sideways and
rotational motion).

@{
*/


/** Position3d device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Robot geometry in robot cs: pose gives the position3d and
      orientation, size gives the extent.  These values are filled in
      by playerc_position3d_get_geom(). */
  double pose[3];
  double size[2];

  /** Device position (m). */
  double pos_x, pos_y, pos_z;

  /** Device orientation (radians). */
  double pos_roll, pos_pitch, pos_yaw;

  /** Linear velocity (m/s). */
  double vel_x, vel_y, vel_z;

  /** Angular velocity (radians/sec). */
  double vel_roll, vel_pitch, vel_yaw;

  /** Stall flag [0, 1]. */
  int stall;

} playerc_position3d_t;


/** Create a position3d device proxy. */
playerc_position3d_t *playerc_position3d_create(playerc_client_t *client,
                                                int index);

/** Destroy a position3d device proxy. */
void playerc_position3d_destroy(playerc_position3d_t *device);

/** Subscribe to the position3d device */
int playerc_position3d_subscribe(playerc_position3d_t *device, int access);

/** Un-subscribe from the position3d device */
int playerc_position3d_unsubscribe(playerc_position3d_t *device);

/** Enable/disable the motors */
int playerc_position3d_enable(playerc_position3d_t *device, int enable);

/** Get the position3d geometry.  The writes the result into the proxy
    rather than returning it to the caller. */
int playerc_position3d_get_geom(playerc_position3d_t *device);

/** Set the target speed.  vx : forward speed (m/s).  vy : sideways
    speed (m/s); vz : vertical speed (m/s). vr : roll speed (rad/s) .
    vp : pitch speed (rad/s) . vt : theta speed (rad/s).
    All speeds are defined in the robot coordinate system. */
int playerc_position3d_set_velocity(playerc_position3d_t *device,
                                    double vx, double vy, double vz,
                                    double vr, double vp, double vt,
                                    int state);

/** For compatibility with old position3d interface */
int playerc_position3d_set_speed(playerc_position3d_t *device,
                                 double vx, double vy, double vz, int state);

/** Set the target pose (gx, gy, ga, gr, gp, gt) is the target pose in the
    odometric coordinate system. */
int playerc_position3d_set_pose(playerc_position3d_t *device,
                                double gx, double gy, double gz,
                                double gr, double gp, double gt);


/** Set the target pose (pos,vel) define desired position and motion speed */
int playerc_position3d_set_pose_with_vel(playerc_position3d_t *device,
                                         player_pose3d_t pos,
                                         player_pose3d_t vel);

/** For compatibility with old position3d interface */
int playerc_position3d_set_cmd_pose(playerc_position3d_t *device,
                                    double gx, double gy, double gz);

/** Set the velocity mode. This is driver dependent. */
int playerc_position3d_set_vel_mode(playerc_position3d_t *device, int mode);

/** Set the odometry offset */
int playerc_position3d_set_odom(playerc_position3d_t *device,
                                double ox, double oy, double oz,
                                double oroll, double opitch, double oyaw);

/** Reset the odometry offset */
int playerc_position3d_reset_odom(playerc_position3d_t *device);

/** @} */
/**************************************************************************/


/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_power power

The power proxy provides an interface through which battery levels can
be monitored.

@{
*/

/** @brief Power device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** status bits. Bitwise-and with PLAYER_POWER_MASK_ values to see
      which fields are being set by the driver. */
  int valid;

  /** Battery charge (Volts). */
  double charge;

  /** Battery charge (percent full). */
  double percent;

  /** energy stored (Joules) */
  double joules;

  /** power currently being used (Watts). Negative numbers indicate
      charging. */
  double watts;

  /** charging flag. TRUE if charging, else FALSE */
  int charging;

} playerc_power_t;


/** @brief Create a power device proxy. */
playerc_power_t *playerc_power_create(playerc_client_t *client, int index);

/** @brief Destroy a power device proxy. */
void playerc_power_destroy(playerc_power_t *device);

/** @brief Subscribe to the power device. */
int playerc_power_subscribe(playerc_power_t *device, int access);

/** @brief Un-subscribe from the power device. */
int playerc_power_unsubscribe(playerc_power_t *device);


/** @} */
/**************************************************************************/



/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_ptz ptz

The ptz proxy provides an interface to pan-tilt units such as the Sony
PTZ camera.

@{
*/

/** @brief PTZ device data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** The current ptz pan and tilt angles.  pan : pan angle (+ve to
      the left, -ve to the right).  tilt : tilt angle (+ve upwrds, -ve
      downwards). */
  double pan, tilt;

  /** The current zoom value (field of view angle). */
  double zoom;

} playerc_ptz_t;


/** @brief Create a ptz proxy. */
playerc_ptz_t *playerc_ptz_create(playerc_client_t *client, int index);

/** @brief Destroy a ptz proxy. */
void playerc_ptz_destroy(playerc_ptz_t *device);

/** @brief Subscribe to the ptz device. */
int playerc_ptz_subscribe(playerc_ptz_t *device, int access);

/** @brief Un-subscribe from the ptz device. */
int playerc_ptz_unsubscribe(playerc_ptz_t *device);

/** @brief Set the pan, tilt and zoom values.

@param device Pointer to proxy object.
@param pan Pan value, in radians; 0 = centered.
@param tilt Tilt value, in radians; 0 = level.
@param zoom Zoom value, in radians (corresponds to camera field of view).

*/
int playerc_ptz_set(playerc_ptz_t *device, double pan, double tilt, double zoom);

/** @brief Set the pan, tilt and zoom values (and speed)

@param device Pointer to proxy object.
@param pan Pan value, in radians; 0 = centered.
@param tilt Tilt value, in radians; 0 = level.
@param zoom Zoom value, in radians (corresponds to camera field of view).
@param panspeed Pan speed, in radians/sec.
@param tiltspeed Tilt speed, in radians/sec.

*/
int playerc_ptz_set_ws(playerc_ptz_t *device, double pan, double tilt, double zoom,
                       double panspeed, double tiltspeed);

/** @brief Change control mode (select velocity or position control)

  @param device Pointer to proxy object.
  @param mode Desired mode (@ref PLAYER_PTZ_VELOCITY_CONTROL or @ref PLAYER_PTZ_POSITION_CONTROL)

  @returns 0 on success, -1 on error, -2 on NACK.
*/
int playerc_ptz_set_control_mode(playerc_ptz_t *device, int mode);

/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_sonar sonar

The sonar proxy provides an interface to the sonar range sensors built
into robots such as the ActiveMedia Pioneer series.

@{
*/

/** @brief Sonar proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** Number of pose values. */
  int pose_count;

  /** Pose of each sonar relative to robot (m, m, radians).  This
      structure is filled by calling playerc_sonar_get_geom(). */
  player_pose_t poses[PLAYERC_SONAR_MAX_SAMPLES];

  /** Number of points in the scan. */
  int scan_count;

  /** Scan data: range (m). */
  double scan[PLAYERC_SONAR_MAX_SAMPLES];

} playerc_sonar_t;


/** @brief Create a sonar proxy. */
playerc_sonar_t *playerc_sonar_create(playerc_client_t *client, int index);

/** @brief Destroy a sonar proxy. */
void playerc_sonar_destroy(playerc_sonar_t *device);

/** @brief Subscribe to the sonar device. */
int playerc_sonar_subscribe(playerc_sonar_t *device, int access);

/** @brief Un-subscribe from the sonar device. */
int playerc_sonar_unsubscribe(playerc_sonar_t *device);

/** @brief Get the sonar geometry.

This writes the result into the proxy
rather than returning it to the caller.
*/
int playerc_sonar_get_geom(playerc_sonar_t *device);

/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_wifi wifi

The wifi proxy is used to query the state of a wireless network.  It
returns information such as the link quality and signal strength of
access points or of other wireless NIC's on an ad-hoc network.

@{
*/

/** @brief Individual link info. */
typedef struct
{
  /** Mac accress. */
  char mac[32];

  /** IP address. */
  char ip[32];

  /** ESSID id */
  char essid[32];

  /** Mode (master, ad-hoc, etc). */
  int mode;

  /** Encrypted? */
  int encrypt;

  /** Frequency (MHz). */
  double freq;

  /** Link properties. */
  int qual, level, noise;

} playerc_wifi_link_t;


/** @brief Wifi device proxy. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** A list containing info for each link. */
  playerc_wifi_link_t links[PLAYERC_WIFI_MAX_LINKS];
  int link_count;

} playerc_wifi_t;


/** @brief Create a wifi proxy. */
playerc_wifi_t *playerc_wifi_create(playerc_client_t *client, int index);

/** @brief Destroy a wifi proxy. */
void playerc_wifi_destroy(playerc_wifi_t *device);

/** @brief Subscribe to the wifi device. */
int playerc_wifi_subscribe(playerc_wifi_t *device, int access);

/** @brief Un-subscribe from the wifi device. */
int playerc_wifi_unsubscribe(playerc_wifi_t *device);

/** @brief Get link state. */
playerc_wifi_link_t *playerc_wifi_get_link(playerc_wifi_t *device, int link);


/** @} */
/**************************************************************************/

/***************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_simulation simulation

The simulation proxy is used to interact with objects in a simulation.

@todo write playerc_proxy_simulation description

@{
*/

/** @brief Simulation device proxy. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

} playerc_simulation_t;


/** @brief Create a new simulation proxy */
playerc_simulation_t *playerc_simulation_create(playerc_client_t *client, int index);

/** @brief Destroy a simulation proxy */
void playerc_simulation_destroy(playerc_simulation_t *device);

/** @brief Subscribe to the simulation device */
int playerc_simulation_subscribe(playerc_simulation_t *device, int access);

/** @brief Un-subscribe from the simulation device */
int playerc_simulation_unsubscribe(playerc_simulation_t *device);

/** @brief Set the 2D pose of a named simulation object */
int playerc_simulation_set_pose2d(playerc_simulation_t *device, char* name,
                                  double gx, double gy, double ga);

/** @brief Get the 2D pose of a named simulation object */
int playerc_simulation_get_pose2d(playerc_simulation_t *device, char* identifier,
          double *x, double *y, double *a);

/** @brief Set the value of a named integer property of a named
    simulation object */
int playerc_simulation_set_property_int(playerc_simulation_t *device,
					char* name,
					char* property,
					int value );
/** @brief Set the value of a named floating point property of a named
    simulation object */
int playerc_simulation_set_property_double(playerc_simulation_t *device,
					   char* name,
					   char* property,
					   double value );
/** @brief Set the value of a named string property of a named
    simulation object */
int playerc_simulation_set_property_string(playerc_simulation_t *device,
					   char* name,
					   char* property,
					   char* value );
/** @} */
/***************************************************************************/


/**************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_speech speech

The speech proxy provides an interface to a speech synthesis system.

@{
*/

/** Speech proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;
} playerc_speech_t;


/** Create a speech proxy. */
playerc_speech_t *playerc_speech_create(playerc_client_t *client, int index);

/** Destroy a speech proxy. */
void playerc_speech_destroy(playerc_speech_t *device);

/** Subscribe to the speech device. */
int playerc_speech_subscribe(playerc_speech_t *device, int access);

/** Un-subscribe from the speech device. */
int playerc_speech_unsubscribe(playerc_speech_t *device);

/** Set the output for the speech device. */
int playerc_speech_say (playerc_speech_t *device, const char *);


/** @} */
/***************************************************************************/

/**************************************************************************/
/** @ingroup playerc_proxies
 * @defgroup playerc_proxy_speech_recognition speech recognition

The speech recognition proxy provides an interface to a speech recognition system.

@{
*/

/** Speech recognition proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  char rawText[PLAYER_SPEECH_RECOGNITION_TEXT_LEN];
  // Just estimating that no more than 20 words will be spoken between updates.
  // Assuming that the longest word is <= 30 characters.
  char words[20][30];
  int wordCount;
} playerc_speechrecognition_t;


/** Create a speech recognition proxy. */
playerc_speechrecognition_t *playerc_speechrecognition_create(playerc_client_t *client, int index);

/** Destroy a speech recognition proxy. */
void playerc_speechrecognition_destroy(playerc_speechrecognition_t *device);

/** Subscribe to the speech recognition device. */
int playerc_speechrecognition_subscribe(playerc_speechrecognition_t *device, int access);

/** Un-subscribe from the speech recognition device */
int playerc_speechrecognition_unsubscribe(playerc_speechrecognition_t *device);
  
/** @} */
/***************************************************************************/

/**************************************************************************/
/** @ingroup playerc_proxies
    @defgroup playerc_proxy_rfid rfid

The rfid proxy provides an interface to a RFID reader.

@{
*/

/** @brief Structure describing a single RFID tag. */
typedef struct
{
    /** Tag type. */
    uint32_t type;
    /** GUID count. */
    uint32_t guid_count;
    /** The Globally Unique IDentifier (GUID) of the tag. */
    uint8_t guid[PLAYERC_RFID_MAX_GUID];
}  playerc_rfidtag_t;

/** @brief RFID proxy data. */
typedef struct
{
  /** Device info; must be at the start of all device structures. */
  playerc_device_t info;

  /** The number of RFID tags found. */
  uint16_t tags_count;

  /** The list of RFID tags. */
  playerc_rfidtag_t tags[PLAYERC_RFID_MAX_TAGS];
} playerc_rfid_t;


/** @brief Create a rfid proxy. */
playerc_rfid_t *playerc_rfid_create(playerc_client_t *client, int index);

/** @brief Destroy a rfid proxy. */
void playerc_rfid_destroy(playerc_rfid_t *device);

/** @brief Subscribe to the rfid device. */
int playerc_rfid_subscribe(playerc_rfid_t *device, int access);

/** @brief Un-subscribe from the rfid device. */
int playerc_rfid_unsubscribe(playerc_rfid_t *device);

/** @} */
/***************************************************************************/

/**************************************************************************/
/** @ingroup playerc_proxies
    @defgroup playerc_proxy_wsn wsn

The wsn proxy provides an interface to a Wireless Sensor Network.

@{
*/

/** Note: the structure describing the WSN node's data packet is declared in
          Player. */

/** @brief WSN proxy data. */
typedef struct
{
    /** Device info; must be at the start of all device structures.         */
    playerc_device_t info;

    /** The type of WSN node.                                               */
    uint32_t node_type;
    /** The ID of the WSN node.                                             */
    uint32_t node_id;
    /** The ID of the WSN node's parent (if existing).                      */
    uint32_t node_parent_id;
    /** The WSN node's data packet.                                         */
    player_wsn_node_data_t data_packet;
} playerc_wsn_t;


/** @brief Create a wsn proxy. */
playerc_wsn_t *playerc_wsn_create(playerc_client_t *client, int index);

/** @brief Destroy a wsn proxy. */
void playerc_wsn_destroy(playerc_wsn_t *device);

/** @brief Subscribe to the wsn device. */
int playerc_wsn_subscribe(playerc_wsn_t *device, int access);

/** @brief Un-subscribe from the wsn device. */
int playerc_wsn_unsubscribe(playerc_wsn_t *device);

/** Set the device state. */
int playerc_wsn_set_devstate(playerc_wsn_t *device, int node_id,
                             int group_id, int devnr, int state);

/** Put the node in sleep mode (0) or wake it up (1). */
int playerc_wsn_power(playerc_wsn_t *device, int node_id, int group_id,
                      int value);

/** Change the data type to RAW or converted engineering units. */
int playerc_wsn_datatype(playerc_wsn_t *device, int value);

/** Change data delivery frequency. */
int playerc_wsn_datafreq(playerc_wsn_t *device, int node_id, int group_id,
                         double frequency);

/** @} */
/***************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
