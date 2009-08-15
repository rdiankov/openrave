/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Player communication packet structures and codes
 * CVS:  $Id: player.h,v 1.92.2.4 2007/03/07 02:07:24 gerkey Exp $
 */


#ifndef PLAYER_H
#define PLAYER_H

/* Include values from the configure script */
#include "playerconfig.h"

/** @ingroup libplayercore
 * @defgroup message_basics Messaging basics
 * Interface-independent message types, sizes, units, address structures, etc.
 */

/** @ingroup message_basics
 * @defgroup message_constants Miscellaneous constants
 * Maximum message lengths, etc.
 * @{ */
/** The largest possible message */
#define PLAYER_MAX_MESSAGE_SIZE 8388608 /*8MB*/
/** Maximum payload in a message */
#define PLAYER_MAX_PAYLOAD_SIZE (PLAYER_MAX_MESSAGE_SIZE - sizeof(player_msghdr_t))
/** Maximum length for a driver name */
#define PLAYER_MAX_DRIVER_STRING_LEN 64
/** The maximum number of devices the server will support. */
#define PLAYER_MAX_DEVICES             256
/** Default maximum length for a message queue */
#define PLAYER_MSGQUEUE_DEFAULT_MAXLEN 32
/** String that is spit back as a banner on connection */
#define PLAYER_IDENT_STRING    "Player v."
/** Length of string that is spit back as a banner on connection */
#define PLAYER_IDENT_STRLEN 32
/** Length of authentication key */
#define PLAYER_KEYLEN       32
/** @} */

/** @ingroup message_basics
 * @defgroup message_types Message types
 * The Player message types
 */
/** @ingroup message_types
 * @{ */

/** A data message.  Such messages are asynchronously published from
devices, and are usually used to reflect some part of the device's state.
*/
#define PLAYER_MSGTYPE_DATA      1

/** A command message.  Such messages are asynchronously published to
devices, and are usually used to change some aspect of the device's state. */
#define PLAYER_MSGTYPE_CMD       2

/** A request message.  Such messages are published synchronously to
devices, usually to get or set some aspect of the device's state that is
not available in data or command messages.  Every request message gets
a response message (either PLAYER_MSGTYPE_RESP_ACK or
PLAYER_MSGTYPE_RESP_NACK). */
#define PLAYER_MSGTYPE_REQ       3

/** A positive response message.  Such messages are published in response
to a PLAYER_MSGTYPE_REQ.  This message indicates that the underlying driver
received, interpreted, and processed the request.  Any requested data is in
the body of this response message. */
#define PLAYER_MSGTYPE_RESP_ACK  4

/** A synch message.  Only used in @ref PLAYER_DATAMODE_PULL mode.
Sent at the end of the set of messages that are sent in response to a
@ref PLAYER_PLAYER_REQ_DATA request. */
#define PLAYER_MSGTYPE_SYNCH     5

/** A negative response message.  Such messages are published in response
to a PLAYER_MSGTYPE_REQ.  This messages indicates that the underlying
driver did not process the message.  Possible causes include: the driver's
message queue was full, the driver failed to interpret the request, or the
the driver does not support the request.   This message will have no data
in the body.*/
#define PLAYER_MSGTYPE_RESP_NACK 6

/** @} */


/** @ingroup message_basics
 * @defgroup message_codes Interface codes
 * An integer code is assigned to each interface.  See @ref interfaces for
 * detailed descriptions of each interface.
 */

/** @ingroup message_codes
 * @{ */

#define PLAYER_NULL_CODE           256 // /dev/null analogue
#define PLAYER_PLAYER_CODE         1   // the server itself
#define PLAYER_POWER_CODE          2   // power subsystem
#define PLAYER_GRIPPER_CODE        3   // gripper
#define PLAYER_POSITION2D_CODE     4   // device that moves about in the plane
#define PLAYER_SONAR_CODE          5   // fixed range-finder
#define PLAYER_LASER_CODE          6   // scanning range-finder
#define PLAYER_BLOBFINDER_CODE     7   // visual blobfinder
#define PLAYER_PTZ_CODE            8   // pan-tilt-zoom unit
#define PLAYER_AUDIO_CODE          9   // audio I/O
#define PLAYER_FIDUCIAL_CODE       10  // fiducial detector
#define PLAYER_SPEECH_CODE         12  // speech I/O
#define PLAYER_GPS_CODE            13  // GPS unit
#define PLAYER_BUMPER_CODE         14  // bumper array
#define PLAYER_TRUTH_CODE          15  // ground-truth (via Stage;
#define PLAYER_DIO_CODE            20  // digital I/O
#define PLAYER_AIO_CODE            21  // analog I/O
#define PLAYER_IR_CODE             22  // IR array
#define PLAYER_WIFI_CODE           23  // wifi card status
#define PLAYER_WAVEFORM_CODE       24  // fetch raw waveforms
#define PLAYER_LOCALIZE_CODE       25  // localization
#define PLAYER_MCOM_CODE           26  // multicoms
#define PLAYER_SOUND_CODE          27  // sound file playback
#define PLAYER_AUDIODSP_CODE       28  // audio dsp I/O
#define PLAYER_AUDIOMIXER_CODE     29  // audio I/O
#define PLAYER_POSITION3D_CODE     30  // 3-D position
#define PLAYER_SIMULATION_CODE     31  // simulators
#define PLAYER_BLINKENLIGHT_CODE   33  // blinking lights
#define PLAYER_NOMAD_CODE          34  // Nomad robot
#define PLAYER_CAMERA_CODE         40  // camera device
#define PLAYER_MAP_CODE            42  // get a map
#define PLAYER_PLANNER_CODE        44  // 2D motion planner
#define PLAYER_LOG_CODE            45  // log read/write control
#define PLAYER_ENERGY_CODE         46  // energy consumption
//#define PLAYER_MOTOR_CODE          47  // motor interface
#define PLAYER_JOYSTICK_CODE       49  // Joytstick
#define PLAYER_SPEECH_RECOGNITION_CODE  50  // speech recognition
#define PLAYER_OPAQUE_CODE         51  // plugin interface
#define PLAYER_POSITION1D_CODE     52  // 1-D position
#define PLAYER_ACTARRAY_CODE       53  // Actuator Array interface
#define PLAYER_LIMB_CODE           54  // Limb interface
#define PLAYER_GRAPHICS2D_CODE     55  // Graphics2D interface
#define PLAYER_RFID_CODE           56  // RFID reader interface
#define PLAYER_WSN_CODE            57  // Wireless Sensor Networks interface
#define PLAYER_GRAPHICS3D_CODE     58  // Graphics3D interface
#define PLAYER_VISIONSERVER_CODE   60  // vision server interface providing vision objects 
/** @} */

/** @ingroup message_basics
 * @defgroup message_strings Interface string names
 * Used in configuration file parsing and console output, each interface is
 * assigned a string name. See @ref interfaces for
 * detailed descriptions of each interface.
 */

/** @ingroup message_strings
 * @{ */

#define PLAYER_ACTARRAY_STRING        "actarray"
#define PLAYER_AIO_STRING             "aio"
#define PLAYER_AUDIO_STRING           "audio"
#define PLAYER_AUDIODSP_STRING        "audiodsp"
#define PLAYER_AUDIOMIXER_STRING      "audiomixer"
#define PLAYER_BLINKENLIGHT_STRING    "blinkenlight"
#define PLAYER_BLOBFINDER_STRING      "blobfinder"
#define PLAYER_BUMPER_STRING          "bumper"
#define PLAYER_CAMERA_STRING          "camera"
#define PLAYER_ENERGY_STRING          "energy"
#define PLAYER_DIO_STRING             "dio"
#define PLAYER_GRIPPER_STRING         "gripper"
#define PLAYER_FIDUCIAL_STRING        "fiducial"
#define PLAYER_GPS_STRING             "gps"
#define PLAYER_IR_STRING              "ir"
#define PLAYER_JOYSTICK_STRING        "joystick"
#define PLAYER_LASER_STRING           "laser"
#define PLAYER_LIMB_STRING            "limb"
#define PLAYER_LOCALIZE_STRING        "localize"
#define PLAYER_LOG_STRING             "log"
#define PLAYER_MAP_STRING             "map"
#define PLAYER_MCOM_STRING            "mcom"
//#define PLAYER_MOTOR_STRING           "motor"
#define PLAYER_NOMAD_STRING           "nomad"
#define PLAYER_NULL_STRING            "null"
#define PLAYER_OPAQUE_STRING          "opaque"
#define PLAYER_PLANNER_STRING         "planner"
#define PLAYER_PLAYER_STRING          "player"
#define PLAYER_POSITION1D_STRING      "position1d"
#define PLAYER_POSITION2D_STRING      "position2d"
#define PLAYER_POSITION3D_STRING      "position3d"
#define PLAYER_POWER_STRING           "power"
#define PLAYER_PTZ_STRING             "ptz"
#define PLAYER_RFID_STRING            "rfid"
#define PLAYER_SIMULATION_STRING      "simulation"
#define PLAYER_SONAR_STRING           "sonar"
#define PLAYER_SOUND_STRING            "sound"
#define PLAYER_SPEECH_STRING          "speech"
#define PLAYER_SPEECH_RECOGNITION_STRING  "speech_recognition"
#define PLAYER_TRUTH_STRING           "truth"
#define PLAYER_WAVEFORM_STRING        "waveform"
#define PLAYER_WIFI_STRING            "wifi"
#define PLAYER_GRAPHICS2D_STRING       "graphics2d"
#define PLAYER_GRAPHICS3D_STRING       "graphics3d"
#define PLAYER_WSN_STRING             "wsn"
#define PLAYER_VISIONSERVER_STRING    "visionserver"

/** @} */

/** @ingroup message_basics
 * @defgroup address_structs Address structures
 * %Device and message address structures.
 * @{ */

/** @brief A device address.

 Devices are identified by 12-byte addresses of this form. Some of the
 fields are transport-dependent in their interpretation. */
typedef struct player_devaddr
{
  /** The "host" on which the device resides.  Transport-dependent. */
  uint32_t host;
  /** The "robot" or device collection in which the device resides.
      Transport-dependent */
  uint32_t robot;
  /** The interface provided by the device; must be one of PLAYER_*_CODE */
  uint16_t interf;
  /** Which device of that interface */
  uint16_t index;
} player_devaddr_t;

/** @brief Generic message header.

 Every message starts with this header.*/
typedef struct player_msghdr
{
  /** Device to which this message pertains */
  player_devaddr_t addr;
  /** Message type; must be one of PLAYER_MSGTYPE_* */
  uint8_t type;
  /** Message subtype; interface specific */
  uint8_t subtype;
  /** Time associated with message contents (seconds since epoch) */
  double timestamp;
  /** For keeping track of associated messages.  Transport-specific. */
  uint32_t seq;
  /** Size in bytes of the payload to follow */
  uint32_t size;
} player_msghdr_t;
/** @} */

/** @ingroup message_basics
 * @defgroup utility_structs General-purpose message structures.
 * These structures often appear inside other structures.
 * @{ */

/** @brief A point in the plane */
typedef struct player_point_2d
{
  /** X [m] */
  float px;
  /** Y [m] */
  float py;
} player_point_2d_t;


/** @brief A point in 3D space */
typedef struct player_point_3d
{
  /** X [m] */
  float px;
  /** Y [m] */
  float py;
  /** Z [m] */
  float pz;
} player_point_3d_t;


/** @brief A pose in the plane */
typedef struct player_pose
{
  /** X [m] */
  float px;
  /** Y [m] */
  float py;
  /** yaw [rad] */
  float pa;
} player_pose_t;

/** @brief A pose in space */
typedef struct player_pose3d
{
  /** X [m] */
  float px;
  /** Y [m] */
  float py;
  /** Z [m] */
  float pz;
  /** roll [rad] */
  float proll;
  /** pitch [rad] */
  float ppitch;
  /** yaw [rad] */
  float pyaw;
} player_pose3d_t;

/** @brief A rectangular bounding box, used to define the size of an object */
typedef struct player_bbox
{
  /** Width [m] */
  float sw;
  /** Length [m] */
  float sl;
} player_bbox_t;

/** @brief A rectangular bounding box, used to define the size of an object */
typedef struct player_bbox3d
{
  /** Width [m] */
  float sw;
  /** Length [m] */
  float sl;
  /** Height [m] */
  float sh;
} player_bbox3d_t;

/** @brief A line segment, used to construct vector-based maps */
typedef struct player_segment
{
  /** Endpoints [m] */
  float x0;
  /** Endpoints [m] */
  float y0;
  /** Endpoints [m] */
  float x1;
  /** Endpoints [m] */
  float y1;
} player_segment_t;

/** @brief A color descriptor */
typedef struct player_color
{
  /** Alpha (transparency) channel */
  uint8_t alpha;
  /** Red color channel */
  uint8_t red;
  /** Green color channel */
  uint8_t green;
  /** Blue color channel */
  uint8_t blue;
} player_color_t;

/** @brief A boolean variable, 0 for false anything else for true */
typedef struct player_bool
{
  /** state */
  uint8_t state;
} player_bool_t;

/** @} */

/**
@ingroup message_basics
@defgroup units Units
Standard units used in Player messages.

In the interest of using MKS units (http://en.wikipedia.org/wiki/Mks) the
internal message structure will use the following unit base types.

Base units
- kilogram [kg]
- second   [s]
- meter    [m]
- ampere   [A]
- radian   [rad]
- watt     [W]
- degree Celcsius [C]
- hertz    [Hz]
- decibel  [dB]
- volts    [V]

@note see float.h and limits.h for the limits of floats and integers on your
system

*/

// /////////////////////////////////////////////////////////////////////////////
//
//             Here starts the alphabetical list of interfaces
//                       (please keep it that way)
//
// /////////////////////////////////////////////////////////////////////////////

/**
@ingroup libplayercore
@defgroup interfaces Interface specifications

All Player communication occurs through <i>interfaces</i>, which specify
the syntax and semantics for a set of messages. See the tutorial @ref
tutorial_devices for a discussion of what an interface is.

Below are the details.  For each interface, the following is given:
- Relevant constants (size limits, etc.)
- %Message subtypes:
  - Data subtypes : codes for each kind of data message defined by the interface
  - Command subtypes : codes for each kind of command message define by
  the interfce.
  - Request/reply subtypes: codes for each kind of request/reply message
  defined by the interface.  Also specified are the interaction semantics,
  such as when to send a null request and when to expect a null response.
  A "null" request or response is a zero-length message.
- Utility structures : structures that appear inside messages.
- %Message structures:
  - Data message structures : data messages that can be published via
  this interface.
  - Command message structures : command messages that can be sent to
  this interface.
  - Request/reply message structures : request messages that can be sent
  to this interface and reply messages that can be expected from it.

It can be the case that a given message can be sent as data or in response
to a request.  A common example is geometry.  For many devices geometry
is fixed and so need only be requested once.  For others geometry may
change dynamically and so the device will publish it periodically.

@todo
  - Normalize subtype names (PLAYER_PTZ_REQ_GEOM vs PLAYER_POSITION2D_REQ_GET_GEOM)
  - Normalize subtype numbers (PLAYER_POSITION2D_SET_ODOM = 6 and PLAYER_POSITION2D_RESET_ODOM = 5, while position3d has PLAYER_POSITION3D_REQ_SET_ODOM = 5 and PLAYER_POSITION_3D_REQ_RESET_ODOM = 6)
*/

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_actarray actarray
@brief An array of actuators

The actuator array interface provides access to an array of actuators.
*/

/**
@ingroup interface_actarray
@{
*/

/** Maximum number of actuators */
#define PLAYER_ACTARRAY_NUM_ACTUATORS     16


/** Maximum trajectory size in floats  */
#define PLAYER_ACTARRAY_NUM_TRAJ_FLOATS     256

/** if timestamps bit is set in trajformat field,
    timestamp information is included as the first value in a point */
#define PLAYER_ACTARRAY_TRAJFMT_FLAG_TIMESTAMPS 0x01

/** If synchronize bit is set, multiple joints must stay in sync when
    interpolating in between trajectory points.  If false, each joint can
    use whatever speed profile it wants, as long as it hits the points
    at the same time as the other joints. */
#define PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCHRONIZE 0x02

/** Enables the synchronization of multiple actarray driver devices
    by waiting for PLAYER_ACTARRAY_START_CMD before the driver can start.
    The driver itself should return 1 when the start request is sent. After
    that, a start command can be sent to start trajectory following immediately.
    This command is necessary because a lot of drivers have retiming steps after
    the initial trajectory is set, which makes trajectory synchronization 
    with different devices impossible */
#define PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCEXECUTION 0x04

/** If this flag is enabled then the robot should hold its position
    and keep returning ACTSTATE_STALLED if it ever gets into a stalled
    state. The condition can be cleared by cancelling the trajectory
    (PLAYER_ACTARRAY_POWER_CANCELCURRENT).  If the flag is not specified,
    the robot will continue applying movement commands until the stall
    condition disappears */
#define PLAYER_ACTARRAY_TRAJFMT_FLAG_HOLD_ON_STALL 0x08

/** The next flag designates whether or not each trajectory point also
    includes a blend radius, for blending between adjacent segments.  The
    blend radius is specified in radians, and should appear after the
    timestamp (if any) but before the joint values */
#define PLAYER_ACTARRAY_TRAJFMT_FLAG_BLEND_RADIUS 0x10

/** Idle state code */
#define PLAYER_ACTARRAY_ACTSTATE_IDLE     1
/** Moving state code */
#define PLAYER_ACTARRAY_ACTSTATE_MOVING   2
/** Braked state code */
#define PLAYER_ACTARRAY_ACTSTATE_BRAKED   3
/** Stalled state code */
#define PLAYER_ACTARRAY_ACTSTATE_STALLED  4
/** Ready to start a trajectory, only used when the PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCEXECUTION
 flag is sent through a trajectory command
*/
#define PLAYER_ACTARRAY_ACTSTATE_READY    5

/** Linear type code */
#define PLAYER_ACTARRAY_TYPE_LINEAR       1
/** Rotary type code */
#define PLAYER_ACTARRAY_TYPE_ROTARY       2

/** Request subtype: power */
#define PLAYER_ACTARRAY_POWER_REQ         1
/** Request subtype: brakes */
#define PLAYER_ACTARRAY_BRAKES_REQ        2
/** Request subtype: get geometry */
#define PLAYER_ACTARRAY_GET_GEOM_REQ      3
/** Request subtype: speed */
#define PLAYER_ACTARRAY_SPEED_REQ         4

/** Command subtype: position */
#define PLAYER_ACTARRAY_POS_CMD           1
/** Command subtype: speed */
#define PLAYER_ACTARRAY_SPEED_CMD         2
/** Command subtype: home */
#define PLAYER_ACTARRAY_HOME_CMD          3
/** Command subtype: trajectory */
#define PLAYER_ACTARRAY_TRAJECTORY_CMD    4
/** Command subtype: start (trajectory) */
#define PLAYER_ACTARRAY_START_CMD         5

/** power request: turn robot off and stop all commands */
#define PLAYER_ACTARRAY_POWER_OFF        0
/** power request: turn robot on */
#define PLAYER_ACTARRAY_POWER_ON      1
/** power request:  cancel the current executing trajectory. If there is no currently
    running trajectory or if the robot is in a ready state, will return a NACK. */
#define PLAYER_ACTARRAY_POWER_CANCELCURRENT 2

/** Data subtype: state */
#define PLAYER_ACTARRAY_DATA_STATE        1

/** @brief Structure containing a single actuator's information */
typedef struct player_actarray_actuator
{
  /** The position of the actuator in m or rad depending on the type. */
  float position;
  /** The speed of the actuator in m/s or rad/s depending on the type. */
  float speed;
  /** The current strain/torque on the motor from outside forces. If possible,
      strain should be filtered to remove any feedforward/gravity/dynamics terms. */
  float current;
  /** The current state of the actuator. */
  uint8_t state;
} player_actarray_actuator_t;

/** @brief Data: state (@ref PLAYER_ACTARRAY_DATA_STATE)

The actuator array data packet. */
typedef struct player_actarray_data
{
  /** The number of actuators in the array. */
  uint32_t actuators_count;
  /** The actuator data. */
  player_actarray_actuator_t actuators[PLAYER_ACTARRAY_NUM_ACTUATORS];
  /** current trajectory robot is executing. If 0, then robot is not executing a trajectory, but can still be moving.  */
  uint32_t trajectory_id;
  /** how far the robot is along the current trajectory */
  float trajectory_time;
} player_actarray_data_t;

/** @brief Actuator geometry */
typedef struct player_actarray_actuatorgeom
{
  /** The type of the actuator - linear or rotary. */
  uint8_t type;
  /** The range of motion of the actuator, in m or rad depending on the type. */
  float min;
  /** The range of motion of the actuator, in m or rad depending on the type. */
  float centre;
  /** The range of motion of the actuator, in m or rad depending on the type. */
  float max;
  /** The range of motion of the actuator, in m or rad depending on the type. */
  float home;
  /** The configured speed setting of the actuator - different from current speed. */
  float config_speed;
  /** max velocity in m or rad per second */
  float max_velocity;
  /** max acceleration in m or rad per second^2 */
  float max_accel;
  /** If the actuator has brakes or not. */
  uint8_t hasbrakes;
} player_actarray_actuatorgeom_t;

/** @brief Request/reply: get geometry

Send a null @ref PLAYER_ACTARRAY_GET_GEOM_REQ request to receive the geometry in
this form. */
typedef struct player_actarray_geom
{
  /** The number of actuators in the array. */
  uint32_t actuators_count;
  /** The geometry information for each actuator in the array. */
  player_actarray_actuatorgeom_t actuators[PLAYER_ACTARRAY_NUM_ACTUATORS];
} player_actarray_geom_t;

/** @brief Command: Joint position control (@ref PLAYER_ACTARRAY_POS_CMD)

Tells a joint to attempt to move to a requested position. */
typedef struct player_actarray_position_cmd
{
  /** The joint to command. */
  int32_t joint;
  /** The position to move to. */
  float position;
} player_actarray_position_cmd_t;

/** @brief Command: Joint speed control (@ref PLAYER_ACTARRAY_SPEED_CMD)

Tells a joint to attempt to move at a requested speed. */
typedef struct player_actarray_speed_cmd
{
  /** The joint to command. */
  int32_t joint;
  /** The speed to move at. */
  float speed;
} player_actarray_speed_cmd_t;

/** @brief Command: Joint home (@ref PLAYER_ACTARRAY_HOME_CMD)

Tells a joint (or the whole array) to go to home position. */
typedef struct player_actarray_home_cmd
{
  /** The joint to command - set to -1 to command all. */
  int32_t joint;
} player_actarray_home_cmd_t;

/** @brief Command: Send a trajectory for the actarray to follow

Sends a variable length trajectory to the actuators. The packet itself
has options to constrain speed and velocity (with trajformat). Since
each packet is fixed length, if the trajectory exceeds the preallocated limit
it will be sent in multiple packets. Each packet has its own unique id.*/
typedef struct player_actarray_trajectory_cmd
{
  int numpoints;

  /** defines several properties of the trajectory through the
      PLAYER_ACTARRAY_TRAJFMT_FLAG_X bit fields */
  int trajformat;

  /** packet id */
  int packetid;

  /** unique id of trajectory. If a trajectory with the same id has already been sent,
      replace it with the current trajectory. If a trajectory with the same id is already being executed,
      then replace it only if time_of_divergence < current time along trajectory. If the executed time,
      is much farther along, then hold the current position by putting the brakes on. */
  uint32_t trajectory_id;

  /** Divergence time is used when this trajectory's id matches the current executing trajectory on the robot. */
  float time_of_divergence;

  /** The total packets of the trajectory is stored in nTotalPackets */
  int nTotalPackets;

  /** trajectory data */
  float data[PLAYER_ACTARRAY_NUM_TRAJ_FLOATS];
    
} player_actarray_trajectory_cmd_t;

/** @brief Command: start a trajectory (@ref PLAYER_ACTARRAY_START_CMD)

Tells the robot to start a trajectory if it is in the ready state. Only valid when
the PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCEXECUTION command is set
*/
typedef struct player_actarray_start_cmd
{
  /** The time in microseconds that the trajectory should start in. If 0, start right away */
    int64_t timestamp;
} player_actarray_start_cmd_t;

/** @brief Request/reply: Power.

Send a @ref PLAYER_ACTARRAY_POWER_REQ request to turn the power to all actuators
in the array on or off and to control the current execution behavior of the robot.
Be careful when turning power on that the array is not obstructed from its
home position in case it moves to it (common behaviour). Null response. */
typedef struct player_actarray_power_config
{
  /** Power setting; one of the PLAYER_ACTARRAY_POWER_X fields */
  uint8_t value;
} player_actarray_power_config_t;

/** @brief Request/reply: Brakes.

Send a @ref PLAYER_ACTARRAY_BRAKES_REQ request to turn brakes on or off */
typedef struct player_actarray_brakes_config
{
  /** Brakes setting; 0 for off, 1 for on */
  uint8_t value;
} player_actarray_brakes_config_t;

/** @brief Request/reply: Speed.

Send a @ref PLAYER_ACTARRAY_SPEED_REQ request to set the speed of a joint for
all subsequent movements. Null response. */
typedef struct player_actarray_speed_config
{
  /** Joint to set speed for. */
  int32_t joint;
  /** Speed setting in mrad/s. */
  float speed;
} player_actarray_speed_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_aio aio
@brief Analog I/O

The @p aio interface provides access to an analog I/O device.
*/

/**
@ingroup interface_aio
@{ */

/** The maximum number of analog I/O samples */
#define PLAYER_AIO_MAX_INPUTS  8
/** The maximum number of analog I/O samples */
#define PLAYER_AIO_MAX_OUTPUTS 8

/** Command subtype: state */
#define PLAYER_AIO_CMD_STATE              1

/** Data subtype: state */
#define PLAYER_AIO_DATA_STATE             1

/** @brief Data: state (@ref PLAYER_AIO_DATA_STATE)

The @p aio interface returns data regarding the current state of the
analog inputs. */
typedef struct player_aio_data
{
  /** number of valid samples */
  uint32_t voltages_count;
  /** the samples [V] */
  float voltages[PLAYER_AIO_MAX_INPUTS];
} player_aio_data_t;

/** @brief Command: state (@ref PLAYER_AIO_CMD_STATE)

The @p aio interface allows for the voltage level on one output to be set */
typedef struct player_aio_cmd
{
  /** Which I/O output to command */
  uint32_t id;
  /** Voltage level to set */
  float voltage;
} player_aio_cmd_t;

/** @} */


// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_audio audio
@brief Audible tone emission / detection (deprecated)

@deprecated Use the @ref interface_audiodsp interface instead

The @p audio interface is used to control sound hardware, if equipped.
*/

/**
@ingroup interface_audio
@{ */

/** Data buffer size */
#define PLAYER_AUDIO_DATA_BUFFER_SIZE    20
/** Command buffer size */
#define PLAYER_AUDIO_COMMAND_BUFFER_SIZE (3*sizeof(short))
/** Number of frequency / amplitude pairs to report */
#define PLAYER_AUDIO_PAIRS               5

/** @brief Data: tones detected

The @p audio interface reads the audio stream from @p /dev/audio (which
is assumed to be associated with a sound card connected to a microphone)
and performs some analysis on it.  @ref PLAYER_AUDIO_PAIRS number
of frequency/amplitude pairs are then returned as data. */
typedef struct player_audio_data
{
  /** Number of frequencies */
  uint32_t frequency_count;
  /** [Hz] */
  float frequency[PLAYER_AUDIO_PAIRS];
  /** Number of amplitudes */
  uint32_t amplitude_count;
  /** [dB] */
  float amplitude[PLAYER_AUDIO_PAIRS];
} player_audio_data_t;

/** @brief Command: tone to emit

The @p audio interface accepts commands to produce fixed-frequency tones
through @p /dev/dsp (which is assumed to be associated with a sound card
to which a speaker is attached). */
typedef struct player_audio_cmd
{
  /** Frequency to play [Hz] */
  float frequency;
  /** Amplitude to play [dB] */
  float amplitude;
  /** Duration to play [s] */
  float duration;
} player_audio_cmd_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_audiodsp audiodsp
@brief Audible tone emission / detection

The @p audiodsp interface is used to control sound hardware, if equipped.
*/

/**
@ingroup interface_audiodsp
@{ */

/** Maximum number of frequencies to report */
#define PLAYER_AUDIODSP_MAX_FREQS 8
/** Maximum length of a BPSK bitstring to emit */
#define PLAYER_AUDIODSP_MAX_BITSTRING_LEN 64

/** Request/reply subtype: set configuration */
#define PLAYER_AUDIODSP_SET_CONFIG 1
/** Request/reply subtype: get configuration */
#define PLAYER_AUDIODSP_GET_CONFIG 2

/** Command subtype: play tone */
#define PLAYER_AUDIODSP_PLAY_TONE  1
/** Command subtype: play chirp */
#define PLAYER_AUDIODSP_PLAY_CHIRP 2
/** Command subtype: replay (last tone, last chirp ?) */
#define PLAYER_AUDIODSP_REPLAY     3

/** Data subtype: detected tones */
#define PLAYER_AUDIODSP_DATA_TONES 1

/** @brief Data: detected tones (@ref PLAYER_AUDIODSP_DATA_TONES)

The @p audiodsp interface reads the audio stream from @p /dev/dsp (which
is assumed to be associated with a sound card connected to a microphone)
and performs some analysis on it.  @ref PLAYER_AUDIODSP_MAX_FREQS number of
frequency/amplitude pairs are then returned as data. */
typedef struct player_audiodsp_data
{
  /** Number of frequencies */
  uint32_t frequency_count;
  /** [Hz] */
  float frequency[PLAYER_AUDIODSP_MAX_FREQS];
  /** Number of amplitudes */
  uint32_t amplitude_count;
  /** [Db] */
  float amplitude[PLAYER_AUDIODSP_MAX_FREQS];

} player_audiodsp_data_t;

/** @brief Command: tone / chirp to play

The @p audiodsp interface accepts commands to produce fixed-frequency
tones or binary phase shift keyed(BPSK) chirps through @p /dev/dsp
(which is assumed to be associated with a sound card to which a speaker is
attached). The command subtype, which should be @ref PLAYER_AUDIODSP_PLAY_TONE,
@ref PLAYER_AUDIODSP_PLAY_CHIRP, or @ref PLAYER_AUDIODSP_REPLAY, determines what
to do.*/
typedef struct player_audiodsp_cmd
{
  /** Frequency to play [Hz] */
  float frequency;
  /** Amplitude to play [dB] */
  float amplitude;
  /** Duration to play [s] */
  float duration;
  /** Length of bit string */
  uint32_t bit_string_count;
  /** BitString to encode in sine wave */
  uint8_t bit_string[PLAYER_AUDIODSP_MAX_BITSTRING_LEN];
} player_audiodsp_cmd_t;

/** @brief Request/reply : Get/set audio properties.

Send a null @ref PLAYER_AUDIODSP_GET_CONFIG request to receive the audiodsp
configuration.  Send a full @ref PLAYER_AUDIODSP_SET_CONFIG request to modify
the configuration (and receive a null response).

The sample format is defined in sys/soundcard.h, and defines the byte
size and endian format for each sample.

The sample rate defines the Hertz at which to sample.

Mono or stereo sampling is defined in the channels parameter where
1==mono and 2==stereo. */
typedef struct player_audiodsp_config
{
  /** Format with which to sample */
  int32_t format;
  /** Sample rate [Hz] */
  float frequency;
  /** Number of channels to use. 1=mono, 2=stereo */
  uint32_t channels;
} player_audiodsp_config_t;
/** @} */

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_audiomixer audiomixer
@brief Sound level control

The @p audiomixer interface is used to control sound levels.
*/

/**
@ingroup interface_audiomixer
@{ */

/** Command subtype: set master level */
#define PLAYER_AUDIOMIXER_SET_MASTER 1
/** Command subtype: set PCM level */
#define PLAYER_AUDIOMIXER_SET_PCM    2
/** Command subtype: set line in level */
#define PLAYER_AUDIOMIXER_SET_LINE   3
/** Command subtype: set microphone level */
#define PLAYER_AUDIOMIXER_SET_MIC    4
/** Command subtype: set input gain level */
#define PLAYER_AUDIOMIXER_SET_IGAIN  5
/** Command subtype: set output gain level */
#define PLAYER_AUDIOMIXER_SET_OGAIN  6

/** Request/reply subtype: get levels */
#define PLAYER_AUDIOMIXER_GET_LEVELS 1

/** @brief Command: set level

The @p audiomixer interface accepts commands to set the left and right
volume levels of various channels. The channel is determined by the
subtype of the command: @ref PLAYER_AUDIOMIXER_SET_MASTER for the master volume,
@ref PLAYER_AUDIOMIXER_SET_PCM for the PCM volume, @ref PLAYER_AUDIOMIXER_SET_LINE for
the line in volume, @ref PLAYER_AUDIOMIXER_SET_MIC for the microphone volume,
@ref PLAYER_AUDIOMIXER_SET_IGAIN for the input gain, and @ref PLAYER_AUDIOMIXER_SET_OGAIN
for the output gain.
*/
typedef struct player_audiomixer_cmd
{
  /** Left level */
  uint32_t left;
  /** Right level */
  uint32_t right;

} player_audiomixer_cmd_t;

/** @brief Request/reply: Get levels

Send a null @ref PLAYER_AUDIOMIXER_GET_LEVELS request to receive the
current state of the mixer levels.
*/
typedef struct player_audiomixer_config
{
  /** Levels */
  uint32_t master_left;
  /** Levels */
  uint32_t master_right;
  /** Levels */
  uint32_t pcm_left;
  /** Levels */
  uint32_t pcm_right;
  /** Levels */
  uint32_t line_left;
  /** Levels */
  uint32_t line_right;
  /** Levels */
  uint32_t mic_left;
  /** Levels */
  uint32_t mic_right;
  /** Levels */
  uint32_t i_gain;
  /** Levels */
  uint32_t o_gain;
} player_audiomixer_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_blinkenlight blinkenlight
@brief A blinking light

The @p blinkenlight interface is used to switch on and off a flashing
indicator light, and to set it's flash period.

This interface accepts no configuration requests.
*/
/** @ingroup interface_blinkenlight
 * @{ */

/** Data subtype: state */
#define PLAYER_BLINKENLIGHT_DATA_STATE 1

/** Command subtype: state */
#define PLAYER_BLINKENLIGHT_CMD_STATE      1
/** Command subtype: light */
#define PLAYER_BLINKENLIGHT_CMD_POWER      2
/** Command subtype: color */
#define PLAYER_BLINKENLIGHT_CMD_COLOR      3
/** Command subtype: period */
#define PLAYER_BLINKENLIGHT_CMD_PERIOD     4
/** Command subtype: dutycycle */
#define PLAYER_BLINKENLIGHT_CMD_DUTYCYCLE  5

/** @brief Data: state (@ref PLAYER_BLINKENLIGHT_DATA_STATE)
The @p blinkenlight data provides the current state of the indicator
light.*/
typedef struct player_blinkenlight_data
{
  /** FALSE: disabled, TRUE: enabled */
  uint8_t enable;
  /** flash period (duration of one whole on-off cycle) [s]. */
  float period;
  /** flash duty cycle (ratio of time-on to time-off in one cycle). */
  float dutycycle;
  /** the color of the light */
  player_color_t color;
} player_blinkenlight_data_t;

/** @brief Command: state (@ref PLAYER_BLINKENLIGHT_CMD_STATE)
This @p blinkenlight command sets the complete current state of the
indicator light. */
typedef struct player_blinkenlight_cmd
{
  /** FALSE: disabled, TRUE: enabled */
  uint8_t enable;
  /** flash period (duration of one whole on-off cycle) [s]. */
  float period;
  /** flash duty cycle (ratio of time-on to time-off in one cycle). */
  float dutycycle;
  /** the color of the light */
  player_color_t color;
} player_blinkenlight_cmd_t;

/** @brief Command: power (@ref PLAYER_BLINKENLIGHT_CMD_POWER)
This @p blinkenlight command turns the light on or off.
*/
typedef struct player_blinkenlight_cmd_power
{
  /** FALSE: off, TRUE: on */
  uint8_t enable;
} player_blinkenlight_cmd_power_t;

/** @brief Command: color (@ref PLAYER_BLINKENLIGHT_CMD_COLOR)
This @p blinkenlight command sets the color of the light.
*/
typedef struct player_blinkenlight_cmd_color
{
  /** the color of the light */
  player_color_t color;
} player_blinkenlight_cmd_color_t;

/** @brief Command: period (@ref PLAYER_BLINKENLIGHT_CMD_PERIOD)
This @p blinkenlight command sets the duration of one on/off blink cycle in seconds.
*/
typedef struct player_blinkenlight_cmd_period
{
  /** flash period (duration of one whole on-off cycle) [s]. */
  float period;
} player_blinkenlight_cmd_period_t;

/** @brief Command: dutycycle (@ref
PLAYER_BLINKENLIGHT_CMD_DUTYCYCLE) This @p blinkenlight command sets
the ratio of light-on to light-off time in one on/off blink cycle.
*/
typedef struct player_blinkenlight_cmd_dutycycle
{
  /** flash duty cycle (ratio of time-on to time-off in one cycle). */
  float dutycycle;
} player_blinkenlight_cmd_dutycycle_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_blobfinder blobfinder
@brief A visual blob-detection system

The blobfinder interface provides access to devices that detect blobs
in images.
*/


/** @ingroup interface_blobfinder
 * @{ */

/** The maximum number of blobs in total. */
#define PLAYER_BLOBFINDER_MAX_BLOBS 256

/** Data subtype: detected blobs */
#define PLAYER_BLOBFINDER_DATA_BLOBS 1

/** Request/reply subtype: set tracking color */
#define PLAYER_BLOBFINDER_REQ_SET_COLOR         1
/** Request/reply subtype: set imager parameters */
#define PLAYER_BLOBFINDER_REQ_SET_IMAGER_PARAMS 2

//#define PLAYER_BLOBFINDER_DATA_BLOBS 1


/** @brief Structure describing a single blob. */
typedef struct player_blobfinder_blob
{
  /** Blob id. */
  uint32_t id;
  /** A descriptive color for the blob (useful for gui's).  The color
      is stored as packed 32-bit RGB, i.e., 0x00RRGGBB. */
  uint32_t color;
  /** The blob area [pixels]. */
  uint32_t area;
  /** The blob centroid [pixels]. */
  uint32_t x;
  /** The blob centroid [pixels]. */
  uint32_t y;
  /** Bounding box for the blob [pixels]. */
  uint32_t left;
  /** Bounding box for the blob [pixels]. */
  uint32_t right;
  /** Bounding box for the blob [pixels]. */
  uint32_t top;
  /** Bounding box for the blob [pixels]. */
  uint32_t bottom;
  /** Range to the blob center [meters] */
  float range;
} player_blobfinder_blob_t;

/** @brief Data: detected blobs (@ref PLAYER_BLOBFINDER_DATA_BLOBS)

The list of detected blobs, returned as data by @p blobfinder devices. */
typedef struct player_blobfinder_data
{
  /** The image dimensions. [pixels] */
  uint32_t width;
  /** The image dimensions. [pixels] */
  uint32_t height;
  /** The number of blobs */
  uint32_t blobs_count;
  /** The list of blobs */
  player_blobfinder_blob_t blobs[PLAYER_BLOBFINDER_MAX_BLOBS];
} player_blobfinder_data_t;


/** @brief Request/reply: Set tracking color.

For some sensors (ie CMUcam), simple blob tracking tracks only one color.
To set the tracking color, send a @ref PLAYER_BLOBFINDER_REQ_SET_COLOR request
with the format below, including the RGB color ranges (max and min).
Values of -1 will cause the track color to be automatically set to the
current window color.  This is useful for setting the track color by
holding the tracking object in front of the lens.  Null response.
*/
typedef struct player_blobfinder_color_config
{
  /** RGB minimum and max values (0-255) **/
  uint32_t rmin;
  /** RGB minimum and max values (0-255) **/
  uint32_t rmax;
  /** RGB minimum and max values (0-255) **/
  uint32_t gmin;
  /** RGB minimum and max values (0-255) **/
  uint32_t gmax;
  /** RGB minimum and max values (0-255) **/
  uint32_t bmin;
  /** RGB minimum and max values (0-255) **/
  uint32_t bmax;
} player_blobfinder_color_config_t;


/** @brief Configuration request: Set imager params.

Imaging sensors that do blob tracking generally have some sorts of
image quality parameters that you can tweak.  The following ones
are implemented here:
   - brightness  (0-255)
   - contrast    (0-255)
   - auto gain   (0=off, 1=on)
   - color mode  (0=RGB/AutoWhiteBalance Off,  1=RGB/AutoWhiteBalance On,
                2=YCrCB/AWB Off, 3=YCrCb/AWB On)
To set the params, send a @ref PLAYER_BLOBFINDER_REQ_SET_IMAGER_PARAMS request
with the format below.  Any values set to -1 will be left unchanged.
Null response.
*/
typedef struct player_blobfinder_imager_config
{
  /** Brightness: (0-255)  -1=no change. */
  int32_t brightness;
  /** Contrast: (0-255)  -1=no change. */
  int32_t contrast;
  /** Color Mode
      ( 0=RGB/AutoWhiteBalance Off,  1=RGB/AutoWhiteBalance On,
      2=YCrCB/AWB Off, 3=YCrCb/AWB On)  -1=no change.
  */
  int32_t  colormode;
  /** AutoGain:   0=off, 1=on.  -1=no change. */
  int32_t  autogain;
} player_blobfinder_imager_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/**
@ingroup interfaces
@defgroup interface_bumper bumper
@brief An array of bumpers

The @p bumper interface returns data from a bumper array.  This interface
accepts no commands.
*/

/** @ingroup interface_bumper
 * @{ */

/** Maximum number of bumper samples */
#define PLAYER_BUMPER_MAX_SAMPLES 32

/** Request/reply subtype: get geometry */
#define PLAYER_BUMPER_GET_GEOM    1

/** Data subtype: state */
#define PLAYER_BUMPER_DATA_STATE  1
/** Data subtype: geometry */
#define PLAYER_BUMPER_DATA_GEOM  2

/** @brief Data: state (@ref PLAYER_BUMPER_DATA_GEOM)

The @p bumper interface gives current bumper state*/
typedef struct player_bumper_data
{
  /** the number of valid bumper readings */
  uint32_t bumpers_count;
  /** array of bumper values */
  uint8_t bumpers[PLAYER_BUMPER_MAX_SAMPLES];
} player_bumper_data_t;

/** @brief The geometry of a single bumper */
typedef struct player_bumper_define
{
  /** the local pose of a single bumper */
  player_pose_t pose;
  /** length of the sensor [m] */
  float length;
  /** radius of curvature [m] - zero for straight lines */
  float radius;
} player_bumper_define_t;

/** @brief Data AND Request/reply: bumper geometry

To query the geometry of a bumper array, send a null
@ref PLAYER_BUMPER_GET_GEOM request.  The response will be in this form.  This
message may also be sent as data with the subtype @ref PLAYER_BUMPER_DATA_GEOM
(e.g., from a robot whose bumper can move with respect to its body)
*/
typedef struct player_bumper_geom
{
  /** The number of valid bumper definitions. */
  uint32_t bumper_def_count;
  /** geometry of each bumper */
  player_bumper_define_t bumper_def[PLAYER_BUMPER_MAX_SAMPLES];
} player_bumper_geom_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 @defgroup interface_camera camera
 @brief Camera imagery

The camera interface is used to see what the camera sees.  It is
intended primarily for server-side (i.e., driver-to-driver) data
transfers, rather than server-to-client transfers.  Image data can be
in may formats (see below), but is always packed (i.e., pixel rows are
byte-aligned).
*/

/** @ingroup interface_camera
 * @{ */

/** Data subtype: state */
#define PLAYER_CAMERA_DATA_STATE             1

/** Maximum image width, in pixels */
#define PLAYER_CAMERA_IMAGE_WIDTH  1920
/** Maximum image height, in pixels */
#define PLAYER_CAMERA_IMAGE_HEIGHT 1080
/** Maximum image size, in pixels */
#define PLAYER_CAMERA_IMAGE_SIZE  (PLAYER_CAMERA_IMAGE_WIDTH * PLAYER_CAMERA_IMAGE_HEIGHT * 4)

/** Image format : 8-bit monochrome. */
#define PLAYER_CAMERA_FORMAT_MONO8  1
/** Image format : 16-bit monochrome (network byte order). */
#define PLAYER_CAMERA_FORMAT_MONO16 2
/** Image format : 16-bit color (5 bits R, 6 bits G, 5 bits B). */
#define PLAYER_CAMERA_FORMAT_RGB565 4
/** Image format : 24-bit color (8 bits R, 8 bits G, 8 bits B). */
#define PLAYER_CAMERA_FORMAT_RGB888 5

/** Compression method: raw */
#define PLAYER_CAMERA_COMPRESS_RAW  0
/** Compression method: jpeg */
#define PLAYER_CAMERA_COMPRESS_JPEG 1

/** @brief Data: state (@ref PLAYER_CAMERA_DATA_STATE) */
typedef struct player_camera_data
{
  /** Image dimensions [pixels]. */
  uint32_t width;
  /** Image dimensions [pixels]. */
  uint32_t height;
  /** Image bits-per-pixel (8, 16, 24, 32). */
  uint32_t bpp;
  /** Image format (must be compatible with depth). */
  uint32_t format;
  /** Some images (such as disparity maps) use scaled pixel values;
      for these images, fdiv specifies the scale divisor (i.e., divide
      the integer pixel value by fdiv to recover the real pixel value). */
  uint32_t fdiv;
  /** Image compression; @ref PLAYER_CAMERA_COMPRESS_RAW indicates no
      compression. */
  uint32_t compression;
  /** Size of image data as stored in image buffer (bytes) */
  uint32_t image_count;
  /** Compressed image data (byte-aligned, row major order).
      Multi-byte image formats (such as MONO16) must be converted
      to network byte ordering. */
  uint8_t image[PLAYER_CAMERA_IMAGE_SIZE];
} player_camera_data_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_dio dio
 * @brief Digital I/O

The @p dio interface provides access to a digital I/O device.
*/

/** @ingroup interface_dio
 * @{ */

/** Data subtype: input values */
#define PLAYER_DIO_DATA_VALUES 1

/** Command subtype: output values */
#define PLAYER_DIO_CMD_VALUES 1

/** @brief Data: input values (@ref PLAYER_DIO_DATA_VALUES)

The @p dio interface returns data regarding the current state of the
digital inputs. */
typedef struct player_dio_data
{
  /** number of samples */
  uint32_t count;
  /** bitfield of samples */
  uint32_t digin;
} player_dio_data_t;


/** @brief Command: output values (@ref PLAYER_DIO_CMD_VALUES)

The @p dio interface accepts 4-byte commands which consist of the ouput
bitfield */
typedef struct player_dio_cmd
{
  /** the command */
  uint32_t count;
  /** output bitfield */
  uint32_t digout;
} player_dio_cmd_t;

/** @} */

/* REMOVE ENERGY DEVICE - USE POWER INSTEAD - RTV 2005.12.04 */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_energy energy
 * @brief Energy storage / consumption

DEPRECATED: THE FUNCTIONALITY OF THE ENERGY DEVICE HAS BEEN FOLDED INTO THE
POWER DEVICE. YOU SHOULD CONSIDER USING THE POWER DEVICE INSTEAD. NO
NEW DEVELOPMENT SHOULD BE DONE ON THE ENERGY DEVICE AND IT WILL BE
REMOVED FROM HERE VERY SOON.

The @p energy interface provides data about energy storage, consumption
and charging.  This interface accepts no commands.
*/

/** @ingroup interface_energy
 * @{ */

/** Data subtype: state */
#define PLAYER_ENERGY_DATA_STATE 1

/** Request subtype: set charging policy */
#define PLAYER_ENERGY_SET_CHARGING_POLICY_REQ 1

/** @brief Data: state (@ref PLAYER_ENERGY_DATA_STATE)

The @p energy interface reports he amount of energy stored, current rate
of energy consumption or aquisition, and whether or not the device is
connected to a charger. */
typedef struct player_energy_data
{
  /** energy stored [J]. */
  float joules;
  /** estimated current power consumption (negative values) or
      aquisition (positive values) [W]. */
  float watts;
  /** charge exchange status: if 1, the device is currently receiving
      charge from another energy device. If -1 the device is currently
      providing charge to another energy device. If 0, the device is
      not exchanging charge with an another device. */
  int32_t charging;
} player_energy_data_t;

/** @brief Request/reply: set charging policy
 *
 * Send a @ref PLAYER_ENERGY_SET_CHARGING_POLICY_REQ request to change the
 * charging policy. */
typedef struct player_energy_chargepolicy_config
{
  /** uint8_tean controlling recharging. If FALSE, recharging is
      disabled. Defaults to TRUE */
  uint8_t enable_input;
  /** uint8_tean controlling whether others can recharge from this
      device. If FALSE, charging others is disabled. Defaults to TRUE.*/
  uint8_t enable_output;
} player_energy_chargepolicy_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_fiducial fiducial
 * @brief Fiducial (marker) detection

The fiducial interface provides access to devices that detect coded
fiducials (markers) placed in the environment.  It can also be used
for devices the detect natural landmarks.
*/

/** @ingroup interface_fiducial
 * @{ */

/** The maximum number of fiducials that can be detected at one time. */
#define PLAYER_FIDUCIAL_MAX_SAMPLES 32

/** Data subtype: detected fiducials */
#define PLAYER_FIDUCIAL_DATA_SCAN 1

/** Request/reply subtype: get geometry */
#define PLAYER_FIDUCIAL_REQ_GET_GEOM     1
/** Request/reply subtype: get field of view */
#define PLAYER_FIDUCIAL_REQ_GET_FOV      2
/** Request/reply subtype: set field of view */
#define PLAYER_FIDUCIAL_REQ_SET_FOV      3
/** Request/reply subtype: get ID */
#define PLAYER_FIDUCIAL_REQ_GET_ID       7
/** Request/reply subtype: set ID */
#define PLAYER_FIDUCIAL_REQ_SET_ID       8

/** @brief Info on a single detected fiducial

The fiducial data packet contains a list of these.
*/
typedef struct player_fiducial_item
{
  /** The fiducial id.  Fiducials that cannot be identified get id
      -1. */
  int32_t id;
  /** Fiducial pose relative to the detector. */
  player_pose3d_t pose;
  /** Uncertainty in the measured pose . */
  player_pose3d_t upose;
} player_fiducial_item_t;


/** @brief Data: detected fiducials (@ref PLAYER_FIDUCIAL_DATA_SCAN)

The fiducial data packet (all fiducials). */
typedef struct player_fiducial_data
{
  /** The number of detected fiducials */
  uint32_t fiducials_count;
  /** List of detected fiducials */
  player_fiducial_item_t fiducials[PLAYER_FIDUCIAL_MAX_SAMPLES];

} player_fiducial_data_t;

/** @brief Request/reply: Get geometry.

The geometry (pose and size) of the fiducial device can be queried by
sending a null @ref PLAYER_FIDUCIAL_REQ_GET_GEOM request.
*/
typedef struct player_fiducial_geom
{
  /** Pose of the detector in the robot cs */
  player_pose_t pose;
  /** Size of the detector */
  player_bbox_t size;
  /** Dimensions of the fiducials in units of (m, m). */
  player_bbox_t fiducial_size;
} player_fiducial_geom_t;

/** @brief Request/reply: Get/set sensor field of view.

The field of view of the fiducial device can be set using the
@ref PLAYER_FIDUCIAL_REQ_SET_FOV request (response will be null), and queried
using a null @ref PLAYER_FIDUCIAL_REQ_GET_FOV request.
*/
typedef struct player_fiducial_fov
{
  /** The minimum range of the sensor [m] */
  float min_range;
  /** The maximum range of the sensor [m] */
  float max_range;
  /** The receptive angle of the sensor [rad]. */
  float view_angle;
} player_fiducial_fov_t;

/** @brief Request/reply: Get/set fiducial ID.

Some fiducial finder devices display their own fiducial. Send a null
@ref PLAYER_FIDUCIAL_REQ_GET_ID request to get the identifier displayed by the
fiducial.

Some devices can dynamically change the identifier they display. Send
a @ref PLAYER_FIDUCIAL_REQ_SET_ID request to set the currently displayed
value. Make the request with the player_fiducial_id_t structure. The
device replies with the same structure with the id field set to the value
it actually used. You should check this value, as the device may not be
able to display the value you requested.

Currently supported by the stg_fiducial driver.
*/
typedef struct player_fiducial_id
{
  /** The value displayed */
  uint32_t id;
} player_fiducial_id_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_visionserver visionserver
 * @brief Visionserver object tracking

The visionserver interface provides access to a system that tracks objects. The current objects
can be queried by the visionobject interface.
*/

/** @ingroup interface_visionserver
 * @{ */

/** Request/reply subtype: get geometry */
#define PLAYER_VISIONSERVER_REQ_GET_GEOM           1
#define PLAYER_VISIONSERVER_REQ_SET_GEOM           2

/** Data subtype: state */
#define PLAYER_VISIONSERVER_DATA_STATE        1

#define PLAYER_VISIONSERVER_MAX_LENGTH 32
#define PLAYER_VISIONSERVER_MAX_OBJECTS 16
#define PLAYER_VISIONSERVER_MODEL_NAME 128

/** @brief Structure containing pose information of the visionobject */
typedef struct player_visionobject_pose
{
  /** Translation  */
  double translation[3];
  /** Rotation of the vision object in quaterion format (w,x,y,z) */
  double rotation[4];
} player_visionobject_pose_t;

/** @brief Request/reply: Get visionobject ID.

Some visionobject finder devices display their own visionobject. Send a null
@ref PLAYER_VISIONOBJECT_REQ_GET_ID request to get the identifier displayed by the
visionobject.
*/
typedef struct player_visionobject_id
{
  /** The value displayed */
  uint32_t id;
  char modelname[PLAYER_VISIONSERVER_MODEL_NAME];
} player_visionobject_id_t;

/** @brief Structure containing information about each object the server is tracking.

Use the playerid to query a visionobject interface. */
typedef struct player_visionserver_object
{
  int playerid;

  /** Id specific to the visionobject */
  player_visionobject_id_t id;
  player_visionobject_pose_t pose;
} player_visionserver_object_t;

/** @brief Data: state (@ref PLAYER_VISIONSERVER_DATA_STATE)

The visionserver data packet. */
typedef struct player_visionserver_data
{
  /** status is 0 if objects are the same since last frame, 1 otherwise */
  int status;
    
  /** the number of objects currently tracking */
  int numobjects;
  player_visionserver_object_t objectdata[PLAYER_VISIONSERVER_MAX_OBJECTS];
} player_visionserver_data_t;


/** @brief Request/reply: Get/Set algorithm.

The visionserver algorithm in use can be queried by
sending a null @ref PLAYER_VISIONSERVER_REQ_GET_GEOM request.

To set the algorithm, send a @ref PLAYER_VISIONSERVER_REQ_SET_GEOM with
the algorithm name in the type field.
*/
typedef struct player_visionserver_geom
{
  /** Type of  vision server (can explain what objects are tracked, etc) */
  char type[PLAYER_VISIONSERVER_MAX_LENGTH];
} player_visionserver_geom_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_gps gps
 * @brief Global positioning system

The @p gps interface provides access to an absolute position system,
such as GPS.
*/

/** @ingroup interface_gps
 * @{ */

/** Data subtype: state */
#define PLAYER_GPS_DATA_STATE 1

/** @brief Data: state (@ref PLAYER_GPS_DATA_STATE)

The @p gps interface gives current global position and heading information.

@todo: Review the units used here
*/
typedef struct player_gps_data
{
  /** GPS (UTC) time, in seconds and microseconds since the epoch. */
  uint32_t time_sec;
  /** GPS (UTC) time, in seconds and microseconds since the epoch. */
  uint32_t time_usec;
  /** Latitude in degrees / 1e7 (units are scaled such that the
      effective resolution is roughly 1cm).  Positive is north of
      equator, negative is south of equator. */
  int32_t latitude;
  /** Longitude in degrees / 1e7 (units are scaled such that the
      effective resolution is roughly 1cm).  Positive is east of prime
      meridian, negative is west of prime meridian. */
  int32_t longitude;
  /** Altitude, in millimeters.  Positive is above reference (e.g.,
      sea-level), and negative is below. */
  int32_t altitude;
  /** UTM WGS84 coordinates, easting [m] */
  double utm_e;
  /** UTM WGS84 coordinates, northing [m] */
  double utm_n;
  /** Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix */
  uint32_t quality;
  /** Number of satellites in view. */
  uint32_t num_sats;
  /** Horizontal dilution of position (HDOP), times 10 */
  uint32_t hdop;
  /** Vertical dilution of position (VDOP), times 10 */
  uint32_t vdop;
  /** Horizonal error [m] */
  double err_horz;
  /** Vertical error [m] */
  double err_vert;
} player_gps_data_t;

/** @} */

////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_graphics2d graphics2d
 * @brief Two-dimensional graphics interface

The @p graphics2d interface provides an interface to graphics
devices. Drivers can implement this interface to provide clients and
other drivers with graphics output. For example, Stage models present
this interface to allow clients to draw in the Stage window.
*/

/** @ingroup interface_graphics2d
 * @{ */

/** The maximum number of points that can be described in a packet. */
#define PLAYER_GRAPHICS2D_MAX_POINTS 64

/** Command subtype: clear the drawing area (send an empty message) */
#define PLAYER_GRAPHICS2D_CMD_CLEAR 1
/** Command subtype: draw points */
#define PLAYER_GRAPHICS2D_CMD_POINTS 2
/** Command subtype: draw a polyline */
#define PLAYER_GRAPHICS2D_CMD_POLYLINE 3
/** Command subtype: draw a polygon */
#define PLAYER_GRAPHICS2D_CMD_POLYGON 4

/** @brief Data: This interface produces no data. */

/** @brief Requests: This interface accepts no requests. */

/** @brief Command: Draw points (@ref PLAYER_GRAPHICS2D_CMD_POINTS)
Draw some points.
*/
typedef struct player_graphics2d_cmd_points
{
  /** Number of points in this packet. */
  uint16_t count;
  /** Array of points. */
  player_point_2d_t points[PLAYER_GRAPHICS2D_MAX_POINTS];
  /** Color in which the points should be drawn. */
  player_color_t color;
} player_graphics2d_cmd_points_t;

/** @brief Command: Draw polyline (@ref PLAYER_GRAPHICS2D_CMD_POLYLINE)
Draw a series of straight line segments between a set of points.
*/
typedef struct player_graphics2d_cmd_polyline
{
  /** Number of points in this packet. */
  uint16_t count;
  /** Array of points to be joined by lines. */
  player_point_2d_t points[PLAYER_GRAPHICS2D_MAX_POINTS];
  /** Color in which the line should be drawn. */
  player_color_t color;
} player_graphics2d_cmd_polyline_t;

/** @brief Command: Draw polygon (@ref PLAYER_GRAPHICS2D_CMD_POLYGON)
Draw a polygon.
*/
typedef struct player_graphics2d_cmd_polygon
{
  /** Number of points in this packet. */
  uint16_t count;
  /** array of points defining the polygon. */
  player_point_2d_t points[PLAYER_GRAPHICS2D_MAX_POINTS];
  /** Color in which the outline should be drawn. */
  player_color_t color;
  /** Color in which the polygon should be filled. */
  player_color_t fill_color;
  /** If non-zero, the polygon should be drawn filled, else empty. */
  uint8_t filled;
} player_graphics2d_cmd_polygon_t;

/** @} */

////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_graphics3d graphics3d
 * @brief Three-dimensional graphics interface

The @p graphics3d interface provides an interface to graphics
devices. Drivers can implement this interface to provide clients and
other drivers with graphics output. 

The interface uses an openGL style of command where a type is specified along 
with a series of verticies. The interpretation depends on the command type

Graphics items should be accumulated until an explicit clear command is issued
*/

/** @ingroup interface_graphics3d
 * @{ */

/** The maximum number of points that can be described in a packet. */
#define PLAYER_GRAPHICS3D_MAX_POINTS 64

/** Command subtype: clear the drawing area (send an empty message) */
#define PLAYER_GRAPHICS3D_CMD_CLEAR 1
/** Command subtype: draw items */
#define PLAYER_GRAPHICS3D_CMD_DRAW 2


/** Drawmode: enumeration that defines the drawing mode */
typedef enum player_graphics3d_draw_mode
{
	PLAYER_DRAW_POINTS,
	PLAYER_DRAW_LINES,
	PLAYER_DRAW_LINE_STRIP,
	PLAYER_DRAW_LINE_LOOP,
	PLAYER_DRAW_TRIANGLES,
	PLAYER_DRAW_TRIANGLE_STRIP,
	PLAYER_DRAW_TRIANGLE_FAN,
	PLAYER_DRAW_QUADS,
	PLAYER_DRAW_QUAD_STRIP,
	PLAYER_DRAW_POLYGON
} player_graphics3d_draw_mode_t;


/** @brief Data: This interface produces no data. */

/** @brief Requests: This interface accepts no requests. */

/** @brief Command: Draw points (@ref PLAYER_GRAPHICS2D_CMD_POINTS)
Draw some points.
*/
typedef struct player_graphics3d_cmd_draw
{
  /** The drawing mode defining how teh verticies should be interpreted */
  uint32_t draw_mode;
  /** Number of points in this packet. */
  uint32_t points_count;
  /** Array of points. */
  player_point_3d_t points[PLAYER_GRAPHICS3D_MAX_POINTS];
  /** Color in which the points should be drawn. */
  player_color_t color;
  
} player_graphics3d_cmd_draw_t;


/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_gripper gripper
 * @brief An actuated gripper

The @p gripper interface provides access to a robotic gripper.

@todo This interface is VERY Pioneer-specific, and should really be generalized.
*/

/** @ingroup interface_gripper
 * @{ */

/** Data subtype: state */
#define PLAYER_GRIPPER_DATA_STATE 1

/** Command subtype: state */
#define PLAYER_GRIPPER_CMD_STATE 1

/** Request/reply subtype: get geometry*/
#define PLAYER_GRIPPER_REQ_GET_GEOM 1

/** Command codes */
#define GRIPopen   1
/** Command codes */
#define GRIPclose  2
/** Command codes */
#define GRIPstop   3
/** Command codes */
#define LIFTup     4
/** Command codes */
#define LIFTdown   5
/** Command codes */
#define LIFTstop   6
/** Command codes */
#define GRIPstore  7
/** Command codes */
#define GRIPdeploy 8
/** Command codes */
#define GRIPhalt   15
/** Command codes */
#define GRIPpress  16
/** Command codes */
#define LIFTcarry  17

/** @brief Data: state (@ref PLAYER_GRIPPER_DATA_STATE)

The @p gripper interface returns 2 bytes that represent the current
state of the gripper; the format is given below.  Note that the exact
interpretation of this data may vary depending on the details of your
gripper and how it is connected to your robot (e.g., General I/O vs. User
I/O for the Pioneer gripper).

The following list defines how the data can be interpreted for some
Pioneer robots and Stage:

- state (unsigned byte)
  - bit 0: Paddles open
  - bit 1: Paddles closed
  - bit 2: Paddles moving
  - bit 3: Paddles error
  - bit 4: Lift is up
  - bit 5: Lift is down
  - bit 6: Lift is moving
  - bit 7: Lift error
- beams (unsigned byte)
  - bit 0: Gripper limit reached
  - bit 1: Lift limit reached
  - bit 2: Outer beam obstructed
  - bit 3: Inner beam obstructed
  - bit 4: Left paddle open
  - bit 5: Right paddle open
*/
typedef struct player_gripper_data
{
  /** The current gripper lift*/
  uint32_t state;
  /** The current gripper breakbeam state */
  uint32_t beams;
} player_gripper_data_t;

/** @brief Command: state (@ref PLAYER_GRIPPER_CMD_STATE)

The @p gripper interface accepts 2-byte commands, the format of which
is given below.  These two bytes are sent directly to the gripper;
refer to Table 3-3 page 10 in the Pioneer 2 Gripper Manual for a list of
commands. The first byte is the command. The second is the argument for
the LIFTcarry and GRIPpress commands, but for all others it is ignored. */
typedef struct player_gripper_cmd
{
  /** the command */
  uint32_t cmd;
  /** optional argument */
  uint32_t arg;
} player_gripper_cmd_t;


/** @brief Request/reply: get geometry

The geometry (pose and size) of the gripper device can be queried
by sending a null @ref PLAYER_GRIPPER_REQ_GET_GEOM request.
*/
typedef struct player_gripper_geom
{
  /** Gripper pose, in robot cs (m, m, rad). */
  player_pose_t pose;
  /** Gripper dimensions (m, m). */
  player_bbox_t size;
} player_gripper_geom_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_ir ir
 * @brief Array of infrared rangers

The @p ir interface provides access to an array of infrared (IR) range
sensors.
*/

/** @ingroup interface_ir
 * @{ */

/** Maximum number of samples */
#define PLAYER_IR_MAX_SAMPLES 32

/** Request/reply subtype: get pose */
#define PLAYER_IR_POSE        1
/** Request/reply subtype: set power */
#define PLAYER_IR_POWER       2

/** Data subtype: ranges */
#define PLAYER_IR_DATA_RANGES 1

/** @brief Data: ranges (@ref PLAYER_IR_DATA_RANGES)

The @p ir interface returns range readings from the IR array. */
typedef struct player_ir_data
{
  /** number of samples */
  uint32_t voltages_count;
  /** voltages [V] */
  float voltages[PLAYER_IR_MAX_SAMPLES];
  /** number of samples */
  uint32_t ranges_count;
  /** ranges [m] */
  float ranges[PLAYER_IR_MAX_SAMPLES];
} player_ir_data_t;

/** @brief Request/reply: get pose

To query the pose of the IRs, send a null @ref PLAYER_IR_POSE request.*/
typedef struct player_ir_pose
{
  /** the number of ir samples returned by this robot */
  uint32_t poses_count;
  /** the pose of each IR detector on this robot */
  player_pose_t poses[PLAYER_IR_MAX_SAMPLES];
} player_ir_pose_t;

/** @brief Request/reply: set power

To turn IR power on and off, send a @ref PLAYER_IR_POWER request.
Null response. */
typedef struct player_ir_power_req
{
  /** FALSE for power off, TRUE for power on */
  uint8_t state;
} player_ir_power_req_t;
/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_joystick joystick
 * @brief Joystick control

The joystick interface provides access to the state of a joystick.
It allows another driver or a (possibly off-board) client to read and
use the state of a joystick.
*/

/** @ingroup interface_joystick
 * @{ */

/** Data subtype: state */
#define PLAYER_JOYSTICK_DATA_STATE 1

/** @brief Data: state (@ref PLAYER_JOYSTICK_DATA_STATE)

The joystick data packet, which contains the current state of the
joystick */
typedef struct player_joystick_data
{
  /** Current joystick position (unscaled) */
  int32_t xpos;
  /** Current joystick position (unscaled) */
  int32_t ypos;
  /** Scaling factors */
  int32_t xscale;
  /** Scaling factors */
  int32_t yscale;
  /** Button states (bitmask) */
  uint32_t buttons;
} player_joystick_data_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_laser laser
 * @brief Laser range-finder

The laser interface provides access to a single-origin scanning
range sensor, such as a SICK laser range-finder (e.g., @ref
driver_sicklms200).

Devices supporting the laser interface can be configured to scan at
different angles and resolutions.  As such, the data returned by the
laser interface can take different forms.  To make interpretation of the
data simple, the laser data packet contains some extra fields before
the actual range data.  These fields tell the client the starting and
ending angles of the scan, the angular resolution of the scan, and the
number of range readings included.  Scans proceed counterclockwise about
the laser (0 degrees is forward).  The laser can return a maximum of
@ref PLAYER_LASER_MAX_SAMPLES readings; this limits the valid combinations
of scan width and angular resolution.
*/

/** @ingroup interface_laser
 * @{ */

/** The maximum number of laser range values */
#define PLAYER_LASER_MAX_SAMPLES  1024

/** Data subtype: scan */
#define PLAYER_LASER_DATA_SCAN        1
/** Data subtype: pose-stamped scan */
#define PLAYER_LASER_DATA_SCANPOSE    2

/** Request/reply subtype: get geometry */
#define PLAYER_LASER_REQ_GET_GEOM     1
/** Request/reply subtype: get configuration */
#define PLAYER_LASER_REQ_SET_CONFIG   2
/** Request/reply subtype: set configuration */
#define PLAYER_LASER_REQ_GET_CONFIG   3
/** Request/reply subtype: set power */
#define PLAYER_LASER_REQ_POWER        4

/** @brief Data: scan (@ref PLAYER_LASER_DATA_SCAN)

The basic laser data packet.  */
typedef struct player_laser_data
{
  /** Start and end angles for the laser scan [rad].  */
  float min_angle;
  /** Start and end angles for the laser scan [rad].  */
  float max_angle;
  /** Angular resolution [rad].  */
  float resolution;
  /** Maximum range [m]. */
  float max_range;
  /** Number of range readings.  */
  uint32_t ranges_count;
  /** Range readings [m]. */
  float ranges[PLAYER_LASER_MAX_SAMPLES];
  /** Number of intensity readings */
  uint32_t intensity_count;
  /** Intensity readings. */
  uint8_t intensity[PLAYER_LASER_MAX_SAMPLES];
  /** A unique, increasing, ID for the scan */
  uint32_t id;
} player_laser_data_t;

/** @brief Data: pose-stamped scan (@ref PLAYER_LASER_DATA_SCANPOSE)

A laser scan with a pose that indicates the (possibly esimated) pose of the
laser when the scan was taken. */
typedef struct player_laser_data_scanpose
{
  /** The scan data */
  player_laser_data_t scan;
  /** The global pose of the laser at the time the scan was acquired */
  player_pose_t pose;
} player_laser_data_scanpose_t;

/** @brief Request/reply: Get geometry.

The laser geometry (position and size) can be queried by sending a
null @ref PLAYER_LASER_REQ_GET_GEOM request. */
typedef struct player_laser_geom
{
  /** Laser pose, in robot cs (m, m, rad). */
  player_pose_t pose;
  /** Laser dimensions (m, m). */
  player_bbox_t size;
} player_laser_geom_t;

/** @brief Request/reply: Get/set scan properties.

The scan configuration (resolution, aperture, etc) can be queried by
sending a null @ref PLAYER_LASER_REQ_GET_CONFIG request and modified by
sending a @ref PLAYER_LASER_REQ_SET_CONFIG request.  In either case, the
current configuration (after attempting any requested modification) will
be returned in the response.  Read the documentation for your driver to
determine what configuration values are permissible. */
typedef struct player_laser_config
{
  /** Start and end angles for the laser scan [rad].*/
  float min_angle;
  /** Start and end angles for the laser scan [rad].*/
  float max_angle;
  /** Scan resolution [rad].  */
  float resolution;
  /** Maximum range [m] */
  float max_range;
  /** Range Resolution [m] */
  float range_res;
  /** Enable reflection intensity data. */
  uint8_t  intensity;
} player_laser_config_t;

/** @brief Request/reply: Turn power on/off.

Send a @ref PLAYER_LASER_REQ_POWER request to turn laser power on or off
(assuming your hardware supports it). */
typedef struct player_laser_power_config
{
  /** FALSE to turn laser off, TRUE to turn laser on */
  uint8_t state;
} player_laser_power_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_limb limb
 * @brief A multi-jointed limb

The limb interface provides access to a multi-jointed limb

@todo Use pre-defined types, like player_pose3d_t, wherever possible
 */

/** @ingroup interface_limb
 * @{ */

/** Idle */
#define PLAYER_LIMB_STATE_IDLE        1
/** Brakes are on */
#define PLAYER_LIMB_STATE_BRAKED      2
/** Moving to target */
#define PLAYER_LIMB_STATE_MOVING      3
/** Target was out of reach */
#define PLAYER_LIMB_STATE_OOR         4
/** Target was blocked by collision */
#define PLAYER_LIMB_STATE_COLL        5

/** Data subtype: state */
#define PLAYER_LIMB_DATA              1

/** Command subtype: home */
#define PLAYER_LIMB_HOME_CMD          1
/** Command subtype: stop */
#define PLAYER_LIMB_STOP_CMD          2
/** Command subtype: set pose */
#define PLAYER_LIMB_SETPOSE_CMD       3
/** Command subtype: set position */
#define PLAYER_LIMB_SETPOSITION_CMD   4
/** Command subtype: vector move */
#define PLAYER_LIMB_VECMOVE_CMD       5

/** Request/reply: power */
#define PLAYER_LIMB_POWER_REQ         1
/** Request/reply: brakes */
#define PLAYER_LIMB_BRAKES_REQ        2
/** Request/reply: geometry */
#define PLAYER_LIMB_GEOM_REQ          3
/** Request/reply: speed */
#define PLAYER_LIMB_SPEED_REQ         4

/** @brief Data: state (@ref PLAYER_LIMB_DATA)

  The limb data packet. */
typedef struct player_limb_data
{
  /** The position of the end effector. */
  player_point_3d_t position;
  /** The approach vector of the end effector. */
  player_point_3d_t approach;
  /** The orientation vector of the end effector (a vector in a
  predefined direction on the end effector, generally from fingertip to
  fingertip). */
  player_point_3d_t orientation;
  /** The state of the limb. */
  uint8_t state;
} player_limb_data_t;

/** @brief Command: home (@ref PLAYER_LIMB_HOME_CMD)

Tells the end effector to return to its home position. */
typedef struct player_limb_home_cmd
{
} player_limb_home_cmd_t;

/** @brief Command: stop (@ref PLAYER_LIMB_STOP_CMD)

Tells the limb to stop moving immediatly. */
typedef struct player_limb_stop_cmd
{
} player_limb_stop_cmd_t;

/** @brief Command: Set end effector pose (@ref PLAYER_LIMB_SETPOSE_CMD)

Provides a fully-described pose (position, normal vector and
orientation vector) for the end effector to move to. */
typedef struct player_limb_setpose_cmd
{
  /** Position of the end effector. */
  player_point_3d_t position;
  /** Approach vector. */
  player_point_3d_t approach;
  /** Orientation vector. */
  player_point_3d_t orientation;
} player_limb_setpose_cmd_t;

/** @brief Command: Set end effector position (@ref PLAYER_LIMB_SETPOSITION_CMD)

Set the position of the end effector without worrying about a
specific orientation. */
typedef struct player_limb_setposition_cmd
{
  /** Position of the end effector. */
  player_point_3d_t position;
} player_limb_setposition_cmd_t;

/** @brief Command: Vector move the end effector (@ref PLAYER_LIMB_VECMOVE_CMD)

Move the end effector along the provided vector from its current
position for the provided distance. */
typedef struct player_limb_vecmove_cmd
{
  /** Direction vector to move in. */
  player_point_3d_t direction;
  /** Distance to move. */
  float length;
} player_limb_vecmove_cmd_t;

/** @brief Request/reply: Power.

Turn the power to the limb by sending a @ref PLAYER_LIMB_POWER_REQ request. Be
careful when turning power on that the limb is not obstructed from its
home position in case it moves to it (common behaviour). Null reponse*/
typedef struct player_limb_power_req
{
  /** Power setting; 0 for off, 1 for on. */
  uint8_t value;
} player_limb_power_req_t;

/** @brief Request/reply: Brakes.

Turn the brakes of the limb on or off by sending a @ref PLAYER_LIMB_BRAKES_REQ
request. Null response*/
typedef struct player_limb_brakes_req
{
  /** Brakes setting; 0 for off, 1 for on. */
  uint8_t value;
} player_limb_brakes_req_t;

/** @brief Request/reply: get geometry

Query geometry by sending a null @ref PLAYER_LIMB_GEOM_REQ reqest.*/
typedef struct player_limb_geom_req
{
  /** The base position of the end-effector in robot coordinates. */
  player_point_3d_t basePos;
} player_limb_geom_req_t;

/** @brief Request/reply: Speed.

Set the speed of the end effector for all subsequent movements by sending
a @ref PLAYER_LIMB_SPEED_REQ request. Null response. */
typedef struct player_limb_speed_req
{
  /** Speed setting in m/s. */
  float speed;
} player_limb_speed_req_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_localize localize
 * @brief Multi-hypothesis planar localization system

The @p localize interface provides pose information for the robot.
Generally speaking, localization drivers will estimate the pose of the
robot by comparing observed sensor readings against a pre-defined map
of the environment.  See, for the example, the @ref driver_amcl
driver, which implements a probabilistic Monte-Carlo localization
algorithm.

*/

/** @ingroup interface_localize
 * @{ */

/** The maximum number of pose hypotheses. */
#define PLAYER_LOCALIZE_MAX_HYPOTHS   10
/** The maximum number of particles (for particle-based localization
 * algorithms) */
#define PLAYER_LOCALIZE_PARTICLES_MAX 100

/** Data subtype: pose hypotheses */
#define PLAYER_LOCALIZE_DATA_HYPOTHS      1

/** Request/reply subtype: set pose hypothesis */
#define PLAYER_LOCALIZE_REQ_SET_POSE      1
/** Request/reply subtype: get particle set */
#define PLAYER_LOCALIZE_REQ_GET_PARTICLES 2

/** @brief Hypothesis format.

Since the robot pose may be ambiguous (i.e., the robot may at any
of a number of widely spaced locations), the @p localize interface is
capable of returning more that one hypothesis. */
typedef struct player_localize_hypoth
{
  /** The mean value of the pose estimate (m, m, rad). */
  player_pose_t mean;
  /** The covariance matrix pose estimate (m$^2$, rad$^2$). */
  double cov[3];
  /** The weight coefficient for linear combination (alpha) */
  double alpha;
} player_localize_hypoth_t;

/** @brief Data: hypotheses (@ref PLAYER_LOCALIZE_DATA_HYPOTHS)

The @p localize interface returns a data packet containing an an array
of hypotheses. */
typedef struct player_localize_data
{
  /** The number of pending (unprocessed observations) */
  uint32_t pending_count;
  /** The time stamp of the last observation processed. */
  double pending_time;
  /** The number of pose hypotheses. */
  uint32_t hypoths_count;
  /** The array of the hypotheses. */
  player_localize_hypoth_t hypoths[PLAYER_LOCALIZE_MAX_HYPOTHS];
} player_localize_data_t;

/** @brief Request/reply: Set the robot pose estimate.

Set the current robot pose hypothesis by sending a
@ref PLAYER_LOCALIZE_REQ_SET_POSE request.  Null response. */
typedef struct player_localize_set_pose
{
  /** The mean value of the pose estimate (m, m, rad). */
  player_pose_t mean;
  /** The diagonal elements of the covariance matrix pose estimate
      (m$^2$, rad$^2$). */
  double cov[3];
} player_localize_set_pose_t;

/** @brief A particle */
typedef struct player_localize_particle
{
  /** The particle's pose (m,m,rad) */
  player_pose_t pose;
  /** The weight coefficient for linear combination (alpha) */
  double alpha;
} player_localize_particle_t;

/** @brief Request/reply: Get particles.

To get (usually a subset of) the current particle set (assuming
the underlying driver uses a particle filter), send a null
@ref PLAYER_LOCALIZE_REQ_GET_PARTICLES request. */
typedef struct player_localize_get_particles
{
  /** The best (?) pose (mm, mm, arc-seconds). */
  player_pose_t mean;
  /** The variance of the best (?) pose (mm^2) */
  double variance;
  /** The number of particles included */
  uint32_t particles_count;
  /** The particles */
  player_localize_particle_t particles[PLAYER_LOCALIZE_PARTICLES_MAX];
} player_localize_get_particles_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_log log
 * @brief Log read / write control

The @p log interface provides start/stop control of data logging/playback.
A log device either writes data from one or more devices to a file, or
it reads logged data from a file and plays it back as if it were being
produced live by one or more devices.
*/

/** @ingroup interface_log
 * @{ */

/** Types of log device: read */
#define  PLAYER_LOG_TYPE_READ       1
/** Types of log device: write */
#define  PLAYER_LOG_TYPE_WRITE      2

/** Request/reply subtype: set write state */
#define PLAYER_LOG_REQ_SET_WRITE_STATE  1
/** Request/reply subtype: set read state */
#define PLAYER_LOG_REQ_SET_READ_STATE   2
/** Request/reply subtype: get state */
#define PLAYER_LOG_REQ_GET_STATE        3
/** Request/reply subtype: rewind */
#define PLAYER_LOG_REQ_SET_READ_REWIND  4
/** Request/reply subtype: set filename to write */
#define PLAYER_LOG_REQ_SET_FILENAME     5

/** @brief Request/reply: Set write state

To start or stop data logging, send a @ref PLAYER_LOG_REQ_SET_WRITE_STATE request.
 Null response. */
typedef struct player_log_set_write_state
{
  /** State: FALSE=disabled, TRUE=enabled */
  uint8_t state;
} player_log_set_write_state_t;

/** @brief Request/reply: Set playback state

To start or stop data playback, send a @ref PLAYER_LOG_REQ_SET_READ_STATE
request. Null response.*/
typedef struct player_log_set_read_state
{
  /** State: FALSE=disabled, TRUE=enabled */
  uint8_t state;
} player_log_set_read_state_t;

/** @brief Request/reply: Rewind playback

To rewind log playback to beginning of logfile, send a
@ref PLAYER_LOG_REQ_SET_READ_REWIND request.  Does not affect playback state
(i.e., whether it is started or stopped.  Null response. */
typedef struct player_log_set_read_rewind
{
} player_log_set_read_rewind_t;

/** @brief Request/reply: Get state.

To find out whether logging/playback is enabled or disabled, send a null
@ref PLAYER_LOG_REQ_GET_STATE request. */
typedef struct player_log_get_state
{
  /** The type of log device, either @ref PLAYER_LOG_TYPE_READ or
      @ref PLAYER_LOG_TYPE_WRITE */
  uint8_t type;
  /** Logging/playback state: FALSE=disabled, TRUE=enabled */
  uint8_t state;
} player_log_get_state_t;

/** @brief Request/reply: Set filename

To set the name of the file to write to when logging, send a
@ref PLAYER_LOG_REQ_SET_FILENAME request.  Null response. */
typedef struct player_log_set_filename
{
  /** Length of filename */
  uint32_t filename_count;
  /** Filename; max 255 chars + terminating NULL */
  char filename[256];
} player_log_set_filename_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_map map
 * @brief Access maps

The @p map interface provides access to maps.  Depending on the underlying
driver, the map may be provided as an occupancy grid, or as a set of
segments (or both).  In either case, the map is retrieved by request only.
Segment (aka vector) maps are delivered in one message, whereas grid
maps are delivered in tiles, via a sequence of requests.
*/

/** @ingroup interface_map
 * @{ */

/** The maximum size of a grid map tile, in cells

(max_payload - 12 (zlib header)) / 1.001 (zlib overhead) -  20 (tile meta-data)
*/
#define PLAYER_MAP_MAX_TILE_SIZE (((int)((PLAYER_MAX_PAYLOAD_SIZE-12)/1.001)) - 20 - 1)

/** The maximum number of segments in a vector map

(2097152 - 30 (msg header) - 20 (meta-data to accompany the lines)) /
16 (size of each line) = 131068 */
#define PLAYER_MAP_MAX_SEGMENTS 131068

/** Data subtype: grid map metadata */
#define PLAYER_MAP_DATA_INFO               1

/** Request/reply subtype: get grid map metadata  */
#define PLAYER_MAP_REQ_GET_INFO            1
/** Request/reply subtype: get grid map tile  */
#define PLAYER_MAP_REQ_GET_DATA            2
/** Request/reply subtype: get vector map */
#define PLAYER_MAP_REQ_GET_VECTOR          3

/** @brief Data AND Request/reply: Map information.

To retrieve the size and scale information of a map, send a null
@ref PLAYER_MAP_REQ_GET_INFO request. This message can also be sent as data,
with the subtype @ref PLAYER_MAP_DATA_INFO, depending on the underlying
driver. */
typedef struct player_map_info
{
  /** The scale of the map [m/pixel]. */
  float scale;
  /** The size of the map [pixels]. */
  uint32_t width;
  /** The size of the map [pixels]. */
  uint32_t height;
  /** The origin of the map [m, m, rad]. That is, the real-world pose of
   * cell (0,0) in the map */
  player_pose_t origin;
} player_map_info_t;

/** @brief Request/reply: get grid map tile

To request a grid map tile, send a @ref PLAYER_MAP_REQ_GET_DATA request with
the tile origin and size you want.  Set data_count to 0 and leave the
data field empty.  The response will contain origin, size, and occupancy
data for a tile.  Note that the response tile may not be exactly the
same as the tile you requested (e.g., your requested tile is too large
or runs off the map). */
typedef struct player_map_data
{
  /** The tile origin [pixels]. */
  uint32_t col;
  /** The tile origin [pixels]. */
  uint32_t row;
  /** The size of the tile [pixels]. */
  uint32_t width;
  /** The size of the tile [pixels]. */
  uint32_t height;
  /** The number of cells */
  uint32_t data_count;
  /** Cell occupancy value (empty = -1, unknown = 0, occupied = +1). */
  int8_t data[PLAYER_MAP_MAX_TILE_SIZE];
} player_map_data_t;

/** @brief Request/reply: get vector map

A vector map is represented as line segments.  To retrieve the vector map,
send a null @ref PLAYER_MAP_REQ_GET_VECTOR request. */
typedef struct player_map_data_vector
{
  /** The minimum and maximum coordinates of all the line segments [meters] */
  float minx;
  /** The minimum and maximum coordinates of all the line segments [meters] */
  float maxx;
  /** The minimum and maximum coordinates of all the line segments [meters] */
  float miny;
  /** The minimum and maximum coordinates of all the line segments [meters] */
  float maxy;
  /** The number of line segments  */
  uint32_t segments_count;
  /** Line segments */
  player_segment_t segments[PLAYER_MAP_MAX_SEGMENTS];
} player_map_data_vector_t;
/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_mcom mcom
 * @brief Client - client communication.

The @p mcom interface is designed for exchanging information between
clients.  A client sends a message of a given "type" and "channel". This
device stores adds the message to that channel's stack.  A second client
can then request data of a given "type" and "channel".  Push, Pop,
Read, and Clear operations are defined, but their semantics can vary,
based on the stack discipline of the underlying driver.  For example,
the @ref driver_lifomcom driver enforces a last-in-first-out stack.

@todo Is this interface used and/or needed any more?
*/

/** @ingroup interface_mcom
 * @{ */

/** size of the data field in messages */
#define MCOM_DATA_LEN            128
/** number of buffers to keep per channel */
#define  MCOM_N_BUFS             10
/** size of channel name */
#define MCOM_CHANNEL_LEN        8
/** returns this if empty */
#define  MCOM_EMPTY_STRING          "(EMPTY)"
/** request ids */
#define  PLAYER_MCOM_PUSH         0
/** request ids */
#define  PLAYER_MCOM_POP          1
/** request ids */
#define  PLAYER_MCOM_READ         2
/** request ids */
#define  PLAYER_MCOM_CLEAR        3
/** request ids */
#define  PLAYER_MCOM_SET_CAPACITY 4
/** ?? */
#define MCOM_COMMAND_BUFFER_SIZE (sizeof(player_mcom_config_t))

/** @brief A piece of data. */
typedef struct player_mcom_data
{
  /** a flag */
  char full;
  /** length of data */
  uint32_t data_count;
  /** the data */
  char data[MCOM_DATA_LEN];
} player_mcom_data_t;

/** @brief Configuration request to the device. */
typedef struct player_mcom_config
{
  /** Which request.  Should be one of the defined request ids. */
  uint32_t command;
  /** The "type" of the data. */
  uint32_t type;
  /** length of channel name */
  uint32_t channel_count;
  /** The name of the channel. */
  char channel[MCOM_CHANNEL_LEN];
  /** The data. */
  player_mcom_data_t data;
} player_mcom_config_t;

/** @brief Configuration reply from the device. */
typedef struct player_mcom_return
{
  /** The "type" of the data */
  uint32_t type;
  /** length of channel name */
  uint32_t channel_count;
  /** The name of the channel. */
  char channel[MCOM_CHANNEL_LEN];
  /** The data. */
  player_mcom_data_t data;
} player_mcom_return_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_opaque opaque
 * @brief A generic interface for user-defined messages

The @p opaque interface allows you to send user-specified messages.  With this
interface a user can send custom commands to their drivers/plugins.
*/

/** @ingroup interface_opaque
 * @{ */

/** Data subtype: generic state */
#define PLAYER_OPAQUE_DATA_STATE             1

/** Data subtype: generic command */
#define PLAYER_OPAQUE_CMD                    2

/** Data subtype: generic request */
#define PLAYER_OPAQUE_REQ                    3

/** Maximum message size is 1 MB */
#define PLAYER_OPAQUE_MAX_SIZE            1048576

/** @brief data */
typedef struct player_opaque_data
{
  /** Size of data as stored in buffer (bytes) */
  uint32_t data_count;
  /** The data we will be sending */
  uint8_t data[PLAYER_OPAQUE_MAX_SIZE];
} player_opaque_data_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_planner planner
 * @brief A planar path-planner

The @p planner interface provides control of a 2-D motion planner.
*/

/** @ingroup interface_planner
 * @{ */

/** maximum number of waypoints in a single plan */
#define PLAYER_PLANNER_MAX_WAYPOINTS 128

/** Data subtype: state */
#define PLAYER_PLANNER_DATA_STATE 1

/** Command subtype: state */
#define PLAYER_PLANNER_CMD_GOAL 1

/** Request subtype: get waypoints */
#define PLAYER_PLANNER_REQ_GET_WAYPOINTS 1
/** Request subtype: enable / disable planner */
#define PLAYER_PLANNER_REQ_ENABLE        2

/** @brief Data: state (@ref PLAYER_PLANNER_DATA_STATE)

The @p planner interface reports the current execution state of the
planner. */
typedef struct player_planner_data
{
  /** Did the planner find a valid path? */
  uint8_t valid;
  /** Have we arrived at the goal? */
  uint8_t done;
  /** Current location (m,m,rad) */
  player_pose_t pos;
  /** Goal location (m,m,rad) */
  player_pose_t goal;
  /** Current waypoint location (m,m,rad) */
  player_pose_t waypoint;
  /** Current waypoint index (handy if you already have the list
      of waypoints). May be negative if there's no plan, or if
      the plan is done */
  int32_t waypoint_idx;
  /** Number of waypoints in the plan */
  uint32_t waypoints_count;
} player_planner_data_t;

/** @brief Command: state (@ref PLAYER_PLANNER_CMD_GOAL)

The @p planner interface accepts a new goal. */
typedef struct player_planner_cmd
{
  /** Goal location (m,m,rad) */
  player_pose_t goal;
} player_planner_cmd_t;

/** @brief Request/reply: Get waypoints

To retrieve the list of waypoints, send a null
@ref PLAYER_PLANNER_REQ_GET_WAYPOINTS request.
*/
typedef struct player_planner_waypoints_req
{
  /** Number of waypoints to follow */
  uint32_t waypoints_count;
  /** The waypoints */
  player_pose_t waypoints[PLAYER_PLANNER_MAX_WAYPOINTS];
} player_planner_waypoints_req_t;

/** @brief Request/reply: Enable/disable robot motion

To enable or disable the planner, send a @ref PLAYER_PLANNER_REQ_ENABLE
request.  When disabled, the planner will stop the robot.  When enabled, the planner should resume plan execution.  Null response.
*/
typedef struct player_planner_enable_req
{
  /** state: TRUE to enable, FALSE to disable */
  uint8_t state;
} player_planner_enable_req_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_player player
 * @brief Player: the meta-device

The @p player device represents the server itself, and is used in
configuring the behavior of the server.  There is only one such device
(with index 0) and it is always open.

@todo Determine what, if any, data delivery modes and requests are needed.
*/

/** @ingroup interface_player
 * @{ */

/** Device access mode: open */
#define PLAYER_OPEN_MODE   1
/** Device access mode: close */
#define PLAYER_CLOSE_MODE  2
/** Device access mode: error */
#define PLAYER_ERROR_MODE  3


/** Data delivery mode: Send data from all subscribed devices
all the time (i.e. when it's ready on the server). */
#define PLAYER_DATAMODE_PUSH  1
/** Data delivery mode: Only on request, send data from all
subscribed devices. A PLAYER_MSGTYPE_SYNCH packet follows each
set of data. Request should be made automatically by client
libraries when they begin reading. */
#define PLAYER_DATAMODE_PULL   2

/** Request/reply subtype: get device list */
#define PLAYER_PLAYER_REQ_DEVLIST     1
/** Request/reply subtype: get driver info */
#define PLAYER_PLAYER_REQ_DRIVERINFO  2
/** Request/reply subtype: (un)subscribe to device */
#define PLAYER_PLAYER_REQ_DEV         3

#define PLAYER_PLAYER_REQ_DATA        4
#define PLAYER_PLAYER_REQ_DATAMODE    5
#define PLAYER_PLAYER_REQ_AUTH        7
#define PLAYER_PLAYER_REQ_NAMESERVICE 8
#define PLAYER_PLAYER_REQ_IDENT       9
#define PLAYER_PLAYER_REQ_ADD_REPLACE_RULE 10


/** @brief Request/reply: Get the list of available devices.

    It's useful for applications such as viewer programs
    and test suites that tailor behave differently depending on which
    devices are available.  To request the list, send a null
    @ref PLAYER_PLAYER_REQ_DEVLIST. */
typedef struct player_device_devlist
{
  /** The number of devices */
  uint32_t devices_count;
  /** The list of available devices. */
  player_devaddr_t devices[PLAYER_MAX_DEVICES];
} player_device_devlist_t;

/** @brief Request/reply: Get the driver name for a particular device.

To get a name, send a @ref PLAYER_PLAYER_REQ_DRIVERINFO request that
specifies the address of the desired device in the addr field.
Set driver_name_count to 0 and leave driver_name empty. The response
will contain the driver name. */
typedef struct player_device_driverinfo
{
  /** The device identifier. */
  player_devaddr_t addr;
  /** Length of the driver name */
  uint32_t driver_name_count;
  /** The driver name (returned) */
  char driver_name[PLAYER_MAX_DRIVER_STRING_LEN];
} player_device_driverinfo_t;

/** @brief Request/reply: (un)subscribe to a device

This is the most important request!  Before interacting with a device,
the client must request appropriate access.    Valid access modes are:
- @ref PLAYER_OPEN_MODE : subscribe to the device.  You will receive any data
published by the device and you may send it commands and/or requests.
- @ref PLAYER_CLOSE_MODE : unsubscribe from the device.
- @ref PLAYER_ERROR_MODE : the requested access was not granted (only appears
in responses)

To request access, send a @ref PLAYER_PLAYER_REQ_DEV request that specifies
the desired device address in the addr field and the desired access mode
in access.  Set driver_name_count to 0 and leave driver_name empty.
The response will indicate the granted access in the access field and
the name of the underyling driver in the driver_name field.  Note that
the granted access may not be the same as the requested access (e.g.,
if initialization of the driver failed).   */
typedef struct player_device_req
{
  /** Address of the device */
  player_devaddr_t addr;
  /** The requested / granted access */
  uint8_t access;
  /** Length of driver name */
  uint32_t driver_name_count;
  /** The name of the underlying driver */
  char driver_name[PLAYER_MAX_DRIVER_STRING_LEN];
} player_device_req_t;

/** @brief Configuration request: Get data.

When the server is in a PLAYER_DATAMODE_PULL data delivery mode, the
client can request a single round of data by sending a zero-argument
request with type code @p PLAYER_PLAYER_REQ_DATA.  The response will
be a zero-length acknowledgement. */
typedef struct player_device_data_req
{
} player_device_data_req_t;

/** @brief Configuration request: Change data delivery mode.

The Player server supports two data modes, described above.
By default, the server operates in @p PLAYER_DATAMODE_PUSH mode. To
switch to a different mode send a request with the format given
below. The server's reply will be a zero-length acknowledgement. */
typedef struct player_device_datamode_req
{
  /** The requested mode */
  uint8_t mode;

} player_device_datamode_req_t;


/** @brief Configuration request: Authentication.

@todo Add support for this mechanism to libplayertcp.  Right now, it's disabled.

If server authentication has been enabled (by providing '-key &lt;key&gt;'
on the command-line); then each client must
authenticate itself before otherwise interacting with the server.
To authenticate, send a request with this format.

If the key matches the server's key then the client is authenticated,
the server will reply with a zero-length acknowledgement, and the client
can continue with other operations.  If the key does not match, or if
the client attempts any other server interactions before authenticating,
then the connection will be closed immediately.  It is only necessary
to authenticate each client once.

Note that this support for authentication is @b NOT a security mechanism.
The keys are always in plain text, both in memory and when transmitted
over the network; further, since the key is given on the command-line,
there is a very good chance that you can find it in plain text in the
process table (in Linux try 'ps -ax | grep player').  Thus you should
not use an important password as your key, nor should you rely on
Player authentication to prevent bad guys from driving your robots (use
a firewall instead).  Rather, authentication was introduced into Player
to prevent accidentally connecting one's client program to someone else's
robot.  This kind of accident occurs primarily when Stage is running in
a multi-user environment.  In this case it is very likely that there
is a Player server listening on port 6665, and clients will generally
connect to that port by default, unless a specific option is given.

This mechanism was never really used, and may be removed. */
typedef struct player_device_auth_req
{
  /** Length of key */
  uint32_t auth_key_count;
  /** The authentication key */
  uint8_t auth_key[PLAYER_KEYLEN];

} player_device_auth_req_t;


/** @brief Nameservice request.

@todo Update this structure and add support for it to libplayertcp.  Right now it's disabled.
*/
typedef struct player_device_nameservice_req
{
  /** Length of robot name */
  uint32_t name_count;
  /** The robot name */
  uint8_t name[PLAYER_MAX_DRIVER_STRING_LEN];
  /** The corresponding port */
  uint16_t port;
} player_device_nameservice_req_t;

/** @brief Configuration request: Add client queue replace rule.

Allows the client to add a replace rule to their server queue. Replace
rules define which messages will be replaced when new data arrives.
If you are not updating frequently from ther server then the use of
replace rules for data packets will stop any queue overflow messages

Each field in the request type corresponds to the equivalent field in
the message header use -1 for a dont care value.
 */
typedef struct player_add_replace_rule_req
{
  /** Interface to set replace rule for (-1 for wildcard) */
  int32_t interf;
  /** index to set replace rule for (-1 for wildcard) */
  int32_t index;
  /** message type to set replace rule for (-1 for wildcard), i.e. PLAYER_MSGTYPE_DATA */
  int32_t type;
  /** message subtype to set replace rule for (-1 for wildcard) */
  int32_t subtype;
  /** Should we replace these messages */
  int32_t replace ;
} player_add_replace_rule_req_t;


/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_position1d position1d
 * @brief A 1-D linear actuator

The @p position1d interface is used to control linear actuators
*/

/** @ingroup interface_position1d
 * @{ */

/** Request/reply subtype: get geometry */
#define PLAYER_POSITION1D_REQ_GET_GEOM          1
/** Request/reply subtype: motor power */
#define PLAYER_POSITION1D_REQ_MOTOR_POWER       2
/** Request/reply subtype: velocity mode */
#define PLAYER_POSITION1D_REQ_VELOCITY_MODE     3
/** Request/reply subtype: position mode */
#define PLAYER_POSITION1D_REQ_POSITION_MODE     4
/** Request/reply subtype: set odometry */
#define PLAYER_POSITION1D_REQ_SET_ODOM          5
/** Request/reply subtype: reset odometry */
#define PLAYER_POSITION1D_REQ_RESET_ODOM        6
/** Request/reply subtype: set speed PID params */
#define PLAYER_POSITION1D_REQ_SPEED_PID         7
/** Request/reply subtype: set position PID params */
#define PLAYER_POSITION1D_REQ_POSITION_PID      8
/** Request/reply subtype: set speed profile params */
#define PLAYER_POSITION1D_REQ_SPEED_PROF        9

/** Data subtype: state */
#define PLAYER_POSITION1D_DATA_STATE             1
/** Data subtype: geometry */
#define PLAYER_POSITION1D_DATA_GEOM              2

/** Command subtype: velocity command */
#define PLAYER_POSITION1D_CMD_VEL                1
/** Command subtype: position command */
#define PLAYER_POSITION1D_CMD_POS                2

/** Status byte: limit min */
#define PLAYER_POSITION1D_STATUS_LIMIT_MIN 0
/** Status byte: limit center */
#define PLAYER_POSITION1D_STATUS_LIMIT_CEN 1
/** Status byte: limit max */
#define PLAYER_POSITION1D_STATUS_LIMIT_MAX 2
/** Status byte: limit over current */
#define PLAYER_POSITION1D_STATUS_OC 3
/** Status byte: limit trajectory complete */
#define PLAYER_POSITION1D_STATUS_TRAJ_COMPLETE 4
/** Status byte: enabled */
#define PLAYER_POSITION1D_STATUS_ENABLED 5

/** @brief Data: state (@ref PLAYER_POSITION1D_DATA_STATE)

The @p position interface returns data regarding the odometric pose and
velocity of the robot, as well as motor stall information. */
typedef struct player_position1d_data
{
  /** position [m] or [rad] depending on actuator type*/
  float pos;
  /** translational velocities [m/s] or [rad/s] depending on actuator type*/
  float vel;
  /** Is the motor stalled? */
  uint8_t stall;
  /** bitfield of extra data in the following order:
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
  uint8_t status;

} player_position1d_data_t;

/** @brief Command: state (@ref PLAYER_POSITION1D_CMD_VEL)

The @p position1d interface accepts new velocities for
the robot's motors (drivers may support position control, speed control,
or both). */
typedef struct player_position1d_cmd_vel
{
  /** velocity [m/s] or [rad/s] */
  float vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position1d_cmd_vel_t;

/** @brief Command: state (@ref PLAYER_POSITION1D_CMD_POS)

The @p position1d interface accepts new positions for
the robot's motors (drivers may support position control, speed control,
or both). */
typedef struct player_position1d_cmd_pos
{
  /** position [m] or [rad] */
  float pos;
  /** velocity at which to move to the position [m/s] or [rad/s] */
  float vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position1d_cmd_pos_t;

/** @brief Request/reply: Query geometry.

To request robot geometry, send a null
@ref PLAYER_POSITION1D_REQ_GET_GEOM. */
typedef struct player_position1d_geom
{
  /** Pose of the robot base, in the robot cs (m, m, rad). */
  player_pose_t pose;
  /** Dimensions of the base (m, m). */
  player_bbox_t size;
} player_position1d_geom_t;

/** @brief Request/reply: Motor power.

On some robots, the motor power can be turned on and off from software.
To do so, send a @ref PLAYER_POSITION1D_REQ_MOTOR_POWER request with the format
given below, and with the appropriate @p state (zero for motors off and
non-zero for motors on).  Null response.

Be VERY careful with this command!  You are very likely to start the
robot running across the room at high speed with the battery charger
still attached.
*/
typedef struct player_position1d_power_config
{
  /** FALSE for off, TRUE for on */
  uint8_t state;
} player_position1d_power_config_t;

/** @brief Request/reply: Change velocity control.

Some robots offer different velocity control modes.  It can be changed by
sending a @ref PLAYER_POSITION1D_REQ_VELOCITY_MODE request with the format given
below, including the appropriate mode.  No matter which mode is used, the
external client interface to the @p position1d device remains the same.
Null response.
*/
typedef struct player_position1d_velocity_mode_config
{
  /** driver-specific */
  uint32_t value;
} player_position1d_velocity_mode_config_t;

/** @brief Request/reply: Reset odometry.

To reset the robot's odometry to x = 0, send a @ref PLAYER_POSITION1D_REQ_RESET_ODOM
request.  Null response. */
typedef struct player_position1d_reset_odom_config
{
  /** driver-specific */
  uint32_t value;
} player_position1d_reset_odom_config_t;

/** @brief Request/reply: Change control mode.

To change the control mode, send a @ref PLAYER_POSITION1D_REQ_POSITION_MODE reqeust.
Null response.
*/
typedef struct player_position1d_position_mode
{
  /** 0 for velocity mode, 1 for position mode */
  uint32_t state;
} player_position1d_position_mode_req_t;

/** @brief Request/reply: Set odometry.

To set the robot's odometry
to a particular state, send a @ref PLAYER_POSITION1D_REQ_SET_ODOM request.
Null response. */
typedef struct player_position1d_set_odom
{
  /** (x) [m] or [rad] */
  float pos;
} player_position1d_set_odom_req_t;

/** @brief Request/reply: Set velocity PID parameters.

To set velocity PID parameters, send a @ref PLAYER_POSITION1D_REQ_SPEED_PID request.
Null response.
*/
typedef struct player_position1d_speed_pid
{
  /** PID parameters */
  float kp;
  /** PID parameters */
  float ki;
  /** PID parameters */
  float kd;
} player_position1d_speed_pid_req_t;

/** @brief Request/reply: Set position PID parameters.

To set position PID parameters, send a @ref PLAYER_POSITION1D_REQ_POSITION_PID request.
Null response.
*/
typedef struct player_position1d_position_pid
{
  /** PID parameters */
  float kp;
  /** PID parameters */
  float ki;
  /** PID parameters */
  float kd;
} player_position1d_position_pid_req_t;

/** @brief Request/reply: Set linear speed profile parameters.

To set linear speed profile parameters, send a
@ref PLAYER_POSITION1D_REQ_SPEED_PROF requst.  Null response.
*/
typedef struct player_position1d_speed_prof
{
  /** max speed [m/s] or [rad/s] */
  float speed;
  /** max acceleration [m/s^2] or [rad/s^2] */
  float acc;
} player_position1d_speed_prof_req_t;
/** @} */


// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_position2d position2d
 * @brief Planar mobile robot

The @p position2d interface is used to control mobile robot bases in 2D.
*/

/** @ingroup interface_position2d
 * @{ */

/** Request/reply subtype: get geometry */
#define PLAYER_POSITION2D_REQ_GET_GEOM          1
/** Request/reply subtype: motor power */
#define PLAYER_POSITION2D_REQ_MOTOR_POWER       2
/** Request/reply subtype: velocity mode */
#define PLAYER_POSITION2D_REQ_VELOCITY_MODE     3
/** Request/reply subtype: position mode */
#define PLAYER_POSITION2D_REQ_POSITION_MODE     4
/** Request/reply subtype: set odometry */
#define PLAYER_POSITION2D_REQ_SET_ODOM          5
/** Request/reply subtype: reset odometry */
#define PLAYER_POSITION2D_REQ_RESET_ODOM        6
/** Request/reply subtype: set speed PID params */
#define PLAYER_POSITION2D_REQ_SPEED_PID         7
/** Request/reply subtype: set position PID params */
#define PLAYER_POSITION2D_REQ_POSITION_PID      8
/** Request/reply subtype: set speed profile params */
#define PLAYER_POSITION2D_REQ_SPEED_PROF        9

/** Data subtype: state */
#define PLAYER_POSITION2D_DATA_STATE             1
/** Data subtype: geometry */
#define PLAYER_POSITION2D_DATA_GEOM              2

/** Command subtype: velocity command */
#define PLAYER_POSITION2D_CMD_VEL                1
/** Command subtype: position command */
#define PLAYER_POSITION2D_CMD_POS              2
/** Command subtype: carlike command */
#define PLAYER_POSITION2D_CMD_CAR              3

/** @brief Data: state (@ref PLAYER_POSITION2D_DATA_STATE)

The @p position interface returns data regarding the odometric pose and
velocity of the robot, as well as motor stall information. */
typedef struct player_position2d_data
{
  /** position [m,m,rad] (x, y, yaw)*/
  player_pose_t pos;
  /** translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
  player_pose_t vel;
  /** Are the motors stalled? */
  uint8_t stall;
} player_position2d_data_t;

/** @brief Command: velocity (@ref PLAYER_POSITION2D_CMD_VEL)

The @p position interface accepts new velocities
for the robot's motors (drivers may support position control, speed control,
or both). */
typedef struct player_position2d_cmd_vel
{
  /** translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
  player_pose_t vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position2d_cmd_vel_t;

/** @brief Command: position (@ref PLAYER_POSITION2D_CMD_POS)

The @p position interface accepts new positions
for the robot's motors (drivers may support position control, speed control,
or both). */
typedef struct player_position2d_cmd_pos
{
  /** position [m,m,rad] (x, y, yaw)*/
  player_pose_t pos;
  /** velocity at which to move to the position [m/s] or [rad/s] */
  player_pose_t vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position2d_cmd_pos_t;

/** @brief Command: carlike (@ref PLAYER_POSITION2D_CMD_CAR)

The @p position interface accepts new carlike velocity commands (speed and turning angle)
for the robot's motors (only supported by some drivers). */
typedef struct player_position2d_cmd_car
{
  /** forward velocity (m/s) */
  double velocity;
  /** turning angle (rad) */
  double angle;
} player_position2d_cmd_car_t;


/** @brief Data AND Request/reply: geometry.

To request robot geometry, send a null @ref PLAYER_POSITION2D_REQ_GET_GEOM
request.  Depending on the underlying driver, this message may also be
sent as data, with subtype @ref PLAYER_POSITION2D_DATA_GEOM. */
typedef struct player_position2d_geom
{
  /** Pose of the robot base, in the robot cs (m, m, rad). */
  player_pose_t pose;
  /** Dimensions of the base (m, m). */
  player_bbox_t size;
} player_position2d_geom_t;

/** @brief Request/reply: Motor power.

On some robots, the motor power can be turned on and off from software.
To do so, send a @ref PLAYER_POSITION2D_REQ_MOTOR_POWER request with the
format given below, and with the appropriate @p state (zero for motors
off and non-zero for motors on).  Null response.

Be VERY careful with this command!  You are very likely to start the
robot running across the room at high speed with the battery charger
still attached.
*/
typedef struct player_position2d_power_config
{
  /** FALSE for off, TRUE for on */
  uint8_t state;
} player_position2d_power_config_t;

/** @brief Request/reply: Change velocity control.

Some robots offer different velocity control modes.  It can be changed by
sending a @ref PLAYER_POSITION2D_REQ_VELOCITY_MODE request with the format
given below, including the appropriate mode.  No matter which mode is
used, the external client interface to the @p position device remains
the same.  Null response.

The @ref driver_p2os driver offers two modes of velocity control:
separate translational and rotational control and direct wheel control.
When in the separate mode, the robot's microcontroller internally
computes left and right wheel velocities based on the currently commanded
translational and rotational velocities and then attenuates these values
to match a nice predefined acceleration profile.  When in the direct
mode, the microcontroller simply passes on the current left and right
wheel velocities.  Essentially, the separate mode offers smoother but
slower (lower acceleration) control, and the direct mode offers faster
but jerkier (higher acceleration) control.  Player's default is to use
the direct mode.  Set @a mode to zero for direct control and non-zero
for separate control.

For the @ref driver_reb driver, 0 is direct velocity control,
1 is for velocity-based heading PD controller.
*/
typedef struct player_position2d_velocity_mode_config
{
  /** driver-specific */
  uint32_t value;
} player_position2d_velocity_mode_config_t;

/** @brief Request/reply: Reset odometry.

To reset the robot's odometry to @f$(x, y, yaw) = (0,0,0)@f$, send
a @ref PLAYER_POSITION2D_REQ_RESET_ODOM request.  Null response. */
typedef struct player_position2d_reset_odom_config
{
} player_position2d_reset_odom_config_t;

/** @brief Request/reply: Change control mode.

To change control mode, send a @ref PLAYER_POSITION2D_REQ_POSITION_MODE request.
Null response.
*/
typedef struct player_position2d_position_mode_req
{
  /** 0 for velocity mode, 1 for position mode */
  uint32_t state;
} player_position2d_position_mode_req_t;

/** @brief Request/reply: Set odometry.

To set the robot's odometry to a particular state, send a
@ref PLAYER_POSITION2D_REQ_SET_ODOM request.  Null response. */
typedef struct player_position2d_set_odom_req
{
  /** (x, y, yaw) [m, m, rad] */
  player_pose_t pose;
} player_position2d_set_odom_req_t;

/** @brief Request/reply: Set velocity PID parameters.
 *
 * To set velocity PID parameters, send a @ref PLAYER_POSITION2D_REQ_SPEED_PID
 * request.  Null response.
 */
typedef struct player_position2d_speed_pid_req
{
  /** PID parameters */
  float kp;
  /** PID parameters */
  float ki;
  /** PID parameters */
  float kd;
} player_position2d_speed_pid_req_t;

/** @brief Request/reply: Set position PID parameters.
 *
 * To set position PID parameters, send a
 * @ref PLAYER_POSITION2D_REQ_POSITION_PID
 * request.  Null response.
 */
typedef struct player_position2d_position_pid_req
{
  /** PID parameters */
  float kp;
  /** PID parameters */
  float ki;
  /** PID parameters */
  float kd;
} player_position2d_position_pid_req_t;

/** @brief Request/reply: Set linear speed profile parameters.
 *
 * To set linear speed profile parameters, send a
 * @ref PLAYER_POSITION2D_REQ_SPEED_PROF request.  Null response. */
typedef struct player_position2d_speed_prof_req
{
  /** max speed [m/s] */
  float speed;
  /** max acceleration [m/s^2] */
  float acc;
} player_position2d_speed_prof_req_t;
/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_position3d position3d
 * @brief A robot that moves in 3-D

The position3d interface is used to control mobile robot bases in 3D
(i.e., pitch and roll are important).
*/

/** @ingroup interface_position3d
 * @{ */

/** Data subtype: state */
#define PLAYER_POSITION3D_DATA_STATE 1
/** Data subtype: geometry */
#define PLAYER_POSITION3D_DATA_GEOMETRY 2

/** Command subtype: velocity control */
#define  PLAYER_POSITION3D_CMD_SET_VEL      1
/** Command subtype: position control */
#define  PLAYER_POSITION3D_CMD_SET_POS      2

/** Request/reply subtype: get geometry */
#define PLAYER_POSITION3D_GET_GEOM          1
/** Request/reply subtype: motor power */
#define PLAYER_POSITION3D_MOTOR_POWER       2
/** Request/reply subtype: velocity mode */
#define PLAYER_POSITION3D_VELOCITY_MODE     3
/** Request/reply subtype: position mode */
#define PLAYER_POSITION3D_POSITION_MODE     4
/** Request/reply subtype: reset odometry */
#define PLAYER_POSITION3D_RESET_ODOM        5
/** Request/reply subtype: set odometry */
#define PLAYER_POSITION3D_SET_ODOM          6
/** Request/reply subtype: set speed PID params */
#define PLAYER_POSITION3D_SPEED_PID         7
/** Request/reply subtype: set position PID params */
#define PLAYER_POSITION3D_POSITION_PID      8
/** Request/reply subtype: set speed profile params */
#define PLAYER_POSITION3D_SPEED_PROF        9

/** @brief Data: state (@ref PLAYER_POSITION3D_DATA_STATE)

This interface returns data regarding the odometric pose and velocity
of the robot, as well as motor stall information.  */
typedef struct player_position3d_data
{
  /** (x, y, z, roll, pitch, yaw) position [m, m, m, rad, rad, rad] */
  player_pose3d_t pos;
  /** (x, y, z, roll, pitch, yaw) velocity [m, m, m, rad, rad, rad] */
  player_pose3d_t vel;
  /** Are the motors stalled? */
  uint8_t stall;
} player_position3d_data_t;

/** @brief Command: position (@ref PLAYER_POSITION3D_CMD_SET_POS)

It accepts new positions and/or velocities for the robot's motors
(drivers may support position control, speed control, or both).  */
typedef struct player_position3d_cmd_pos
{
  /** (x, y, z, roll, pitch, yaw) position [m, m, m, rad, rad, rad] */
  player_pose3d_t pos;
  /** velocity at which to move to the position [m/s] or [rad/s] */
  player_pose3d_t vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position3d_cmd_pos_t;

/** @brief Command: velocity (@ref PLAYER_POSITION3D_CMD_SET_VEL)

It accepts new positions and/or velocities for the robot's motors
(drivers may support position control, speed control, or both).  */
typedef struct player_position3d_cmd_vel
{
  /** (x, y, z, roll, pitch, yaw) velocity [m, m, m, rad, rad, rad] */
  player_pose3d_t vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position3d_cmd_vel_t;

/** @brief Request/reply: Query geometry.

To request robot geometry, send a null @ref PLAYER_POSITION3D_GET_GEOM request. */
typedef struct player_position3d_geom
{
  /** Pose of the robot base, in the robot cs (m, m, m, rad, rad, rad).*/
  player_pose3d_t pose;
  /** Dimensions of the base (m, m, m). */
  player_bbox3d_t size;
} player_position3d_geom_t;

/** @brief Request/reply: Motor power.

On some robots, the motor power can be turned on and off from software.
To do so, send a @ref PLAYER_POSITION3D_MOTOR_POWER request with the format
given below, and with the appropriate @p state (zero for motors off
and non-zero for motors on).  Null response.

Be VERY careful with this command!  You are very likely to start the
robot running across the room at high speed with the battery charger
still attached.  */
typedef struct player_position3d_power_config
{
  /** FALSE for off, TRUE for on */
  uint8_t state;
} player_position3d_power_config_t;

/** @brief Request/reply: Change position control.

To change control mode, send a @ref PLAYER_POSITION3D_POSITION_MODE request.
Null response.
*/
typedef struct player_position3d_position_mode_req
{
  /** 0 for velocity mode, 1 for position mode */
  uint32_t value;
} player_position3d_position_mode_req_t;

/** @brief Request/reply: Change velocity control.

Some robots offer different velocity control modes.  It can be changed by
sending a @ref PLAYER_POSITION3D_VELOCITY_MODE request with the format given
below, including the appropriate mode.  No matter which mode is used, the
external client interface to the @p position3d device remains the same.
Null response. */
typedef struct player_position3d_velocity_mode_config
{
  /** driver-specific */
  uint32_t value;
} player_position3d_velocity_mode_config_t;

/** @brief Request/reply: Set odometry.

To set the robot's odometry to a particular state, send a
@ref PLAYER_POSITION3D_SET_ODOM request.  Null response. */
typedef struct player_position3d_set_odom_req
{
  /** (x, y, z, roll, pitch, yaw) position [m, m, m, rad, rad, rad] */
  player_pose3d_t pos;
} player_position3d_set_odom_req_t;

/** @brief Request/reply: Reset odometry.

To reset the robot's odometry to @f$(x,y,\theta) = (0,0,0)@f$,
send a @ref PLAYER_POSITION3D_RESET_ODOM request.  Null response. */
typedef struct player_position3d_reset_odom_config
{
} player_position3d_reset_odom_config_t;

/** @brief Request/reply: Set velocity PID parameters.
 *
 * To set velocity PID parameters, send a @ref PLAYER_POSITION3D_SPEED_PID
 * request.  Null response. */
typedef struct player_position3d_speed_pid_req
{
  /** PID parameters */
  float kp;
  /** PID parameters */
  float ki;
  /** PID parameters */
  float kd;
} player_position3d_speed_pid_req_t;

/** @brief Request/reply: Set position PID parameters.
 *
 * To set position PID parameters, send a @ref PLAYER_POSITION3D_POSITION_PID
 * request.  Null response. */
typedef struct player_position3d_position_pid_req
{
  /** PID parameters */
  float kp;
  /** PID parameters */
  float ki;
  /** PID parameters */
  float kd;
} player_position3d_position_pid_req_t;

/** @brief Request/reply: Set speed profile parameters.
 *
 * To set speed profile parameters, send a @ref PLAYER_POSITION3D_SPEED_PROF
 * request.  Null response. */
typedef struct player_position3d_speed_prof_req
{
  /** max speed [rad/s] */
  float speed;
  /** max acceleration [rad/s^2] */
  float acc;
} player_position3d_speed_prof_req_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_power power
 * @brief Power system

The @p power interface provides access to a robot's power
subsystem. Includes the functionality of the old Player @p energy device, which is
now deprecated.
*/

/** @ingroup interface_power
 * @{ */

/** Data subtype: voltage */
#define PLAYER_POWER_DATA_STATE 1

/** Request subtype: set charging policy */
#define PLAYER_POWER_SET_CHARGING_POLICY_REQ 1

/** bit masks for the  player_power_data_t mask field */
#define PLAYER_POWER_MASK_VOLTS 1
#define PLAYER_POWER_MASK_WATTS 2
#define PLAYER_POWER_MASK_JOULES 4
#define PLAYER_POWER_MASK_PERCENT 8
#define PLAYER_POWER_MASK_CHARGING 16

/** @brief Data: voltage (@ref PLAYER_POWER_DATA_STATE)

The @p power interface returns data in this format. */
typedef struct player_power_data
{
  /** Status bits. The driver will set the bits to indicate which fields
      it is using. Bitwise-and with PLAYER_POWER_MASK_X values to see
      which fields are being set.*/
  uint32_t valid;

  /** Battery voltage [V] */
  float  volts;
  /** Percent of full charge [%] */
  float percent;
  /** energy stored [J]. */
  float joules;
  /** estimated current energy consumption (negative values) or
      aquisition (positive values) [W]. */
  float watts;
  /** charge exchange status: if 1, the device is currently receiving
      charge from another energy device. If -1 the device is currently
      providing charge to another energy device. If 0, the device is
      not exchanging charge with an another device. */
  int32_t charging;

} player_power_data_t;


/** @brief Request/reply: set charging policy
 *
 * Send a @ref PLAYER_ENERGY_SET_CHARGING_POLICY_REQ request to change the
 * charging policy. */
typedef struct player_power_chargepolicy_config
{
  /** uint8_tean controlling recharging. If FALSE, recharging is
      disabled. Defaults to TRUE */
  uint8_t enable_input;
  /** uint8_tean controlling whether others can recharge from this
      device. If FALSE, charging others is disabled. Defaults to TRUE.*/
  uint8_t enable_output;
} player_power_chargepolicy_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_ptz ptz
 * @brief Pan-tilt-zoom unit

The ptz interface is used to control a pan-tilt-zoom unit, such as a camera.
*/

/** @ingroup interface_ptz
 * @{ */

/** Maximum command length for use with @ref PLAYER_PTZ_REQ_GENERIC request;
    based on the Sony EVID30 camera right now. */
#define PLAYER_PTZ_MAX_CONFIG_LEN  32

/** Control mode, for use with @ref PLAYER_PTZ_REQ_CONTROL_MODE */
#define PLAYER_PTZ_VELOCITY_CONTROL 0
/** Control mode, for use with @ref PLAYER_PTZ_REQ_CONTROL_MODE */
#define PLAYER_PTZ_POSITION_CONTROL 1

/** Request/reply subtype: generic */
#define PLAYER_PTZ_REQ_GENERIC         1
/** Request/reply subtype: control mode */
#define PLAYER_PTZ_REQ_CONTROL_MODE    2
/** Request/reply subtype: autoservo */
#define PLAYER_PTZ_REQ_AUTOSERVO       3
/** Request/reply subtype: geometry */
#define PLAYER_PTZ_REQ_GEOM            4

/** Data subtype: state */
#define PLAYER_PTZ_DATA_STATE      1
/** Data subtype: geometry */
#define PLAYER_PTZ_DATA_GEOM       2

/** Command subtype: state */
#define PLAYER_PTZ_CMD_STATE       1

/** @brief Data: state (@ref PLAYER_PTZ_DATA_STATE)

The ptz interface returns data reflecting the current state of the
Pan-Tilt-Zoom unit. */
typedef struct player_ptz_data
{
  /** Pan [rad] */
  float pan;
  /** Tilt [rad] */
  float tilt;
  /** Field of view [rad] */
  float zoom;
  /** Current pan velocity [rad/s] */
  float panspeed;
  /** Current tilt velocity [rad/s] */
  float tiltspeed;
} player_ptz_data_t;

/** @brief Command: state (@ref PLAYER_PTZ_CMD_STATE)

The ptz interface accepts commands that set change the state of the unit.
Note that the commands are absolute, not relative. */
typedef struct player_ptz_cmd
{
  /** Desired pan angle [rad] */
  float pan;
  /** Desired tilt angle [rad] */
  float tilt;
  /** Desired field of view [rad]. */
  float zoom;
  /** Desired pan velocity [rad/s] */
  float panspeed;
  /** Desired tilt velocity [rad/s] */
  float tiltspeed;
} player_ptz_cmd_t;

/** @brief Request/reply: Query geometry.

To request ptz geometry, send a null @ref PLAYER_PTZ_REQ_GEOM request. */
typedef struct player_ptz_geom
{
  /** Pose of the ptz base*/
  player_pose3d_t pos;
  /** Dimensions of the base [m, m, m]. */
  player_bbox3d_t size;
} player_ptz_geom_t;

/** @brief Request/reply: Generic request

To send a unit-specific command to the unit, use the
@ref PLAYER_PTZ_REQ_GENERIC request.  Whether data is returned depends on the
command that was sent.  The device may fill in "config" with a reply if
applicable. */
typedef struct player_ptz_req_generic
{
  /** Length of data in config buffer */
  uint32_t  config_count;
  /** Buffer for command/reply */
  uint32_t  config[PLAYER_PTZ_MAX_CONFIG_LEN];
} player_ptz_req_generic_t;

/** @brief Request/reply: Control mode.

To switch between position and velocity control (for those drivers that
support it), send a @ref PLAYER_PTZ_REQ_CONTROL_MODE request.  Note that this
request changes how the driver interprets forthcoming commands from all
clients.  Null response. */
typedef struct player_ptz_req_control_mode
{
  /** Mode to use: must be either @ref PLAYER_PTZ_VELOCITY_CONTROL or
      @ref PLAYER_PTZ_POSITION_CONTROL. */
  uint32_t mode;
} player_ptz_req_control_mode_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_simulation simulation
 * @brief A robot simulator

Player devices may either be real hardware or virtual devices
generated by a simulator such as Stage or Gazebo.  This interface
provides direct access to a simulator.

This interface doesn't do much yet. It is in place to later support things
like pausing and restarting the simulation clock, saving and loading,
etc. It is documented because it is used by the stg_simulation driver;
required by all stageclient drivers (stg_*).

Note: the Stage and Gazebo developers should confer on the best design
for this interface. Suggestions welcome on playerstage-developers.
*/

/** @ingroup interface_simulation
 * @{ */

/** The maximum length of a string indentifying a simulation object or
    object property. */
#define PLAYER_SIMULATION_IDENTIFIER_MAXLEN 64

/** Request/reply subtype: set 2D pose */
#define PLAYER_SIMULATION_REQ_SET_POSE2D                         1
/** Request/reply subtype: get 2D pose */
#define PLAYER_SIMULATION_REQ_GET_POSE2D                         2
/** Request/reply subtype: set integer property value */
#define PLAYER_SIMULATION_REQ_SET_PROPERTY_INT                   3
/** Request/reply subtype: get integer property value */
#define PLAYER_SIMULATION_REQ_GET_PROPERTY_INT                   4
/** Request/reply subtype: set floating point property value */
#define PLAYER_SIMULATION_REQ_SET_PROPERTY_FLOAT                 5
/** Request/reply subtype: get floating point property value */
#define PLAYER_SIMULATION_REQ_GET_PROPERTY_FLOAT                 6
/** Request/reply subtype: set string property value */
#define PLAYER_SIMULATION_REQ_SET_PROPERTY_STRING                7
/** Request/reply subtype: get string property value */
#define PLAYER_SIMULATION_REQ_GET_PROPERTY_STRING                8

/** @brief Data

Just a placeholder for now; data will be added in future.
*/
typedef struct player_simulation_data
{
  /** A single byte of as-yet-unspecified data. Useful for experiments. */
  uint8_t data;
} player_simulation_data_t;

/** @brief Command

Just a placeholder for now; data will be added in future.
*/
typedef struct player_simulation_cmd
{
  /** A single byte of as-yet-unspecified command. Useful for experiments. */
  uint8_t cmd;
} player_simulation_cmd_t;

/** @brief Request/reply: get/set 2D pose of a named simulation object

To retrieve the pose of an object in a simulator, send a null
@ref PLAYER_SIMULATION_REQ_GET_POSE2D request.  To set the pose of an object
in a simulator, send a @ref PLAYER_SIMULATION_REQ_SET_POSE2D request (response
will be null). */
typedef struct player_simulation_pose2d_req
{
  /** Length of name */
  uint32_t name_count;
  /** the identifier of the object we want to locate */
  char name[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** the desired pose in (m, m, rad) */
  player_pose_t pose;
} player_simulation_pose2d_req_t;

/** @brief Request/reply: get/set integer property of a named simulation object

To retrieve an integer property of an object in a simulator, send a
@ref PLAYER_SIMULATION_REQ_GET_PROPERTY_INT request. The server will
reply with the integer value filled in. To set a integer property,
send a completely filled in @ref PLAYER_SIMULATION_REQ_SET_PROPERTY_INT
request. The server will respond with an ACK if the property was
successfully set to your value, else a NACK.  */
typedef struct player_simulation_property_int_req
{
  /** Length of name */
  uint32_t name_count;
  /** The identifier of the object we want to locate */
  char name[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** Length of property identifier */
  uint32_t prop_count;
  /** The identifier of the property we want to get/set */
  char prop[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** The value of the property */
  int32_t value;
} player_simulation_property_int_req_t;

/** @brief Request/reply: get/set floating-point property of a named
    simulation object

    Behaves identically to the integer version, but for double-precision
    floating-pont values. */
typedef struct player_simulation_property_float_req
{
  /** Length of name */
  uint32_t name_count;
  /** The identifier of the object we want to locate */
  char name[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** Length of property identifier */
  uint32_t prop_count;
  /** The identifier of the property we want to get/set */
  char prop[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** The value of the property */
  double value;
} player_simulation_property_float_req_t;

/** @brief Request/reply: get/set string property of a named
    simulation object

    Behaves identically to the integer version, but for strings.*/
typedef struct player_simulation_property_string_req
{
  /** Length of name */
  uint32_t name_count;
  /** The identifier of the object we want to locate */
  char name[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** Length of property identifier */
  uint32_t prop_count;
  /** The identifier of the property we want to get/set */
  char prop[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
  /** Length of the data string. */
  uint32_t value_count;
  /** The data string. */
  char value[PLAYER_SIMULATION_IDENTIFIER_MAXLEN];
} player_simulation_property_string_req_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_sonar sonar
 * @brief Array of ultrasonic rangers

The @p sonar interface provides access to a collection of fixed range
sensors, such as a sonar array.
*/

/** @ingroup interface_sonar
 * @{ */

/** maximum number of sonar samples in a data packet */
#define PLAYER_SONAR_MAX_SAMPLES 64

/** Request/reply subtype: get geometry */
#define PLAYER_SONAR_REQ_GET_GEOM    1
/** Request/reply subtype: power */
#define PLAYER_SONAR_REQ_POWER       2

/** Data subtype: ranges */
#define PLAYER_SONAR_DATA_RANGES     1
/** Data subtype: geometry */
#define PLAYER_SONAR_DATA_GEOM       2

/** @brief Data: ranges (@ref PLAYER_SONAR_DATA_RANGES)

The @p sonar interface returns up to @ref PLAYER_SONAR_MAX_SAMPLES range
readings from a robot's sonars. */
typedef struct player_sonar_data
{
  /** The number of valid range readings. */
  uint32_t ranges_count;
  /** The range readings [m] */
  float ranges[PLAYER_SONAR_MAX_SAMPLES];
} player_sonar_data_t;

/** @brief Data AND Request/reply: geometry.

To query the geometry of the sonar transducers, send a null
@ref PLAYER_SONAR_REQ_GET_GEOM request.  Depending on the underlying
driver, this message can also be sent as data with the subtype
@ref PLAYER_SONAR_DATA_GEOM. */
typedef struct player_sonar_geom
{
  /** The number of valid poses. */
  uint32_t poses_count;
  /** Pose of each sonar, in robot cs */
  player_pose_t poses[PLAYER_SONAR_MAX_SAMPLES];
} player_sonar_geom_t;

/** @brief Request/reply: Sonar power.

On some robots, the sonars can be turned on and off from software.
To do so, send a @ref PLAYER_SONAR_REQ_POWER request.  Null response. */
typedef struct player_sonar_power_config
{
  /** Turn power off TRUE or FALSE */
  uint8_t state;
} player_sonar_power_config_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_sound sound
 * @brief Play sound clips

The @p sound interface allows playback of a pre-recorded sound (e.g.,
on an Amigobot).
*/

/** @ingroup interface_sound
 * @{ */

/** Commmand subtype: play clip */
#define PLAYER_SOUND_CMD_IDX 1

/** @brief Command: play clip (@ref PLAYER_SOUND_CMD_IDX)

The @p sound interface accepts an index of a pre-recorded sound file
to play.  */
typedef struct player_sound_cmd
{
  /** Index of sound to be played. */
  uint32_t index;
} player_sound_cmd_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_speech speech
 * @brief Speech synthesis

The @p speech interface provides access to a speech synthesis system.
*/

/** @ingroup interface_speech
 * @{ */

/** Maximum string length */
#define PLAYER_SPEECH_MAX_STRING_LEN 256

/** Command subtype: say a string */
#define PLAYER_SPEECH_CMD_SAY 1

/** @brief Command: say a string (@ref PLAYER_SPEECH_CMD_SAY)

The @p speech interface accepts a command that is a string to
be given to the speech synthesizer.*/
typedef struct player_speech_cmd
{
  /** Length of string */
  uint32_t string_count;
  /** The string to say */
  char string[PLAYER_SPEECH_MAX_STRING_LEN];
} player_speech_cmd_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_speech_recognition speech_recognition
 * @brief Speech recognition

The speech recognition interface provides access to a speech recognition
server.
*/

/** @ingroup interface_speech_recognition
 * @{ */

/** Maximum length of string to recognize */
#define PLAYER_SPEECH_RECOGNITION_TEXT_LEN 256

/** Data subtype: recognized string */
#define PLAYER_SPEECH_RECOGNITION_DATA_STRING 1

/** @brief Data: recognized string (@ref PLAYER_SPEECH_MAX_STRING_LEN)

The speech recognition data packet.  */
typedef struct player_speech_recognition_data
{
  /** Length of text */
  uint32_t text_count;
  /** Recognized text */
  char text[PLAYER_SPEECH_RECOGNITION_TEXT_LEN];
} player_speech_recognition_data_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
@defgroup interface_truth truth (deprecated)
@brief Access to true state

@deprecated This interface has been superceded by the @ref
interface_simulation interface.

The @p truth interface provides access to the absolute state of entities.
Note that, unless your robot has superpowers, @p truth devices are
only avilable in simulation.
*/

/** @ingroup interface_truth
 * @{ */

/** Data subtype */
#define PLAYER_TRUTH_DATA_POSE 0x01
/** Data subtype */
#define PLAYER_TRUTH_DATA_FIDUCIAL_ID 0x02

/** Request subtypes. */
#define PLAYER_TRUTH_REQ_SET_POSE 0x03
/** Request subtypes. */
#define PLAYER_TRUTH_REQ_SET_FIDUCIAL_ID 0x04
/** Request subtypes. */
#define PLAYER_TRUTH_REQ_GET_FIDUCIAL_ID 0x05

/** @brief Data

The @p truth interface returns data concerning the current state of an
entity. */
typedef struct player_truth_pose
{
  /** Object pose in the world . */
  player_pose3d_t pose;
}  player_truth_pose_t;

/** @brief Configuration request: Get/set fiducial ID number.

To get the fiducial ID of an object, use the following request, filling
in only the subtype with @ref PLAYER_TRUTH_REQ_GET_FIDUCIAL_ID. The server will
respond with the ID field filled in.  To set the fiducial ID, set the
subtype to @ref PLAYER_TRUTH_REQ_SET_FIDUCIAL_ID and fill in the ID field with
the desired value. */
typedef struct player_truth_fiducial_id
{
  /** Packet subtype.  Must be either @ref PLAYER_TRUTH_REQ_GET_FIDUCIAL_ID or
    @ref PLAYER_TRUTH_REQ_SET_FIDUCIAL_ID */
  uint8_t subtype;
  /** the fiducial ID */
  int16_t id;
}  player_truth_fiducial_id_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_waveform waveform
 * @brief Digital waveforms

The @p waveform interface is used to receive arbitrary digital samples,
say from a digital audio device.
*/

/** @ingroup interface_waveform
 * @{ */

/** Maximum length of waveform */
#define PLAYER_WAVEFORM_DATA_MAX 4096

/** Data subtype: sample */
#define PLAYER_WAVEFORM_DATA_SAMPLE 1

/** @brief Data: sample (@ref PLAYER_WAVEFORM_DATA_SAMPLE)

The @p waveform interface reads a digitized waveform from the target
device.*/
typedef struct player_waveform_data
{
  /** Bit rate - bits per second */
  uint32_t rate;
  /** Depth - bits per sample */
  uint32_t depth;
  /** Samples - the number of bytes of raw data */
  uint32_t data_count;
  /** data - an array of raw data */
  uint8_t data[PLAYER_WAVEFORM_DATA_MAX];
} player_waveform_data_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_wifi wifi
 * @brief WiFi signal information

The @p wifi interface provides access to the state of a wireless network
interface.
*/

/** @ingroup interface_wifi
 * @{ */

/** The maximum number of remote hosts to report on */
#define PLAYER_WIFI_MAX_LINKS   32

/** link quality is in dBm */
#define PLAYER_WIFI_QUAL_DBM     1
/** link quality is relative */
#define PLAYER_WIFI_QUAL_REL     2
/** link quality is unknown */
#define PLAYER_WIFI_QUAL_UNKNOWN 3

/** unknown operating mode */
#define PLAYER_WIFI_MODE_UNKNOWN 0
/** driver decides the mode */
#define PLAYER_WIFI_MODE_AUTO    1
/** ad hoc mode */
#define PLAYER_WIFI_MODE_ADHOC   2
/** infrastructure mode (multi cell network, roaming) */
#define PLAYER_WIFI_MODE_INFRA   3
/** access point, master mode */
#define PLAYER_WIFI_MODE_MASTER  4
/** repeater mode */
#define PLAYER_WIFI_MODE_REPEAT  5
/** secondary/backup repeater */
#define PLAYER_WIFI_MODE_SECOND  6

/** Request/reply subtype: */
#define PLAYER_WIFI_MAC          1
/** Request/reply subtype: */
#define PLAYER_WIFI_IWSPY_ADD    2
/** Request/reply subtype: */
#define PLAYER_WIFI_IWSPY_DEL    3
/** Request/reply subtype: */
#define PLAYER_WIFI_IWSPY_PING   4

/** Data subtype: state */
#define PLAYER_WIFI_DATA_STATE 1

/** @brief Link information for one host.

The @p wifi interface returns data regarding the signal characteristics
of remote hosts as perceived through a wireless network interface; this
is the format of the data for each host. */
typedef struct player_wifi_link
{
  /** MAC address. */
  uint32_t mac_count;
  uint8_t mac[32];
  /** IP address. */
  uint32_t ip_count;
  uint8_t ip[32];
  /** ESSID. */
  uint32_t essid_count;
  uint8_t essid[32];
  /** Mode (master, adhoc, etc). */
  uint32_t mode;
  /** Frequency [MHz]. */
  uint32_t freq;
  /** Encryted?. */
  uint32_t encrypt;
  /** Link quality */
  uint32_t qual;
  /** Link level */
  uint32_t level;
  /** Link noise */
  uint32_t noise;
} player_wifi_link_t;

/** @brief Data: state (@ref PLAYER_WIFI_DATA_STATE)

The complete data packet format. */
typedef struct player_wifi_data
{
  /** length of said list */
  uint32_t links_count;
  /** A list of links */
  player_wifi_link_t links[PLAYER_WIFI_MAX_LINKS];
  /** mysterious throughput calculated by driver */
  uint32_t throughput;
  /** current bitrate of device */
  uint32_t bitrate;
  /** operating mode of device */
  uint32_t mode;
  /** Indicates type of link quality info we have */
  uint32_t qual_type;
  /** Maximum value for quality */
  uint32_t maxqual;
  /** Maximum value for level */
  uint32_t maxlevel;
  /** Maximum value for noise. */
  uint32_t maxnoise;
  /** MAC address of current access point/cell */
  char ap[32];
} player_wifi_data_t;

/** @brief Request/reply: */
typedef struct player_wifi_mac_req
{
  /** MAC address. */
  uint32_t mac_count;
  uint8_t mac[32];
} player_wifi_mac_req_t;

/** @brief Request/reply: */
typedef struct player_wifi_iwspy_addr_req
{
  /** Address ?? */
  char      address[32];
} player_wifi_iwspy_addr_req_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_rfid rfid
 * @brief RFID reader

The RFID interface provides access to a RFID reader (driver implementations
include RFID readers such as Skyetek M1 and Inside M300).
*/

/** @ingroup interface_rfid
 * @{ */

/** The maximum nr of tags that can be detected in the RF field in total. */
#define PLAYER_RFID_MAX_TAGS 30
/** The maximum number of digits one RFID tag can have. */
#define PLAYER_RFID_MAX_GUID 8

/** Data subtype */
#define PLAYER_RFID_DATA         1

/** Request/reply: put the reader in sleep mode (0) or wake it up (1). */
#define PLAYER_RFID_REQ_POWER    1
/** Request/reply: read data from the RFID tag - to be implemented.    */
#define PLAYER_RFID_REQ_READTAG  2
/** Request/reply: write data to the RFID tag - to be implemented.     */
#define PLAYER_RFID_REQ_WRITETAG 3
/** Request/reply: lock data blocks of a RFID tag - to be implemented. */
#define PLAYER_RFID_REQ_LOCKTAG  4

/** @brief Structure describing a single RFID tag. */
typedef struct player_rfid_tag
{
  /** Tag type. */
  uint32_t type;
  /** GUID count. */
  uint32_t guid_count;
  /** The Globally Unique IDentifier (GUID) of the tag. */
  char guid[PLAYER_RFID_MAX_GUID];
} player_rfid_tag_t;

/** @brief Data

The RFID data packet.  */
typedef struct player_rfid_data
{
  /** The number of RFID tags found. */
  uint32_t tags_count;
  /** The list of RFID tags. */
  player_rfid_tag_t tags[PLAYER_RFID_MAX_TAGS];
} player_rfid_data_t;

/** @brief Command

Just a placeholder for now; data will be added in future.
*/
typedef struct player_rfid_cmd
{
  /** A single byte of as-yet-unspecified command. Useful for experiments. */
  uint8_t temp;
} player_rfid_cmd_t;

/** @} */

// /////////////////////////////////////////////////////////////////////////////
/** @ingroup interfaces
 * @defgroup interface_wsn wsn
 * @brief Wireless Sensor Networks

The WSN interface provides access to a Wireless Sensor Network (driver
implementations include WSN's such as Crossbow's MICA2 motes and TeCo's RCore
particles).

The current implementation supports a single group of network nodes. Support
for multiple groups will be added in the future.
 */

/** @ingroup interface_wsn
 * @{ */

/** Data subtypes                                                               */
#define PLAYER_WSN_DATA           1

/** Command subtype: set device state                                           */
#define PLAYER_WSN_CMD_DEVSTATE   1

/** Request/reply: put the node in sleep mode (0) or wake it up (1).            */
#define PLAYER_WSN_REQ_POWER      1
/** Request/reply: change the data type to RAW or converted metric units.       */
#define PLAYER_WSN_REQ_DATATYPE   2
/** Request/reply: change the receiving data frequency.                         */
#define PLAYER_WSN_REQ_DATAFREQ   3

/** @brief Structure describing the WSN node's data packet.                     */
typedef struct player_wsn_node_data
{
	/** The node's light measurement from a light sensor.                   */
	float light;
	/** The node's accoustic measurement from a microphone.                 */
	float mic;
	/** The node's acceleration on X-axis from an acceleration sensor.      */
	float accel_x;
	/** The node's acceleration on y-axis from an acceleration sensor.      */
	float accel_y;
	/** The node's acceleration on Z-axis from an acceleration sensor.      */
	float accel_z;
	/** The node's magnetic measurement on X-axis from a magnetometer.      */
	float magn_x;
	/** The node's magnetic measurement on Y-axis from a magnetometer.      */
	float magn_y;
	/** The node's magnetic measurement on Z-axis from a magnetometer.      */
	float magn_z;
	/** The node's templerature measurement from a temperature sensor.      */
	float temperature;
	/** The node's remaining battery voltage.                               */
	float battery;
} player_wsn_node_data_t;

/** @brief Data (@ref PLAYER_WSN_DATA)

The WSN data packet describes a wireless sensor network node.                   */
typedef struct player_wsn_data
{
	/** The type of WSN node.                                               */
	uint32_t node_type;
	/** The ID of the WSN node.                                             */
	uint32_t node_id;
	/** The ID of the WSN node's parent (if existing).                      */
	uint32_t node_parent_id;
	/** The WSN node's data packet.                                         */
	player_wsn_node_data_t data_packet;
} player_wsn_data_t;

/** @brief Command: set device state (@ref PLAYER_WSN_CMD_DEVSTATE)
This @p wsn command sets the state of the node's indicator lights or
its buzzer/sounder (if equipped with one).                                      */
typedef struct player_wsn_cmd
{
	/** The ID of the WSN node. -1 for all.                                 */
	int32_t node_id;
	/** The Group ID of the WSN node. -1 for all.                           */
	int32_t group_id;
	/** The device number.                                                  */
	uint32_t device;
	/** The state: 0=disabled, 1=enabled.                                   */
	uint8_t enable;
} player_wsn_cmd_t;

/** @brief Request/reply: Put the node in sleep mode (0) or wake it up (1).

Send a @ref PLAYER_WSN_REQ_POWER request to power or wake up a node in the WSN.
Null response.                                                                  */
typedef struct player_wsn_power_config
{
	/** The ID of the WSN node. -1 for all.                                 */
	int32_t node_id;
	/** The Group ID of the WSN node. -1 for all.                           */
	int32_t group_id;
	/** Power setting: 0 for off, 1 for on.                                 */
	uint8_t value;
} player_wsn_power_config_t;

/** @brief Request/reply: change the data type to RAW or converted engineering
units.

Send a @ref PLAYER_WSN_REQ_DATATYPE request to switch between RAW or converted
engineering units values in the data packet. Null response.                     */
typedef struct player_wsn_datatype_config
{
	/** Data type setting: 0 for RAW values, 1 for converted units.         */
	uint8_t value;
} player_wsn_datatype_config_t;

/** @brief Request/reply: Change data delivery frequency.

By default, the frequency set for receiving data is set on the wireless node.
Send a @ref PLAYER_WSN_REQ_DATAFREQ request to change the frequency. Fill in
the node ID or set -1 for all nodes. Null response.                             */
typedef struct player_wsn_datafreq_config
{
	/** The ID of the WSN node. -1 for all.                                 */
	int32_t node_id;
	/** The Group ID of the WSN node. -1 for all.                           */
	int32_t group_id;
	/** Requested frequency in Hz.                                          */
	double  frequency;
} player_wsn_datafreq_config_t;
/** @} */

#endif /* PLAYER_H */
