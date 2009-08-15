/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2003  John Sweeney & Brian Gerkey
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
  Desc: Driver for the Segway RMP
  Author: John Sweeney and Brian Gerkey
  Date:
  CVS: $Id: segwayrmp.cc,v 1.40.2.4 2007/04/19 16:40:55 gerkey Exp $
*/

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_segwayrmp segwayrmp
 * @brief Segway RMP mobile robot

@todo This driver is currently disabled because it needs to be updated to
the Player 2.0 API.

The driver has been updated to the point that it compiles, but a few key requests
have been disabled and it has not been tested in anyway.

The segwayrmp driver provides control of a Segway RMP (Robotic
Mobility Platform), which is an experimental robotic version of the
Segway HT (Human Transport), a kind of two-wheeled, self-balancing
electric scooter.

@par Compile-time dependencies

- libcanlib (from Kvaser)

@par Notes

- Because of its power, weight, height, and dynamics, the Segway RMP is
a potentially dangerous machine.  Be @e very careful with it.

- Although the RMP does not actually support motor power control from 
software, for safety you must explicitly enable the motors using a
@p PLAYER_POSITION_MOTOR_POWER_REQ or @p PLAYER_POSITION3D_MOTOR_POWER_REQ
(depending on which interface you are using).  You must @e also enable
the motors in the command packet, by setting the @p state field to 1.

- For safety, this driver will stop the RMP (i.e., send zero velocities)
if no new command has been received from a client in the previous 400ms or so.
Thus, even if you want to continue moving at a constant velocity, you must
continuously send your desired velocities.

- Most of the configuration requests have not been tested.

- Currently, the only supported type of CAN I/O is "kvaser", which uses
Kvaser, Inc.'s CANLIB interface library.  This library provides access
to CAN cards made by Kvaser, such as the LAPcan II.  However, the CAN
I/O subsystem within this driver is modular, so that it should be pretty
straightforward to add support for other CAN cards.


@par Provides

- @ref interface_position2d
  - This interface returns odometry data, and accepts velocity commands.

- @ref interface_position3d
  - This interface returns odometry data (x, y and yaw) from the wheel
  encoders, and attitude data (pitch and roll) from the IMU.  The
  driver accepts velocity commands (x vel and yaw vel).

- @ref interface_power
  - Returns the current battery voltage (72 V when fully charged).

@par Configuration requests

- position interface
  - PLAYER_POSITION_POWER_REQ

- position3d interface
  - PLAYER_POSITION_POWER_REQ

@par Requires

- none

@par Configuration file options

- canio (string)
  - Default: "kvaser"
  - Type of CANbus driver.  Other option is "rmpusb".

- max_xspeed (length / sec)
  - Default: 0.5 m/s
  - Maximum linear speed

- max_yawspeed (angle / sec)
  - Default: 40 deg/sec
  - Maximum angular speed
      
@par Example 

@verbatim
driver
(
  name "segwayrmp"
  provides ["position:0" "position3d:0" "power:0"]
)
@endverbatim

@author John Sweeney, Brian Gerkey, Andrew Howard
*/
/** @} */

  
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>

#define PLAYER_ENABLE_MSG 0


#include "rmp_frame.h"
#include "segwayrmp.h"
#include <libplayercore/playercore.h>


// Number of RMP read cycles, without new speed commands from clients,
// after which we'll stop the robot (for safety).  The read loop
// seems to run at about 50Hz, or 20ms per cycle.
#define RMP_TIMEOUT_MSECS 800


////////////////////////////////////////////////////////////////////////////////
// A factory creation function
Driver* SegwayRMP_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
  return ((Driver*) (new SegwayRMP(cf, section)));
}


////////////////////////////////////////////////////////////////////////////////
// A driver registration function.
void SegwayRMP_Register(DriverTable* table)
{
  table->AddDriver("segwayrmp", SegwayRMP_Init);
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
SegwayRMP::SegwayRMP(ConfigFile* cf, int section)
    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
  memset(&this->position_id, 0, sizeof(player_devaddr_t));
  memset(&this->position3d_id, 0, sizeof(player_devaddr_t));
  memset(&this->power_id, 0, sizeof(player_devaddr_t));

  // Do we create a position interface?
  if(cf->ReadDeviceAddr(&(this->position_id), section, "provides", 
                      PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position_id) != 0)
    {
      this->SetError(-1);    
      return;
    }
  }

  // Do we create a position3d interface?
  if(cf->ReadDeviceAddr(&(this->position3d_id), section, "provides", 
                        PLAYER_POSITION3D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position3d_id) != 0)
    {
      this->SetError(-1);    
      return;
    }
  }

  // Do we create a power interface?
  if(cf->ReadDeviceAddr(&(this->power_id), section, "provides", 
                        PLAYER_POWER_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->power_id) != 0)
    {
      this->SetError(-1);    
      return;
    }
  }

  this->canio = NULL;
  this->caniotype = cf->ReadString(section, "canio", "kvaser");
  this->caniochannel = cf->ReadInt(section,"canio_channel",0);
  this->max_xspeed = (int) (1000.0 * cf->ReadLength(section, "max_xspeed", 0.5));
  if(this->max_xspeed < 0)
    this->max_xspeed = -this->max_xspeed;
  this->max_yawspeed = (int) (RTOD(cf->ReadAngle(section, "max_yawspeed", 40)));
  if(this->max_yawspeed < 0)
    this->max_yawspeed = -this->max_yawspeed;
  
  this->length = cf->ReadLength(section, "length", 0.610);
  this->width  = cf->ReadLength(section, "width", 0.508);

  return;
}


SegwayRMP::~SegwayRMP()
{
  return;
}

int
SegwayRMP::Setup()
{
   printf("SEGWAYRMP: Modified by Garratt\n");
  // Clear the command buffers
#if 0
  if (this->position_id.code)
    ClearCommand(this->position_id);
  if (this->position3d_id.code)
    ClearCommand(this->position3d_id);
#endif

  PLAYER_MSG0(2, "CAN bus initializing");

  if(!strcmp(this->caniotype, "kvaser"))
    assert(this->canio = new CANIOKvaser);
  else if (!strcmp(this->caniotype,"rmpusb"))
      assert(this->canio = new CANIOrmpusb);
  else
  {
    PLAYER_ERROR1("Unknown CAN I/O type: \"%s\"", this->caniotype);
    return(-1);
  }

  // start the CAN at 500 kpbs
  if(this->canio->Init() < 0) 
  {
    PLAYER_ERROR("error on CAN Init");
    return(-1);
  }

  // Initialize odometry
  this->odom_x = this->odom_y = this->odom_yaw = 0.0;

  this->curr_xspeed = this->curr_yawspeed = 0.0;
  this->motor_allow_enable = false;
  this->motor_enabled = false;
  this->firstread = true;
  // set the initial time
  gettimeofday(&(this->lasttime),NULL);

  this->StartThread();

  return(0);
}

int
SegwayRMP::Shutdown()
{
  PLAYER_MSG0(2, "Shutting down CAN bus");
  fflush(stdout);
  
  // TODO: segfaulting in here somewhere on client disconnect, but only 
  // sometimes.  
  //
  // UPDATE: This might have been fixed by moving the call to StopThread()
  // to before the sending of zero velocities.   There could have been
  // a race condition, since Shutdown() is called from the server's thread 
  // context.

  StopThread();
  
  // send zero velocities, for some semblance of safety
  CanPacket pkt;

  MakeVelocityCommand(&pkt,0,0);
  
  Write(pkt);

  // shutdown the CAN
  canio->Shutdown();
  delete canio;
  canio = NULL;
  
  return(0);
}

// Main function for device thread.
void 
SegwayRMP::Main()
{
   printf("SEGWAYRMP: Modified by Garratt\n");
  //unsigned char buffer[256];
  //size_t buffer_len;
  //player_position2d_cmd_t position_cmd;
  //player_position3d_cmd_t position3d_cmd;
  //void *client;
  CanPacket pkt;
  //int32_t xspeed,yawspeed;
  //bool got_command;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);


  PLAYER_MSG0(2, "starting main loop");
  
  for(;;)
  {
    pthread_testcancel();
    ProcessMessages(); 
    // Read from the RMP
    int ret=ReadFrame();
    if (ret < 0) {
        PLAYER_ERROR("ReadFrame() errored; bailing");
        pthread_exit(NULL);
    }

    if (ret == 1) {  // received a full frame of state info
        // Send data to clients
        Publish(this->position_id, NULL, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
                &this->position_data, sizeof(this->position_data), NULL);
        Publish(this->position3d_id, NULL, PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
                &this->position3d_data, sizeof(this->position3d_data), NULL);
        Publish(this->power_id, NULL, PLAYER_MSGTYPE_DATA, PLAYER_POWER_DATA_STATE,
                &this->power_data, sizeof(this->power_data), NULL);
    }

    // check to see how long since the last command
    struct timeval curr;
    // get the current time
    gettimeofday(&curr,NULL);

    // calculate how long since the last command
    double msecs = (curr.tv_sec - this->lasttime.tv_sec)*1000 + (curr.tv_usec - this->lasttime.tv_usec) / 1000;
    
    if (msecs > RMP_TIMEOUT_MSECS) {
        if(curr_xspeed || curr_yawspeed) {
            PLAYER_WARN1("timeout exceeded: %d msecs since last command stopping robot", (int) msecs);
            curr_xspeed = 0;
            curr_yawspeed = 0;
        }
    }
    
    if(!motor_enabled) 
    {
      curr_xspeed = 0;
      curr_yawspeed = 0;
    }

    // make a velocity command... could be zero
    MakeVelocityCommand(&pkt,static_cast<int> (curr_xspeed),static_cast<int> (curr_yawspeed));
    if(Write(pkt) < 0) {
        PLAYER_ERROR("error on write");
    }
    usleep(500);
  }
}

int
SegwayRMP::ProcessMessage(MessageQueue * resp_queue,
                          player_msghdr * hdr,
                          void * data)
{
  /// @todo
  /// Handle config requests
  
  // 2-D velocity command
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
                           PLAYER_POSITION2D_CMD_VEL, 
                           this->position_id))
  {

    player_position2d_cmd_vel_t* cmd = (player_position2d_cmd_vel_t*)data;
    this->curr_xspeed = cmd->vel.px * 1000.0; //Added multiply by 1000.0, BMS.
    this->curr_yawspeed = cmd->vel.pa * 180.0/M_PI;  //Added multiply by 1000.0, BMS.
    this->motor_enabled = cmd->state & this->motor_allow_enable;
//    printf("SEGWAYRMP: motors given command: %f  %f %d\n", cmd->vel.px, cmd->vel.pa,(int)this->motor_enabled);
//    printf("SEGWAYRMP: motors state: %d\n", this->motor_enabled);
    gettimeofday(&(this->lasttime),NULL);
    return 0;
  }
  // 3-D velocity command
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
                           PLAYER_POSITION3D_CMD_SET_VEL,
                           this->position3d_id))
  {
    player_position3d_cmd_vel_t* cmd = (player_position3d_cmd_vel_t*)data;
    this->curr_xspeed = cmd->vel.px * 1000.0; //Added multiply by 1000.0, BMS.
    this->curr_yawspeed = cmd->vel.pyaw * 180.0/M_PI; ////Added multiply by 1000.0, BMS.
    this->motor_enabled = cmd->state & this->motor_allow_enable;
    gettimeofday(&(this->lasttime),NULL);
    return 0;
  }

  if (hdr->type == PLAYER_MSGTYPE_REQ && hdr->addr.interf == position_id.interf
          && hdr->addr.index == position_id.index)
  {
    return HandlePositionConfig(resp_queue, hdr->subtype, data, hdr->size);
  }

  if (hdr->type == PLAYER_MSGTYPE_REQ && hdr->addr.interf == position3d_id.interf
          && hdr->addr.index == position3d_id.index)
  {
    return HandlePosition3DConfig(resp_queue, hdr->subtype, data, hdr->size);
  }



  return(-1);
}
  
// helper to handle config requests
// returns 1 to indicate we wrote to the CAN bus
// returns 0 to indicate we did NOT write to CAN bus
int
SegwayRMP::HandlePositionConfig(MessageQueue* client, uint32_t subtype, void* buffer, size_t len)
{
  uint16_t rmp_cmd,rmp_val;
  //player_rmp_config_t *rmp;
  CanPacket pkt;
  
  switch(subtype) 
  {
    case PLAYER_POSITION2D_REQ_MOTOR_POWER:
      // just set a flag telling us whether we should
      // act on motor commands
      // set the commands to 0... think it will automatically
      // do this for us.  
      if(((char *) buffer)[0]) 
        this->motor_allow_enable = true;
      else
        this->motor_allow_enable = false;

      printf("SEGWAYRMP: motors state: %d\n", this->motor_allow_enable);

      Publish(position_id, client, PLAYER_MSGTYPE_RESP_ACK,subtype);
      return 0;

    case PLAYER_POSITION2D_REQ_GET_GEOM:
      player_position2d_geom_t geom;
      geom.pose.px = 0;
      geom.pose.py = 0;
      geom.pose.pa = 0;
      geom.size.sw = this->width;
      geom.size.sl = this->length;

      Publish(position_id, client, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM, &geom, sizeof(geom),NULL);
      return 0;

    case PLAYER_POSITION2D_REQ_RESET_ODOM:
      // we'll reset all the integrators

      MakeStatusCommand(&pkt, (uint16_t)RMP_CAN_CMD_RST_INT, 
                        (uint16_t)RMP_CAN_RST_ALL);
      if (Write(pkt) < 0) {
          Publish(position_id, client, PLAYER_MSGTYPE_RESP_NACK,PLAYER_POSITION2D_REQ_RESET_ODOM);
      } else {
          Publish(position_id, client, PLAYER_MSGTYPE_RESP_ACK,PLAYER_POSITION2D_REQ_RESET_ODOM);
      }

      odom_x = odom_y = odom_yaw = 0.0;
      firstread = true;

      // return 1 to indicate that we wrote to the CAN bus this time
      return(0);

/*    case PLAYER_POSITION_RMP_VELOCITY_SCALE:
      rmp_cmd = RMP_CAN_CMD_MAX_VEL;
      rmp = (player_rmp_config_t *)buffer;
      rmp_val = ntohs(rmp->value);

      printf("SEGWAYRMP: velocity scale %d\n", rmp_val);

      MakeStatusCommand(&pkt, rmp_cmd, rmp_val);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);

    case PLAYER_POSITION_RMP_ACCEL_SCALE:
      rmp_cmd = RMP_CAN_CMD_MAX_ACCL;
      rmp = (player_rmp_config_t *)buffer;
      rmp_val = ntohs(rmp->value);

      MakeStatusCommand(&pkt, rmp_cmd, rmp_val);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);

    case PLAYER_POSITION_RMP_TURN_SCALE:
      rmp_cmd = RMP_CAN_CMD_MAX_TURN;
      rmp = (player_rmp_config_t *)buffer;
      rmp_val = ntohs(rmp->value);

      MakeStatusCommand(&pkt, rmp_cmd, rmp_val);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);

    case PLAYER_POSITION_RMP_GAIN_SCHEDULE:
      rmp_cmd = RMP_CAN_CMD_GAIN_SCHED;
      rmp = (player_rmp_config_t *)buffer;
      rmp_val = ntohs(rmp->value);

      MakeStatusCommand(&pkt, rmp_cmd, rmp_val);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);

    case PLAYER_POSITION_RMP_CURRENT_LIMIT:
      rmp_cmd = RMP_CAN_CMD_CURR_LIMIT;
      rmp = (player_rmp_config_t *)buffer;
      rmp_val = ntohs(rmp->value);

      MakeStatusCommand(&pkt, rmp_cmd, rmp_val);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);

    case PLAYER_POSITION_RMP_RST_INTEGRATORS:
      rmp_cmd = RMP_CAN_CMD_RST_INT;
      rmp = (player_rmp_config_t *)buffer;
      rmp_val = ntohs(rmp->value);

      MakeStatusCommand(&pkt, rmp_cmd, rmp_val);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if (Write(pkt) < 0) {
          if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
            PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
        } else {
          if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
            PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
        }
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);

    case PLAYER_POSITION_RMP_SHUTDOWN:
      MakeShutdownCommand(&pkt);

      if(Write(pkt) < 0)
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      else
      {
        if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK,NULL))
          PLAYER_ERROR("SEGWAY: Failed to PutReply\n");
      }
      // return 1 to indicate that we wrote to the CAN bus this time
      return(1);
*/
    default:
      printf("segwayrmp received unknown config request %d\n", 
             subtype);
/*      if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL))
        PLAYER_ERROR("Failed to PutReply in segwayrmp\n");*/
      break;
  }

  // return -1, to indicate that we did NOT handle the message
  return(-1);
}

// helper to handle config requests
// returns 1 to indicate we wrote to the CAN bus
// returns 0 to indicate we did NOT write to CAN bus
int
SegwayRMP::HandlePosition3DConfig(MessageQueue* client, uint32_t subtype, void* buffer, size_t len)
{
  switch(subtype) 
  {
    case PLAYER_POSITION3D_MOTOR_POWER:
      // just set a flag telling us whether we should
      // act on motor commands
      // set the commands to 0... think it will automatically
      // do this for us.  
      if(((char*)buffer)[0]) 
        this->motor_allow_enable = true;
      else
        this->motor_allow_enable = false;

      printf("SEGWAYRMP: motors state: %d\n", this->motor_allow_enable);

      Publish(this->position3d_id, client, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_MOTOR_POWER);
      return 0;

    default:
      printf("segwayrmp received unknown config request %d\n", 
             subtype);
  }

  // return -1, to indicate that we did process the message
  return(-1);
}

int SegwayRMP::ReadFrame() {
    CanPacket pkt[CANIOrmpusb::MAX_EXTRACTED_PACKETS];
    rmp_frame_t data_frame;
    data_frame.ready = 0;
    
    int packets=canio->ReadFrame(pkt);
    if (packets < 0) {
        return -1;
    }
    while (packets>0) {
        data_frame.AddPacket(pkt[packets-1]);
        packets--;
    }
    if(data_frame.IsReady()) 
    {
      UpdateData(&data_frame);
      data_frame.ready = 0;
      return 1;
    }
    return 0;
}

int
SegwayRMP::Read()
{
    CanPacket pkt;
  int ret;
  rmp_frame_t data_frame;
  
  //static struct timeval last;
  //struct timeval curr;

  data_frame.ready = 0;

  ret=0;
  // read until we've gotten all 8 packets for this cycle on this channel
  while((ret = canio->ReadPacket(&pkt)) > 0)
  {
    // then we got a new packet...
    //printf("SEGWAYIO: pkt: %s\n", pkt.toString());

    data_frame.AddPacket(pkt);

    // if data has been filled in, then let's make it the latest data 
    // available to player...
    if(data_frame.IsReady()) 
    {
      UpdateData(&data_frame);
      data_frame.ready = 0;
      break;
    }
  }
  if (ret < 0) 
  {
    PLAYER_ERROR2("error (%d) reading packet on channel %d",
                  ret, this->caniochannel);
  }

  return(0);
}

void
SegwayRMP::UpdateData(rmp_frame_t * data_frame)
{
  int delta_lin_raw, delta_ang_raw;
  double delta_lin, delta_ang;
  double tmp;

  // Get the new linear and angular encoder values and compute
  // odometry.  Note that we do the same thing here, regardless of 
  // whether we're presenting 2D or 3D position info.
  delta_lin_raw = Diff(this->last_raw_foreaft,
                       data_frame->foreaft,
                       this->firstread);
  this->last_raw_foreaft = data_frame->foreaft;
  
  delta_ang_raw = Diff(this->last_raw_yaw,
                       data_frame->yaw,
                       this->firstread);
  this->last_raw_yaw = data_frame->yaw;
  
  delta_lin = (double)delta_lin_raw / (double)RMP_COUNT_PER_M;
  delta_ang = DTOR((double)delta_ang_raw / 
                   (double)RMP_COUNT_PER_REV * 360.0);
  
  // First-order odometry integration
  this->odom_x += delta_lin * cos(this->odom_yaw);
  this->odom_y += delta_lin * sin(this->odom_yaw);
  this->odom_yaw += delta_ang;
  
  // Normalize yaw in [0, 360]
  this->odom_yaw = atan2(sin(this->odom_yaw), cos(this->odom_yaw));
  if (this->odom_yaw < 0)
    this->odom_yaw += 2 * M_PI;
  
  // first, do 2D info.
  this->position_data.pos.px = this->odom_x;
  this->position_data.pos.py = this->odom_y;
  this->position_data.pos.pa = this->odom_yaw;
  
  // combine left and right wheel velocity to get foreward velocity
  // change from counts/s into mm/s
  this->position_data.vel.px = ((double)data_frame->left_dot +
                          (double)data_frame->right_dot) /
                         (double)RMP_COUNT_PER_M_PER_S 
                         / 2.0;
  
  // no side speeds for this bot
  this->position_data.vel.py = 0;
  
  // from counts/sec into deg/sec.  also, take the additive
  // inverse, since the RMP reports clockwise angular velocity as
  // positive.
  this->position_data.vel.pa = 
          DTOR(-(double)data_frame->yaw_dot / (double)RMP_COUNT_PER_DEG_PER_S);
  
  this->position_data.stall = 0;
  
  // now, do 3D info.
  this->position3d_data.pos.px = this->odom_x;
  this->position3d_data.pos.py = this->odom_y;
  // this robot doesn't fly
  this->position3d_data.pos.pz = 0;

  /// TODO left off here
  
  // normalize angles to [0,360]
  tmp = NORMALIZE(DTOR((double)data_frame->roll /
                       (double)RMP_COUNT_PER_DEG));
  if(tmp < 0)
    tmp += 2*M_PI;
  this->position3d_data.pos.proll = tmp;//htonl((int32_t)rint(tmp * 1000.0));
  
  // normalize angles to [0,360]
  tmp = NORMALIZE(DTOR((double)data_frame->pitch /
                       (double)RMP_COUNT_PER_DEG));
  if(tmp < 0)
    tmp += 2*M_PI;
  this->position3d_data.pos.ppitch = tmp;//htonl((int32_t)rint(tmp * 1000.0));
  
  this->position3d_data.pos.pyaw = tmp;//htonl(((int32_t)(this->odom_yaw * 1000.0)));
  
  // combine left and right wheel velocity to get foreward velocity
  // change from counts/s into m/s
  this->position3d_data.vel.px = 
    ((double)data_frame->left_dot +
                          (double)data_frame->right_dot) /
                         (double)RMP_COUNT_PER_M_PER_S 
                          / 2.0;
  // no side or vertical speeds for this bot
  this->position3d_data.vel.py = 0;
  this->position3d_data.vel.pz = 0;
  
  this->position3d_data.vel.proll = 
    (double)data_frame->roll_dot /
                        (double)RMP_COUNT_PER_DEG_PER_S * M_PI / 180;
  this->position3d_data.vel.ppitch = 
    (double)data_frame->pitch_dot /
                        (double)RMP_COUNT_PER_DEG_PER_S * M_PI / 180;
  // from counts/sec into millirad/sec.  also, take the additive
  // inverse, since the RMP reports clockwise angular velocity as
  // positive.

  // This one uses left_dot and right_dot, which comes from odometry
  this->position3d_data.vel.pyaw = 
    (double)(data_frame->right_dot - data_frame->left_dot) /
                         (RMP_COUNT_PER_M_PER_S * RMP_GEOM_WHEEL_SEP * M_PI);
  // This one uses yaw_dot, which comes from the IMU
  //data.position3d_data.yawspeed = 
  //  htonl((int32_t)(-rint((double)data_frame->yaw_dot / 
  //                        (double)RMP_COUNT_PER_DEG_PER_S * M_PI / 180 * 1000)));
  
  this->position3d_data.stall = 0;
  
  // fill in power data.  the RMP returns a percentage of full,
  // and the specs for the HT say that it's a 72 volt system.  assuming
  // that the RMP is the same, we'll convert to decivolts for Player.
  this->power_data.volts = 
    data_frame->battery * 72;
  
  firstread = false;  
}  


int
SegwayRMP::Write(CanPacket& pkt)
{
  return(canio->WritePacket(pkt));
}

/* Creates a status CAN packet from the given arguments
 */  
void
SegwayRMP::MakeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val)
{
  int16_t trans,rot;

  pkt->id = RMP_CAN_ID_COMMAND;
  pkt->PutSlot(2, cmd);
 
  // it was noted in the windows demo code that they
  // copied the 8-bit value into both bytes like this
  pkt->PutByte(6, val);
  pkt->PutByte(7, val);

  trans = (int16_t) rint((double)this->curr_xspeed * 
                         (double)RMP_COUNT_PER_MM_PER_S);

  if(trans > RMP_MAX_TRANS_VEL_COUNT)
    trans = RMP_MAX_TRANS_VEL_COUNT;
  else if(trans < -RMP_MAX_TRANS_VEL_COUNT)
    trans = -RMP_MAX_TRANS_VEL_COUNT;

  rot = (int16_t) rint((double)this->curr_yawspeed * 
                       (double)RMP_COUNT_PER_DEG_PER_SS);

  if(rot > RMP_MAX_ROT_VEL_COUNT)
    rot = RMP_MAX_ROT_VEL_COUNT;
  else if(rot < -RMP_MAX_ROT_VEL_COUNT)
    rot = -RMP_MAX_ROT_VEL_COUNT;

  // put in the last speed commands as well
  pkt->PutSlot(0,(uint16_t)trans);
  pkt->PutSlot(1,(uint16_t)rot);
  
  if(cmd) 
  {
    printf("SEGWAYIO: STATUS: cmd: %02x val: %02x pkt: %s\n", 
	   cmd, val, pkt->toString());
  }
}

/* takes a player command (in host byte order) and turns it into CAN packets 
 * for the RMP 
 */
void
SegwayRMP::MakeVelocityCommand(CanPacket* pkt, 
                               int32_t xspeed, 
                               int32_t yawspeed)
{
  pkt->id = RMP_CAN_ID_COMMAND;
  pkt->PutSlot(2, (uint16_t)RMP_CAN_CMD_NONE);
  
  // we only care about cmd.xspeed and cmd.yawspeed
  // translational velocity is given to RMP in counts 
  // [-1176,1176] ([-8mph,8mph])

  // player is mm/s
  // 8mph is 3576.32 mm/s
  // so then mm/s -> counts = (1176/3576.32) = 0.32882963

  if(xspeed > this->max_xspeed)
  {
    PLAYER_WARN2("xspeed thresholded! (%d > %d)", xspeed, this->max_xspeed);
    xspeed = this->max_xspeed;
  }
  else if(xspeed < -this->max_xspeed)
  {
    PLAYER_WARN2("xspeed thresholded! (%d < %d)", xspeed, -this->max_xspeed);
    xspeed = -this->max_xspeed;
  }

  this->curr_xspeed = xspeed;

  int16_t trans = (int16_t) rint((double)xspeed * 
                                 (double)RMP_COUNT_PER_MM_PER_S);

  if(trans > RMP_MAX_TRANS_VEL_COUNT)
    trans = RMP_MAX_TRANS_VEL_COUNT;
  else if(trans < -RMP_MAX_TRANS_VEL_COUNT)
    trans = -RMP_MAX_TRANS_VEL_COUNT;

  if(yawspeed > this->max_yawspeed)
  {
    PLAYER_WARN2("yawspeed thresholded! (%d > %d)", 
                 yawspeed, this->max_yawspeed);
    yawspeed = this->max_yawspeed;
  }
  else if(yawspeed < -this->max_yawspeed)
  {
    PLAYER_WARN2("yawspeed thresholded! (%d < %d)", 
                 yawspeed, -this->max_yawspeed);
    yawspeed = -this->max_yawspeed;
  }
  this->curr_yawspeed = yawspeed;

  // rotational RMP command \in [-1024, 1024]
  // this is ripped from rmi_demo... to go from deg/s to counts
  // deg/s -> count = 1/0.013805056
  int16_t rot = (int16_t) rint((double)yawspeed * 
                               (double)RMP_COUNT_PER_DEG_PER_S);

  if(rot > RMP_MAX_ROT_VEL_COUNT)
    rot = RMP_MAX_ROT_VEL_COUNT;
  else if(rot < -RMP_MAX_ROT_VEL_COUNT)
    rot = -RMP_MAX_ROT_VEL_COUNT;

  // We've noticed that the Segway can't turn in place with a pure commanded
  // rotation; this is an attempt to help.
  // From measurements, we found that adding an extra translation of 22 of the
  // same sign as the rotation seems to help.
  if (rot > 0) {
      trans += 16;
  } else if (rot < 0) {
      trans -= 19;
  }

  pkt->PutSlot(0, (uint16_t)trans);
  pkt->PutSlot(1, (uint16_t)rot);

  static int printcount =0;
  if (++printcount > 100) {
      printf("trans=%d rot=%d\n",trans,rot);
      printcount=0;
  }

  // alternate between the three config commands we want to add to the packet
  static int config_command=0;
  if (++config_command==1) {
      pkt->PutSlot(2, (uint16_t)10);  // set the scaling value (parameter 10)
      pkt->PutSlot(3, (uint16_t)16);  // to 1.0 (16 means 1.0)
  } else if (config_command == 2) {
      pkt->PutSlot(2, (uint16_t)15);  // set the balance lockout mode (param 15)
      pkt->PutSlot(3, (uint16_t)1);  // to locked-out (value 1)
  } else if (config_command == 3) {
      pkt->PutSlot(2, (uint16_t)14);  // set the current limit scale factor (param 14)
      pkt->PutSlot(3, (uint16_t)256);  // to 1.0 (value 256 means 1.0)
      config_command = 0;
  }

}

void
SegwayRMP::MakeShutdownCommand(CanPacket* pkt)
{
  pkt->id = RMP_CAN_ID_SHUTDOWN;

  printf("SEGWAYIO: SHUTDOWN: pkt: %s\n",
	 pkt->toString());
}

// Calculate the difference between two raw counter values, taking care
// of rollover.
int
SegwayRMP::Diff(uint32_t from, uint32_t to, bool first)
{
  int diff1, diff2;
  static uint32_t max = (uint32_t)pow(2,32)-1;

  // if this is the first time, report no change
  if(first)
    return(0);

  diff1 = to - from;

  /* find difference in two directions and pick shortest */
  if(to > from)
    diff2 = -(from + max - to);
  else 
    diff2 = max - from + to;

  if(abs(diff1) < abs(diff2)) 
    return(diff1);
  else
    return(diff2);
}
