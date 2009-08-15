/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2005 -
 *     Brian Gerkey
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
 * $Id: functiontable.c,v 1.54.2.2 2007/01/31 22:23:25 gerkey Exp $
 *
 * Functions for looking up the appropriate XDR pack/unpack function for a
 * given message type and subtype.
 */

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <libplayercore/error.h>

#include "playerxdr.h"
#include "functiontable.h"

static playerxdr_function_t init_ftable[] =
{
  /* This list is currently alphabetized, please keep it that way! */

  /* actarray messages */
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_DATA, PLAYER_ACTARRAY_DATA_STATE,
   (player_pack_fn_t)player_actarray_data_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_POWER_REQ,
   (player_pack_fn_t)player_actarray_power_config_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_BRAKES_REQ,
   (player_pack_fn_t)player_actarray_brakes_config_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_GET_GEOM_REQ,
   (player_pack_fn_t)player_actarray_geom_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_SPEED_REQ,
   (player_pack_fn_t)player_actarray_speed_config_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_POS_CMD,
   (player_pack_fn_t)player_actarray_position_cmd_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_SPEED_CMD,
   (player_pack_fn_t)player_actarray_speed_cmd_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_HOME_CMD,
   (player_pack_fn_t)player_actarray_home_cmd_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_TRAJECTORY_CMD,
   (player_pack_fn_t)player_actarray_trajectory_cmd_pack},
  {PLAYER_ACTARRAY_CODE, PLAYER_MSGTYPE_CMD, PLAYER_ACTARRAY_START_CMD,
   (player_pack_fn_t)player_actarray_start_cmd_pack},

  /* aio messages */
  {PLAYER_AIO_CODE, PLAYER_MSGTYPE_DATA, PLAYER_AIO_DATA_STATE,
   (player_pack_fn_t)player_aio_data_pack},
  {PLAYER_AIO_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AIO_CMD_STATE,
   (player_pack_fn_t)player_aio_cmd_pack},



  /* audiodsp messages */
  {PLAYER_AUDIODSP_CODE, PLAYER_MSGTYPE_DATA, PLAYER_AUDIODSP_DATA_TONES,
   (player_pack_fn_t)player_audiodsp_data_pack},
  {PLAYER_AUDIODSP_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIODSP_PLAY_TONE,
   (player_pack_fn_t)player_audiodsp_cmd_pack},
  {PLAYER_AUDIODSP_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIODSP_PLAY_CHIRP,
   (player_pack_fn_t)player_audiodsp_cmd_pack},
  {PLAYER_AUDIODSP_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIODSP_REPLAY,
   (player_pack_fn_t)player_audiodsp_cmd_pack},
  {PLAYER_AUDIODSP_CODE, PLAYER_MSGTYPE_REQ, PLAYER_AUDIODSP_GET_CONFIG,
   (player_pack_fn_t)player_audiodsp_config_pack},
  {PLAYER_AUDIODSP_CODE, PLAYER_MSGTYPE_REQ, PLAYER_AUDIODSP_SET_CONFIG,
   (player_pack_fn_t)player_audiodsp_config_pack},

  /* audiomixer messages */
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_MASTER,
   (player_pack_fn_t)player_audiomixer_cmd_pack},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_PCM,
   (player_pack_fn_t)player_audiomixer_cmd_pack},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_LINE,
   (player_pack_fn_t)player_audiomixer_cmd_pack},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_MIC,
   (player_pack_fn_t)player_audiomixer_cmd_pack},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_IGAIN,
   (player_pack_fn_t)player_audiomixer_cmd_pack},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_OGAIN,
   (player_pack_fn_t)player_audiomixer_cmd_pack},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_AUDIOMIXER_GET_LEVELS,
   (player_pack_fn_t)player_audiomixer_config_pack},

  /* blinkenlight messages */
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_MSGTYPE_DATA, PLAYER_BLINKENLIGHT_DATA_STATE,
   (player_pack_fn_t)player_blinkenlight_data_pack},
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_MSGTYPE_CMD, PLAYER_BLINKENLIGHT_CMD_STATE,
   (player_pack_fn_t)player_blinkenlight_cmd_pack},
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_MSGTYPE_CMD, PLAYER_BLINKENLIGHT_CMD_POWER,
   (player_pack_fn_t)player_blinkenlight_cmd_power_pack},
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_MSGTYPE_CMD, PLAYER_BLINKENLIGHT_CMD_COLOR,
   (player_pack_fn_t)player_blinkenlight_cmd_color_pack},
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_MSGTYPE_CMD, PLAYER_BLINKENLIGHT_CMD_DUTYCYCLE,
   (player_pack_fn_t)player_blinkenlight_cmd_dutycycle_pack},
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_MSGTYPE_CMD, PLAYER_BLINKENLIGHT_CMD_PERIOD,
   (player_pack_fn_t)player_blinkenlight_cmd_period_pack},

  /* blobfinder messages */
  {PLAYER_BLOBFINDER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_BLOBFINDER_DATA_BLOBS,
   (player_pack_fn_t)player_blobfinder_data_pack},
  {PLAYER_BLOBFINDER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_BLOBFINDER_REQ_SET_COLOR,
   (player_pack_fn_t)player_blobfinder_color_config_pack},
  {PLAYER_BLOBFINDER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_BLOBFINDER_REQ_SET_IMAGER_PARAMS,
   (player_pack_fn_t)player_blobfinder_imager_config_pack},

  /* bumper messages */
  {PLAYER_BUMPER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_BUMPER_DATA_STATE,
   (player_pack_fn_t)player_bumper_data_pack},
  {PLAYER_BUMPER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_BUMPER_DATA_GEOM,
   (player_pack_fn_t)player_bumper_geom_pack},
  {PLAYER_BUMPER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_BUMPER_GET_GEOM,
   (player_pack_fn_t)player_bumper_geom_pack},

  /* camera messages */
  {PLAYER_CAMERA_CODE, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
   (player_pack_fn_t)player_camera_data_pack},

 /* dio messages */
  {PLAYER_DIO_CODE, PLAYER_MSGTYPE_DATA, PLAYER_DIO_DATA_VALUES,
   (player_pack_fn_t)player_dio_data_pack},
  {PLAYER_DIO_CODE, PLAYER_MSGTYPE_CMD, PLAYER_DIO_CMD_VALUES,
   (player_pack_fn_t)player_dio_cmd_pack},

  /* fiducial messages */
  {PLAYER_FIDUCIAL_CODE, PLAYER_MSGTYPE_DATA, PLAYER_FIDUCIAL_DATA_SCAN,
   (player_pack_fn_t)player_fiducial_data_pack},
  {PLAYER_FIDUCIAL_CODE, PLAYER_MSGTYPE_REQ, PLAYER_FIDUCIAL_REQ_GET_GEOM,
   (player_pack_fn_t)player_fiducial_geom_pack},
  {PLAYER_FIDUCIAL_CODE, PLAYER_MSGTYPE_REQ, PLAYER_FIDUCIAL_REQ_GET_FOV,
   (player_pack_fn_t)player_fiducial_fov_pack},
  {PLAYER_FIDUCIAL_CODE, PLAYER_MSGTYPE_REQ, PLAYER_FIDUCIAL_REQ_SET_FOV,
   (player_pack_fn_t)player_fiducial_fov_pack},
  {PLAYER_FIDUCIAL_CODE, PLAYER_MSGTYPE_REQ, PLAYER_FIDUCIAL_REQ_GET_ID,
   (player_pack_fn_t)player_fiducial_id_pack},
  {PLAYER_FIDUCIAL_CODE, PLAYER_MSGTYPE_REQ, PLAYER_FIDUCIAL_REQ_SET_ID,
   (player_pack_fn_t)player_fiducial_id_pack},

  /* gps messages */
  {PLAYER_GPS_CODE, PLAYER_MSGTYPE_DATA, PLAYER_GPS_DATA_STATE,
   (player_pack_fn_t)player_gps_data_pack},

  /* graphics2d messages */
  {PLAYER_GRAPHICS2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRAPHICS2D_CMD_CLEAR,
   (player_pack_fn_t)player_graphics2d_cmd_points_pack},
  {PLAYER_GRAPHICS2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRAPHICS2D_CMD_POINTS,
   (player_pack_fn_t)player_graphics2d_cmd_points_pack},
  {PLAYER_GRAPHICS2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRAPHICS2D_CMD_POLYGON,
   (player_pack_fn_t)player_graphics2d_cmd_polygon_pack},
  {PLAYER_GRAPHICS2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRAPHICS2D_CMD_POLYLINE,
   (player_pack_fn_t)player_graphics2d_cmd_polyline_pack},

  /* graphics3d messages */
  {PLAYER_GRAPHICS3D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRAPHICS3D_CMD_CLEAR,
   (player_pack_fn_t)player_graphics3d_cmd_draw_pack},
  {PLAYER_GRAPHICS3D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRAPHICS3D_CMD_DRAW,
   (player_pack_fn_t)player_graphics3d_cmd_draw_pack},

  /* gripper messages */
  {PLAYER_GRIPPER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_GRIPPER_DATA_STATE,
   (player_pack_fn_t)player_gripper_data_pack},
  {PLAYER_GRIPPER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_GRIPPER_CMD_STATE,
   (player_pack_fn_t)player_gripper_cmd_pack},
  {PLAYER_GRIPPER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_GRIPPER_REQ_GET_GEOM,
   (player_pack_fn_t)player_gripper_geom_pack},

  /* ir messages */
  {PLAYER_IR_CODE, PLAYER_MSGTYPE_DATA, PLAYER_IR_DATA_RANGES,
    (player_pack_fn_t)player_ir_data_pack},
  {PLAYER_IR_CODE, PLAYER_MSGTYPE_REQ, PLAYER_IR_POSE,
    (player_pack_fn_t)player_ir_pose_pack},
  {PLAYER_IR_CODE, PLAYER_MSGTYPE_REQ, PLAYER_IR_POWER,
    (player_pack_fn_t)player_ir_power_req_pack},

  /* ir messages */
  {PLAYER_JOYSTICK_CODE, PLAYER_MSGTYPE_DATA, PLAYER_JOYSTICK_DATA_STATE,
    (player_pack_fn_t)player_joystick_data_pack},

  /* laser messages */
  {PLAYER_LASER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN,
    (player_pack_fn_t)player_laser_data_pack},
  {PLAYER_LASER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCANPOSE,
    (player_pack_fn_t)player_laser_data_scanpose_pack},
  {PLAYER_LASER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM,
    (player_pack_fn_t)player_laser_geom_pack},
  {PLAYER_LASER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG,
    (player_pack_fn_t)player_laser_config_pack},
  {PLAYER_LASER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_SET_CONFIG,
    (player_pack_fn_t)player_laser_config_pack},
  {PLAYER_LASER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_POWER,
    (player_pack_fn_t)player_laser_power_config_pack},

  /* limb messages */
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_DATA, PLAYER_LIMB_DATA,
    (player_pack_fn_t)player_limb_data_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_CMD, PLAYER_LIMB_HOME_CMD,
    (player_pack_fn_t)player_limb_home_cmd_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_CMD, PLAYER_LIMB_STOP_CMD,
    (player_pack_fn_t)player_limb_stop_cmd_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_CMD, PLAYER_LIMB_SETPOSE_CMD,
    (player_pack_fn_t)player_limb_setpose_cmd_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_CMD, PLAYER_LIMB_SETPOSITION_CMD,
    (player_pack_fn_t)player_limb_setposition_cmd_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_CMD, PLAYER_LIMB_VECMOVE_CMD,
    (player_pack_fn_t)player_limb_vecmove_cmd_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LIMB_POWER_REQ,
    (player_pack_fn_t)player_limb_power_req_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LIMB_BRAKES_REQ,
    (player_pack_fn_t)player_limb_brakes_req_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LIMB_GEOM_REQ,
    (player_pack_fn_t)player_limb_geom_req_pack},
  {PLAYER_LIMB_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LIMB_SPEED_REQ,
    (player_pack_fn_t)player_limb_speed_req_pack},

  /* localize messages */
  {PLAYER_LOCALIZE_CODE, PLAYER_MSGTYPE_DATA, PLAYER_LOCALIZE_DATA_HYPOTHS,
    (player_pack_fn_t)player_localize_data_pack},
  {PLAYER_LOCALIZE_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOCALIZE_REQ_SET_POSE,
    (player_pack_fn_t)player_localize_set_pose_pack},
  {PLAYER_LOCALIZE_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOCALIZE_REQ_GET_PARTICLES,
    (player_pack_fn_t)player_localize_get_particles_pack},

  /* log messages */
  {PLAYER_LOG_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOG_REQ_SET_WRITE_STATE,
    (player_pack_fn_t)player_log_set_write_state_pack},
  {PLAYER_LOG_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOG_REQ_SET_READ_STATE,
    (player_pack_fn_t)player_log_set_read_state_pack},
  {PLAYER_LOG_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOG_REQ_GET_STATE,
    (player_pack_fn_t)player_log_get_state_pack},
  {PLAYER_LOG_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOG_REQ_SET_READ_REWIND,
    (player_pack_fn_t)player_log_set_read_rewind_pack},
  {PLAYER_LOG_CODE, PLAYER_MSGTYPE_REQ, PLAYER_LOG_REQ_SET_FILENAME,
    (player_pack_fn_t)player_log_set_filename_pack},

  /* map messages */
  {PLAYER_MAP_CODE, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_INFO,
    (player_pack_fn_t)player_map_info_pack},
  {PLAYER_MAP_CODE, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_DATA,
    (player_pack_fn_t)player_map_data_pack},
  {PLAYER_MAP_CODE, PLAYER_MSGTYPE_DATA, PLAYER_MAP_DATA_INFO,
    (player_pack_fn_t)player_map_info_pack},
  {PLAYER_MAP_CODE, PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_VECTOR,
    (player_pack_fn_t)player_map_data_vector_pack},

  /* opaque messages */
  {PLAYER_OPAQUE_CODE, PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,
    (player_pack_fn_t)player_opaque_data_pack},
  {PLAYER_OPAQUE_CODE, PLAYER_MSGTYPE_CMD, PLAYER_OPAQUE_CMD,
    (player_pack_fn_t)player_opaque_data_pack},
  {PLAYER_OPAQUE_CODE, PLAYER_MSGTYPE_REQ, PLAYER_OPAQUE_REQ,
    (player_pack_fn_t)player_opaque_data_pack},

  /* planner messages */
  {PLAYER_PLANNER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_PLANNER_DATA_STATE,
    (player_pack_fn_t)player_planner_data_pack},
  {PLAYER_PLANNER_CODE, PLAYER_MSGTYPE_CMD, PLAYER_PLANNER_CMD_GOAL,
    (player_pack_fn_t)player_planner_cmd_pack},
  {PLAYER_PLANNER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLANNER_REQ_ENABLE,
    (player_pack_fn_t)player_planner_enable_req_pack},
  {PLAYER_PLANNER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLANNER_REQ_GET_WAYPOINTS,
    (player_pack_fn_t)player_planner_waypoints_req_pack},

  /* player messages */
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_DEVLIST,
    (player_pack_fn_t)player_device_devlist_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_DRIVERINFO,
    (player_pack_fn_t)player_device_driverinfo_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_DEV,
    (player_pack_fn_t)player_device_req_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_DATA,
    (player_pack_fn_t)player_device_data_req_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_DATAMODE,
    (player_pack_fn_t)player_device_datamode_req_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_AUTH,
    (player_pack_fn_t)player_device_auth_req_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_NAMESERVICE,
    (player_pack_fn_t)player_device_nameservice_req_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PLAYER_REQ_ADD_REPLACE_RULE,
    (player_pack_fn_t)player_add_replace_rule_req_pack},
  {PLAYER_PLAYER_CODE, PLAYER_MSGTYPE_SYNCH, 0,
    (player_pack_fn_t)player_add_replace_rule_req_pack},

  /* position1d messages */
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_DATA, PLAYER_POSITION1D_DATA_STATE,
    (player_pack_fn_t)player_position1d_data_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_DATA, PLAYER_POSITION1D_DATA_GEOM,
    (player_pack_fn_t)player_position1d_geom_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION1D_CMD_VEL,
    (player_pack_fn_t)player_position1d_cmd_vel_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION1D_CMD_POS,
    (player_pack_fn_t)player_position1d_cmd_pos_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_GET_GEOM,
    (player_pack_fn_t)player_position1d_geom_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_MOTOR_POWER,
    (player_pack_fn_t)player_position1d_power_config_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_VELOCITY_MODE,
    (player_pack_fn_t)player_position1d_velocity_mode_config_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_POSITION_MODE,
    (player_pack_fn_t)player_position1d_position_mode_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_SET_ODOM,
    (player_pack_fn_t)player_position1d_set_odom_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_SPEED_PID,
    (player_pack_fn_t)player_position1d_speed_pid_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_POSITION_PID,
    (player_pack_fn_t)player_position1d_position_pid_pack},
  {PLAYER_POSITION1D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION1D_REQ_SPEED_PROF,
    (player_pack_fn_t)player_position1d_speed_prof_pack},


  /* position2d messages */
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
    (player_pack_fn_t)player_position2d_data_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL,
    (player_pack_fn_t)player_position2d_cmd_vel_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_POS,
    (player_pack_fn_t)player_position2d_cmd_pos_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_CAR,
    (player_pack_fn_t)player_position2d_cmd_car_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM,
    (player_pack_fn_t)player_position2d_geom_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_MOTOR_POWER,
    (player_pack_fn_t)player_position2d_power_config_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_VELOCITY_MODE,
    (player_pack_fn_t)player_position2d_velocity_mode_config_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_POSITION_MODE,
    (player_pack_fn_t)player_position2d_position_mode_req_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_SET_ODOM,
    (player_pack_fn_t)player_position2d_set_odom_req_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_SPEED_PID,
    (player_pack_fn_t)player_position2d_speed_pid_req_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_POSITION_PID,
    (player_pack_fn_t)player_position2d_position_pid_req_pack},
  {PLAYER_POSITION2D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_SPEED_PROF,
    (player_pack_fn_t)player_position2d_speed_prof_req_pack},

  /* position3d messages */
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
    (player_pack_fn_t)player_position3d_data_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_GEOMETRY,
    (player_pack_fn_t)player_position3d_data_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION3D_CMD_SET_POS,
    (player_pack_fn_t)player_position3d_cmd_pos_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_CMD, PLAYER_POSITION3D_CMD_SET_VEL,
    (player_pack_fn_t)player_position3d_cmd_vel_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION3D_MOTOR_POWER,
    (player_pack_fn_t)player_position3d_power_config_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION3D_POSITION_MODE,
    (player_pack_fn_t)player_position3d_position_mode_req_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION3D_RESET_ODOM,
    (player_pack_fn_t)player_position3d_reset_odom_config_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION3D_SET_ODOM,
    (player_pack_fn_t)player_position3d_set_odom_req_pack},
  {PLAYER_POSITION3D_CODE, PLAYER_MSGTYPE_REQ, PLAYER_POSITION3D_VELOCITY_MODE,
    (player_pack_fn_t)player_position3d_velocity_mode_config_pack},
  

  /* power messages */
  {PLAYER_POWER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_POWER_DATA_STATE,
    (player_pack_fn_t)player_power_data_pack},

  /* ptz messages */
  {PLAYER_PTZ_CODE, PLAYER_MSGTYPE_DATA, PLAYER_PTZ_DATA_STATE,
    (player_pack_fn_t)player_ptz_data_pack},
  {PLAYER_PTZ_CODE, PLAYER_MSGTYPE_CMD, PLAYER_PTZ_CMD_STATE,
    (player_pack_fn_t)player_ptz_cmd_pack},
  {PLAYER_PTZ_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PTZ_REQ_GEOM,
    (player_pack_fn_t)player_ptz_geom_pack},
  {PLAYER_PTZ_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PTZ_REQ_GENERIC,
    (player_pack_fn_t)player_ptz_req_generic_pack},
  {PLAYER_PTZ_CODE, PLAYER_MSGTYPE_REQ, PLAYER_PTZ_REQ_CONTROL_MODE,
    (player_pack_fn_t)player_ptz_req_control_mode_pack},

  /* rfid messages */
  {PLAYER_RFID_CODE, PLAYER_MSGTYPE_DATA, PLAYER_RFID_DATA,
    (player_pack_fn_t)player_rfid_data_pack},

  /* simulation messages */
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_SET_POSE2D,
    (player_pack_fn_t)player_simulation_pose2d_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_GET_POSE2D,
    (player_pack_fn_t)player_simulation_pose2d_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_GET_PROPERTY_INT,
    (player_pack_fn_t)player_simulation_property_int_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_SET_PROPERTY_INT,
    (player_pack_fn_t)player_simulation_property_int_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_GET_PROPERTY_FLOAT,
    (player_pack_fn_t)player_simulation_property_float_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_SET_PROPERTY_FLOAT,
    (player_pack_fn_t)player_simulation_property_float_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_GET_PROPERTY_STRING,
    (player_pack_fn_t)player_simulation_property_string_req_pack},
  {PLAYER_SIMULATION_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SIMULATION_REQ_SET_PROPERTY_STRING,
    (player_pack_fn_t)player_simulation_property_string_req_pack},

  /* sonar messages */
  {PLAYER_SONAR_CODE, PLAYER_MSGTYPE_DATA, PLAYER_SONAR_DATA_RANGES,
    (player_pack_fn_t)player_sonar_data_pack},
  {PLAYER_SONAR_CODE, PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM,
    (player_pack_fn_t)player_sonar_geom_pack},

  /* speech messages */
  {PLAYER_SPEECH_CODE, PLAYER_MSGTYPE_CMD, PLAYER_SPEECH_CMD_SAY,
    (player_pack_fn_t)player_speech_cmd_pack},

  /* speech recognition messages */
  {PLAYER_SPEECH_RECOGNITION_CODE, PLAYER_MSGTYPE_DATA, PLAYER_SPEECH_RECOGNITION_DATA_STRING,
    (player_pack_fn_t)player_speech_recognition_data_pack},

  /* waveform messages */
  {PLAYER_WAVEFORM_CODE, PLAYER_MSGTYPE_DATA, PLAYER_WAVEFORM_DATA_SAMPLE,
    (player_pack_fn_t)player_waveform_data_pack},

  /* wifi messages */
  {PLAYER_WIFI_CODE, PLAYER_MSGTYPE_DATA, PLAYER_WIFI_DATA_STATE,
    (player_pack_fn_t)player_wifi_data_pack},
  {PLAYER_WIFI_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WIFI_MAC,
    (player_pack_fn_t)player_wifi_mac_req_pack},
  {PLAYER_WIFI_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WIFI_IWSPY_ADD,
    (player_pack_fn_t)player_wifi_iwspy_addr_req_pack},
  {PLAYER_WIFI_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WIFI_IWSPY_DEL,
    (player_pack_fn_t)player_wifi_iwspy_addr_req_pack},
  {PLAYER_WIFI_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WIFI_IWSPY_PING,
    (player_pack_fn_t)player_wifi_iwspy_addr_req_pack},

  /* wsn messages */
  {PLAYER_WSN_CODE, PLAYER_MSGTYPE_DATA, PLAYER_WSN_DATA,
    (player_pack_fn_t)player_wsn_data_pack},
  {PLAYER_WSN_CODE, PLAYER_MSGTYPE_CMD, PLAYER_WSN_CMD_DEVSTATE,
   (player_pack_fn_t)player_wsn_cmd_pack},
  {PLAYER_WSN_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WSN_REQ_POWER,
   (player_pack_fn_t)player_wsn_power_config_pack},
  {PLAYER_WSN_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WSN_REQ_DATATYPE,
   (player_pack_fn_t)player_wsn_datatype_config_pack},
  {PLAYER_WSN_CODE, PLAYER_MSGTYPE_REQ, PLAYER_WSN_REQ_DATAFREQ,
   (player_pack_fn_t)player_wsn_datafreq_config_pack},

  /* visionserver messages */
  {PLAYER_VISIONSERVER_CODE, PLAYER_MSGTYPE_DATA, PLAYER_VISIONSERVER_DATA_STATE,
   (player_pack_fn_t)player_visionserver_data_pack},
  {PLAYER_VISIONSERVER_CODE, PLAYER_MSGTYPE_REQ, PLAYER_VISIONSERVER_REQ_GET_GEOM,
   (player_pack_fn_t)player_visionserver_geom_pack},
  
  /* This NULL element signals the end of the list; don't remove it */
  {0,0,0,NULL}
};

static playerxdr_function_t* ftable;
static int ftable_len;

void
playerxdr_ftable_init()
{
  playerxdr_function_t* f;
  ftable_len = 0;
  for(f = init_ftable; f->func; f++)
    ftable_len++;

  ftable = (playerxdr_function_t*)calloc(ftable_len,
                                         sizeof(playerxdr_function_t));
  assert(ftable);

  memcpy(ftable,init_ftable,ftable_len*sizeof(playerxdr_function_t));
}

int
playerxdr_ftable_add(playerxdr_function_t f, int replace)
{
  if(playerxdr_get_func(f.interf, f.type, f.subtype))
  {
    // It's already in the table.  Did the caller say to replace?
    if(!replace)
    {
      // Nope; return an error
      return(-1);
    }
    else
    {
      // Yes; replace (it's clearly inefficient to iterate through the
      // table again to find the entry to replace, but the table is pretty
      // small and this doesn't happen very often)
      int i;
      playerxdr_function_t* curr;

      for(i=0;i<ftable_len;i++)
      {
        curr = ftable + i;
        // Make sure the interface, type, and subtype match exactly
        if((curr->interf == f.interf) &&
           (curr->subtype == f.subtype) &&
           (curr->type == f.type))
        {
          curr->func = f.func;
          return(0);
        }
      }
      PLAYER_ERROR("unable to find entry to replace");
      return(-1);
    }
  }
  else
  {
    // Not in the table; add it
    ftable = (playerxdr_function_t*)realloc(ftable,
                                            ((ftable_len+1)*
                                             sizeof(playerxdr_function_t)));
    assert(ftable);
    ftable[ftable_len++] = f;
    return(0);
  }
}

player_pack_fn_t
playerxdr_get_func(uint16_t interf, uint8_t type, uint8_t subtype)
{
  playerxdr_function_t* curr=NULL;
  int i;

  if(!ftable_len)
    return(NULL);

  // The supplied type can be RESP_ACK if the registered type is REQ.
  if (type == PLAYER_MSGTYPE_RESP_ACK || type == PLAYER_MSGTYPE_RESP_NACK)
    type = PLAYER_MSGTYPE_REQ;

  for(i=0;i<ftable_len;i++)
  {
    curr = ftable + i;
    // Make sure the interface and subtype match exactly.
    // match anyway if interface = 0 (universal data types)
 
    if((curr->interf == interf || curr->interf == 0) &&
      curr->type == type &&
      curr->subtype == subtype)
      return(curr->func);
  }

  printf( "interface %d %d\n", curr->interf, interf );
  printf( "type %d %d\n", curr->type, type );
  printf( "subtype %d %d\n", curr->subtype, subtype );

  return(NULL);
}

