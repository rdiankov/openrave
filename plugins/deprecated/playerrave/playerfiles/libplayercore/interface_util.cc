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

#include <string.h>

#include <libplayercore/player.h>  // for interface constants
#include <libplayercore/interface_util.h> // for player_interface_t type

/* this array lists the interfaces that Player knows how to load
 *
 * NOTE: the last element must be NULL
 */
static player_interface_t interfaces[] = {
  {PLAYER_NULL_CODE, PLAYER_NULL_STRING},
  {PLAYER_LOG_CODE, PLAYER_LOG_STRING},
  {PLAYER_LASER_CODE, PLAYER_LASER_STRING},
  {PLAYER_BLOBFINDER_CODE, PLAYER_BLOBFINDER_STRING},
  {PLAYER_SPEECH_CODE, PLAYER_SPEECH_STRING},
  {PLAYER_AUDIO_CODE, PLAYER_AUDIO_STRING},
  {PLAYER_AUDIODSP_CODE, PLAYER_AUDIODSP_STRING},
  {PLAYER_FIDUCIAL_CODE, PLAYER_FIDUCIAL_STRING},
  {PLAYER_PTZ_CODE, PLAYER_PTZ_STRING},
  {PLAYER_GRIPPER_CODE, PLAYER_GRIPPER_STRING},
  {PLAYER_POWER_CODE, PLAYER_POWER_STRING},
  {PLAYER_BUMPER_CODE, PLAYER_BUMPER_STRING},
  {PLAYER_AIO_CODE, PLAYER_AIO_STRING},
  {PLAYER_DIO_CODE, PLAYER_DIO_STRING},
  {PLAYER_POSITION1D_CODE, PLAYER_POSITION1D_STRING},
  {PLAYER_POSITION2D_CODE, PLAYER_POSITION2D_STRING},
  {PLAYER_POSITION3D_CODE, PLAYER_POSITION3D_STRING},
  {PLAYER_SONAR_CODE, PLAYER_SONAR_STRING},
  {PLAYER_WIFI_CODE, PLAYER_WIFI_STRING},
  {PLAYER_IR_CODE, PLAYER_IR_STRING},
  {PLAYER_WAVEFORM_CODE, PLAYER_WAVEFORM_STRING},
  {PLAYER_LOCALIZE_CODE, PLAYER_LOCALIZE_STRING},
  {PLAYER_MCOM_CODE, PLAYER_MCOM_STRING},
  {PLAYER_SIMULATION_CODE, PLAYER_SIMULATION_STRING},
  {PLAYER_SOUND_CODE, PLAYER_SOUND_STRING},
  {PLAYER_AUDIOMIXER_CODE, PLAYER_AUDIOMIXER_STRING},
  {PLAYER_TRUTH_CODE, PLAYER_TRUTH_STRING},
  {PLAYER_GPS_CODE, PLAYER_GPS_STRING},
  {PLAYER_SIMULATION_CODE, PLAYER_SIMULATION_STRING},
  {PLAYER_BLINKENLIGHT_CODE, PLAYER_BLINKENLIGHT_STRING},
  {PLAYER_LASER_CODE, PLAYER_LASER_STRING},
  {PLAYER_CAMERA_CODE, PLAYER_CAMERA_STRING},
  {PLAYER_BLOBFINDER_CODE, PLAYER_BLOBFINDER_STRING},
  {PLAYER_NOMAD_CODE, PLAYER_NOMAD_STRING},
  {PLAYER_ENERGY_CODE, PLAYER_ENERGY_STRING},
  {PLAYER_MAP_CODE, PLAYER_MAP_STRING},
  {PLAYER_PLANNER_CODE, PLAYER_PLANNER_STRING},
  {PLAYER_JOYSTICK_CODE, PLAYER_JOYSTICK_STRING},
  {PLAYER_SPEECH_RECOGNITION_CODE, PLAYER_SPEECH_RECOGNITION_STRING},
  {PLAYER_OPAQUE_CODE, PLAYER_OPAQUE_STRING},
  {PLAYER_ACTARRAY_CODE, PLAYER_ACTARRAY_STRING},
  {PLAYER_LIMB_CODE, PLAYER_LIMB_STRING},
  {PLAYER_GRAPHICS2D_CODE, PLAYER_GRAPHICS2D_STRING},
  {PLAYER_GRAPHICS3D_CODE, PLAYER_GRAPHICS3D_STRING},
  {PLAYER_RFID_CODE, PLAYER_RFID_STRING},
  {PLAYER_WSN_CODE, PLAYER_WSN_STRING},
  {PLAYER_VISIONSERVER_CODE, PLAYER_VISIONSERVER_STRING},
  {0,NULL}
};

/*
 * looks through the array of available interfaces for one which the given
 * name.  if found, interface is filled out (the caller must provide storage)
 * and zero is returned.  otherwise, -1 is returned.
 */
int
lookup_interface(const char* name, player_interface_t* interface)
{
  for(int i=0; interfaces[i].interf; i++)
  {
    if(!strcmp(name, interfaces[i].name))
    {
      *interface = interfaces[i];
      return(0);
    }
  }
  return(-1);
}

/*
 * looks through the array of available interfaces for one which the given
 * code.  if found, interface is filled out (the caller must provide storage)
 * and zero is returned.  otherwise, -1 is returned.
 */
int
lookup_interface_code(int code, player_interface_t* interface)
{
  for(int i=0; interfaces[i].interf; i++)
  {
    if(code == interfaces[i].interf)
    {
      *interface = interfaces[i];
      return(0);
    }
  }
  return(-1);
}

/*
 * looks through the array of interfaces, starting at startpos, for the first
 * entry that has the given code, and return the name.
 * leturns 0 when the end of the * array is reached.
 */
const char*
lookup_interface_name(unsigned int startpos, int code)
{
  if(startpos > sizeof(interfaces))
    return 0;
  for(int i = startpos; interfaces[i].interf != 0; i++)
  {
    if(code == interfaces[i].interf)
    {
      return interfaces[i].name;
    }
  }
  return 0;
}
