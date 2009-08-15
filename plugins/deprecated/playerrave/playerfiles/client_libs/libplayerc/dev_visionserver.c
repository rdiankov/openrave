/*
 *  libplayerc : a Player client library
 *  Copyright (C)
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

#include <string.h>
#include <stdlib.h>

#include "playerc.h"
#include "error.h"

// Local declarations
void playerc_visionserver_putmsg(playerc_visionserver_t *device,
                             player_msghdr_t *header,
                             player_visionserver_data_t *data, size_t len);

// Create an visionserver proxy
playerc_visionserver_t *playerc_visionserver_create(playerc_client_t *client, int index)
{
  playerc_visionserver_t *device;

  device = malloc(sizeof(playerc_visionserver_t));
  memset(device, 0, sizeof(playerc_visionserver_t));
  playerc_device_init(&device->info, client, PLAYER_VISIONSERVER_CODE, index,
                       (playerc_putmsg_fn_t) playerc_visionserver_putmsg);

  return device;
}

// Destroy an visionserver proxy
void playerc_visionserver_destroy(playerc_visionserver_t *device)
{
  playerc_device_term(&device->info);
  free(device);
}

// Subscribe to the visionserver device
int playerc_visionserver_subscribe(playerc_visionserver_t *device, int access)
{
  return playerc_device_subscribe(&device->info, access);
}

// Un-subscribe from the visionserver device
int playerc_visionserver_unsubscribe(playerc_visionserver_t *device)
{
  return playerc_device_unsubscribe(&device->info);
}

void playerc_visionserver_putmsg(playerc_visionserver_t *device,
                             player_msghdr_t *header,
                             player_visionserver_data_t *data, size_t len)
{
  if((header->type == PLAYER_MSGTYPE_DATA) && (header->subtype == PLAYER_VISIONSERVER_DATA_STATE))
  {
      device->_data = *data;
  }
  else
    PLAYERC_WARN2("skipping visionserver message with unknown type/subtype: %d/%d\n",
                  header->type, header->subtype);
}

// Get the visionserver geometry
int playerc_visionserver_get_geom(playerc_visionserver_t *device)
{
  player_visionserver_geom_t geom;
  int ii = 0, result = 0;

  if((result = playerc_client_request(device->info.client, &device->info,
                            PLAYER_VISIONSERVER_REQ_GET_GEOM,
                            NULL, (void*)&geom, sizeof(geom))) < 0)
    return result;

  device->_geom = geom;
  return 0;
}
