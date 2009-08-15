/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
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
 * $Id: actarrayproxy.cc,v 1.7 2005/11/08 17:07:00 bradkratochvil Exp $
 *
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <cassert>
#include <sstream>
#include <iomanip>

#include "playerc++.h"
#include "debug.h"

using namespace PlayerCc;

ActArrayProxy::ActArrayProxy(PlayerClient *aPc, uint aIndex)
  : ClientProxy(aPc, aIndex),
  mDevice(NULL)
{
  Subscribe(aIndex);
  mInfo = &(mDevice->info);
}

ActArrayProxy::~ActArrayProxy()
{
  Unsubscribe();
}

void ActArrayProxy::Subscribe(uint aIndex)
{
  scoped_lock_t lock(mPc->mMutex);
  mDevice = playerc_actarray_create(mClient, aIndex);
  if (mDevice==NULL)
    throw PlayerError("ActArrayProxy::ActArrayProxy()", "could not create");

  if (playerc_actarray_subscribe(mDevice, PLAYER_OPEN_MODE) != 0)
    throw PlayerError("ActArrayProxy::ActArrayProxy()", "could not subscribe");
}

void ActArrayProxy::Unsubscribe(void)
{
  assert(mDevice!=NULL);
  scoped_lock_t lock(mPc->mMutex);
  playerc_actarray_unsubscribe(mDevice);
  playerc_actarray_destroy(mDevice);
  mDevice = NULL;
}

// interface that all proxies SHOULD provide
std::ostream& std::operator << (std::ostream& os, const PlayerCc::ActArrayProxy& a)
{
  player_actarray_actuator_t data;
  player_actarray_actuatorgeom_t geom;

  int old_precision = os.precision(3);
  std::_Ios_Fmtflags old_flags = os.flags();
  os.setf(std::ios::fixed);

  os << a.GetCount () << " actuators:" << std::endl;
  os << "Act \tType\tMin\tCentre\tMax\tHome"
        "\tCfgSpd\tPos\tSpeed\tState\tBrakes" << std::endl;
  for (uint ii = 0; ii < a.GetCount (); ii++)
  {
    data = a.GetActuatorData(ii);
    geom = a.GetActuatorGeom(ii);
    os <<  ii << '\t'
       << (geom.type ? "Linear" : "Rotary") << '\t'
       << geom.min << '\t'
       << geom.centre << '\t'
       << geom.max << '\t'
       << geom.home << '\t'
       << geom.config_speed << '\t'
       << data.position << '\t'
       << data.speed << '\t'
       << static_cast<int> (data.state)
       << '\t' << (geom.hasbrakes ? "Y" : "N")
       << std::endl;
  }

  os.precision(old_precision);
  os.flags(old_flags);

  return os;
}

// Power control
void ActArrayProxy::SetPowerConfig(uint8_t aVal)
{
  scoped_lock_t lock(mPc->mMutex);
  int ret = playerc_actarray_power(mDevice, aVal);

  if (ret == -2)
    throw PlayerError("ActArrayProxy::SetPowerConfig", "NACK", ret);
  else if (ret != 0)
    throw PlayerError("ActArrayProxy::SetPowerConfig",
                      playerc_error_str(),
                      ret);
}

// Brakes control
void ActArrayProxy::SetBrakesConfig(uint8_t aVal)
{
  scoped_lock_t lock(mPc->mMutex);
  int ret = playerc_actarray_brakes(mDevice, aVal);

  if (ret == -2)
    throw PlayerError("ActArrayProxy::SetBrakesConfig", "NACK", ret);
  else if (ret != 0)
    throw PlayerError("ActArrayProxy::SetBrakesConfig",
                      playerc_error_str(),
                      ret);
}

// Speed config
void ActArrayProxy::SetSpeedConfig (uint aJoint, float aSpeed)
{
  scoped_lock_t lock(mPc->mMutex);
  int ret = playerc_actarray_speed_config(mDevice, aJoint, aSpeed);

  if (ret == -2)
    throw PlayerError("ActArrayProxy::SetSpeedConfig", "NACK", ret);
  else if (ret != 0)
    throw PlayerError("ActArrayProxy::SetSpeedConfig",
                      playerc_error_str(),
                      ret);
}

// Send an actuator to a position
void ActArrayProxy::MoveTo(uint aJoint, float aPosition)
{
  scoped_lock_t lock(mPc->mMutex);
  playerc_actarray_position_cmd(mDevice, aJoint, aPosition);
}

// Move an actuator at a speed
void ActArrayProxy::MoveAtSpeed(uint aJoint, float aSpeed)
{
  scoped_lock_t lock(mPc->mMutex);
  playerc_actarray_speed_cmd(mDevice, aJoint, aSpeed);
}

// Send an actuator, or all actuators, home
void ActArrayProxy::MoveHome(int aJoint)
{
  scoped_lock_t lock(mPc->mMutex);
  playerc_actarray_home_cmd(mDevice, aJoint);
}

int ActArrayProxy::MoveWithTrajectory(int numpoints, int trajformat, float time_of_divergence, uint32_t trajectory_id, const float* pData)
{
  scoped_lock_t lock(mPc->mMutex);
  return playerc_actarray_trajectory_cmd(mDevice, numpoints, trajformat, time_of_divergence, trajectory_id, pData);
}

// Send an actuator, or all actuators, home
void ActArrayProxy::Start(int64_t timestamp)
{
  scoped_lock_t lock(mPc->mMutex);
  playerc_actarray_start_cmd(mDevice, timestamp);
}

player_actarray_actuator_t ActArrayProxy::GetActuatorData(uint aJoint) const
{
  if (aJoint > mDevice->actuators_count)
  {
    scoped_lock_t lock(mPc->mMutex);
    player_actarray_actuator_t empty;
    memset(&empty, 0, sizeof(player_actarray_actuator_t));
    return empty;
  }
  else
    return GetVar(mDevice->actuators_data[aJoint]);
}

// Same again for getting actuator geometry
player_actarray_actuatorgeom_t ActArrayProxy::GetActuatorGeom(uint aJoint) const
{
  if (aJoint > mDevice->actuators_count)
  {
    scoped_lock_t lock(mPc->mMutex);
    player_actarray_actuatorgeom_t empty;
    memset(&empty, 0, sizeof(player_actarray_actuatorgeom_t));
    return empty;
  }
  else
    return GetVar(mDevice->actuators_geom[aJoint]);
}

uint32_t ActArrayProxy::GetTrajectoryId() const
{
    return GetVar(mDevice->trajectory_id);
}

float ActArrayProxy::GetTrajectoryTime() const
{
    return GetVar(mDevice->trajectory_time);
}

void ActArrayProxy::RequestGeometry(void)
{
  scoped_lock_t lock(mPc->mMutex);
  int ret = playerc_actarray_get_geom(mDevice);

  if (ret == -2)
    throw PlayerError("ActArrayProxy::RequestGeometry", "NACK", ret);
  else if (ret != 0)
    throw PlayerError("ActArrayProxy::RequestGeometry",
                      playerc_error_str(),
                      ret);
}
