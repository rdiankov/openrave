/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
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
/********************************************************************
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ********************************************************************/

/*
 * $Id: visionobjectproxy.cc $
 */

#include "playerc++.h"

#ifdef HAVE_BOOST_SIGNALS
#include <boost/bind.hpp>
#endif

using namespace PlayerCc;

VisionserverProxy::VisionserverProxy(PlayerClient *aPc, uint aIndex)
  : ClientProxy(aPc, aIndex),
  mDevice(NULL), _bChanged(false)
{
  Subscribe(aIndex);
  // how can I get this into the clientproxy.cc?
  // right now, we're dependent on knowing its device type
  mInfo = &(mDevice->info);

#ifdef HAVE_BOOST_SIGNALS
  ConnectReadSignal(boost::bind(&VisionserverProxy::UpdateChanged, this));
#endif
}

VisionserverProxy::~VisionserverProxy()
{
  Unsubscribe();
  //DisconnectReadSignal
}

void
VisionserverProxy::Subscribe(uint aIndex)
{
  scoped_lock_t lock(mPc->mMutex);
  mDevice = playerc_visionserver_create(mClient, aIndex);
  if (NULL==mDevice)
    throw PlayerError("VisionserverProxy::VisionserverProxy()", "could not create");

  if (0 != playerc_visionserver_subscribe(mDevice, PLAYER_OPEN_MODE))
    throw PlayerError("VisionserverProxy::VisionserverProxy()", "could not subscribe");
}

void
VisionserverProxy::Unsubscribe()
{
  assert(NULL!=mDevice);
  scoped_lock_t lock(mPc->mMutex);
  playerc_visionserver_unsubscribe(mDevice);
  playerc_visionserver_destroy(mDevice);
  mDevice = NULL;
}

void VisionserverProxy::UpdateChanged()
{
    _bChanged |= !!(GetVar(mDevice->_data.status)&1);
}

std::ostream& std::operator << (std::ostream &os, const PlayerCc::VisionserverProxy &c)
{
    /*
  os << "#Visionserver (" << c.GetInterface() << ":" << c.GetIndex() << ")" << std::endl;
  os << c.GetCount() << std::endl;
  for (unsigned int i=0;i < c.GetCount();i++)
  {
    player_visionserver_item_t item = c.GetVisionserverItem(i);
    os << "  " << i << " - " << item.id << ": "
       << item.pose << " " << item.upose << std::endl;
  }
    */
  os << "Not implemented";
  return os;
}

// Get the visionserver geometry.  The writes the result into the proxy
// rather than returning it to the caller.
void
VisionserverProxy::RequestGeometry()
{
  scoped_lock_t lock(mPc->mMutex);
  if (0 != playerc_visionserver_get_geom(mDevice))
    throw PlayerError("VisionserverProxy::RequestGeometry()", "error getting geom");
  return;
}
