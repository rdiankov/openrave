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
 * $Id: playerclient.cc,v 1.19.2.2 2006/09/25 17:00:16 gerkey Exp $
 *
 * The C++ client
 */

#include <cassert>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <functional>

#include <time.h>

#include "playerc++.h"
#include "playerclient.h"

#include "debug.h"

using namespace PlayerCc;

PlayerClient::PlayerClient(const std::string aHostname, uint aPort, 
                           int aTransport) :
  mClient(NULL),
  mHostname(aHostname),
  mPort(aPort),
  mTransport(aTransport)
{
#ifdef HAVE_BOOST_THREAD
  mIsStop=true;
  mThread = NULL;
#endif
  Connect(mHostname, mPort);
}

PlayerClient::~PlayerClient()
{
#ifdef HAVE_BOOST_THREAD
  if (!mIsStop)
  {
    StopThread();
  }
#endif

  Disconnect();
}

void PlayerClient::Connect(const std::string aHostname, uint aPort)
{
  assert("" != aHostname);
  assert(0  != aPort);

  LOG("Connecting " << *this);

  mClient = playerc_client_create(NULL, aHostname.c_str(), aPort);
  playerc_client_set_transport(mClient, mTransport);
  if (mClient == NULL)
  {
    throw PlayerError("PlayerClient::Connect()", playerc_error_str());
  }
  if (0 != playerc_client_connect(mClient))
  {
  	playerc_client_destroy(mClient);
    throw PlayerError("PlayerClient::Connect()", playerc_error_str());
  }
  EVAL(mClient);
}

void PlayerClient::Disconnect()
{
  LOG("Disconnecting " << *this);

  std::for_each(mProxyList.begin(),
                mProxyList.end(),
                std::mem_fun(&ClientProxy::Unsubscribe));

  if (NULL != mClient)
  {
    playerc_client_disconnect(mClient);
    playerc_client_destroy(mClient);
    mClient = NULL;
  }
}

void PlayerClient::StartThread()
{
#ifdef HAVE_BOOST_THREAD
  assert(NULL == mThread);
  mThread = new boost::thread(boost::bind(&PlayerClient::RunThread, this));
  mIsStop = false;
#else
  throw PlayerError("PlayerClient::StartThread","Thread support not included");
#endif
}

void PlayerClient::StopThread()
{
#ifdef HAVE_BOOST_THREAD
  if (mIsStop) // at least do the check
    return;

  Stop();
  assert(mThread);
  mThread->join();
  delete mThread;
  mThread = NULL;
  PRINT("joined");
#else
  throw PlayerError("PlayerClient::StopThread","Thread support not included");
#endif
}

// non-blocking
void PlayerClient::RunThread()
{
#ifdef HAVE_BOOST_THREAD
  mIsStop = false;
  PRINT("starting run");
  while (!mIsStop)
  {
    if( mClient->mode == PLAYER_DATAMODE_PUSH){
      if (Peek())
      {
        Read();
      }
    } else {
      Read();
    };
    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC);
    // we sleep for 0 seconds
    boost::thread::sleep(xt);
  }
#else
  throw PlayerError("PlayerClient::RunThread","Thread support not included");
#endif

}

// blocking
void PlayerClient::Run(uint aTimeout)
{
  timespec sleep = {0,aTimeout*1000000};
  mIsStop = false;
  PRINT("starting run");
  while (!mIsStop)
  {
    if( mClient->mode == PLAYER_DATAMODE_PUSH){
      if (Peek())
      {
        Read();
      }
    } else {
      Read();
    };
    nanosleep(&sleep, NULL);
  }
}

void PlayerClient::Stop()
{
  mIsStop = true;
}

bool PlayerClient::Peek(uint aTimeout)
{
  ClientProxy::scoped_lock_t lock(mMutex);
  //EVAL(playerc_client_peek(mClient, aTimeout));
  return playerc_client_peek(mClient, aTimeout);
}


void PlayerClient::ReadIfWaiting()
{
  if (Peek())
    Read();
}

void PlayerClient::Read()
{
  assert(NULL!=mClient);
  PRINT("read()");
  // first read the data
  {
    ClientProxy::scoped_lock_t lock(mMutex);
    if (NULL==playerc_client_read(mClient))
    {
      throw PlayerError("PlayerClient::Read()", playerc_error_str());
    }
  }

  std::for_each(mProxyList.begin(),
                mProxyList.end(),
                std::mem_fun(&ClientProxy::ReadSignal));
}

void PlayerClient::RequestDeviceList()
{
  ClientProxy::scoped_lock_t lock(mMutex);
  if (0!=playerc_client_get_devlist(mClient))
  {
    throw PlayerError("PlayerClient::RequestDeviceList()", playerc_error_str());
  }
}

std::list<playerc_device_info_t> PlayerClient::GetDeviceList()
{
  std::list<playerc_device_info_t> dev_list;
  for (int i=0; i < mClient->devinfo_count; ++i)
  {
    PRINT(mClient->devinfos[i]);
    dev_list.push_back(mClient->devinfos[i]);
  }

  return dev_list;
}

// change data delivery mode
// valid modes are given in include/messages.h
void PlayerClient::SetDataMode(uint aMode)
{
  assert((aMode==PLAYER_DATAMODE_PULL)||(aMode==PLAYER_DATAMODE_PUSH));

  ClientProxy::scoped_lock_t lock(mMutex);
  if (0!=playerc_client_datamode(mClient, aMode))
  {
    throw PlayerError("PlayerClient::SetDataMode()", playerc_error_str());
  }

}

// add replace rule
void PlayerClient::SetReplaceRule(bool aReplace,
                                  int aType,
                                  int aSubtype,
                                  int aInterf)
{
  ClientProxy::scoped_lock_t lock(mMutex);
  if (0!=playerc_client_set_replace_rule(mClient,
                                         aInterf,
                                         -1,
                                         aType,
                                         aSubtype,
                                         aReplace))
  {
    throw PlayerError("PlayerClient::SetReplaceRule()", playerc_error_str());
  }
}

int PlayerClient::LookupCode(std::string aName) const
{
  return playerc_lookup_code(aName.c_str());
}

std::string PlayerClient::LookupName(int aCode) const
{
  return std::string(playerc_lookup_name(aCode));
}

std::ostream&
std::operator << (std::ostream& os, const PlayerCc::PlayerClient& c)
{
  return os << c.GetHostname()
            << ": "
            << c.GetPort();
}
