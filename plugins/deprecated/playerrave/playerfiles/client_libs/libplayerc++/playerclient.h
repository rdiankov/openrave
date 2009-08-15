/*
  $Id: playerclient.h,v 1.20.2.2 2006/09/25 17:00:16 gerkey Exp $
*/

#ifndef PLAYERCLIENT_H
#define PLAYERCLIENT_H

#include "libplayerc++/playerc++config.h"
#include "libplayerc++/utility.h"

#include <string>
#include <list>

#ifdef HAVE_BOOST_SIGNALS
  #include <boost/signal.hpp>
  #include <boost/bind.hpp>
#endif

#ifdef HAVE_BOOST_THREAD
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/thread.hpp>
  #include <boost/thread/xtime.hpp>
#else
  // we have to define this so we don't have to
  // comment out all the instances of scoped_lock
  // in all the proxies
  namespace boost
  {
    class thread
    {
      public:
        thread() {};
    };

    class mutex
    {
      public:
        mutex() {};
        class scoped_lock
        {
          public: scoped_lock(mutex m) {};
        };
    };
  }

#endif

namespace PlayerCc
{

class ClientProxy;

/** @brief The PlayerClient is used for communicating with the player server
 *
 * One PlayerClient object is used to control each connection to
 * a Player server.  Contained within this object are methods for changing the
 * connection parameters and obtaining access to devices.
 *
 * Since the threading functionality of the PlayerClient is built on Boost,
 * these options are conditionally available based on the Boost threading
 * library being present on the system.  The StartThread() and StopThread() are
 * the only functions conditionally available based on this.
*/
class PlayerClient
{
  friend class ClientProxy;

  // our thread type
  typedef boost::thread thread_t;

  // our mutex type
  typedef boost::mutex mutex_t;

  private:
    // list of proxies associated with us
    std::list<PlayerCc::ClientProxy*> mProxyList;

    std::list<playerc_device_info_t> mDeviceList;

    // Connect to the indicated host and port.
    // @exception throws PlayerError if unsuccessfull
    void Connect(const std::string aHostname, uint aPort);

    // Disconnect from server.
    void Disconnect();

    //  our c-client from playerc
    playerc_client_t* mClient;

    // The hostname of the server, stored for convenience
    std::string mHostname;

    // The port number of the server, stored for convenience
    uint mPort;

    // Which transport (TCP or UDP) we're using
    unsigned int mTransport;

    // Is the thread currently stopped or stopping?
    bool mIsStop;

    // This is the thread where we run @ref Run()
    thread_t* mThread;

    // A helper function for starting the thread
    void RunThread();

  public:

    /// Make a client and connect it as indicated.
    PlayerClient(const std::string aHostname=PLAYER_HOSTNAME,
                 uint aPort=PLAYER_PORTNUM,
                 int transport=PLAYERC_TRANSPORT_TCP);

    /// destructor
    ~PlayerClient();

    /// A mutex for handling synchronization
    mutex_t mMutex;

    // ideally, we'd use the read_write mutex, but I was having some problems
    // (with their code) because it's under development
    //boost::read_write_mutex mMutex;

    /// Start the run thread
    void StartThread();

    /// Stop the run thread
    void StopThread();

    /// This starts a blocking loop on @ref Read()
    void Run(uint aTimeout=10); // aTimeout in ms

    /// Stops the @ref Run() loop
    void Stop();

    /// @brief Check whether there is data waiting on the connection, blocking
    /// for up to @p timeout milliseconds (set to 0 to not block).
    ///
    /// @returns
    /// - false if there is no data waiting
    /// - true if there is data waiting
    bool Peek(uint timeout=0);

    /// @brief Set connection retry limit, which is the number of times
    /// that we'll try to reconnect to the server after a socket error.
    /// Set to -1 for inifinite retry.
    void SetRetryLimit(int limit) { playerc_client_set_retry_limit(this->mClient,limit); }

    /// @brief Get connection retry limit, which is the number of times
    /// that we'll try to reconnect to the server after a socket error.
    int GetRetryLimit() { return(this->mClient->retry_limit); }

    /// @brief Set connection retry time, which is number of seconds to
    /// wait between reconnection attempts.
    void SetRetryTime(double time) { playerc_client_set_retry_time(this->mClient,time); }

    /// @brief Set the request time limit.
    void SetRequestTimeout(uint timeoutsecs) { playerc_client_set_request_timeout(this->mClient,timeoutsecs); }
    /// @brief Get connection retry time, which is number of seconds to
    /// wait between reconnection attempts.
    double GetRetryTime() { return(this->mClient->retry_time); }

    /// @brief A blocking Read
    ///
    /// Use this method to read data from the server, blocking until at
    /// least one message is received.  Use @ref PlayerClient::Peek() to check
    /// whether any data is currently waiting.
    /// In pull mode, this will block until all data waiting on the server has
    /// been received, ensuring as up to date data as possible.
    void Read();

    /// @brief A nonblocking Read
    ///
    /// Use this method if you want to read in a nonblocking manner.  This
    /// is the equivalent of checking if Peek is true and then reading
    void ReadIfWaiting();

//    /// @brief You can change the rate at which your client receives data from the
//    /// server with this method.  The value of @p freq is interpreted as Hz;
//    /// this will be the new rate at which your client receives data (when in
//    /// continuous mode).
//    ///
//    /// @exception throws PlayerError if unsuccessfull
//     void SetFrequency(uint aFreq);

    /// @brief Set whether the client operates in Push/Pull modes
    ///
    /// You can toggle the mode in which the server sends data to your
    /// client with this method.  The @p mode should be one of
    ///   - @ref PLAYER_DATAMODE_PUSH (all data)
    ///   - @ref PLAYER_DATAMODE_PULL (data on demand)
    /// When in pull mode, it is highly recommended that a replace rule is set
    /// for data packets to prevent the server message queue becoming flooded.
    /// For a more detailed description of data modes, see @ref
    /// libplayerc_datamodes.
    ///
    /// @exception throws PlayerError if unsuccessfull
    void SetDataMode(uint aMode);

    /// @brief Set a replace rule for the clients queue on the server.
    ///
    /// If a rule with the same pattern already exists, it will be replaced
    /// with the new rule (i.e., its setting to replace will be updated).
    /// @param aReplace Should we replace these messages? true/false
    /// @param aType type of message to set replace rule for
    ///          (-1 for wildcard).  See @ref message_types.
    /// @param aSubtype message subtype to set replace rule for (-1 for
    ///          wildcard).
    /// @param aInterf Interface to set replace rule for (-1 for wildcard).
    ///          This can be used to set the replace rule for all members of a
    ///          certain interface type.  See @ref interfaces.
    ///
    /// @exception throws PlayerError if unsuccessfull
    ///
    /// @see ClientProxy::SetReplaceRule, PlayerClient::SetDataMode
    void SetReplaceRule(bool aReplace,
                        int aType = -1,
                        int aSubtype = -1,
                        int aInterf = -1);

    /// Get the list of available device ids. The data is written into the
    /// proxy structure rather than retured to the caller.
    void RequestDeviceList();

    std::list<playerc_device_info_t> GetDeviceList();

    /// Returns the hostname
    std::string GetHostname() const { return(mHostname); };

    /// Returns the port
    uint GetPort() const { return(mPort); };

    /// Get the interface code for a given name
    int LookupCode(std::string aName) const;

    /// Get the name for a given interface code
    std::string LookupName(int aCode) const;
};



}

namespace std
{
  std::ostream& operator << (std::ostream& os, const PlayerCc::PlayerClient& c);
}

#endif

