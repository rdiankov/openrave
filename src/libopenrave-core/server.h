// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef RAVE_SERVER
#define RAVE_SERVER

#ifndef _WIN32
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif

class RaveServer;

// socket just accepts connections
class Socket
{
public: 
	struct PACKET
	{
		string cmd, arg;
	};

    Socket(RaveServer* pserver);
    ~Socket();
	bool Accept(int server_sockfd);
    void Close();

    /// returns true if connection is valid, otherwise false
    bool IsInit() { return bInit; }

    void SendData(const void* pdata, int size);
	bool ReadLine(string& s);	

    RaveServer* GetServer() { return _pserver; }

private:
    int client_sockfd;	
	int client_len;

	struct sockaddr_in client_address;
    RaveServer* _pserver;
    
    bool bInit;
};

/** manages all connections to matlab. At the moment, server can handle only one connectino at a time
*/
class RaveServer : public RaveServerBase
{
    /****
     *  Network Functions
     *  every function X has two C functions: orX and worX. orX is launched in the socket thread, worX is launched in the main worker thread.
    */

    /// \param in is the data passed from the network
    /// \param out is the return data that will be passed to the client
    /// \param ppPassData is a pointer to a void taht willl be passed to the worker thread function
    typedef bool (*OpenRaveNetworkFn)(char* in, string& out, void** ppPassData, RaveServer* pserv);
    typedef bool (*OpenRaveWorkerFn)(char* in, void* pdata, RaveServer* pserv);

    /// each network function has a function to intially processes the data on the socket function
    /// and one that is executed on the main worker thread to avoid multithreading data synchronization issues
    struct RAVENETWORKFN
    {
        RAVENETWORKFN() : fnSocketThread(NULL), fnWorker(NULL), bReturnResult(false) {}
        RAVENETWORKFN(OpenRaveNetworkFn socket, OpenRaveWorkerFn worker, bool bReturnResult) : fnSocketThread(socket), fnWorker(worker), bReturnResult(bReturnResult) {}

        OpenRaveNetworkFn fnSocketThread;
        OpenRaveWorkerFn fnWorker;
        bool bReturnResult; // if true, function is expected to return a result
    };

public:
    struct WORKERSTRUCT
    {
        OpenRaveWorkerFn fnWorker;
        void* pdata;
        string args;
    };

    RaveServer(EnvironmentBase* penv);
    ~RaveServer();

    void Destroy();
    void Reset();

    bool Init(int port);

    /// worker thread called from the main thread
    void Worker();

    bool IsInit() { return bInitThread; }
    bool IsClosing() { return bCloseThread; }

    // called from threads other than the main worker to wait until 
    void SyncWithWorkerThread();
    void ScheduleWorker(const WORKERSTRUCT& w); // multithread safe

private:

    static void* listen_thread(void* param);
    void* _listen_thread();

    static void* read_thread(void* param);
    void* _read_thread(Socket* psocket);

    int _nPort; ///< port used for listening to incoming connections

    pthread_t servthread;
    pthread_mutex_t _mutexWorker;
    pthread_cond_t _condWorker;

    PhysicsEngineBase* _pphysics;
    CollisionCheckerBase* _pcolchecker;

    list<pthread_t> listSocketThreads;

    bool bInitThread;
    bool bCloseThread;
    bool bDestroying;

    struct sockaddr_in server_address;
	int server_sockfd, server_len;

    FILE* pfLog;

    list<WORKERSTRUCT> listWorkers;

    map<string, RAVENETWORKFN> mapNetworkFns;

    bool _bWorking; ///< worker thread processing current work items
    
    friend bool worSetOptions(char* in, void* pData, RaveServer*);
    friend bool orEnvLoadScene(char* in, string& out, void **ppPassData, RaveServer* pserv);
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(Socket)
BOOST_TYPEOF_REGISTER_TYPE(RaveServer::WORKERSTRUCT)
BOOST_TYPEOF_REGISTER_TYPE(pthread_t)

#endif

#endif
