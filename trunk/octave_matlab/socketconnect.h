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
#ifndef TEST_H
#include <cstdlib>
#include <cstdio>

#ifdef _WIN32
#include <winsock2.h>

#define CLOSESOCKET closesocket
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#define SOCKET_ERROR -1

#include <fcntl.h>
#define SOCKET int
#define CLOSESOCKET close
#endif

#include <iostream>
#include <fstream>
#include <locale>

#include <string>
#include <cstring>
#include "mex.h"

//settings for connection to the server
#ifdef _WIN32
#define RAVE_SERVER "c:/orHost.txt"
#else
#define RAVE_SERVER "~/orHost.txt"
//#define RAVE_SERVER (std::string("/home/") + std::string(cuserid(NULL)) + std::string("/orHost.txt")).c_str()
#endif

#define RAVE_PORT 4765

#ifdef _WIN32
typedef int socklen_t;
#endif

#include <string>
using namespace std;

class HostConnection
{
public:
    HostConnection(bool bCreate, int port = 4765) {
        _port = port;
        if( bCreate ) {
            sockd = ConnectToHost(_ip,_port);
            if (sockd < 0)
                mexErrMsgTxt("Could not connect");
        }
        else sockd = -1;
    }

    HostConnection(std::string* ip) {
        if( ip != NULL ) _ip = *ip;
        _port = 4765;
        sockd = ConnectToHost(_ip,_port);
        if (sockd < 0)
            mexErrMsgTxt("Could not connect");
    }


    HostConnection(int port) {
        _port = port;
        sockd = ConnectToHost(_ip,_port);
        if (sockd < 0)
            mexErrMsgTxt("Could not connect");

    }

    HostConnection(std::string* ip,int port)
    {
        if( ip != NULL ) _ip = *ip;
        _port = port;
        sockd = ConnectToHost(_ip,_port);
        if (sockd < 0)
            mexErrMsgTxt("Could not connect");
    }

    ~HostConnection() {
        if( sockd > 0 ) {
            CLOSESOCKET(sockd);
#ifdef _WIN32
            WSACleanup();
#endif
        }
    }

    int ReadSocket(int sockfd, void* vptr, int sizetoread)
    {
        int read = 0, rc;
        char* pbuf = (char*)vptr;

        while(sizetoread > 0) {
            if ( (rc = recv(sockfd, pbuf, sizetoread,0)) <= 0 ) {
                return rc;
            }

            pbuf += rc;
            sizetoread -= rc;
            read += rc;
        }

        return read;
    }

    int ConnectToHost(std::string& ip, int port)
    {
        int                  sck;
        struct sockaddr_in   writer;
        struct hostent       *hp;

        // set to the classic locale so that number serialization/hashing works correctly
        std::locale::global(std::locale::classic());

        if(ip.size() == 0) {
            std::ifstream orHost;
            orHost.open(RAVE_SERVER);
            orHost >> ip;

            if( orHost.fail() )
                ip = "localhost";
            orHost.close();
        }

#ifdef _WIN32
        WORD      wVersionRequested;
        WSADATA   wsaData;

        wVersionRequested = MAKEWORD(1,1);
        if (WSAStartup(wVersionRequested, &wsaData) != 0) {
            mexErrMsgTxt("Init Failed");
            return -1;
        }
#endif

        hp = gethostbyname( ip.c_str());

        if( hp == 0x0 ) {
            mexErrMsgTxt("Host unknown");
            return -1;
        }

        memset( (char *) &writer, 0, sizeof(writer) );
        memcpy( (char *) &writer.sin_addr, hp->h_addr, hp->h_length );
        writer.sin_family = hp->h_addrtype;
        if(port == 0)
            writer.sin_port = htons( (short)RAVE_PORT );
        else
            writer.sin_port = htons( (short)port );

        sck = socket( AF_INET, SOCK_STREAM, 0 );
        if ( sck < 0 ) {
            mexErrMsgTxt("Could not open socket.");
            return -1;
        }

        if( connect( sck, (struct sockaddr *) &writer, sizeof(writer) ) < 0 )  {
            char str[255];
            sprintf(str, "Could not establish connection with %s", ip.c_str());
            mexErrMsgTxt(str);
            return -1;
        }
        return sck;
    }

    int Readline(void *vptr, int maxlen)
    {
        int n, rc;
        char    c, *buffer;

        buffer = (char*)vptr;

        for ( n = 1; n < maxlen; n++ ) {

            if ( (rc = recv(sockd, &c, 1,0)) == 1 ) {
                *buffer++ = c;
                if ( c == '\n' )
                    break;
            }
            else if ( rc == 0 ) {
                if ( n == 1 )
                    return 0;
                else
                    break;
            }
            else {
                return -1;
            }

        }
        *buffer = 0;
        return n;
    }

    int ReadServerBuffer(string& out)
    {
        int size;
        if( ReadSocket(sockd, &size, 4) != 4 ) {
            printf("error in reading\n");
            return -1;
        }

        out.resize(size);
        return ReadSocket(sockd, &out[0], size);
    }

    int Writeline(const void *vptr, int n)
    {
        int      nleft;
        int      nwritten;
        const char *buffer;

        buffer = (const char*)vptr;
        nleft  = n;

        while ( nleft > 0 ) {
            if ( (nwritten = send(sockd, buffer, nleft,0)) == SOCKET_ERROR ) {
                return -1;
            }
            nleft  -= nwritten;
            buffer += nwritten;
        }

        return n;
    }

    int sockd, _port;
    string _ip;
};

#define TEST_H
#endif
