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
#include "socketconnect.h"
#include "mex.h"
#include <vector>
using namespace std;

#ifndef _WIN32
#include <errno.h>
#endif

void mexFunction(int nlhs,       mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    HostConnection* c = NULL;
    std::string ip;
    int port=4765;

    if(nrhs == 0) {
        c = new HostConnection(true);
    }
    else if( nrhs == 1 || nrhs == 3 ) {
        c = new HostConnection(false);
        c->sockd = (int)mxGetScalar(prhs[0]);

        if( nrhs == 3 ) {
            ip.resize(mxGetNumberOfElements(prhs[1])+1);
            mxGetString(prhs[1], &ip[0], ip.size());
            port = (int)mxGetScalar(prhs[2]);
        }

        // test if the socket is valid, if not
        // check if closed, only for linux systems
#ifndef _WIN32
        int opts = MSG_PEEK|MSG_DONTWAIT;
	char temp;
        int ret = recv(c->sockd, &temp, 1, opts);
	
	//printf("ret=%d, errno=%d(%d)\n", ret, errno, EAGAIN);
	if( (ret == 0 || errno==0) || (ret < 0 && errno != EWOULDBLOCK && errno != EAGAIN) ) {
	  //printf("new socket\n");
	  delete c;
	  c = new HostConnection(&ip, port);
	}
	else {
	  // retrieve the host address
	  struct sockaddr_in addr;
	  int len = sizeof(addr);
	  if( getsockname(c->sockd, (struct sockaddr*)&addr, (socklen_t*)&len) == 0 ) {
	    char strip[32];
	    unsigned int curip = ntohl(addr.sin_addr.s_addr);
	    sprintf(strip, "%d.%d.%d.%d\n", curip>>24,(curip>>16)&0xff,(curip>>8)&0xff,curip&0xff);
	    c->_ip = strip;
	    c->_port = ntohs(addr.sin_port);
	  } 
	}
#else
        u_long flags = 1;
        ioctlsocket(c->sockd, FIONBIO, &flags);
        int opts = MSG_PEEK;
        int ret;
        ret = recv(c->sockd, NULL, 0, opts);

        if( ret == 0 || WSAGetLastError() != WSAEWOULDBLOCK ) {
            // failed, create a default
            //printf("failed, create default: %s:%d\n", ip.c_str(), port);
            delete c;
            c = new HostConnection(&ip, port);
        }
        else {
            flags = 0;
            ioctlsocket(c->sockd, FIONBIO, &flags);

            //printf("here\n");
            // retrieve the host address
            struct sockaddr_in addr;
            int len = sizeof(addr);
            if( getsockname(c->sockd, (struct sockaddr*)&addr, (socklen_t*)&len) == 0 ) {
                char strip[32];
                unsigned int curip = ntohl(addr.sin_addr.s_addr);
                sprintf(strip, "%d.%d.%d.%d\n", curip>>24,(curip>>16)&0xff,(curip>>8)&0xff,curip&0xff);
                c->_ip = strip;
                c->_port = ntohs(addr.sin_port);
            }
        }

#endif
    }
    else if( nrhs == 2 ) { // ip/port pair
        ip.resize(mxGetNumberOfElements(prhs[0])+1);
        mxGetString(prhs[0], &ip[0], ip.size());
        c = new HostConnection(&ip,(int)mxGetScalar(prhs[1]));
    }
    else
    {
        mexErrMsgTxt("orcreate takes 0-2 arguments: (command string,[server ip (string)],[port (int)])");
    }

    plhs[0] = mxCreateString(c->_ip.c_str());
    plhs[1] = mxCreateDoubleScalar(c->_port);
    plhs[2] = mxCreateDoubleScalar(c->sockd);

    c->sockd = -1;
    delete c;
}
