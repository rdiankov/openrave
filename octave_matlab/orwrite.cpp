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
void mexFunction(int nlhs,       mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    bool bCloseSocket = true;
    HostConnection* c = NULL;
    if(nrhs == 1) {
        c = new HostConnection(true);
    }
    else if(nrhs == 2) { // regular sockid (don't release!)
        c = new HostConnection(false);
        c->sockd = (int)mxGetScalar(prhs[1]);
        bCloseSocket = false;
    }
    else if( nrhs == 3 ) { // ip/port pair
        std::string ip;
        ip.resize(mxGetNumberOfElements(prhs[1])+1);
        mxGetString(prhs[1], &ip[0], ip.size());
        c = new HostConnection(&ip,(int)mxGetScalar(prhs[2]));
    }
    else
    {
        mexErrMsgTxt("orsend takes 1 or 3 arguments: (command string,[server ip (string)],[port (int)])");
    }

    std::vector<char> v(mxGetNumberOfElements(prhs[0])+1);
    mxGetString(prhs[0], &v[0], v.size());
    v[mxGetNumberOfElements(prhs[0])] = '\n';
    c->Writeline(&v[0],v.size());

    if( !bCloseSocket )
        c->sockd = -1;
    delete c;
}
