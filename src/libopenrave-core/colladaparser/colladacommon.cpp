// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "colladacommon.h"

#include <dae/daeRawResolver.h>
#include <dae/daeStandardURIResolver.h>

namespace OpenRAVE
{
static boost::shared_ptr<DAE> s_dae;
static boost::mutex s_daemutex;
boost::shared_ptr<DAE> GetGlobalDAE(bool resetdefaults)
{
    if( !s_dae ) {
        s_dae.reset(new DAE());
        RaveAddCallbackForDestroy(boost::bind(SetGlobalDAE,boost::shared_ptr<DAE>()));
    }
    if( resetdefaults ) {
        // load the normal resolvers
        s_dae->getURIResolvers().list().clear();
        s_dae->getURIResolvers().list().append(new daeRawResolver(*s_dae));
        s_dae->getURIResolvers().list().append(new daeStandardURIResolver(*s_dae));
        // destroy all previous documents
        s_dae->clear();
        // reset options?
        //_dae->getIOPlugin()->setOption()
    }
    return s_dae;
}

void SetGlobalDAE(boost::shared_ptr<DAE> newdae)
{
    RAVELOG_VERBOSE("resetting global collada DAE\n");
    s_dae = newdae;
}

boost::mutex& GetGlobalDAEMutex()
{
    return s_daemutex;
}
}
