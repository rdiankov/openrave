// -*- coding: utf-8 -*-
// Copyright (C) 2006-2016 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
#include "plugindefs.h"

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/lexical_cast.hpp>

#ifdef OPENRAVE_HAS_LAPACK
#include "jacobianinverse.h"
#endif

#include "ikfastsolver.h"

#ifdef OPENRAVE_IKFAST_FLOAT32
IkSolverBasePtr CreateIkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<float> > ikfunctions, const vector<dReal>& vfreeinc, dReal ikthreshold)

{
    return IkSolverBasePtr(new IkFastSolver<float>(penv,sinput,ikfunctions,vfreeinc,ikthreshold));
}
#endif

IkSolverBasePtr CreateIkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<double> > ikfunctions, const vector<dReal>& vfreeinc, dReal ikthreshold)
{
    return IkSolverBasePtr(new IkFastSolver<double>(penv,sinput,ikfunctions,vfreeinc,ikthreshold));
}
