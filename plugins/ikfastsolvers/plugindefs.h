// -*- coding: utf-8 --*
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_PLUGINDEFS_H
#define OPENRAVE_PLUGINDEFS_H

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#include <stdint.h>
#include <fstream>
#include <iostream>
#include <cmath>

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace OpenRAVE;

template <typename Real>
class IKSolutionTemplate
{
public:
    typedef Real IKReal;
    /// Gets a solution given its free parameters
    /// \param pfree The free parameters required, range is in [-pi,pi]
    void GetSolution(Real* psolution, const Real* pfree) const {
        for(size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].freeind < 0 ) {
                psolution[i] = basesol[i].foffset;
            }
            else {
                BOOST_ASSERT(pfree != NULL);
                psolution[i] = pfree[basesol[i].freeind]*basesol[i].fmul + basesol[i].foffset;
                if( psolution[i] > PI ) {
                    psolution[i] -= 2*PI;
                }
                else if( psolution[i] < -PI ) {
                    psolution[i] += 2*PI;
                }
            }
        }
    }

    /// Gets the free parameters the solution requires to be set before a full solution can be returned
    /// \return vector of indices indicating the free parameters
    const std::vector<int>& GetFree() const {
        return vfree;
    }

    struct VARIABLE
    {
        VARIABLE() : fmul(0), foffset(0), freeind(-1), maxsolutions(-1) {
            indices[0] = indices[1] = -1;
        }
        IKReal fmul, foffset; ///< joint value is fmul*sol[freeind]+foffset
        signed char freeind; ///< if >= 0, mimics another joint
        unsigned char maxsolutions; ///< max possible indices, 0 if controlled by free index or a free joint itself
        unsigned char indices[2]; ///< unique index of the solution used to keep track on what part it came from. sometimes a solution can be repeated for different indices. store at least another repeated root
    };

    std::vector<VARIABLE> basesol;           ///< solution and their offsets if joints are mimiced
    std::vector<int> vfree;

    bool Validate() const {
        for(size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].maxsolutions == (unsigned char)-1) {
                throw OPENRAVE_EXCEPTION_FORMAT("max solutions for joint %d not initialized\n",i,ORE_Assert);
            }
            if( basesol[i].maxsolutions > 0 ) {
                if( basesol[i].indices[0] >= basesol[i].maxsolutions ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("index %d >= max solutions %d for joint %d\n",(int)basesol[i].indices[0]%(int)basesol[i].maxsolutions%i,ORE_Assert);
                }
                if( basesol[i].indices[1] != (unsigned char)-1 && basesol[i].indices[1] >= basesol[i].maxsolutions ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("2nd index %d >= max solutions %d for joint %d\n",(int)basesol[i].indices[0]%(int)basesol[i].maxsolutions%i,ORE_Assert);
                }
            }
        }
        return true;
    }

    void GetSolutionIndices(std::vector<unsigned int>& v) const {
        v.resize(0);
        v.push_back(0);
        for(int i = (int)basesol.size()-1; i >= 0; --i) {
            if( basesol[i].maxsolutions != (unsigned char)-1 && basesol[i].maxsolutions > 1 ) {
                for(size_t j = 0; j < v.size(); ++j) {
                    v[j] *= basesol[i].maxsolutions;
                }
                size_t orgsize=v.size();
                if( basesol[i].indices[1] != (unsigned char)-1 ) {
                    for(size_t j = 0; j < orgsize; ++j) {
                        v.push_back(v[j]+basesol[i].indices[1]);
                    }
                }
                if( basesol[i].indices[0] != (unsigned char)-1 ) {
                    for(size_t j = 0; j < orgsize; ++j) {
                        v[j] += basesol[i].indices[0];
                    }
                }
            }
        }
    }
};

typedef IKSolutionTemplate<float> IKSolutionFloat;
typedef IKSolutionTemplate<double> IKSolutionDouble;

#define IKFAST_ASSERT BOOST_ASSERT
#define IKFAST_REAL double
#define IKFAST_NO_MAIN

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TEMPLATE(IKSolutionTemplate, 1)
#endif

#endif
