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
/**
   \file   ravep.h
   \brief  Defines the private headers that every source file used to build openrave must include (used in place of rave.h).
 */

#ifndef RAVE_RAVEP_H
#define RAVE_RAVEP_H

#define RAVE_PRIVATE

#include <cstdio>
#include <cmath>
#include <cstdlib>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); ++(it))
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); ++(it))
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include <time.h>

#ifdef _WIN32
static const char s_filesep = '\\';
#else
static const char s_filesep = '/';
#endif

// need the prototypes in order to keep them free of the OpenRAVE namespace
namespace OpenRAVEXMLParser
{
class InterfaceXMLReader;
class KinBodyXMLReader;
class LinkXMLReader;
class JointXMLReader;
class ManipulatorXMLReader;
class AttachedSensorXMLReader;
class RobotXMLReader;
class EnvironmentXMLReader;
}

class Environment;
class RaveDatabase;
class XFileReader;

#include "openrave-core.h"
#include <openrave/utils.h>

// exports from libopenrave.h
namespace OpenRAVE
{

class ColladaReader;
class ColladaWriter;

bool RaveInvertFileLookup(std::string& newfilename, const std::string& filename);

RobotBasePtr CreateGenericRobot(EnvironmentBasePtr penv, std::istream& sinput);
TrajectoryBasePtr CreateGenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput);
PhysicsEngineBasePtr CreateGenericPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput);
CollisionCheckerBasePtr CreateGenericCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput);

enum MassType
{
    MT_None = 0,
    MT_MimicGeom,
    MT_MimicGeomMass,
    MT_Box,
    MT_BoxMass,         // use total mass instead of density
    MT_Sphere,
    MT_SphereMass,
    MT_Custom,         // manually specify center of mass and inertia matrix
};

/// \brief mass of objects
class MASS
{
public:
    MASS(const MASS& m) {
        t = m.t;
        fTotalMass = m.fTotalMass;
    }
    MASS() : fTotalMass(0) {
        for(int i = 0; i < 12; ++i) {
            t.m[i] = 0;
        }
    }
    static MASS GetBoxMass(Vector extents, Vector pos, dReal totalmass)
    {
        MASS m;
        m.fTotalMass = totalmass;
        m.t = TransformMatrix();
        m.t.m[0] = totalmass/(dReal)3.0 * (extents.y*extents.y + extents.z*extents.z);
        m.t.m[4*1+1]= totalmass/(dReal)3.0 * (extents.x*extents.x + extents.z*extents.z);
        m.t.m[4*2+2]= totalmass/(dReal)3.0 * (extents.x*extents.x + extents.y*extents.y);
        m.t.trans = pos;
        return m;
    }
    static MASS GetBoxMassD(Vector extents, Vector pos, dReal density)
    {
        return GetBoxMass(extents, pos, 8.0f*extents.x*extents.y*extents.z*density);
    }
    static MASS GetSphericalMass(dReal radius, Vector pos, dReal totalmass)
    {
        MASS m;
        m.fTotalMass = totalmass;
        m.t = TransformMatrix();
        m.t.m[0] = m.t.m[4*1+1] = m.t.m[4*2+2]= (dReal)0.4 * totalmass * radius*radius;
        m.t.trans = pos;
        return m;
    }
    static MASS GetSphericalMassD(dReal radius, Vector pos, dReal density)
    {
        return GetSphericalMass(radius, pos, dReal(4.0)/dReal(3.0) * PI * radius * radius * radius * density);
    }
    static MASS GetCylinderMass(dReal radius, dReal height, Vector pos, dReal totalmass)
    {
        MASS m;
        m.fTotalMass = totalmass;
        m.t = TransformMatrix();
        dReal r2 = radius*radius;
        // axis pointed toward z
        m.t.m[0] = m.t.m[4*1+1] = totalmass*(dReal(0.25)*r2 + (dReal(1.0)/dReal(12.0))*height*height);
        m.t.m[4*2+2] = totalmass*dReal(0.5)*r2;
        return m;
    }
    static MASS GetCylinderMassD(dReal radius, dReal height, Vector pos, dReal density)
    {
        return GetCylinderMass(radius, height, pos, PI*radius*radius*height*density);
    }

    /// \brief adds two masses together
    MASS operator+(const MASS &r) const
    {
        MASS mnew;
        if( fTotalMass+r.fTotalMass == 0 ) {
            return mnew;
        }
        mnew.fTotalMass = fTotalMass + r.fTotalMass;
        mnew.t.trans = (fTotalMass*t.trans+r.fTotalMass*r.t.trans)*((dReal)1.0/mnew.fTotalMass);
        MASS m0=MASS(*this);
        m0.ChangeCenterOfRotation(mnew.t.trans);
        MASS m1=MASS(r);
        m1.ChangeCenterOfRotation(mnew.t.trans);
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                mnew.t.m[4*i+j] = m0.t.m[4*i+j] + m1.t.m[4*i+j];
            }
        }
        return mnew;
    }

    /// \brief adds a mass to the current mass
    MASS& operator+=(const MASS &r)
    {
        MASS mnew = operator+(r);
        t = mnew.t;
        fTotalMass = mnew.fTotalMass;
        return *this;
    }

    const Vector& GetCOM() const {
        return t.trans;
    }

    /// \brief transform the center of mass and inertia matrix by trans
    MASS& ChangeCoordinateSystem(const Transform& trans)
    {
        Vector oldcom = t.trans;
        TransformMatrix trot = matrixFromQuat(trans.rot);
        t = trot.rotate(t.rotate(trot.inverse())); // rotate inertia
        t.trans = trans*oldcom;
        return *this;
    }

    /// \brief changes the center of rotation (ie center of mass)
    MASS& ChangeCenterOfRotation(const Vector& newcor)
    {
        Vector v = newcor-t.trans;
        // translate the inertia tensor
        dReal x2 = v.x*v.x, y2 = v.y*v.y, z2 = v.z*v.z;
        t.m[0] += fTotalMass * (y2+z2);     t.m[1] -= fTotalMass * v.x * v.y;   t.m[2] -= fTotalMass * v.x * v.z;
        t.m[4] -= fTotalMass * v.y * v.x;   t.m[5] += fTotalMass * (x2+z2);     t.m[6] -= fTotalMass * v.y * v.z;
        t.m[8] -= fTotalMass * v.z * v.x;   t.m[9] -= fTotalMass * v.z * v.y;   t.m[10] += fTotalMass * (x2+y2);
        return *this;
    }

    /// \brief computes the frame of reference so that the inertia matrix is reduced to a diagonal
    void GetMassFrame(Transform& tmassframe, Vector& vinertiamoments) const {
        double fCovariance[9] = { t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};
        double eigenvalues[3], eigenvectors[9];
        mathextra::EigenSymmetric3(fCovariance,eigenvalues,eigenvectors);
        TransformMatrix tinertiaframe;
        for(int j = 0; j < 3; ++j) {
            tinertiaframe.m[4*j+0] = eigenvectors[3*j];
            tinertiaframe.m[4*j+1] = eigenvectors[3*j+1];
            tinertiaframe.m[4*j+2] = eigenvectors[3*j+2];
        }
        tmassframe.rot = quatFromMatrix(tinertiaframe);
        tmassframe.trans = t.trans;
        vinertiamoments[0] = eigenvalues[0];
        vinertiamoments[1] = eigenvalues[1];
        vinertiamoments[2] = eigenvalues[2];
    }

    TransformMatrix t;
    dReal fTotalMass;
};

}

using namespace OpenRAVE;
using namespace std;

namespace OpenRAVEXMLParser
{
class InterfaceXMLReadable : public XMLReadable
{
public:
    InterfaceXMLReadable(InterfaceBasePtr pinterface) : XMLReadable(pinterface->GetXMLId()), _pinterface(pinterface) {
    }
    virtual ~InterfaceXMLReadable() {
    }
    InterfaceBasePtr _pinterface;
};

int& GetXMLErrorCount();

/// \param bResetParseDirectory if true, will reset the parse directory to the current working directory
bool ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename);
bool ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata);
BaseXMLReaderPtr CreateEnvironmentReader(EnvironmentBasePtr penv, const AttributesList& atts);
BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, InterfaceType type, InterfaceBasePtr& pinterface, const std::string& xmltag, const AttributesList& atts);
BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, const AttributesList& atts, bool bAddToEnvironment);
bool CreateTriMeshData(EnvironmentBasePtr, const std::string& filename, const Vector &vscale, TriMesh& trimesh, RaveVector<float>&diffuseColor, RaveVector<float>&ambientColor, float &ftransparency);
bool CreateGeometries(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, std::list<KinBody::GeometryInfo>& listGeometries);
}

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define _strnicmp strncasecmp
#define _stricmp strcasecmp
#else
#define _strnicmp strncasecmp
#define _stricmp strcasecmp
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); ++(it))

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assert.hpp>
#include <boost/version.hpp>

bool RaveParseXFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList& atts);
bool RaveParseXFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename,const AttributesList& atts);
bool RaveParseXData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::vector<char>& data,const AttributesList& atts);
bool RaveParseXData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::vector<char>& data,const AttributesList& atts);

#endif
