// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file openrave.h
    \brief  Defines the public headers that every plugin must include in order to use openrave properly.
 */
#ifndef OPENRAVE_H
#define OPENRAVE_H

#include <cstdio>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>

#include <stdint.h>

#ifdef _MSC_VER

#pragma warning(disable:4251)// needs to have dll-interface to be used by clients of class
#pragma warning(disable:4190)// C-linkage specified, but returns UDT 'boost::shared_ptr<T>' which is incompatible with C
#pragma warning(disable:4819)//The file contains a character that cannot be represented in the current code page (932). Save the file in Unicode format to prevent data loss using native typeof

// needed to get typeof working
//#include <boost/typeof/std/string.hpp>
//#include <boost/typeof/std/vector.hpp>
//#include <boost/typeof/std/list.hpp>
//#include <boost/typeof/std/map.hpp>
//#include <boost/typeof/std/set.hpp>
//#include <boost/typeof/std/string.hpp>

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif

#else
#endif

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#include <iomanip>
#include <fstream>
#include <sstream>

// QTBUG-22829 alternative workaround
#ifndef Q_MOC_RUN

#include <boost/version.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/static_assert.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#include <boost/make_shared.hpp>
//#include <boost/cstdint.hpp>

#endif

#if defined(__GNUC__)
#define RAVE_DEPRECATED __attribute__((deprecated))
#else
#define RAVE_DEPRECATED
#endif

#include <openrave/smart_ptr.h>
#include <openrave/openraveexception.h>

/// The entire %OpenRAVE library
namespace OpenRAVE {

#include <openrave/config.h>
#include <openrave/interfacehashes.h>

}

#include <rapidjson/document.h>

#include <openrave/logging.h>

namespace OpenRAVE {

#if OPENRAVE_PRECISION // 1 if double precision
typedef double dReal;
#define g_fEpsilon 1e-15
#else
typedef float dReal;
#define g_fEpsilon 2e-7f
#endif

/// \brief openrave constant for PI, could be replaced by accurate precision number depending on choice of dReal.
static const dReal PI = dReal(3.14159265358979323846);

/// Wrappers of common basic math functions, allows OpenRAVE to control the precision requirements.
/// \ingroup affine_math
//@{

/// \brief exponential
OPENRAVE_API dReal RaveExp(dReal f);
/// \brief logarithm
OPENRAVE_API dReal RaveLog(dReal f);
/// \brief cosine
OPENRAVE_API dReal RaveCos(dReal f);
/// \brief sine
OPENRAVE_API dReal RaveSin(dReal f);
/// \brief tangent
OPENRAVE_API dReal RaveTan(dReal f);
/// \brief base 2 logarithm
OPENRAVE_API dReal RaveLog2(dReal f);
/// \brief base 10 logarithm
OPENRAVE_API dReal RaveLog10(dReal f);
/// \brief arccosine
OPENRAVE_API dReal RaveAcos(dReal f);
/// \brief arcsine
OPENRAVE_API dReal RaveAsin(dReal f);
/// \brief arctangent2 covering entire circle
OPENRAVE_API dReal RaveAtan2(dReal fy, dReal fx);
/// \brief power x^y
OPENRAVE_API dReal RavePow(dReal fx, dReal fy);
/// \brief square-root
OPENRAVE_API dReal RaveSqrt(dReal f);
/// \brief absolute value
OPENRAVE_API dReal RaveFabs(dReal f);
/// \brief ceil
OPENRAVE_API dReal RaveCeil(dReal f);

//@}

class OPENRAVE_LOCAL CaseInsensitiveCompare
{
public:
    bool operator() (const std::string & s1, const std::string& s2) const
    {
        std::string::const_iterator it1=s1.begin();
        std::string::const_iterator it2=s2.begin();

        //has the end of at least one of the strings been reached?
        while ( (it1!=s1.end()) && (it2!=s2.end()) )  {
            if(::toupper(*it1) != ::toupper(*it2)) {     //letters differ?
                // return -1 to indicate 'smaller than', 1 otherwise
                return ::toupper(*it1) < ::toupper(*it2);
            }
            //proceed to the next character in each string
            ++it1;
            ++it2;
        }
        std::size_t size1=s1.size(), size2=s2.size();     // cache lengths
        //return -1,0 or 1 according to strings' lengths
        if (size1==size2) {
            return 0;
        }
        return size1<size2;
    }
};

/// \brief base class for all user data
class OPENRAVE_API UserData
{
public:
    virtual ~UserData() {
    }
};
typedef boost::shared_ptr<UserData> UserDataPtr;
typedef boost::weak_ptr<UserData> UserDataWeakPtr;

/// \brief user data that can serialize/deserialize itself
class OPENRAVE_API SerializableData : public UserData
{
public:
    virtual ~SerializableData() {
    }

    /// \brief output the data of the object
    virtual void Serialize(std::ostream& O, int options=0) const = 0;

    /// \brief initialize the object
    virtual void Deserialize(std::istream& I) = 0;
};
typedef boost::shared_ptr<SerializableData> SerializableDataPtr;
typedef boost::weak_ptr<SerializableData> SerializableDataWeakPtr;

/// \brief Enumeration of all the interfaces.
enum InterfaceType
{
    PT_Planner=1, ///< describes \ref PlannerBase interface
    PT_Robot=2, ///< describes \ref RobotBase interface
    PT_SensorSystem=3, ///< describes \ref SensorSystemBase interface
    PT_Controller=4, ///< describes \ref ControllerBase interface
    PT_Module=5, ///< describes \ref ModuleBase interface
    PT_ProblemInstance=5, ///< describes \ref ModuleBase interface
    PT_IkSolver=6, ///< describes \ref IkSolverBase interface
    PT_InverseKinematicsSolver=6, ///< describes \ref IkSolverBase interface
    PT_KinBody=7, ///< describes \ref KinBody
    PT_PhysicsEngine=8, ///< describes \ref PhysicsEngineBase
    PT_Sensor=9, ///< describes \ref SensorBase
    PT_CollisionChecker=10, ///< describes \ref CollisionCheckerBase
    PT_Trajectory=11, ///< describes \ref TrajectoryBase
    PT_Viewer=12, ///< describes \ref ViewerBase
    PT_SpaceSampler=13, ///< describes \ref SamplerBase
    PT_NumberOfInterfaces=13 ///< number of interfaces, do not forget to update
};

class CollisionReport;
class InterfaceBase;
class IkSolverBase;
class TrajectoryBase;
class ControllerBase;
class PlannerBase;
class RobotBase;
class ModuleBase;
class EnvironmentBase;
class KinBody;
class SensorSystemBase;
class PhysicsEngineBase;
class SensorBase;
class CollisionCheckerBase;
class ViewerBase;
class SpaceSamplerBase;
class IkParameterization;
class ConfigurationSpecification;
class IkReturn;

typedef boost::shared_ptr<CollisionReport> CollisionReportPtr;
typedef boost::shared_ptr<CollisionReport const> CollisionReportConstPtr;
typedef boost::shared_ptr<InterfaceBase> InterfaceBasePtr;
typedef boost::shared_ptr<InterfaceBase const> InterfaceBaseConstPtr;
typedef boost::weak_ptr<InterfaceBase> InterfaceBaseWeakPtr;
typedef boost::shared_ptr<KinBody> KinBodyPtr;
typedef boost::shared_ptr<KinBody const> KinBodyConstPtr;
typedef boost::weak_ptr<KinBody> KinBodyWeakPtr;
typedef boost::shared_ptr<RobotBase> RobotBasePtr;
typedef boost::shared_ptr<RobotBase const> RobotBaseConstPtr;
typedef boost::weak_ptr<RobotBase> RobotBaseWeakPtr;
typedef boost::shared_ptr<CollisionCheckerBase> CollisionCheckerBasePtr;
typedef boost::shared_ptr<CollisionCheckerBase const> CollisionCheckerBaseConstPtr;
typedef boost::weak_ptr<CollisionCheckerBase> CollisionCheckerBaseWeakPtr;
typedef boost::shared_ptr<ControllerBase> ControllerBasePtr;
typedef boost::shared_ptr<ControllerBase const> ControllerBaseConstPtr;
typedef boost::weak_ptr<ControllerBase> ControllerBaseWeakPtr;
typedef boost::shared_ptr<IkSolverBase> IkSolverBasePtr;
typedef boost::shared_ptr<IkSolverBase const> IkSolverBaseConstPtr;
typedef boost::weak_ptr<IkSolverBase> IkSolverBaseWeakPtr;
typedef boost::shared_ptr<PhysicsEngineBase> PhysicsEngineBasePtr;
typedef boost::shared_ptr<PhysicsEngineBase const> PhysicsEngineBaseConstPtr;
typedef boost::weak_ptr<PhysicsEngineBase> PhysicsEngineBaseWeakPtr;
typedef boost::shared_ptr<PlannerBase> PlannerBasePtr;
typedef boost::shared_ptr<PlannerBase const> PlannerBaseConstPtr;
typedef boost::weak_ptr<PlannerBase> PlannerBaseWeakPtr;
typedef boost::shared_ptr<ModuleBase> ModuleBasePtr;
typedef boost::shared_ptr<ModuleBase const> ModuleBaseConstPtr;
typedef boost::weak_ptr<ModuleBase> ModuleBaseWeakPtr;
typedef boost::shared_ptr<SensorBase> SensorBasePtr;
typedef boost::shared_ptr<SensorBase const> SensorBaseConstPtr;
typedef boost::weak_ptr<SensorBase> SensorBaseWeakPtr;
typedef boost::shared_ptr<SensorSystemBase> SensorSystemBasePtr;
typedef boost::shared_ptr<SensorSystemBase const> SensorSystemBaseConstPtr;
typedef boost::weak_ptr<SensorSystemBase> SensorSystemBaseWeakPtr;
typedef boost::shared_ptr<TrajectoryBase> TrajectoryBasePtr;
typedef boost::shared_ptr<TrajectoryBase const> TrajectoryBaseConstPtr;
typedef boost::weak_ptr<TrajectoryBase> TrajectoryBaseWeakPtr;
typedef boost::shared_ptr<ViewerBase> ViewerBasePtr;
typedef boost::shared_ptr<ViewerBase const> ViewerBaseConstPtr;
typedef boost::weak_ptr<ViewerBase> ViewerBaseWeakPtr;
typedef boost::shared_ptr<SpaceSamplerBase> SpaceSamplerBasePtr;
typedef boost::shared_ptr<SpaceSamplerBase const> SpaceSamplerBaseConstPtr;
typedef boost::weak_ptr<SpaceSamplerBase> SpaceSamplerBaseWeakPtr;
typedef boost::shared_ptr<EnvironmentBase> EnvironmentBasePtr;
typedef boost::shared_ptr<EnvironmentBase const> EnvironmentBaseConstPtr;
typedef boost::weak_ptr<EnvironmentBase> EnvironmentBaseWeakPtr;

typedef boost::shared_ptr<IkReturn> IkReturnPtr;
typedef boost::shared_ptr<IkReturn const> IkReturnConstPtr;
typedef boost::weak_ptr<IkReturn> IkReturnWeakPtr;

class BaseXMLReader;
typedef boost::shared_ptr<BaseXMLReader> BaseXMLReaderPtr;
typedef boost::shared_ptr<BaseXMLReader const> BaseXMLReaderConstPtr;
class BaseXMLWriter;
typedef boost::shared_ptr<BaseXMLWriter> BaseXMLWriterPtr;
typedef boost::shared_ptr<BaseXMLWriter const> BaseXMLWriterConstPtr;


///< Cloning Options for interfaces and environments
enum CloningOptions {
    Clone_Bodies = 1, ///< clone all the bodies/robots of the environment, exclude attached interfaces like sensors/controllers
    Clone_Viewer = 2, ///< clone the viewer type, although figures won't be copied, new viewer does try to match views
    Clone_Simulation = 4, ///< clone the physics engine and simulation state (ie, timesteps, gravity)
    Clone_RealControllers = 8, ///< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
    Clone_Sensors = 0x0010, ///< if specified, will clone the sensors attached to the robot and added to the environment
    Clone_Modules = 0x0020, ///< if specified, will clone the modules attached to the environment
    Clone_IgnoreAttachedBodies = 0x00010001, ///< if set, then ignore cloning any attached bodies so _listAttachedBodies becomes empty. Usually used to control grabbing states.
    Clone_All = 0xffffffff,
};

/// base class for readable interfaces
class OPENRAVE_API Readable : public UserData
{
public:
    Readable() {}
    virtual ~Readable() {}

    Readable(const std::string& xmlid) : __xmlid(xmlid) {
    }

    const std::string& GetXMLId() const {
        return __xmlid;
    }

    /// \brief serializes the interface
    ///
    /// \return true if serialized
    virtual bool SerializeXML(BaseXMLWriterPtr writer, int options=0) const = 0;//{
//        return false;
//    }

    /// \return true if serialized
    virtual bool SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const {
        return false;
    }

    /// \return true if deserialized
    virtual bool DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale) {
        return false;
    }

    virtual bool operator==(const Readable& other) {
        return false;
    }

    virtual bool operator!=(const Readable& other) {
        return !operator==(other);
    }

private:
    std::string __xmlid;
};

typedef boost::shared_ptr<Readable> ReadablePtr;
typedef boost::shared_ptr<Readable const> ReadableConstPtr;

/// \brief a list of key-value pairs. It is possible for keys to repeat.
typedef std::list<std::pair<std::string,std::string> > AttributesList;

/// \brief base class for all xml readers. XMLReaders are used to process data from xml files.
///
/// Custom readers can be registered through \ref RaveRegisterXMLReader.
class OPENRAVE_API BaseXMLReader : public boost::enable_shared_from_this<BaseXMLReader>
{
public:
    enum ProcessElement
    {
        PE_Pass=0,     ///< current tag was not supported, so pass onto another class
        PE_Support=1,     ///< current tag will be processed by this class
        PE_Ignore=2,     ///< current tag and all its children should be ignored
    };
    BaseXMLReader() {
    }
    virtual ~BaseXMLReader() {
    }

    /// a readable interface that stores the information processsed for the current tag
    /// This pointer is used to the InterfaceBase class registered readers
    virtual ReadablePtr GetReadable() {
        return ReadablePtr();
    }

    /// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \param atts string of attributes where the first std::string is the attribute name and second is the value
    /// \return true if tag is accepted and this class will process it, otherwise false
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) = 0;

    /// Gets called at the end of each "</type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \return true if XMLReader has finished parsing (one condition is that name==_fieldname) , otherwise false
    virtual bool endElement(const std::string& name) = 0;

    /// gets called for all data in between tags.
    /// \param ch a string to the data
    virtual void characters(const std::string& ch) = 0;

    /// XML filename/resource used for this class (can be empty)
    std::string _filename;
};
typedef boost::function<BaseXMLReaderPtr(InterfaceBasePtr, const AttributesList&)> CreateXMLReaderFn;

/// \brief reads until the tag ends
class OPENRAVE_API DummyXMLReader : public BaseXMLReader
{
public:
    DummyXMLReader(const std::string& fieldname, const std::string& parentname, boost::shared_ptr<std::ostream> osrecord = boost::shared_ptr<std::ostream>());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
    const std::string& GetFieldName() const {
        return _fieldname;
    }
    virtual boost::shared_ptr<std::ostream> GetStream() const {
        return _osrecord;
    }
private:
    std::string _parentname;     /// XML filename
    std::string _fieldname;
    boost::shared_ptr<std::ostream> _osrecord;     ///< used to store the xml data
    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

/// \brief base class for writing to XML files.
///
/// OpenRAVE Interfaces accept a BaseXMLWriter instance and call its write methods to write the data.
class OPENRAVE_API BaseXMLWriter : public boost::enable_shared_from_this<BaseXMLWriter>
{
public:
    virtual ~BaseXMLWriter() {
    }
    /// \brief return the format for the data writing, should be all lower capitals.
    ///
    /// Samples formats are 'openrave', 'collada'
    virtual const std::string& GetFormat() const = 0;

    /// \brief saves character data to the child. Special characters like '<' are automatically converted to fit inside XML.
    ///
    /// \throw openrave_exception throws if this element cannot have character data or the character data was not written
    virtual void SetCharData(const std::string& data) = 0;

    /// \brief returns a writer for child elements
    virtual BaseXMLWriterPtr AddChild(const std::string& xmltag, const AttributesList& atts=AttributesList()) = 0;
};

/// \brief base class for all json readers. JSONReaders are used to process data from json files.
///
/// Custom readers can be registered through \ref RaveRegisterJSONReader.
class OPENRAVE_API BaseJSONReader : public boost::enable_shared_from_this<BaseJSONReader>
{
public:

    BaseJSONReader() {}
    virtual ~BaseJSONReader() {}

    /// a readable interface that stores the information processsed for the current tag
    /// This pointer is used to the InterfaceBase class registered readers
    virtual ReadablePtr GetReadable() {
        return ReadablePtr();
    }

    /// by default, json reader will simply call readable's deserialize function
    virtual void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale=1.0) {
        ReadablePtr pReadable = GetReadable();
        if (!!pReadable) {
            pReadable->DeserializeJSON(value, fUnitScale);
        }
    }
};
typedef boost::shared_ptr<BaseJSONReader> BaseJSONReaderPtr;
typedef boost::shared_ptr<BaseJSONReader const> BaseJSONReaderConstPtr;
typedef boost::function<BaseJSONReaderPtr(InterfaceBasePtr, const AttributesList&)> CreateJSONReaderFn;

} // end namespace OpenRAVE

// define the math functions
#if OPENRAVE_PRECISION // 1 if double precision
#define OPENRAVE_MATH_EXP_DOUBLE RaveExp
#define OPENRAVE_MATH_LOG_DOUBLE RaveLog
#define OPENRAVE_MATH_COS_DOUBLE RaveCos
#define OPENRAVE_MATH_SIN_DOUBLE RaveSin
#define OPENRAVE_MATH_TAN_DOUBLE RaveTan
#define OPENRAVE_MATH_LOG2_DOUBLE RaveLog2
#define OPENRAVE_MATH_LOG10_DOUBLE RaveLog10
#define OPENRAVE_MATH_ACOS_DOUBLE RaveAcos
#define OPENRAVE_MATH_ASIN_DOUBLE RaveAsin
#define OPENRAVE_MATH_ATAN2_DOUBLE RaveAtan2
#define OPENRAVE_MATH_POW_DOUBLE RavePow
#define OPENRAVE_MATH_SQRT_DOUBLE RaveSqrt
#define OPENRAVE_MATH_FABS_DOUBLE RaveFabs
#else // 32bit float
#define OPENRAVE_MATH_EXP_FLOAT RaveExp
#define OPENRAVE_MATH_LOG_FLOAT RaveLog
#define OPENRAVE_MATH_COS_FLOAT RaveCos
#define OPENRAVE_MATH_SIN_FLOAT RaveSin
#define OPENRAVE_MATH_TAN_FLOAT RaveTan
#define OPENRAVE_MATH_LOG2_FLOAT RaveLog2
#define OPENRAVE_MATH_LOG10_FLOAT RaveLog10
#define OPENRAVE_MATH_ACOS_FLOAT RaveAcos
#define OPENRAVE_MATH_ASIN_FLOAT RaveAsin
#define OPENRAVE_MATH_ATAN2_FLOAT RaveAtan2
#define OPENRAVE_MATH_POW_FLOAT RavePow
#define OPENRAVE_MATH_SQRT_FLOAT RaveSqrt
#define OPENRAVE_MATH_FABS_FLOAT RaveFabs
#endif

#include <openrave/geometry.h>
#include <openrave/mathextra.h>

namespace OpenRAVE {
using geometry::RaveVector;
using geometry::RaveTransform;
using geometry::RaveTransformMatrix;
typedef RaveVector<dReal> Vector;
typedef RaveTransform<dReal> Transform;
typedef boost::shared_ptr< RaveTransform<dReal> > TransformPtr;
typedef boost::shared_ptr< RaveTransform<dReal> const > TransformConstPtr;
typedef RaveTransformMatrix<dReal> TransformMatrix;
typedef boost::shared_ptr< RaveTransformMatrix<dReal> > TransformMatrixPtr;
typedef boost::shared_ptr< RaveTransformMatrix<dReal> const > TransformMatrixConstPtr;
typedef geometry::obb<dReal> OBB;
typedef geometry::aabb<dReal> AABB;
typedef geometry::ray<dReal> RAY;

// for compatibility
//@{
using mathextra::dot2;
using mathextra::dot3;
using mathextra::dot4;
using mathextra::normalize2;
using mathextra::normalize3;
using mathextra::normalize4;
using mathextra::cross3;
using mathextra::inv3;
using mathextra::inv4;
using mathextra::lengthsqr2;
using mathextra::lengthsqr3;
using mathextra::lengthsqr4;
using mathextra::mult4;
//@}

/// \brief The types of inverse kinematics parameterizations supported.
///
/// The minimum degree of freedoms required is set in the upper 4 bits of each type.
/// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
/// The lower bits contain a unique id of the type.
enum IkParameterizationType {
    IKP_None=0,
    IKP_Transform6D=0x67000001,     ///< end effector reaches desired 6D transformation
    IKP_Rotation3D=0x34000002,     ///< end effector reaches desired 3D rotation
    IKP_Translation3D=0x33000003,     ///< end effector origin reaches desired 3D translation
    IKP_Direction3D=0x23000004,     ///< direction on end effector coordinate system reaches desired direction
    IKP_Ray4D=0x46000005,     ///< ray on end effector coordinate system reaches desired global ray
    IKP_Lookat3D=0x23000006,     ///< direction on end effector coordinate system points to desired 3D position
    IKP_TranslationDirection5D=0x56000007,     ///< end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
    IKP_TranslationXY2D=0x22000008,     ///< 2D translation along XY plane
    IKP_TranslationXYOrientation3D=0x33000009,     ///< 2D translation along XY plane and 1D rotation around Z axis. The offset of the rotation is measured starting at +X, so at +X is it 0, at +Y it is pi/2.
    IKP_TranslationLocalGlobal6D=0x3600000a,     ///< local point on end effector origin reaches desired 3D global point

    IKP_TranslationXAxisAngle4D=0x4400000b, ///< end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with x-axis  like a cone, angle is from 0-pi. Axes defined in the manipulator base link's coordinate system)
    IKP_TranslationYAxisAngle4D=0x4400000c, ///< end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with y-axis  like a cone, angle is from 0-pi. Axes defined in the manipulator base link's coordinate system)
    IKP_TranslationZAxisAngle4D=0x4400000d, ///< end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with z-axis like a cone, angle is from 0-pi. Axes are defined in the manipulator base link's coordinate system.

    IKP_TranslationXAxisAngleZNorm4D=0x4400000e, ///< end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to z-axis and be rotated at a certain angle starting from the x-axis (defined in the manipulator base link's coordinate system)
    IKP_TranslationYAxisAngleXNorm4D=0x4400000f, ///< end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to x-axis and be rotated at a certain angle starting from the y-axis (defined in the manipulator base link's coordinate system)
    IKP_TranslationZAxisAngleYNorm4D=0x44000010, ///< end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to y-axis and be rotated at a certain angle starting from the z-axis (defined in the manipulator base link's coordinate system)

    IKP_NumberOfParameterizations=16,     ///< number of parameterizations (does not count IKP_None)

    IKP_VelocityDataBit = 0x00008000, ///< bit is set if the data represents the time-derivate velocity of an IkParameterization
    IKP_Transform6DVelocity = IKP_Transform6D|IKP_VelocityDataBit,
    IKP_Rotation3DVelocity = IKP_Rotation3D|IKP_VelocityDataBit,
    IKP_Translation3DVelocity = IKP_Translation3D|IKP_VelocityDataBit,
    IKP_Direction3DVelocity = IKP_Direction3D|IKP_VelocityDataBit,
    IKP_Ray4DVelocity = IKP_Ray4D|IKP_VelocityDataBit,
    IKP_Lookat3DVelocity = IKP_Lookat3D|IKP_VelocityDataBit,
    IKP_TranslationDirection5DVelocity = IKP_TranslationDirection5D|IKP_VelocityDataBit,
    IKP_TranslationXY2DVelocity = IKP_TranslationXY2D|IKP_VelocityDataBit,
    IKP_TranslationXYOrientation3DVelocity = IKP_TranslationXYOrientation3D|IKP_VelocityDataBit,
    IKP_TranslationLocalGlobal6DVelocity = IKP_TranslationLocalGlobal6D|IKP_VelocityDataBit,
    IKP_TranslationXAxisAngle4DVelocity = IKP_TranslationXAxisAngle4D|IKP_VelocityDataBit,
    IKP_TranslationYAxisAngle4DVelocity = IKP_TranslationYAxisAngle4D|IKP_VelocityDataBit,
    IKP_TranslationZAxisAngle4DVelocity = IKP_TranslationZAxisAngle4D|IKP_VelocityDataBit,
    IKP_TranslationXAxisAngleZNorm4DVelocity = IKP_TranslationXAxisAngleZNorm4D|IKP_VelocityDataBit,
    IKP_TranslationYAxisAngleXNorm4DVelocity = IKP_TranslationYAxisAngleXNorm4D|IKP_VelocityDataBit,
    IKP_TranslationZAxisAngleYNorm4DVelocity = IKP_TranslationZAxisAngleYNorm4D|IKP_VelocityDataBit,

    IKP_UniqueIdMask = 0x0000ffff, ///< the mask for the unique ids
    IKP_CustomDataBit = 0x00010000, ///< bit is set if the ikparameterization contains custom data, this is only used when serializing the ik parameterizations
};

class OPENRAVE_API StringReadable: public Readable
{
public:
    StringReadable(const std::string& id, const std::string& data);
    virtual ~StringReadable();
    bool SerializeXML(BaseXMLWriterPtr wirter, int options=0) const override;
    bool SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0) const override;
    bool DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale=1.0) override;
    bool operator==(const Readable& other) override {
        const StringReadable* pOther = dynamic_cast<const StringReadable*>(&other);
        if (!pOther) {
            return false;
        }

        return _data == pOther->_data;
    }

    const std::string& GetData() const;

private:
    std::string _data;
};
typedef boost::shared_ptr<StringReadable> StringReadablePtr;

/// \brief returns a string of the ik parameterization type names
///
/// \param[in] alllowercase If 1, sets all characters to lower case. Otherwise can include upper case in order to match \ref IkParameterizationType definition.
OPENRAVE_API const std::map<IkParameterizationType,std::string>& RaveGetIkParameterizationMap(int alllowercase=0);

/// \brief returns the IkParameterizationType given the unique id detmerined b IKP_UniqueIdMask
OPENRAVE_API IkParameterizationType RaveGetIkTypeFromUniqueId(int uniqueid);

/** \brief A configuration specification references values in the environment that then define a configuration-space which can be searched for.

    It is composed of several groups targetting values for individual bodies. It is serialized into XML. The XML syntax is as follows:

   \code
   <configuration>
     <group name="string" offset="#OFF1" dof="#D1" interpolation="string"/>
     <group name="string" offset="#OFF2" dof="#D2" interpolation="string"/>
   </configuration>
   \endcode
 */
class OPENRAVE_API ConfigurationSpecification
{
public:

    /// \brief A group referencing the values of one body in the environment
    class OPENRAVE_API Group
    {
public:
        Group() : offset(0), dof(0) {
        }

        inline bool operator==(const Group& r) const {
            return offset==r.offset && dof==r.dof && name==r.name && interpolation==r.interpolation;
        }
        inline bool operator!=(const Group& r) const {
            return offset!=r.offset || dof!=r.dof || name!=r.name || interpolation!=r.interpolation;
        }

        /// \brief For each data point, the number of values to offset before data for this group starts.
        int offset;
        /// \brief The number of values in this group.
        int dof;
        /** \brief semantic information on what part of the environment the group refers to.

            Can be composed of multiple words; the first word is the group type, and the words following narrow the specifics. Common types are:

            - \b joint_values - The joint values of a kinbody/robot. The joint names with the name of the body can follow.
            - \b joint_velocities - The joint velocities (1/second) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_accelerations - The joint accelerations (1/second^2) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_jerks - The joint jerks (1/second^3) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_snaps - The joint snaps (1/second^4) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_crackles - The joint crackles (1/second^5) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_pops - The joint pops (1/second^6) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_torques - The joint torques (Newton meter) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b affine_transform - An affine transformation [quaternion, translation]. The name of the body with selected affine dofs (see \ref DOFAffine) can follow.
            - \b affine_velocities - The velocity (1/second) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
            - \b affine_accelerations - The acceleration (1/second^2) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
            - \b affine_jerks - The jerk (1/second^3) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
            - \b ikparam_values - The values of an IkParameterization. The ikparam type is stored as the second value in name
            - \b ikparam_velocities - acceleration of an IkParameterization. The ikparam type is stored as the second value in name
            - \b ikparam_jerks - jerk of an IkParameterization. The ikparam type is stored as the second value in name
            - \b iswaypoint - non-zero if the point represents a major knot point of the trajectory
            - \b grabbody - Grabs the body. The configuration values are 1 for grab and 0 for release. The group name format is: bodyname robotname robotlinkindex [relative_grab_pose]. relative_grab_pose is a 7 value (quaterion + translation) pose of the relative location of the body with respect to the grabbed link. Only 1 DOF is accepted.
         */
        std::string name;
        /** \brief Describes how the data should be interpolated. Common methods are:

            - \b previous - the previous waypoint's value is always chosen
            - \b next - the next waypoint's value is always chosen
            - \b linear - linear interpolation (default)
            - \b quadratic - position is piecewise-quadratic, velocity is piecewise-linear, acceleration is one of -amax, 0, or amax. needs velocity info
            - \b cubic - 3 degree polynomial. needs velocity info.
            - \b quartic - 4 degree polynomial. needs velocity and acceleration info.
            - \b quintic - 5 degree polynomial. needs velocity and acceleration info.
            - \b sextic - 6 degree polynomial. needs velocity, acceleration, and jerk info
         */
        std::string interpolation;
    };

    class Reader : public BaseXMLReader
    {
public:
        Reader(ConfigurationSpecification& spec);
        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
        virtual bool endElement(const std::string& name);
        virtual void characters(const std::string& ch);
protected:
        ConfigurationSpecification& _spec;
        std::stringstream _ss;
        BaseXMLReaderPtr _preader;
    };

    ConfigurationSpecification();
    ConfigurationSpecification(const Group& g);
    ConfigurationSpecification(const ConfigurationSpecification& c);

    virtual ~ConfigurationSpecification() {
    }

    /// \brief return the dimension of the configuraiton space (degrees of freedom)
    virtual int GetDOF() const;

    /// \brief check if the groups form a continguous space
    ///
    /// If there are two or more groups with the same semantic names, will fail. Theese groups should be merged into one.
    /// \return true if valid, otherwise false
    virtual bool IsValid() const;

    /// \brief check if the groups form a continguous space
    ///
    /// If there are two or more groups with the same semantic names, will fail. Theese groups should be merged into one.
    /// \throw openrave_exception if not valid
    virtual void Validate() const;

    virtual bool operator==(const ConfigurationSpecification& r) const;
    virtual bool operator!=(const ConfigurationSpecification& r) const;

    /// \brief JSON serializable
    /// TODO: Ideally we should make it a subclass of openravejson::JsonSerializable, but it requires a lot changes to fix the header files for now.
    virtual void DeserializeJSON(const rapidjson::Value& rValue);
    virtual void SerializeJSON(rapidjson::Value& rValue, rapidjson::Document::AllocatorType& alloc) const;
    virtual void SerializeJSON(rapidjson::Document& d) const {
        SerializeJSON(d, d.GetAllocator());
    }

    /// \brief return the group whose name begins with a particular string.
    ///
    /// If multiple groups exist that begin with the same string, then the shortest one is returned.
    /// \throw openrave_exception if a group is not found
    virtual const Group& GetGroupFromName(const std::string& name) const;

    /// \brief return the group whose name begins with a particular string.
    ///
    /// If multiple groups exist that begin with the same string, then the shortest one is returned.
    /// \throw openrave_exception if a group is not found
    virtual Group& GetGroupFromName(const std::string& name);

    /// \brief finds the most compatible group to the given group
    ///
    /// \param g the group to query, only the Group::name and Group::dof values are used
    /// \param exactmatch if true, will only return groups whose name exactly matches with g.name
    /// \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
    virtual std::vector<Group>::const_iterator FindCompatibleGroup(const Group& g, bool exactmatch=false) const;

    /// \brief finds the most compatible group to the given group
    ///
    /// \param name the name of the group to query
    /// \param exactmatch if true, will only return groups whose name exactly matches with g.name
    /// \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
    virtual std::vector<Group>::const_iterator FindCompatibleGroup(const std::string& name, bool exactmatch=false) const;

    /** \brief Return the most compatible group that represents the time-derivative data of the group.

        For example given a 'joint_values' group, this will return the closest 'joint_velocities' group.
        \param g the group to query, only the Group::name and Group::dof values are used
        \param exactmatch if true, will only return groups whose name exactly matches with g.name
        \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
     */
    virtual std::vector<Group>::const_iterator FindTimeDerivativeGroup(const Group& g, bool exactmatch=false) const;

    /** \brief Return the most compatible group that represents the time-derivative data of the group.

        For example given a 'joint_values' group, this will return the closest 'joint_velocities' group.
        \param name the name of the group to query
        \param exactmatch if true, will only return groups whose name exactly matches with g.name
        \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
     */
    virtual std::vector<Group>::const_iterator FindTimeDerivativeGroup(const std::string& name, bool exactmatch=false) const;

    /** \brief Return the most compatible group that represents the time-integral data of the group.

        For example given a 'joint_velocities' group, this will return the closest 'joint_values' group.
        \param g the group to query, only the Group::name and Group::dof values are used
        \param exactmatch if true, will only return groups whose name exactly matches with g.name
        \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
     */
    virtual std::vector<Group>::const_iterator FindTimeIntegralGroup(const Group& g, bool exactmatch=false) const;

    /** \brief Return the most compatible group that represents the time-integral data of the group.

        For example given a 'joint_velocities' group, this will return the closest 'joint_values' group.
        \param name the name of the group to query
        \param exactmatch if true, will only return groups whose name exactly matches with g.name
        \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
     */
    virtual std::vector<Group>::const_iterator FindTimeIntegralGroup(const std::string& name, bool exactmatch=false) const;

    /** \brief adds velocity, acceleration, etc groups for every position group.

        If the derivative groups already exist, they are checked for and/or modified. Note that the configuration space
        might be re-ordered as a result of this function call. If a new group is added, its interpolation will be
        the derivative of the position group as returned by \ref GetInterpolationDerivative.
        \param deriv The position derivative to add, this must be greater than 0. If 2 is specified, only the acceleration groups of the alread present position groups will be added.
        \param adddeltatime If true will add the 'deltatime' tag, which is necessary for trajectory sampling
     */
    virtual void AddDerivativeGroups(int deriv, bool adddeltatime=false);

    /// \deprecated (12/07/30)
    inline void AddVelocityGroups(bool adddeltatime) RAVE_DEPRECATED {
        AddDerivativeGroups(1,adddeltatime);
    }

    /** \brief converts all the groups to the corresponding velocity groups and returns the specification

        The velocity configuration space will have a one-to-one correspondence with the original configuration.
        The interpolation of each of the groups will correspondingly represent the derivative as returned by \ref GetInterpolationDerivative.
        Only position specifications will be converted, any other groups will be left untouched.
     */
    virtual ConfigurationSpecification ConvertToVelocitySpecification() const;

    /** \brief converts all the groups to the corresponding derivative group and returns the specification

        The new derivative configuration space will have a one-to-one correspondence with the original configuration.
        The interpolation of each of the groups will correspondingly represent the derivative as returned by \ref GetInterpolationDerivative(deriv).
        Only position specifications will be converted, any other groups will be left untouched.
        \param timederivative the number of times to take the time derivative of the position
     */
    virtual ConfigurationSpecification ConvertToDerivativeSpecification(uint32_t timederivative=1) const;

    /// \brief returns a new specification of just particular time-derivative groups.
    ///
    /// \param timederivative the time derivative to query groups from. 0 is positions/joint values, 1 is velocities, 2 is accelerations, etc
    virtual ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

    /** \brief set the offsets of each group in order to get a contiguous configuration space
     */
    virtual void ResetGroupOffsets();

    /// \brief adds the deltatime tag to the end if one doesn't exist and returns the index into the configuration space
    virtual int AddDeltaTimeGroup();

    /** \brief Adds a new group to the specification and returns its new offset.

        If the new group's semantic name does not exist in the current specification, adds it and returns the new offset.
        If the new group's semantic name exists in the current specification and it exactly matches, then function returns the old group's index. If the semantic names match, but parameters do not match, then an openrave_exception is thrown.
        This method is not responsible for merging semantic information
     */
    virtual int AddGroup(const std::string& name, int dof, const std::string& interpolation = "");

    /** \brief Merges all the information from the input group into this group

        For groups that are merged, the interpolation is overwritten if the source group has an empty string.
        \throw openrave_exception throws if groups do not contain enough information to be merged or interpolations do not match.
     */
    virtual ConfigurationSpecification& operator+= (const ConfigurationSpecification& r);

    /** \brief Return a new specification that holds the merged information from the current and input specification and the input parameter..

        For groups that are merged, the interpolation either has to match for both groups, or one of the groups needs an empty interpolation.
        \throw openrave_exception throws if groups do not contain enough information to be merged or interpolations do not match.
     */
    virtual ConfigurationSpecification operator+ (const ConfigurationSpecification& r) const;

    /** \brief extracts an affine transform given the start of a configuration space point

        Looks for 'affine_transform' groups. If pbody is not initialized, will choose the first affine_transform found.
        \param[inout] t the transform holding the default values, which will be overwritten with the new values.
        \param[in] itdata data in the format of this configuration specification.
        \param[in] timederivative the time derivative of the data to extract
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractTransform(Transform& t, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int timederivative=0) const;

    /** \brief extracts an ikparameterization given the start of a configuration space point

        Looks for 'ikparam' groups.
        \param[inout] ikparam filled with ikparameterization (if found)
        \param[in] itdata data in the format of this configuration specification
        \param[in] timederivative the time derivative of the data to extract
        \param[in] robotname optional name of robot to filter by
        \param[in] manipulatorname optional name of manipulator to filter by
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractIkParameterization(IkParameterization& ikparam, std::vector<dReal>::const_iterator itdata, int timederivative=0, std::string const &robotname="", std::string const &manipulatorname="") const;

    /** \brief extracts the affine values

        Looks for 'affine_X' groups. If pbody is not initialized, will choose the first affine_X found.
        \param[inout] itvalues iterator to vector that holds the default values and will be overwritten with the new values. must be initialized
        \param[in] itdata data in the format of this configuration specification.
        \param[in] affinedofs the format of the affine dofs requested
        \param[in] timederivative the time derivative of the data to extract
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractAffineValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int affinedofs, int timederivative=0) const;

    /** \brief extracts a body's joint values given the start of a configuration space point

        Looks for 'joint_X' groups. If pbody is not initialized, will choose the first joint_X found.
        \param[inout] itvalues iterator to vector that holds the default values and will be overwritten with the new values. must be initialized
        \param[in] itdata data in the format of this configuration specification.
        \param[in] indices the set of DOF indices of the body to extract and write into itvalues.
        \param[in] timederivative the time derivative of the data to extract
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractJointValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative=0) const;

    /// \brief extracts the delta time from the configuration if one exists
    ///
    /// \return true if deltatime exists in the current configuration, otherwise false
    virtual bool ExtractDeltaTime(dReal& deltatime, std::vector<dReal>::const_iterator itdata) const;

    /** \brief inserts a set of joint values into a configuration space point

        Looks for 'joint_X' groups. If pbody is not initialized, will use the first joint_X found.
        \param[inout] itdata data in the format of this configuration specification.
        \param[in] itvalues iterator to joint values to write
        \param[in] indices the set of DOF indices that itvalues represents.
        \param[in] timederivative the time derivative of the data to insert
        \return true if at least one group was found for inserting
     */
    virtual bool InsertJointValues(std::vector<dReal>::iterator itdata, std::vector<dReal>::const_iterator itvalues, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative=0) const;

    /** \brief sets the deltatime field of the data if one exists

        \param[inout] itdata data in the format of this configuration specification.
        \param[in] deltatime the delta time of the time stamp (from previous point)
        \return true if at least one group was found for inserting
     */
    virtual bool InsertDeltaTime(std::vector<dReal>::iterator itdata, dReal deltatime) const;

    /** \brief Adds a new group to the specification and returns its new offset.

        \param g the group whose name, dof, and interpolation are extracted.
        If the new group's semantic name does not exist in the current specification, adds it and returns the new offset.
        If the new group's semantic name exists in the current specification and it exactly matches, then function returns the old group's index. If the semantic names match, but parameters do not match, then an openrave_exception is thrown.
        This method is not responsible for merging semantic information
     */
    virtual int AddGroup(const Group& g);

    /** \brief removes all groups that match a name

        ResetGroupOffsets will be called internally to fix the indices.
        \param groupname the name used to look for groups
        \param exactmatch if true, will remove groups only if the full name matches, otherwise will remove groups that start with groupname
        \return number of groups removed
     */
    virtual int RemoveGroups(const std::string& groupname, bool exactmatch=true);

    /** \brief extracts all the bodies that are used inside this specification

        Because the specification contains names of bodies, an environment is necessary to get the body pointers.
        \param[in] env the environment to extract the bodies from
        \param[out] usedbodies a list of the bodies being used
     */
    virtual void ExtractUsedBodies(EnvironmentBasePtr env, std::vector<KinBodyPtr>& usedbodies) const;

    /** \brief extracts all the unique dof indices that the configuration holds for a particular body

        \param[in] body the body to query for
        \param[out] useddofindices a vector of unique DOF indices targetted for the body
        \param[out] usedconfigindices for every used index, returns the first configuration space index it came from
     */
    virtual void ExtractUsedIndices(KinBodyPtr body, std::vector<int>& useddofindices, std::vector<int>& usedconfigindices) const;

    /// \brief swaps the data between the two configuration specifications as efficiently as possible
    virtual void Swap(ConfigurationSpecification& spec);

    typedef boost::function<int (const std::vector<dReal>&)> SetConfigurationStateFn;
    typedef boost::function<void (std::vector<dReal>&)> GetConfigurationStateFn;

    /// \brief return a function to set the states of the configuration in the environment
    virtual boost::shared_ptr<SetConfigurationStateFn> GetSetFn(EnvironmentBasePtr env) const;

    /// \brief return a function to get the states of the configuration in the environment
    virtual boost::shared_ptr<GetConfigurationStateFn> GetGetFn(EnvironmentBasePtr env) const;

    /** \brief given two compatible groups, convers data represented in the source group to data represented in the target group

        \param ittargetdata iterator pointing to start of target group data that should be overwritten
        \param targetstride the number of elements that to go from the next target point. Necessary if numpoints > 1.
        \param gtarget the target configuration group
        \param itsourcedata iterator pointing to start of source group data that should be read
        \param sourcestride the number of elements that to go from the next source point. Necessary if numpoints > 1.
        \param gsource the source configuration group
        \param numpoints the number of points to convert. The target and source strides are gtarget.dof and gsource.dof
        \param penv [optional] The environment which might be needed to fill in unknown data. Assumes environment is locked.
        \param filluninitialized If there exists target groups that cannot be initialized, then will set default values using the current environment. For example, the current joint values of the body will be used.
        \throw openrave_exception throw f groups are incompatible
     */
    static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized = true);

    /** \brief Converts from one specification to another.

        \param ittargetdata iterator pointing to start of target group data that should be overwritten
        \param targetspec the target configuration specification
        \param itsourcedata iterator pointing to start of source group data that should be read
        \param sourcespec the source configuration specification
        \param numpoints the number of points to convert. The target and source strides are gtarget.dof and gsource.dof
        \param penv [optional] The environment which might be needed to fill in unknown data. Assumes environment is locked.
        \param filluninitialized If there exists target groups that cannot be initialized, then will set default values using the current environment. For example, the current joint values of the body will be used.
     */
    static void ConvertData(std::vector<dReal>::iterator ittargetdata, const ConfigurationSpecification& targetspec, std::vector<dReal>::const_iterator itsourcedata, const ConfigurationSpecification& sourcespec, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized = true);

    /// \brief gets the name of the interpolation that represents the derivative of the passed in interpolation.
    ///
    /// For example GetInterpolationDerivative("quadratic") -> "linear"
    /// \param interpolation the interpolation to start at
    /// \param deriv the number of derivatives to take, should be > 0
    static std::string GetInterpolationDerivative(const std::string& interpolation, int deriv=1);

    std::vector<Group> _vgroups;
};

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const ConfigurationSpecification &spec);
OPENRAVE_API std::istream& operator>>(std::istream& I, ConfigurationSpecification& spec);

typedef boost::shared_ptr<ConfigurationSpecification> ConfigurationSpecificationPtr;
typedef boost::shared_ptr<ConfigurationSpecification const> ConfigurationSpecificationConstPtr;

template <typename T>
inline T NormalizeCircularAnglePrivate(T theta, T min, T max)
{
    if (theta < min) {
        T range = max-min;
        theta += range;
        while (theta < min) {
            theta += range;
        }
    }
    else if (theta > max) {
        T range = max-min;
        theta -= range;
        while (theta > max) {
            theta -= range;
        }
    }
    return theta;
}


/** \brief Parameterization of basic primitives for querying inverse-kinematics solutions.

    Holds the parameterization of a geometric primitive useful for autonomous manipulation scenarios like:
    6D pose, 3D translation, 3D rotation, 3D look at direction, and ray look at direction.
 */
class OPENRAVE_API IkParameterization
{
public:
    /// \deprecated (11/10/12)
    typedef IkParameterizationType Type RAVE_DEPRECATED;
    static const IkParameterizationType Type_None RAVE_DEPRECATED = IKP_None;
    static const IkParameterizationType Type_Transform6D RAVE_DEPRECATED = IKP_Transform6D;
    static const IkParameterizationType Type_Rotation3D RAVE_DEPRECATED =IKP_Rotation3D;
    static const IkParameterizationType Type_Translation3D RAVE_DEPRECATED =IKP_Translation3D;
    static const IkParameterizationType Type_Direction3D RAVE_DEPRECATED = IKP_Direction3D;
    static const IkParameterizationType Type_Ray4D RAVE_DEPRECATED = IKP_Ray4D;
    static const IkParameterizationType Type_Lookat3D RAVE_DEPRECATED = IKP_Lookat3D;
    static const IkParameterizationType Type_TranslationDirection5D RAVE_DEPRECATED = IKP_TranslationDirection5D;
    static const IkParameterizationType Type_TranslationXY2D RAVE_DEPRECATED = IKP_TranslationXY2D;
    static const IkParameterizationType Type_TranslationXYOrientation3D RAVE_DEPRECATED = IKP_TranslationXYOrientation3D;
    static const IkParameterizationType Type_TranslationLocalGlobal6D RAVE_DEPRECATED = IKP_TranslationLocalGlobal6D;
    static const IkParameterizationType Type_NumberOfParameterizations RAVE_DEPRECATED = IKP_NumberOfParameterizations;

    IkParameterization() : _type(IKP_None) {
    }
    /// \brief sets a 6D transform parameterization
    IkParameterization(const Transform &t) {
        SetTransform6D(t);
    }
    /// \brief sets a ray parameterization
    IkParameterization(const RAY &r) {
        SetRay4D(r);
    }
    /// \brief set a custom parameterization using a transform as the source of the data. Not all types are supported with this method.
    IkParameterization(const Transform &t, IkParameterizationType type) {
        _type=type;
        switch(_type) {
        case IKP_Transform6D: SetTransform6D(t); break;
        case IKP_Rotation3D: SetRotation3D(t.rot); break;
        case IKP_Translation3D: SetTranslation3D(t.trans); break;
        case IKP_Lookat3D: SetLookat3D(t.trans); break;
        default:
            throw openrave_exception(str(boost::format("IkParameterization constructor does not support type 0x%x")%_type));
        }
    }

    inline IkParameterizationType GetType() const {
        return _type;
    }

    /// \brief returns a string version of \ref GetType
    inline const std::string& GetName() const;

    /// \brief Returns the minimum degree of freedoms required for the IK type. Does \b not count custom data.
    static int GetDOF(IkParameterizationType type) {
        return (type>>28)&0xf;
    }
    /// \brief Returns the minimum degree of freedoms required for the IK type. Does \b not count custom data.
    inline int GetDOF() const {
        return (_type>>28)&0xf;
    }

    /// \brief Returns the number of values used to represent the parameterization ( >= dof ). Does \b not count custom data.
    static int GetNumberOfValues(IkParameterizationType type) {
        return (type>>24)&0xf;
    }

    /// \brief returns a string of the ik parameterization type names
    ///
    /// \param[in] alllowercase If 1, sets all characters to lower case. Otherwise can include upper case in order to match \ref IkParameterizationType definition.
    static const std::map<IkParameterizationType,std::string>& GetIkParameterizationMap(int alllowercase=0);

    /// \brief returns the IkParameterizationType given the unique id detmerined b IKP_UniqueIdMask
    static IkParameterizationType GetIkTypeFromUniqueId(int uniqueid);

    /// \brief Returns the number of values used to represent the parameterization ( >= dof ). Does \b not count custom data.
    inline int GetNumberOfValues() const {
        return (_type>>24)&0xf;
    }

    inline void SetTransform6D(const Transform& t) {
        _type = IKP_Transform6D; _transform = t;
    }
    inline void SetTransform6DVelocity(const Transform& t) {
        _type = IKP_Transform6DVelocity; _transform = t;
    }
    inline void SetRotation3D(const Vector& quaternion) {
        _type = IKP_Rotation3D; _transform.rot = quaternion;
    }
    inline void SetTranslation3D(const Vector& trans) {
        _type = IKP_Translation3D; _transform.trans = trans;
    }
    inline void SetDirection3D(const Vector& dir) {
        _type = IKP_Direction3D; _transform.rot = dir;
    }
    inline void SetRay4D(const RAY& ray) {
        _type = IKP_Ray4D; _transform.trans = ray.pos; _transform.rot = ray.dir;
    }
    inline void SetLookat3D(const Vector& trans) {
        _type = IKP_Lookat3D; _transform.trans = trans;
    }
    /// \brief the ray direction is not used for IK, however it is needed in order to compute the error
    inline void SetLookat3D(const RAY& ray) {
        _type = IKP_Lookat3D; _transform.trans = ray.pos; _transform.rot = ray.dir;
    }
    inline void SetTranslationDirection5D(const RAY& ray) {
        _type = IKP_TranslationDirection5D; _transform.trans = ray.pos; _transform.rot = ray.dir;
    }
    inline void SetTranslationXY2D(const Vector& trans) {
        _type = IKP_TranslationXY2D; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = 0; _transform.trans.w = 0;
    }
    inline void SetTranslationXYOrientation3D(const Vector& trans) {
        _type = IKP_TranslationXYOrientation3D; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = trans.z; _transform.trans.w = 0;
    }
    inline void SetTranslationLocalGlobal6D(const Vector& localtrans, const Vector& trans) {
        _type = IKP_TranslationLocalGlobal6D; _transform.rot.x = localtrans.x; _transform.rot.y = localtrans.y; _transform.rot.z = localtrans.z; _transform.rot.w = 0; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = trans.z; _transform.trans.w = 0;
    }
    inline void SetTranslationXAxisAngle4D(const Vector& trans, dReal angle) {
        _type = IKP_TranslationXAxisAngle4D;
        _transform.trans = trans;
        _transform.rot.x = angle;
    }
    inline void SetTranslationYAxisAngle4D(const Vector& trans, dReal angle) {
        _type = IKP_TranslationYAxisAngle4D;
        _transform.trans = trans;
        _transform.rot.x = angle;
    }
    inline void SetTranslationZAxisAngle4D(const Vector& trans, dReal angle) {
        _type = IKP_TranslationZAxisAngle4D;
        _transform.trans = trans;
        _transform.rot.x = angle;
    }

    inline void SetTranslationXAxisAngleZNorm4D(const Vector& trans, dReal angle) {
        _type = IKP_TranslationXAxisAngleZNorm4D;
        _transform.trans = trans;
        _transform.rot.x = angle;
    }
    inline void SetTranslationYAxisAngleXNorm4D(const Vector& trans, dReal angle) {
        _type = IKP_TranslationYAxisAngleXNorm4D;
        _transform.trans = trans;
        _transform.rot.x = angle;
    }
    inline void SetTranslationZAxisAngleYNorm4D(const Vector& trans, dReal angle) {
        _type = IKP_TranslationZAxisAngleYNorm4D;
        _transform.trans = trans;
        _transform.rot.x = angle;
    }

    inline const Transform& GetTransform6D() const {
        return _transform;
    }
    inline const Vector& GetRotation3D() const {
        return _transform.rot;
    }
    inline const Vector& GetTranslation3D() const {
        return _transform.trans;
    }
    inline const Vector& GetDirection3D() const {
        return _transform.rot;
    }
    inline const RAY GetRay4D() const {
        return RAY(_transform.trans,_transform.rot);
    }
    inline const Vector& GetLookat3D() const {
        return _transform.trans;
    }
    inline const Vector& GetLookat3DDirection() const {
        return _transform.rot;
    }
    inline const RAY GetTranslationDirection5D() const {
        return RAY(_transform.trans,_transform.rot);
    }
    inline const Vector& GetTranslationXY2D() const {
        return _transform.trans;
    }
    inline const Vector& GetTranslationXYOrientation3D() const {
        return _transform.trans;
    }
    inline std::pair<Vector,Vector> GetTranslationLocalGlobal6D() const {
        return std::make_pair(_transform.rot,_transform.trans);
    }
    inline std::pair<Vector,dReal> GetTranslationXAxisAngle4D() const {
        return std::make_pair(_transform.trans,_transform.rot.x);
    }
    inline std::pair<Vector,dReal> GetTranslationYAxisAngle4D() const {
        return std::make_pair(_transform.trans,_transform.rot.x);
    }
    inline std::pair<Vector,dReal> GetTranslationZAxisAngle4D() const {
        return std::make_pair(_transform.trans,_transform.rot.x);
    }
    inline std::pair<Vector,dReal> GetTranslationXAxisAngleZNorm4D() const {
        return std::make_pair(_transform.trans,_transform.rot.x);
    }
    inline std::pair<Vector,dReal> GetTranslationYAxisAngleXNorm4D() const {
        return std::make_pair(_transform.trans,_transform.rot.x);
    }
    inline std::pair<Vector,dReal> GetTranslationZAxisAngleYNorm4D() const {
        return std::make_pair(_transform.trans,_transform.rot.x);
    }

    /// \brief Computes the distance squared between two IK parmaeterizations.
    inline dReal ComputeDistanceSqr(const IkParameterization& ikparam) const
    {
        const dReal anglemult = 0.4;     // this is a hack that should be removed....
        BOOST_ASSERT(_type==ikparam.GetType());
        switch(_type) {
        case IKP_Transform6D: {
            Transform t0 = GetTransform6D(), t1 = ikparam.GetTransform6D();
            dReal fcos = RaveFabs(t0.rot.dot(t1.rot));
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return (t0.trans-t1.trans).lengthsqr3() + anglemult*facos*facos;
        }
        case IKP_Rotation3D: {
            dReal fcos = RaveFabs(GetRotation3D().dot(ikparam.GetRotation3D()));
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Translation3D:
            return (GetTranslation3D()-ikparam.GetTranslation3D()).lengthsqr3();
        case IKP_Direction3D: {
            dReal fcos = GetDirection3D().dot(ikparam.GetDirection3D());
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Ray4D: {
            Vector pos0 = GetRay4D().pos - GetRay4D().dir*GetRay4D().dir.dot(GetRay4D().pos);
            Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
            dReal fcos = GetRay4D().dir.dot(ikparam.GetRay4D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return (pos0-pos1).lengthsqr3() + anglemult*facos*facos;
        }
        case IKP_Lookat3D: {
            Vector v = GetLookat3D()-ikparam.GetLookat3D();
            dReal s = v.dot3(ikparam.GetLookat3DDirection());
            if( s >= -1 ) {     // ikparam's lookat is always 1 beyond the origin, this is just the convention for testing...
                v -= s*ikparam.GetLookat3DDirection();
            }
            return v.lengthsqr3();
        }
        case IKP_TranslationDirection5D: {
            dReal fcos = GetTranslationDirection5D().dir.dot(ikparam.GetTranslationDirection5D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return (GetTranslationDirection5D().pos-ikparam.GetTranslationDirection5D().pos).lengthsqr3() + anglemult*facos*facos;
        }
        case IKP_TranslationXY2D: {
            return (GetTranslationXY2D()-ikparam.GetTranslationXY2D()).lengthsqr2();
        }
        case IKP_TranslationXYOrientation3D: {
            Vector v0 = GetTranslationXYOrientation3D();
            Vector v1 = ikparam.GetTranslationXYOrientation3D();
            dReal anglediff = v0.z-v1.z;
            if (anglediff < dReal(-PI)) {
                anglediff += dReal(2*PI);
                while (anglediff < dReal(-PI))
                    anglediff += dReal(2*PI);
            }
            else if (anglediff > dReal(PI)) {
                anglediff -= dReal(2*PI);
                while (anglediff > dReal(PI))
                    anglediff -= dReal(2*PI);
            }
            return (v0-v1).lengthsqr2() + anglemult*anglediff*anglediff;
        }
        case IKP_TranslationLocalGlobal6D: {
            std::pair<Vector,Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
            return (p0.first-p1.first).lengthsqr3() + (p0.second-p1.second).lengthsqr3();
        }
        case IKP_TranslationXAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationXAxisAngle4D(), p1 = ikparam.GetTranslationXAxisAngle4D();
            // dot product with axis is always in [0,pi]
            dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
            dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
            return (p0.first-p1.first).lengthsqr3() + (angle0-angle1)*(angle0-angle1);
        }
        case IKP_TranslationYAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationYAxisAngle4D(), p1 = ikparam.GetTranslationYAxisAngle4D();
            // dot product with axis is always in [0,pi]
            dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
            dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
            return (p0.first-p1.first).lengthsqr3() + (angle0-angle1)*(angle0-angle1);
        }
        case IKP_TranslationZAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationZAxisAngle4D(), p1 = ikparam.GetTranslationZAxisAngle4D();
            // dot product with axis is always in [0,pi]
            dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
            dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
            return (p0.first-p1.first).lengthsqr3() + (angle0-angle1)*(angle0-angle1);
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationXAxisAngleZNorm4D(), p1 = ikparam.GetTranslationXAxisAngleZNorm4D();
            dReal anglediff = NormalizeCircularAnglePrivate(p0.second-p1.second, -PI, PI);
            return (p0.first-p1.first).lengthsqr3() + anglediff*anglediff;
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationYAxisAngleXNorm4D(), p1 = ikparam.GetTranslationYAxisAngleXNorm4D();
            dReal anglediff = NormalizeCircularAnglePrivate(p0.second-p1.second, -PI, PI);
            return (p0.first-p1.first).lengthsqr3() + anglediff*anglediff;
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationZAxisAngleYNorm4D(), p1 = ikparam.GetTranslationZAxisAngleYNorm4D();
            dReal anglediff = NormalizeCircularAnglePrivate(p0.second-p1.second, -PI, PI);
            return (p0.first-p1.first).lengthsqr3() + anglediff*anglediff;
        }
        default:
            BOOST_ASSERT(0);
        }
        return 1e30;
    }

    /// \brief Computes the translational distance squared between two IK parmaeterizations.
    inline dReal ComputeTransDistanceSqr(const IkParameterization& ikparam) const
    {
        BOOST_ASSERT(_type==ikparam.GetType());
        switch(_type) {
        case IKP_Transform6D: {
            return (GetTransform6D().trans-ikparam.GetTransform6D().trans).lengthsqr3();
        }
        case IKP_Translation3D:
            return (GetTranslation3D()-ikparam.GetTranslation3D()).lengthsqr3();
        case IKP_Ray4D: {
            Vector pos0 = GetRay4D().pos - GetRay4D().dir*GetRay4D().dir.dot(GetRay4D().pos);
            Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
            return (pos0-pos1).lengthsqr3();
        }
        case IKP_TranslationDirection5D: {
            return (GetTranslationDirection5D().pos-ikparam.GetTranslationDirection5D().pos).lengthsqr3();
        }
        case IKP_TranslationXY2D: {
            return (GetTranslationXY2D()-ikparam.GetTranslationXY2D()).lengthsqr2();
        }
        case IKP_TranslationXYOrientation3D: {
            Vector v0 = GetTranslationXYOrientation3D();
            Vector v1 = ikparam.GetTranslationXYOrientation3D();
            return (v0-v1).lengthsqr2();
        }
        case IKP_TranslationLocalGlobal6D: {
            std::pair<Vector,Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
            return (p0.first-p1.first).lengthsqr3();
        }
        case IKP_TranslationXAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationXAxisAngle4D(), p1 = ikparam.GetTranslationXAxisAngle4D();
            return (p0.first-p1.first).lengthsqr3();
        }
        case IKP_TranslationYAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationYAxisAngle4D(), p1 = ikparam.GetTranslationYAxisAngle4D();
            return (p0.first-p1.first).lengthsqr3();
        }
        case IKP_TranslationZAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationZAxisAngle4D(), p1 = ikparam.GetTranslationZAxisAngle4D();
            return (p0.first-p1.first).lengthsqr3();
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationXAxisAngleZNorm4D(), p1 = ikparam.GetTranslationXAxisAngleZNorm4D();
            return (p0.first-p1.first).lengthsqr3();
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationYAxisAngleXNorm4D(), p1 = ikparam.GetTranslationYAxisAngleXNorm4D();
            return (p0.first-p1.first).lengthsqr3();
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationZAxisAngleYNorm4D(), p1 = ikparam.GetTranslationZAxisAngleYNorm4D();
            return (p0.first-p1.first).lengthsqr3();
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", _type,ORE_InvalidArguments);
        }
        return 1e30;
    }

    /// \brief Computes the rotational distance squared between two IK parmaeterizations.
    inline dReal ComputeRotDistanceSqr(const IkParameterization& ikparam) const
    {
        BOOST_ASSERT(_type==ikparam.GetType());
        switch(_type) {
        case IKP_Transform6D: {
            Transform t0 = GetTransform6D(), t1 = ikparam.GetTransform6D();
            dReal fcos = RaveFabs(t0.rot.dot(t1.rot));
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Rotation3D: {
            dReal fcos = RaveFabs(GetRotation3D().dot(ikparam.GetRotation3D()));
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Direction3D: {
            dReal fcos = GetDirection3D().dot(ikparam.GetDirection3D());
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Ray4D: {
            dReal fcos = GetRay4D().dir.dot(ikparam.GetRay4D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Lookat3D: {
            Vector v = GetLookat3D()-ikparam.GetLookat3D();
            dReal s = v.dot3(ikparam.GetLookat3DDirection());
            if( s >= -1 ) {     // ikparam's lookat is always 1 beyond the origin, this is just the convention for testing...
                v -= s*ikparam.GetLookat3DDirection();
            }
            return v.lengthsqr3();
        }
        case IKP_TranslationDirection5D: {
            dReal fcos = GetTranslationDirection5D().dir.dot(ikparam.GetTranslationDirection5D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_TranslationXYOrientation3D: {
            Vector v0 = GetTranslationXYOrientation3D();
            Vector v1 = ikparam.GetTranslationXYOrientation3D();
            dReal anglediff = v0.z-v1.z;
            if (anglediff < dReal(-PI)) {
                anglediff += dReal(2*PI);
                while (anglediff < dReal(-PI))
                    anglediff += dReal(2*PI);
            }
            else if (anglediff > dReal(PI)) {
                anglediff -= dReal(2*PI);
                while (anglediff > dReal(PI))
                    anglediff -= dReal(2*PI);
            }
            return anglediff*anglediff;
        }
        case IKP_TranslationLocalGlobal6D: {
            std::pair<Vector,Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
            return (p0.second-p1.second).lengthsqr3();
        }
        case IKP_TranslationXAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationXAxisAngle4D(), p1 = ikparam.GetTranslationXAxisAngle4D();
            // dot product with axis is always in [0,pi]
            dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
            dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
            return (angle0-angle1)*(angle0-angle1);
        }
        case IKP_TranslationYAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationYAxisAngle4D(), p1 = ikparam.GetTranslationYAxisAngle4D();
            // dot product with axis is always in [0,pi]
            dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
            dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
            return (angle0-angle1)*(angle0-angle1);
        }
        case IKP_TranslationZAxisAngle4D: {
            std::pair<Vector,dReal> p0 = GetTranslationZAxisAngle4D(), p1 = ikparam.GetTranslationZAxisAngle4D();
            // dot product with axis is always in [0,pi]
            dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
            dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
            return (angle0-angle1)*(angle0-angle1);
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationXAxisAngleZNorm4D(), p1 = ikparam.GetTranslationXAxisAngleZNorm4D();
            dReal anglediff = NormalizeCircularAnglePrivate(p0.second-p1.second, -PI, PI);
            return anglediff*anglediff;
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationYAxisAngleXNorm4D(), p1 = ikparam.GetTranslationYAxisAngleXNorm4D();
            dReal anglediff = NormalizeCircularAnglePrivate(p0.second-p1.second, -PI, PI);
            return anglediff*anglediff;
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            std::pair<Vector,dReal> p0 = GetTranslationZAxisAngleYNorm4D(), p1 = ikparam.GetTranslationZAxisAngleYNorm4D();
            dReal anglediff = NormalizeCircularAnglePrivate(p0.second-p1.second, -PI, PI);
            return anglediff*anglediff;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", _type,ORE_InvalidArguments);
        }
        return 1e30;
    }

    /// \brief fills the iterator with the serialized values of the ikparameterization.
    ///
    /// The container the iterator points to needs to have \ref GetNumberOfValues() available.
    /// Does not support custom data
    /// Don't normalize quaternions since it could hold velocity data.
    inline void GetValues(std::vector<dReal>::iterator itvalues) const
    {
        switch(_type & ~IKP_VelocityDataBit) {
        case IKP_Transform6D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.rot.w;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_Rotation3D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.rot.w;
            break;
        case IKP_Translation3D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_Direction3D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            break;
        case IKP_Ray4D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_Lookat3D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationDirection5D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationXY2D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            break;
        case IKP_TranslationXYOrientation3D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationLocalGlobal6D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationXAxisAngle4D:
        case IKP_TranslationYAxisAngle4D:
        case IKP_TranslationZAxisAngle4D:
        case IKP_TranslationXAxisAngleZNorm4D:
        case IKP_TranslationYAxisAngleXNorm4D:
        case IKP_TranslationZAxisAngleYNorm4D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", _type,ORE_InvalidArguments);
        }
    }

    /// \brief sets a serialized set of values for the IkParameterization
    ///
    /// Function does not handle custom data. Don't normalize quaternions since it could hold velocity data.
    inline void SetValues(std::vector<dReal>::const_iterator itvalues, IkParameterizationType iktype)
    {
        _type = iktype;
        switch(_type & ~IKP_VelocityDataBit) {
        case IKP_Transform6D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.rot.w = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_Rotation3D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.rot.w = *itvalues++;
            break;
        case IKP_Translation3D:
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_Direction3D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            break;
        case IKP_Ray4D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_Lookat3D:
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_TranslationDirection5D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_TranslationXY2D:
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            break;
        case IKP_TranslationXYOrientation3D:
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_TranslationLocalGlobal6D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_TranslationXAxisAngle4D:
        case IKP_TranslationYAxisAngle4D:
        case IKP_TranslationZAxisAngle4D:
        case IKP_TranslationXAxisAngleZNorm4D:
        case IKP_TranslationYAxisAngleXNorm4D:
        case IKP_TranslationZAxisAngleYNorm4D:
            _transform.rot.x = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", _type,ORE_InvalidArguments);
        }
    }

    inline void Set(std::vector<dReal>::const_iterator itvalues, IkParameterizationType iktype) {
        SetValues(itvalues,iktype);
    }

    /** \brief sets named custom data in the ik parameterization

        The custom data is serialized along with the rest of the parameters and can also be part of a configuration specification under the "ikparam_values" anotation.
        The custom data name can have meta-tags for the type of transformation the data undergos when \ref MultiplyTransform is called. For example, if the user wants to have an extra 3 values that represent "direction", then the direction has to be rotated along with all the data or coordinate systems can get lost. The anotations are specified by putting:

        \b _transform=%s_

        somewhere in the string. The %s can be: \b direction, \b point, \b quat, \b ikparam

        If \b ikparam, the first value is expected to be the unique id of the ik type (GetType()&IKP_UniqueIdMask). The other values can be computed from \ref IkParameterization::GetValues

        \param name Describes the type of data, cannot contain spaces or new lines.
        \param values the values representing the data
        \throw openrave_exception throws if the name is invalid
     */
    inline void SetCustomValues(const std::string& name, const std::vector<dReal>& values)
    {
        OPENRAVE_ASSERT_OP_FORMAT0( name.size(), >, 0, "name is empty", ORE_InvalidArguments );
        OPENRAVE_ASSERT_OP_FORMAT0(std::count_if(name.begin(), name.end(), _IsValidCharInName), ==, (int)name.size(), "name has invalid characters",ORE_InvalidArguments);
        _mapCustomData[name] = values;
    }

    /// \brief sets named custom data in the ik parameterization (\see SetCustomValues)
    inline void SetCustomValue(const std::string& name, dReal value)
    {
        OPENRAVE_ASSERT_OP_FORMAT0( name.size(), >, 0, "name is empty", ORE_InvalidArguments );
        OPENRAVE_ASSERT_OP_FORMAT0(std::count_if(name.begin(), name.end(), _IsValidCharInName), ==, (int)name.size(), "name has invalid characters",ORE_InvalidArguments);
        _mapCustomData[name].resize(1);
        _mapCustomData[name][0] = value;
    }

    /// \brief sets named custom data in the ik parameterization (\see SetCustomValues)
    inline void SetCustomValue(const std::string& name, dReal value0, dReal value1, dReal value2)
    {
        OPENRAVE_ASSERT_OP_FORMAT0( name.size(), >, 0, "name is empty", ORE_InvalidArguments );
        OPENRAVE_ASSERT_OP_FORMAT0(std::count_if(name.begin(), name.end(), _IsValidCharInName), ==, (int)name.size(), "name has invalid characters",ORE_InvalidArguments);
        _mapCustomData[name].resize(3);
        _mapCustomData[name][0] = value0;
        _mapCustomData[name][1] = value1;
        _mapCustomData[name][2] = value2;
    }

    /// \brief gets custom data if it exists, returns false if it doesn't
    inline bool GetCustomValues(const std::string& name, std::vector<dReal>& values) const
    {
        std::map<std::string, std::vector<dReal> >::const_iterator it = _mapCustomData.find(name);
        if( it == _mapCustomData.end() ) {
            return false;
        }
        values = it->second;
        return true;
    }

    /// \brief returns number of entries in the custom value. -1 if custom value is not in the map
    inline int GetCustomValueNum(const std::string& name) const {
        std::map<std::string, std::vector<dReal> >::const_iterator it = _mapCustomData.find(name);
        if( it != _mapCustomData.end() ) {
            return (int)it->second.size();
        }

        return -1;
    }

    /// \brief returns the first element of a custom value. If _mapCustomData does not have 'name' and is not > 0, then will return defaultValue
    inline dReal GetCustomValue(const std::string& name, dReal defaultValue) const {
        std::map<std::string, std::vector<dReal> >::const_iterator it = _mapCustomData.find(name);
        if( it != _mapCustomData.end() && it->second.size() > 0 ) {
            return it->second[0];
        }
        return defaultValue;
    }

    /// \brief returns a const reference of the custom data key/value pairs
    inline const std::map<std::string, std::vector<dReal> >& GetCustomDataMap() const
    {
        return _mapCustomData;
    }

    /// \brief clears custom data
    ///
    /// \param name if name is empty, will clear all the data, otherwise will clear only the custom data with that name
    /// \return number of elements erased
    inline size_t ClearCustomValues(const std::string& name=std::string())
    {
        if( name.size() > 0 ) {
            return _mapCustomData.erase(name) > 0;
        }
        else {
            size_t num = _mapCustomData.size();
            _mapCustomData.clear();
            return num;
        }
    }

    static ConfigurationSpecification GetConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation="", const std::string& robotname="", const std::string& manipname="");

    inline ConfigurationSpecification GetConfigurationSpecification(const std::string& interpolation="", const std::string& robotname="", const std::string& manipname="") const
    {
        return GetConfigurationSpecification(GetType(), interpolation, robotname, manipname);
    }

    /// \brief in-place left-transform into a new coordinate system. Equivalent to t * ikparam
    inline IkParameterization& MultiplyTransform(const Transform& t) {
        switch(GetType()) {
        case IKP_Transform6D:
            _transform = t * _transform;
            break;
        case IKP_Transform6DVelocity:
            _transform.trans = t.rotate(_transform.trans);
            _transform.rot = quatMultiply(t.rot,_transform.rot);
            break;
        case IKP_Rotation3D:
        case IKP_Rotation3DVelocity:
            _transform.rot = quatMultiply(t.rot,_transform.rot);
            break;
        case IKP_Translation3D:
            _transform.trans = t * _transform.trans;
            break;
        case IKP_Translation3DVelocity:
            _transform.trans = t.rotate(_transform.trans);
            break;
        case IKP_Direction3D:
        case IKP_Direction3DVelocity:
            _transform.rot = t.rotate(_transform.rot);
            break;
        case IKP_Ray4D:
            _transform.trans = t * _transform.trans;
            _transform.rot = t.rotate(_transform.rot);
            break;
        case IKP_Ray4DVelocity:
            _transform.trans = t.rotate(_transform.trans);
            _transform.rot = t.rotate(_transform.rot);
            break;
        case IKP_Lookat3D:
            SetLookat3D(RAY(t*GetLookat3D(),t.rotate(GetLookat3DDirection())));
            break;
        case IKP_TranslationDirection5D:
            _transform.trans = t * _transform.trans;
            _transform.rot = t.rotate(_transform.rot);
            break;
        case IKP_TranslationDirection5DVelocity:
            _transform.trans = t.rotate(_transform.trans);
            _transform.rot = t.rotate(_transform.rot);
            break;
        case IKP_TranslationXY2D:
            SetTranslationXY2D(t*GetTranslationXY2D());
            break;
        case IKP_TranslationXY2DVelocity:
            _transform.trans = t.rotate(_transform.trans);
            break;
        case IKP_TranslationXYOrientation3D: {
            Vector v = GetTranslationXYOrientation3D();
            Vector voldtrans(v.x,v.y,0);
            Vector vnewtrans = t*voldtrans;
            dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
            SetTranslationXYOrientation3D(Vector(vnewtrans.x,vnewtrans.y,v.z+zangle));
            break;
        }
        case IKP_TranslationXYOrientation3DVelocity: {
            Vector v = GetTranslationXYOrientation3D();
            Vector voldtrans(v.x,v.y,0);
            _transform.trans = t.rotate(voldtrans);
            _transform.trans.z = quatRotate(t.rot,Vector(0,0,v.z)).z;
            break;
        }
        case IKP_TranslationLocalGlobal6D:
            _transform.trans = t*_transform.trans;
            break;
        case IKP_TranslationLocalGlobal6DVelocity:
            _transform.trans = t.rotate(_transform.trans);
            break;
        case IKP_TranslationXAxisAngle4D: {
            _transform.trans = t*_transform.trans;
            // do not support rotations
            break;
        }
        case IKP_TranslationXAxisAngle4DVelocity: {
            _transform.trans = t.rotate(_transform.trans);
            // do not support rotations
            break;
        }
        case IKP_TranslationYAxisAngle4D: {
            _transform.trans = t*_transform.trans;
            // do not support rotations
            break;
        }
        case IKP_TranslationYAxisAngle4DVelocity: {
            _transform.trans = t.rotate(_transform.trans);
            // do not support rotations
            break;
        }
        case IKP_TranslationZAxisAngle4D: {
            _transform.trans = t*_transform.trans;
            // do not support rotations
            break;
        }
        case IKP_TranslationZAxisAngle4DVelocity: {
            _transform.trans = t.rotate(_transform.trans);
            // do not support rotations
            break;
        }

        case IKP_TranslationXAxisAngleZNorm4D: {
            _transform.trans = t*_transform.trans;
            // only support rotation along z-axis
            _transform.rot.x -= normalizeAxisRotation(Vector(0,0,1),t.rot).first;
            break;
        }
        case IKP_TranslationXAxisAngleZNorm4DVelocity: {
            _transform.trans = t.rotate(_transform.trans);
            // only support rotation along z-axis
            _transform.rot.x = quatRotate(t.rot,Vector(0,0,_transform.rot.x)).z;
            break;
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            _transform.trans = t*_transform.trans;
            // only support rotation along x-axis
            _transform.rot.x -= normalizeAxisRotation(Vector(1,0,0),t.rot).first;
            break;
        }
        case IKP_TranslationYAxisAngleXNorm4DVelocity: {
            _transform.trans = t.rotate(_transform.trans);
            // only support rotation along x-axis
            _transform.rot.x = quatRotate(t.rot,Vector(_transform.rot.x,0,0)).x;
            break;
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            _transform.trans = t*_transform.trans;
            // only support rotation along y-axis
            _transform.rot.x -= normalizeAxisRotation(Vector(0,1,0),t.rot).first;
            break;
        }
        case IKP_TranslationZAxisAngleYNorm4DVelocity: {
            _transform.trans = t.rotate(_transform.trans);
            // only support rotation along y-axis
            _transform.rot.x = quatRotate(t.rot,Vector(0,_transform.rot.x,0)).y;
            break;
        }
        default:
            throw openrave_exception(str(boost::format("parameterization 0x%x does not support left-transform")%GetType()));
        }
        for(std::map<std::string, std::vector<dReal> >::iterator it = _mapCustomData.begin(); it != _mapCustomData.end(); ++it) {
            _MultiplyTransform(t, it->first, it->second);
        }
        return *this;
    }

    /** \brief in-place right-transform into a new coordinate system. Equivalent to ikparam*t

        Note that depending on the ikparam type, some information from the passed in transform can get lost or misinterpreted. For example
        Translation3D types do not have a rotation, so assume identity.

        For ik types that have 3D directions stored, the transformation is the following:
        \code
        quatRotate(quatMultiply(quatRotateDirection(Vector(0,0,1),direction), t.rot), Vector(0,0,1))
        \endcode
        Basically it is how the local z axis gets transformed and converting that back to world coordinates.
     */
    inline IkParameterization& MultiplyTransformRight(const Transform& t) {
        switch(GetType()) {
        case IKP_Transform6D:
            _transform *= t;
            break;
//        case IKP_Transform6DVelocity:
//            _transform.trans = t.rotate(_transform.trans);
//            _transform.rot = quatMultiply(t.rot,_transform.rot);
//            break;
        case IKP_Rotation3D:
//        case IKP_Rotation3DVelocity:
            _transform.rot = quatMultiply(_transform.rot,t.rot);
            break;
        case IKP_Translation3D:
            _transform.trans = _transform.trans + t.trans;
            break;
//        case IKP_Translation3DVelocity:
//            _transform.trans = t.rotate(_transform.trans);
//            break;
        case IKP_Direction3D:
//        case IKP_Direction3DVelocity:
            _transform.rot = quatRotate(quatMultiply(quatRotateDirection(Vector(0,0,1),_transform.rot), t.rot), Vector(0,0,1));
            break;
//        case IKP_Ray4D:
//            _transform.trans = t * _transform.trans;
//            _transform.rot = t.rotate(_transform.rot);
//            break;
//        case IKP_Ray4DVelocity:
//            _transform.trans = t.rotate(_transform.trans);
//            _transform.rot = t.rotate(_transform.rot);
//            break;
        case IKP_Lookat3D:
            SetLookat3D(GetLookat3D() + t.trans);
            break;
        case IKP_TranslationDirection5D: {
            Vector qorig = quatRotateDirection(Vector(0,0,1),_transform.rot);
            Vector q = quatMultiply(qorig, t.rot);
            _transform.trans += quatRotate(qorig,t.trans);
            _transform.rot = quatRotate(q, Vector(0,0,1));
            break;
        }
//        case IKP_TranslationDirection5DVelocity:
//            _transform.trans = t.rotate(_transform.trans);
//            _transform.rot = t.rotate(_transform.rot);
//            break;
        case IKP_TranslationXY2D:
            SetTranslationXY2D(GetTranslationXY2D() + t.trans);
            break;
//        case IKP_TranslationXY2DVelocity:
//            _transform.trans = t.rotate(_transform.trans);
//            break;
        case IKP_TranslationXYOrientation3D: {
            Vector v = GetTranslationXYOrientation3D();
            Vector voldtrans(v.x,v.y,0);
            Vector q = quatFromAxisAngle(Vector(0,0,1),v.z);
            Vector vnewtrans = voldtrans + quatRotate(q,t.trans);
            dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
            SetTranslationXYOrientation3D(Vector(vnewtrans.x,vnewtrans.y,v.z+zangle));
            break;
        }
//        case IKP_TranslationXYOrientation3DVelocity: {
//            Vector v = GetTranslationXYOrientation3D();
//            Vector voldtrans(v.x,v.y,0);
//            _transform.trans = t.rotate(voldtrans);
//            _transform.trans.z = quatRotate(t.rot,Vector(0,0,v.z)).z;
//            break;
//        }
        case IKP_TranslationLocalGlobal6D:
            _transform.trans = _transform.trans + t.trans;
            break;
//        case IKP_TranslationLocalGlobal6DVelocity:
//            _transform.trans = t.rotate(_transform.trans);
//            break;
//        case IKP_TranslationXAxisAngle4D: {
//            _transform.trans = t*_transform.trans;
//            // do not support rotations
//            break;
//        }
//        case IKP_TranslationXAxisAngle4DVelocity: {
//            _transform.trans = t.rotate(_transform.trans);
//            // do not support rotations
//            break;
//        }
//        case IKP_TranslationYAxisAngle4D: {
//            _transform.trans = t*_transform.trans;
//            // do not support rotations
//            break;
//        }
//        case IKP_TranslationYAxisAngle4DVelocity: {
//            _transform.trans = t.rotate(_transform.trans);
//            // do not support rotations
//            break;
//        }
//        case IKP_TranslationZAxisAngle4D: {
//            _transform.trans = t*_transform.trans;
//            // do not support rotations
//            break;
//        }
//        case IKP_TranslationZAxisAngle4DVelocity: {
//            _transform.trans = t.rotate(_transform.trans);
//            // do not support rotations
//            break;
//        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            Vector q = quatFromAxisAngle(Vector(0,0,1),_transform.rot.x);
            _transform.trans += quatRotate(q,t.trans);
            // only support rotation along z-axis
            _transform.rot.x -= normalizeAxisRotation(Vector(0,0,1),t.rot).first;
            break;
        }
//        case IKP_TranslationXAxisAngleZNorm4DVelocity: {
//            _transform.trans = t.rotate(_transform.trans);
//            // only support rotation along z-axis
//            _transform.rot.x = quatRotate(t.rot,Vector(0,0,_transform.rot.x)).z;
//            break;
//        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            Vector q = quatFromAxisAngle(Vector(1,0,0),_transform.rot.x);
            _transform.trans += quatRotate(q,t.trans);
            // only support rotation along x-axis
            _transform.rot.x -= normalizeAxisRotation(Vector(1,0,0),t.rot).first;
            break;
        }
//        case IKP_TranslationYAxisAngleXNorm4DVelocity: {
//            _transform.trans = t.rotate(_transform.trans);
//            // only support rotation along x-axis
//            _transform.rot.x = quatRotate(t.rot,Vector(_transform.rot.x,0,0)).x;
//            break;
//        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            Vector q = quatFromAxisAngle(Vector(0,1,0),_transform.rot.x);
            _transform.trans += quatRotate(q,t.trans);
            // only support rotation along y-axis
            _transform.rot.x -= normalizeAxisRotation(Vector(0,1,0),t.rot).first;
            break;
        }
//        case IKP_TranslationZAxisAngleYNorm4DVelocity: {
//            _transform.trans = t.rotate(_transform.trans);
//            // only support rotation along y-axis
//            _transform.rot.x = quatRotate(t.rot,Vector(0,_transform.rot.x,0)).y;
//            break;
//        }
        default:
            throw openrave_exception(str(boost::format("parameterization 0x%x does not support right-transforms")%GetType()));
        }
        for(std::map<std::string, std::vector<dReal> >::iterator it = _mapCustomData.begin(); it != _mapCustomData.end(); ++it) {
            _MultiplyTransformRight(t, it->first, it->second);
        }
        return *this;
    }

    inline IkParameterization operator*(const Transform& t) const {
        IkParameterization iknew(*this);
        iknew.MultiplyTransformRight(t);
        return iknew;
    }

    inline void Swap(IkParameterization& r) {
        std::swap(_transform, r._transform);
        std::swap(_type, r._type);
        _mapCustomData.swap(r._mapCustomData);
    }

    void SerializeJSON(rapidjson::Value& rIkParameterization, rapidjson::Document::AllocatorType& alloc, dReal fUnitScale=1.0) const;

    void DeserializeJSON(const rapidjson::Value& rIkParameterization, dReal fUnitScale=1.0);

    virtual bool operator==(const IkParameterization& other) const {
        return _type == other._type
            && _transform == other._transform
            && _mapCustomData == other._mapCustomData;
    }

    virtual bool operator!=(const IkParameterization& other) const {
        return !operator==(other);
    }

protected:
    inline static bool _IsValidCharInName(char c) {
        return c < 0 || c >= 33;
    }
    inline static void _MultiplyTransform(const Transform& t, const std::string& name, std::vector<dReal>& values)
    {
        size_t startoffset = name.find("_transform=");
        if( startoffset != std::string::npos ) {
            size_t endoffset = name.find("_", startoffset+11);
            std::string transformtype;
            if( endoffset == std::string::npos ) {
                transformtype = name.substr(startoffset+11);
            }
            else {
                transformtype = name.substr(startoffset+11,endoffset-startoffset-11);
            }
            if( transformtype == "direction" ) {
                if (values.size() < 3) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
                }
                Vector v(values[0],values[1], values[2]);
                v = t.rotate(v);
                values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
            }
            else if( transformtype == "point" ) {
                if (values.size() < 3) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
                }
                Vector v(values[0],values[1], values[2]);
                v = t*v;
                values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
            }
            else if( transformtype == "quat" ) {
                if (values.size() < 4) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 4", ORE_InvalidArguments);
                }
                Vector v(values[0],values[1], values[2],values[3]);
                v = quatMultiply(t.rot,v);
                values[0] = v[0]; values[1] = v[1]; values[2] = v[2]; values[3] = v[3];
            }
            else if( transformtype == "ikparam" ) {
                IkParameterizationType newiktype = RaveGetIkTypeFromUniqueId(static_cast<int>(values.at(0)+0.5));
                IkParameterization newikparam;
                OPENRAVE_ASSERT_OP_FORMAT0(IkParameterization::GetNumberOfValues(newiktype)+1, ==, (int)values.size(),"expected values not equal",ORE_InvalidState);
                newikparam.SetValues(values.begin()+1,newiktype);
                newikparam.MultiplyTransform(t);
                newikparam.GetValues(values.begin()+1);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("IkParameterization custom data '%s' does not have a valid transform",name,ORE_InvalidState);
            }
        }
    }

    inline static void _MultiplyTransformRight(const Transform& t, const std::string& name, std::vector<dReal>& values)
    {
        size_t startoffset = name.find("_transform=");
        if( startoffset != std::string::npos ) {
            size_t endoffset = name.find("_", startoffset+11);
            std::string transformtype;
            if( endoffset == std::string::npos ) {
                transformtype = name.substr(startoffset+11);
            }
            else {
                transformtype = name.substr(startoffset+11,endoffset-startoffset-11);
            }
            if( transformtype == "direction" ) {
                if (values.size() < 3) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
                }
                Vector v(values[0],values[1], values[2]);
                v = quatRotate(quatMultiply(quatRotateDirection(Vector(0,0,1),v), t.rot), Vector(0,0,1));
                values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
            }
            else if( transformtype == "point" ) {
                if (values.size() < 3) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
                }
                Vector v(values[0],values[1], values[2]);
                v += t.trans;
                values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
            }
            else if( transformtype == "quat" ) {
                if (values.size() < 4) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 4", ORE_InvalidArguments);
                }
                Vector v(values[0],values[1], values[2],values[3]);
                v = quatMultiply(v,t.rot);
                values[0] = v[0]; values[1] = v[1]; values[2] = v[2]; values[3] = v[3];
            }
            else if( transformtype == "ikparam" ) {
                IkParameterizationType newiktype = RaveGetIkTypeFromUniqueId(static_cast<int>(values.at(0)+0.5));
                IkParameterization newikparam;
                OPENRAVE_ASSERT_OP_FORMAT0(IkParameterization::GetNumberOfValues(newiktype)+1, ==, (int)values.size(),"expected values not equal",ORE_InvalidState);
                newikparam.SetValues(values.begin()+1,newiktype);
                newikparam.MultiplyTransformRight(t);
                newikparam.GetValues(values.begin()+1);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("IkParameterization custom data '%s' does not have a valid transform",name,ORE_InvalidState);
            }
        }
    }

    Transform _transform;
    IkParameterizationType _type;
    std::map<std::string, std::vector<dReal> > _mapCustomData;

    friend IkParameterization operator* (const Transform &t, const IkParameterization &ikparam);
    friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam);
    friend OPENRAVE_API std::istream& operator>>(std::istream& I, IkParameterization& ikparam);
};

inline IkParameterization operator* (const Transform &t, const IkParameterization &ikparam)
{
    IkParameterization local;
    switch(ikparam.GetType()) {
    case IKP_Transform6D:
        local.SetTransform6D(t * ikparam.GetTransform6D());
        break;
    case IKP_Rotation3D:
        local.SetRotation3D(quatMultiply(t.rot,ikparam.GetRotation3D()));
        break;
    case IKP_Translation3D:
        local.SetTranslation3D(t*ikparam.GetTranslation3D());
        break;
    case IKP_Direction3D:
        local.SetDirection3D(t.rotate(ikparam.GetDirection3D()));
        break;
    case IKP_Ray4D:
        local.SetRay4D(RAY(t*ikparam.GetRay4D().pos,t.rotate(ikparam.GetRay4D().dir)));
        break;
    case IKP_Lookat3D:
        local.SetLookat3D(RAY(t*ikparam.GetLookat3D(),t.rotate(ikparam.GetLookat3DDirection())));
        break;
    case IKP_TranslationDirection5D:
        local.SetTranslationDirection5D(RAY(t*ikparam.GetTranslationDirection5D().pos,t.rotate(ikparam.GetTranslationDirection5D().dir)));
        break;
    case IKP_TranslationXY2D:
        local.SetTranslationXY2D(t*ikparam.GetTranslationXY2D());
        break;
    case IKP_TranslationXYOrientation3D: {
        Vector v = ikparam.GetTranslationXYOrientation3D();
        Vector voldtrans(v.x,v.y,0);
        Vector vnewtrans = t*voldtrans;
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        local.SetTranslationXYOrientation3D(Vector(vnewtrans.x,vnewtrans.y,v.z+zangle));
        break;
    }
    case IKP_TranslationLocalGlobal6D:
        local.SetTranslationLocalGlobal6D(ikparam.GetTranslationLocalGlobal6D().first, t*ikparam.GetTranslationLocalGlobal6D().second);
        break;
    case IKP_TranslationXAxisAngle4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationXAxisAngle4D();
        // don't change the angle since don't know the exact direction it is pointing at
        local.SetTranslationXAxisAngle4D(t*p.first,p.second);
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationYAxisAngle4D();
        // don't change the angle since don't know the exact direction it is pointing at
        local.SetTranslationYAxisAngle4D(t*p.first,p.second);
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationZAxisAngle4D();
        // don't change the angle since don't know the exact direction it is pointing at
        local.SetTranslationZAxisAngle4D(t*p.first,p.second);
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationXAxisAngleZNorm4D();
        // don't change the angle since don't know the exact direction it is pointing at
        local.SetTranslationXAxisAngleZNorm4D(t*p.first,p.second);
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationYAxisAngleXNorm4D();
        // don't change the angle since don't know the exact direction it is pointing at
        local.SetTranslationYAxisAngleXNorm4D(t*p.first,p.second);
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationZAxisAngleYNorm4D();
        // don't change the angle since don't know the exact direction it is pointing at
        local.SetTranslationZAxisAngleYNorm4D(t*p.first,p.second);
        break;
    }
    default:
        // internal MultiplyTransform supports more types
        return IkParameterization(ikparam).MultiplyTransform(t);
    }
    local._mapCustomData = ikparam._mapCustomData;
    for(std::map<std::string, std::vector<dReal> >::iterator it = local._mapCustomData.begin(); it != local._mapCustomData.end(); ++it) {
        IkParameterization::_MultiplyTransform(t, it->first,it->second);
    }
    return local;
}

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam);
OPENRAVE_API std::istream& operator>>(std::istream& I, IkParameterization& ikparam);

/// \brief User data for trimesh geometries. Vertices are defined in counter-clockwise order for outward pointing faces.
class OPENRAVE_API TriMesh
{
public:
    std::vector<Vector> vertices;
    std::vector<int32_t> indices;

    void ApplyTransform(const Transform& t);
    void ApplyTransform(const TransformMatrix& t);

    /// append another TRIMESH to this tri mesh
    void Append(const TriMesh& mesh);
    void Append(const TriMesh& mesh, const Transform& trans);

    AABB ComputeAABB() const;
    void serialize(std::ostream& o, int options=0) const;

    friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const TriMesh &trimesh);
    friend OPENRAVE_API std::istream& operator>>(std::istream& I, TriMesh& trimesh);
};

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const TriMesh& trimesh);
OPENRAVE_API std::istream& operator>>(std::istream& I, TriMesh& trimesh);

/// \brief Selects which DOFs of the affine transformation to include in the active configuration.
enum DOFAffine
{
    DOF_NoTransform = 0,
    DOF_X = 1,     ///< can move in the x direction
    DOF_Y = 2,     ///< can move in the y direction
    DOF_Z = 4,     ///< can move in the z direction
    DOF_XYZ=DOF_X|DOF_Y|DOF_Z,     ///< moves in xyz direction

    // DOF_RotationX fields are mutually exclusive
    DOF_RotationAxis = 8,     ///< can rotate around an axis (1 dof)
    DOF_Rotation3D = 16,     ///< can rotate freely (3 dof), the parameterization is
                             ///< theta * v, where v is the rotation axis and theta is the angle about that axis
    DOF_RotationQuat = 32,     ///< can rotate freely (4 dof), parameterization is a quaternion. In order for limits to work correctly, the quaternion is in the space of _vRotationQuatLimitStart. _vRotationQuatLimitStart is always left-multiplied before setting the transform!
    DOF_RotationMask=(DOF_RotationAxis|DOF_Rotation3D|DOF_RotationQuat), ///< mask for all bits representing 3D rotations
    DOF_Transform = (DOF_XYZ|DOF_RotationQuat), ///< translate and rotate freely in 3D space
};

/** \brief returns a string representation of the error code
 */
OPENRAVE_API const char* RaveGetErrorCodeString(OpenRAVEErrorCode error);

/** \brief Given a mask of affine dofs and a dof inside that mask, returns the index where the value could be found.

    \param affinedofs a mask of \ref DOFAffine values
    \param dof a set of values inside affinedofs, the index of the first value is returned
    \throw openrave_exception throws if dof is not present in affinedofs
 */
OPENRAVE_API int RaveGetIndexFromAffineDOF(int affinedofs, DOFAffine dof);

/** \brief Given a mask of affine dofs and an index into the array, returns the affine dof that is being referenced

    \param affinedofs a mask of \ref DOFAffine values
    \param index an index into the affine dof array
    \throw openrave_exception throws if dof if index is out of bounds
 */
OPENRAVE_API DOFAffine RaveGetAffineDOFFromIndex(int affinedofs, int index);

/// \brief Returns the degrees of freedom needed to represent all the values in the affine dof mask.
///
/// \throw openrave_exception throws if
OPENRAVE_API int RaveGetAffineDOF(int affinedofs);

/** \brief Converts the transformation matrix into the specified affine values format.

    \param[out] itvalues an iterator to the vector to write the values to. Will write exactly \ref RaveGetAffineDOF(affinedofs) values.
    \param[in] t the affine transformation to convert
    \param[in] affinedofs the affine format to output values in
    \param[in] axis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
 */
OPENRAVE_API void RaveGetAffineDOFValuesFromTransform(std::vector<dReal>::iterator itvalues, const Transform& t, int affinedofs, const Vector& axis=Vector(0,0,1));

/** \brief Converts the linar and angular velocities into the specified affine values format.

    \param[out] itvalues an iterator to the vector to write the values to. Will write exactly \ref RaveGetAffineDOF(affinedofs) values.
    \param[in] linearvel the linear velocity to convert
    \param[in] angularvel the angular velocity to convert
    \param[in] quatrotation the rotation (in quaternion) of the frame that has the linearvel and angularvel. Used if affinedofs specified \ref DOF_RotationAxis.
    \param[in] affinedofs the affine format to output values in
    \param[in] axis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
 */
OPENRAVE_API void RaveGetAffineDOFValuesFromVelocity(std::vector<dReal>::iterator itvalues, const Vector& linearvel, const Vector& angularvel, const Vector& quatrotation, int affinedofs, const Vector& axis=Vector(0,0,1));

/** \brief Converts affine dof values into a transform.

    Note that depending on what the dof values holds, only a part of the transform will be updated.
    \param[inout] t the output transform
    \param[in] itvalues the start iterator of the affine dof values
    \param[in] affinedofs the affine dof mask
    \param[in] axis optional rotation axis if affinedofs specified (vActvAffineRotationAxis of RobotBase) \ref DOF_RotationAxis
    \param[in] normalize if true will normalize rotations, should set to false if extracting velocity data
 */
OPENRAVE_API void RaveGetTransformFromAffineDOFValues(Transform& t, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& axis=Vector(0,0,1), bool normalize=true);

/** \brief Converts affine dof velocities into linear and angular velocity vectors

    Note that depending on what the dof values holds, only a part of the transform will be updated.
    \param[out] linearvel the output transform
    \param[in] itvalues the start iterator of the affine dof values
    \param[in] affinedofs the affine dof mask
    \param[in] axis optional rotation axis if affinedofs specified (vActvAffineRotationAxis of RobotBase) \ref DOF_RotationAxis
    \param[in] normalize if true will normalize rotations, should set to false if extracting velocity data
    \param[in] quatrotation the quaternion of 3d rotation of the frame where the velocity is being measured at.
 */
OPENRAVE_API void RaveGetVelocityFromAffineDOFVelocities(Vector& linearvel, Vector& angularvel, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& axis=Vector(0,0,1), const Vector& quatrotation = Vector(1,0,0,0));

OPENRAVE_API ConfigurationSpecification RaveGetAffineConfigurationSpecification(int affinedofs,KinBodyConstPtr pbody=KinBodyConstPtr(),const std::string& interpolation="");

}

#include <openrave/plugininfo.h>
#include <openrave/interface.h>
#include <openrave/spacesampler.h>
#include <openrave/kinbody.h>
#include <openrave/trajectory.h>
#include <openrave/module.h>
#include <openrave/collisionchecker.h>
#include <openrave/sensor.h>
#include <openrave/robot.h>
#include <openrave/iksolver.h>
#include <openrave/planner.h>
#include <openrave/controller.h>
#include <openrave/physicsengine.h>
#include <openrave/sensorsystem.h>
#include <openrave/viewer.h>
#include <openrave/environment.h>

namespace OpenRAVE {

/// \name Global Functionality - Interface Creation, Plugin Management, Logging
/// \anchor global_functionality
//@{

/// \brief Returns the a 16 character null-terminated string specifying a hash of the interfaces used for checking changes.
inline const char* RaveGetInterfaceHash(InterfaceType type)
{
    switch(type) {
    case PT_Planner: return OPENRAVE_PLANNER_HASH;
    case PT_Robot: return OPENRAVE_ROBOT_HASH;
    case PT_SensorSystem: return OPENRAVE_SENSORSYSTEM_HASH;
    case PT_Controller: return OPENRAVE_CONTROLLER_HASH;
    case PT_Module: return OPENRAVE_MODULE_HASH;
    case PT_InverseKinematicsSolver: return OPENRAVE_IKSOLVER_HASH;
    case PT_KinBody: return OPENRAVE_KINBODY_HASH;
    case PT_PhysicsEngine: return OPENRAVE_PHYSICSENGINE_HASH;
    case PT_Sensor: return OPENRAVE_SENSOR_HASH;
    case PT_CollisionChecker: return OPENRAVE_COLLISIONCHECKER_HASH;
    case PT_Trajectory: return OPENRAVE_TRAJECTORY_HASH;
    case PT_Viewer: return OPENRAVE_VIEWER_HASH;
    case PT_SpaceSampler: return OPENRAVE_SPACESAMPLER_HASH;
    default:
        throw openrave_exception("failed to find openrave interface type",ORE_InvalidArguments);
        return NULL;
    }
}

/// \brief Safely casts from the base interface class to an openrave interface using static_pointer_cast.
///
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T> RaveInterfaceCast(InterfaceBasePtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T>(pinterface);
        }
        // encode special cases
        if((pinterface->GetInterfaceType() == PT_Robot)&&(T::GetInterfaceTypeStatic() == PT_KinBody)) {
            return boost::static_pointer_cast<T>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// \brief Safely casts from the base interface class to an openrave interface using static_pointer_cast.
///
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T const> RaveInterfaceConstCast(InterfaceBaseConstPtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
        // encode special cases
        if((pinterface->GetInterfaceType() == PT_Robot)&&(T::GetInterfaceTypeStatic() == PT_KinBody)) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// \brief returns a lower case string of the interface type
OPENRAVE_API const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap();
OPENRAVE_API const std::string& RaveGetInterfaceName(InterfaceType type);

/// \brief Returns the openrave home directory where settings, cache, and other files are stored.
///
/// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
OPENRAVE_API std::string RaveGetHomeDirectory();

/// \brief Searches for a filename in the database and returns a full path/URL to it
///
/// \param filename the relative filename in the database
/// \param bRead if true will only return a file if it exists. If false, will return the filename of the first valid database directory.
/// \return a non-empty string if a file could be found.
OPENRAVE_API std::string RaveFindDatabaseFile(const std::string& filename, bool bRead=true);

/// \brief Explicitly initializes the global OpenRAVE state (optional).
///
/// Optional function to initialize openrave plugins, logging, and read OPENRAVE_* environment variables.
/// Although environment creation will automatically make sure this function is called, users might want
/// explicit control of when this happens.
/// Will not do anything if OpenRAVE runtime is already initialized. If OPENRAVE_* environment variables must be re-read, first call \ref RaveDestroy.
/// \param bLoadAllPlugins If true will load all the openrave plugins automatically that can be found in the OPENRAVE_PLUGINS environment path
/// \return 0 if successful, otherwise an error code
OPENRAVE_API int RaveInitialize(bool bLoadAllPlugins=true, int level = Level_Info);

/// \brief Initializes the global state from an already loaded OpenRAVE environment.
///
/// Because of shared object boundaries, it is necessary to pass the global state pointer
/// around. If using plugin.h, this function is automatically called by \ref CreateInterfaceValidated.
/// It is also called by and every InterfaceBase constructor.
/// \param[in] globalstate
OPENRAVE_API void RaveInitializeFromState(UserDataPtr globalstate);

/// \brief A pointer to the global openrave state
/// \return a managed pointer to the state.
OPENRAVE_API UserDataPtr RaveGlobalState();

/// \brief Destroys the entire OpenRAVE state and all loaded environments.
///
/// This functions should be always called before program shutdown in order to assure all
/// resources are relased appropriately.
OPENRAVE_API void RaveDestroy();

/// \brief Add a callback when the OpenRAVE global runtime is destroyed.
///
/// The callback is called after all OpenRAVE environments have been destroyed and
/// before plugins are unloaded.
/// Callback is added only for this run-time. Once the run-time is destroyed/swapped, it will have to be re-added.
/// OpenRAVE runtime is destroyed when \ref RaveDestroy is called or on system exits.
OPENRAVE_API void RaveAddCallbackForDestroy(const boost::function<void()>& fn);

/// \brief Get all the loaded plugins and the interfaces they support.
///
/// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
OPENRAVE_API void RaveGetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins);

/// \brief Get a list of all the loaded interfaces.
OPENRAVE_API void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames);

/// \brief Reloads all the plugins.
///
/// The interfaces currently created remain will continue using the old plugins, so this function is safe in that plugins currently loaded remain loaded until the last interface that uses them is released.
OPENRAVE_API void RaveReloadPlugins();

/// \brief Load a plugin and its interfaces.
///
/// If the plugin is already loaded, will reload it.
/// \param name the filename of the plugin to load
OPENRAVE_API bool RaveLoadPlugin(const std::string& libraryname);

/// \brief Returns true if interface can be created, otherwise false.
OPENRAVE_API bool RaveHasInterface(InterfaceType type, const std::string& interfacename);

OPENRAVE_API InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr env, InterfaceType type,const std::string& interfacename);
OPENRAVE_API RobotBasePtr RaveCreateRobot(EnvironmentBasePtr env, const std::string& name="");
OPENRAVE_API PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ControllerBasePtr RaveCreateController(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API MultiControllerBasePtr RaveCreateMultiController(EnvironmentBasePtr env, const std::string& name="");
OPENRAVE_API ModuleBasePtr RaveCreateModule(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateProblem(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateProblemInstance(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API SensorBasePtr RaveCreateSensor(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API SpaceSamplerBasePtr RaveCreateSpaceSampler(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr env, const std::string& name="");
/// \brief Return an empty trajectory instance.
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr env, const std::string& name="");

/// \deprecated (11/10/01)
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr env, int dof) RAVE_DEPRECATED;

/// \brief returned a fully cloned interface
///
/// \param preference the InterfaceBasePtr to clone
/// \param cloningoptions combination of CO_*
/// \param pcloneenv the environment to create the new clone in. If not specified, will use preference->GetEnv()
template <typename T>
inline boost::shared_ptr<T> RaveClone(boost::shared_ptr<T const> preference, int cloningoptions, EnvironmentBasePtr pcloneenv=EnvironmentBasePtr())
{
    InterfaceBasePtr pcloned = RaveCreateInterface(!pcloneenv ? preference->GetEnv() : pcloneenv, preference->GetInterfaceType(), preference->GetXMLId());
    OPENRAVE_ASSERT_FORMAT(!!pcloned, "Failed to clone interface=%s id=%s", RaveGetInterfaceName(preference->GetInterfaceType())%preference->GetXMLId(), ORE_InvalidArguments);
    boost::shared_ptr<T> pclonedcast = boost::dynamic_pointer_cast<T>(pcloned);
    OPENRAVE_ASSERT_FORMAT(!!pclonedcast, "Interface created but failed to cast interface=%s id=%s", RaveGetInterfaceName(preference->GetInterfaceType())%preference->GetXMLId(), ORE_InvalidArguments);
    pclonedcast->Clone(preference,cloningoptions);
    return pclonedcast;
}

/** \brief Registers a function to create an interface, this allows the interface to be created by other modules.

    \param type interface type
    \param name interface name
    \param interfacehash the hash of the interface being created (use the global defines OPENRAVE_X_HASH)
    \param envhash the hash of the environment (use the global define OPENRAVE_ENVIRONMENT_HASH)
    \param createfn functions to create the interface it takes two parameters: the environment and an istream to the rest of the interface creation arguments.
    \return a handle if function is successfully registered. By destroying the handle, the interface will be automatically unregistered.
    \throw openrave_exception Will throw with ORE_InvalidInterfaceHash if hashes do not match
 */
OPENRAVE_API UserDataPtr RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn);

/** \brief Registers a custom xml reader for a particular interface.

    Once registered, anytime an interface is created through XML and
    the xmltag is seen, the function CreateXMLReaderFn will be called to get a reader for that tag
    \param xmltag the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    \param fn CreateXMLReaderFn(pinterface,atts) - passed in the pointer to the interface where the tag was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
 */
OPENRAVE_API UserDataPtr RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn);

/** \brief Registers a custom json reader for a particular interface.

    Once registered, anytime an interface is created through JSON and
    the id is seen, the function CreateJSONReaderFn will be called to get a reader
    \param id the id specified is seen in the interface, the the custom reader will be created.
    \param fn CreateJSONReaderFn(pinterface,atts) - passed in the pointer to the interface where the id was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
 */
OPENRAVE_API UserDataPtr RaveRegisterJSONReader(InterfaceType type, const std::string& id, const CreateJSONReaderFn& fn);

/// \brief return the environment's unique id, returns 0 if environment could not be found or not registered
OPENRAVE_API int RaveGetEnvironmentId(EnvironmentBaseConstPtr env);

/// \brief get the environment from its unique id
/// \param id the unique environment id returned by \ref RaveGetEnvironmentId
OPENRAVE_API EnvironmentBasePtr RaveGetEnvironment(int id);

/// \brief Return all the created OpenRAVE environments.
OPENRAVE_API void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments);

/// \brief Returns the current registered reader for the interface type/xmlid
///
/// \throw openrave_exception Will throw with ORE_InvalidArguments if registered function could not be found.
OPENRAVE_API BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts);

/// \brief Returns the current registered json reader for the interface type/id
///
/// \throw openrave_exception Will throw with ORE_InvalidArguments if registered function could not be found.
OPENRAVE_API BaseJSONReaderPtr RaveCallJSONReader(InterfaceType type, const std::string& id, InterfaceBasePtr pinterface, const AttributesList& atts);

/** \brief Returns the absolute path of the filename on the local filesystem resolving relative paths from OpenRAVE paths.

    The OpenRAVE paths consist of a list of directories specified by $OPENRAVE_DATA environment variable and custom added user paths.
    Requires boost::filesystem to be installed
    \param filename the filename to look for
    \param curdir the current directory in case the filename is relative
    \return an empty string if file isn't found, otherwise path to full filename on local filesystem
 */
OPENRAVE_API std::string RaveFindLocalFile(const std::string& filename, const std::string& curdir="");

/** \brief Given the absolute filename, return the relative path from one of the OPENRAVE_DATA directories.

    Will check if filename is inside one of the OPENRAVE_DATA directories, and set newfilename to the relative path.
    \return true if inside a OPENRAVE_DATA directory.
 */
OPENRAVE_API bool RaveInvertFileLookup(std::string& newfilename, const std::string& filename);

/// \brief Sets the default data access options for cad resources/robot files
///
/// Controls how files are processed in functions like \ref RaveFindLocalFile
/// \param accessoptions - if 1 will only allow resources inside directories specified from OPERNAVE_DATA environment variable. This allows reject of full paths from unsecure/unauthenticated resources.
OPENRAVE_API void RaveSetDataAccess(int accessoptions);

/// \brief Returns the acess options set
///
/// \see RaveSetDataAccess
OPENRAVE_API int RaveGetDataAccess();

/// \brief Gets the default viewer type name
OPENRAVE_API std::string RaveGetDefaultViewerType();

/** \brief Returns the gettext translated string of the given message id

    \param domainname translation domain name
    \param msgid message id to look for
    \return if a translation was found, it is converted to the locale's codeset and returned. The resulting string is statically allocated and must not be modified or freed. Otherwise msgid is returned.
 */
OPENRAVE_API const char *RaveGetLocalizedTextForDomain(const std::string& domainname, const char *msgid);

//@}

/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API void RaveInitRandomGeneration(uint32_t seed);
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API uint32_t RaveRandomInt();
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API float RaveRandomFloat(IntervalType interval=IT_Closed);
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API double RaveRandomDouble(IntervalType interval=IT_Closed);

/// \deprecated (12/02/06) see \ref OpenRAVE::utils::TokenizeString
bool RaveParseDirectories(const char* pdirs, std::vector<std::string>& vdirs) RAVE_DEPRECATED;

inline bool RaveParseDirectories(const char* pdirs, std::vector<std::string>& vdirs)
{
    vdirs.resize(0);
    if( !pdirs ) {
        return false;
    }
    // search for all directories separated by ':'
    std::string tmp = pdirs;
    std::string::size_type pos = 0, newpos=0;
    while( pos < tmp.size() ) {
#ifdef _WIN32
        newpos = tmp.find(';', pos);
#else
        newpos = tmp.find(':', pos);
#endif
        std::string::size_type n = newpos == std::string::npos ? tmp.size()-pos : (newpos-pos);
        vdirs.push_back(tmp.substr(pos, n));
        if( newpos == std::string::npos ) {
            break;
        }
        pos = newpos+1;
    }
    return true;
}

/// \brief Create the interfaces, see \ref CreateInterfaceValidated.
/// \ingroup plugin_exports
typedef InterfaceBasePtr (*PluginExportFn_OpenRAVECreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, const char* envhash, EnvironmentBasePtr env);

/// \brief Called to fill information about the plugin, see \ref GetPluginAttributesValidated.
/// \ingroup plugin_exports
typedef bool (*PluginExportFn_OpenRAVEGetPluginAttributes)(PLUGININFO* pinfo, int size, const char* infohash);

/// \brief Called before plugin is unloaded from openrave. See \ref DestroyPlugin.
/// \ingroup plugin_exports
typedef void (*PluginExportFn_DestroyPlugin)();

/// \brief Called when OpenRAVE global runtime is finished initializing. See \ref OnRaveInitialized
/// \ingroup plugin_exports
typedef void (*PluginExportFn_OnRaveInitialized)();

/// \brief Called when OpenRAVE global runtime is about to be destroyed. See \ref OnRavePreDestroy.
/// \ingroup plugin_exports
typedef void (*PluginExportFn_OnRavePreDestroy)();

/// \deprecated (12/01/01)
typedef InterfaceBasePtr (*PluginExportFn_CreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr env) RAVE_DEPRECATED;

/// \deprecated (12/01/01)
typedef bool (*PluginExportFn_GetPluginAttributes)(PLUGININFO* pinfo, int size) RAVE_DEPRECATED;

// define inline functions
const std::string& IkParameterization::GetName() const
{
    std::map<IkParameterizationType,std::string>::const_iterator it = RaveGetIkParameterizationMap().find(_type);
    if( it != RaveGetIkParameterizationMap().end() ) {
        return it->second;
    }
    throw openrave_exception(str(boost::format("IkParameterization iktype 0x%x not supported")%_type));
}


} // end namespace OpenRAVE


BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MAJOR>=0&&OPENRAVE_VERSION_MAJOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MINOR>=0&&OPENRAVE_VERSION_MINOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_PATCH>=0&&OPENRAVE_VERSION_PATCH<=255);

// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::InterfaceType)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::UserData)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ModuleBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ControllerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase::PlannerParameters)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase::SensorData)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorSystemBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SimpleSensorSystem)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SimpleSensorSystem::XMLData)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ViewerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SpaceSamplerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::GraphHandle)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkParameterization)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ConfigurationSpecification)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ConfigurationSpecification::Group)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ConfigurationSpecification::Reader)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveVector, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransform, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransformMatrix, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::OrientedBox, 1)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint::MIMIC)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::GEOMPROPERTIES)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TriMesh)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::KinBodyStateSaver)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::BodyState)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::ManageData)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::Manipulator)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::AttachedSensor)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::GRABBED)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::RobotStateSaver)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase::TPOINT)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase::TSEGMENT)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PLUGININFO)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::Readable)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::InterfaceBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::BaseXMLReader)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::BaseXMLWriter)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::EnvironmentBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::EnvironmentBase::BODYSTATE)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RAY)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::AABB)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::OBB)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TRIANGLE)
#endif


#endif
