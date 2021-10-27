#ifndef OPENRAVE_PLUGINDEFS_H
#define OPENRAVE_PLUGINDEFS_H

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
//#include <boost/typeof/std/list.hpp>
//#include <boost/typeof/std/map.hpp>
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

#include <boost/config.hpp>

#ifdef BOOST_NO_CXX11_DECLTYPE

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#else

#define FOREACH(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); )

#endif

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#include <stdint.h>
#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/range/concepts.hpp>
#include <boost/range/detail/any_iterator.hpp>
#include <boost/unordered_map.hpp>

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_oderave", msgid)

using namespace std;

using OpenRAVE::BaseXMLReader;
using OpenRAVE::BaseXMLReaderPtr;
using OpenRAVE::EnvironmentBase;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBody;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::KinBodyWeakPtr;
using OpenRAVE::KinBodyConstPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::RobotBaseConstPtr;
using OpenRAVE::RaveTransform;
using OpenRAVE::RaveTransformMatrix;
using OpenRAVE::RaveVector;
using OpenRAVE::Vector;
using OpenRAVE::Transform;
using OpenRAVE::TransformConstPtr;
using OpenRAVE::TransformMatrix;
using OpenRAVE::CollisionReport;
using OpenRAVE::CollisionReportPtr;
using OpenRAVE::RAY;
using OpenRAVE::InterfaceType;
using OpenRAVE::InterfaceBase;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::InterfaceBaseConstPtr;
using OpenRAVE::PLUGININFO;
using OpenRAVE::openrave_exception;
using OpenRAVE::EnvironmentMutex;
using OpenRAVE::RaveFabs;
using OpenRAVE::ControllerBase;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::TrajectoryBaseConstPtr;
using OpenRAVE::ControllerBase;
using OpenRAVE::AttributesList;


#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/shape/geometric_shapes.h>

#endif
