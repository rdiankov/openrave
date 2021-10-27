// -*- coding: utf-8 -*-
// Copyright (C) 2012-2014 Gustavo Puche, Rosen Diankov, OpenGrasp Team
//
// OpenRAVE Qt/OpenSceneGraph Viewer is licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPENRAVE_QTOSG_H_
#define OPENRAVE_QTOSG_H_

#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_DEPRECATE

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

/// functions that allow plugins to program for the RAVE simulator
#include <cstdio>
#include <cmath>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
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

#include <boost/bind.hpp>
#include <boost/assert.hpp>
#include <boost/thread/condition.hpp>
#include <boost/version.hpp>

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <QtWidgets/QTreeWidget>

using namespace OpenRAVE;
using namespace std;

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

// OSG includes
#include <osg/MatrixTransform>
#include <osg/BlendColor>
#include <osg/Switch>

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/Camera>
#include <osg/Point>
#include <osg/LineWidth>
#include <QtGui>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osgFX/Cartoon>

#include <osgManipulator/Dragger>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <osgText/Text>

#define CALLBACK_EVENT QEvent::Type(QEvent::User+101) // also see qtcoin.h

namespace qtosgrave {

template<class T>
inline T ClampOnRange(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

class MyCallbackEvent : public QEvent
{
public:
    MyCallbackEvent(const boost::function<void()>& fn) : QEvent(CALLBACK_EVENT), _fn(fn) {
    }
    virtual ~MyCallbackEvent() {
    }
    boost::function<void()> _fn;
};

// common used typedefs
typedef osg::ref_ptr<osg::Node> OSGNodePtr;
typedef osg::ref_ptr<osg::Group> OSGGroupPtr;
typedef osg::ref_ptr<osg::Transform> OSGTransformPtr;
typedef osg::ref_ptr<osg::MatrixTransform> OSGMatrixTransformPtr;
typedef osg::ref_ptr<osg::Switch> OSGSwitchPtr;

inline RaveTransform<float> GetRaveTransformFromMatrix(const osg::Matrix &m)
{
    // Debug
    osg::Quat q;
    osg::Vec3d v;
    osg::Vec4d qua;

    // Review Quaternion assign
    RaveTransform<float> t;

    q = m.getRotate();

    // Normalize quat prevents Core crash
    qua = q.asVec4();
    qua.normalize();
    q.set(qua);

    t.rot = Vector(q[3], q[0], q[1], q[2]);
    v = m.getTrans();
    t.trans = Vector(v[0], v[1], v[2]);
    return t;
}

/// sets the transform of a Coin3D SoTransform object
inline osg::Matrix GetMatrixFromRaveTransform(const RaveTransform<float> &t)
{
    osg::Matrix m;
    m.makeRotate(osg::Quat(t.rot.y, t.rot.z, t.rot.w, t.rot.x));
    m.setTrans(t.trans.x, t.trans.y, t.trans.z);
    return m;
}

/// returns the Transform from a Coin3D SoTransform object
inline RaveTransform<float> GetRaveTransform(osg::MatrixTransform& trans)
{
    return GetRaveTransformFromMatrix(trans.getMatrix());
}

/// sets the transform of a Coin3D SoTransform object
inline void SetMatrixTransform(osg::MatrixTransform& trans, const RaveTransform<float> &t)
{
    trans.setMatrix(GetMatrixFromRaveTransform(t));
}

inline osg::Vec3f GetOSGVec3(const RaveVector<float>&v) {
    return osg::Vec3f(v.x, v.y, v.z);
}

// Derive a class from NodeVisitor to find a node with a specific name.
class FindNode : public osg::NodeVisitor
{
public:
    // Traverse all children.
    FindNode( OSGNodePtr node ) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ), _nodeToFind( node ) {
        _found = false;
    }
    // This method gets called for every node in the scene
    //   graph. Check each node to see if its name matches
    //   our target. If so, save the node's address.
    virtual void apply( osg::Node& node )
    {
        if (_nodeToFind.get() == &node) {
            _found = true;
        }
        else {
            // Keep traversing the rest of the scene graph.
            traverse( node );
        }
    }
    bool IsFound() const {
        return _found;
    }
protected:
    OSGNodePtr _nodeToFind;
    bool _found;
};

inline boost::shared_ptr<EnvironmentMutex::scoped_try_lock> LockEnvironmentWithTimeout(EnvironmentBasePtr penv, uint64_t timeout)
{
    // try to acquire the lock
#if BOOST_VERSION >= 103500
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(penv->GetMutex(),boost::defer_lock_t()));
#else
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(penv->GetMutex(),false));
#endif
    uint64_t basetime = utils::GetMicroTime();
    while(utils::GetMicroTime()-basetime<timeout ) {
        if( lockenv->try_lock() ) {
            break;
        }
    }

    if( !*lockenv ) {
        lockenv.reset();
    }
    return lockenv;
}

} // end namespace qtosgrave


#endif /* QTOSG_H_ */
