// -*- coding: utf-8 -*-
// Copyright (C) 2012 Gustavo Puche, Rosen Diankov, OpenGrasp Team
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
#include <assert.h>
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
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>
#include <QtGui/QTreeWidget>

using namespace OpenRAVE;
using namespace std;

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

// OSG includes
#include <osg/MatrixTransform>
#include <osg/BlendColor>
#include <osg/Switch>

namespace qtosgrave {

/// returns the Transform from a Coin3D SoTransform object
inline RaveTransform<float> GetRaveTransform(const osg::MatrixTransform* ptrans)
{
    //    Debug
//  RAVELOG_INFO("\t\tGetRaveTransform(ptrans)\n");

    osg::Matrix m;
    osg::Quat q;
    osg::Vec3d v;
    osg::Vec4d qua;

    //  Review Quaternion assign
    RaveTransform<float> t;
    assert( ptrans != NULL );

    m = ptrans->getMatrix();
    q = m.getRotate();

    //  Normalize quat prevents Core crash
    qua = q.asVec4();
    qua.normalize();
    q.set(qua);

    t.rot = Vector(q[3], q[0], q[1], q[2]);
//  t.rot = Vector(q[0], q[1], q[2],q[3]);
    v = m.getTrans();
    t.trans = Vector(v[0], v[1], v[2]);
    return t;
}


/// sets the transform of a Coin3D SoTransform object
inline void SetMatrixTransform(osg::MatrixTransform* ptrans, const RaveTransform<float>& t)
{
    //    Debug
//  RAVELOG_INFO("\t\tSetMatrixTransform(ptrans,t)\n");

    osg::Matrix mR,mT;
    mR.makeRotate(osg::Quat(t.rot.y, t.rot.z, t.rot.w,t.rot.x));
    mT.makeTranslate(t.trans.x, t.trans.y, t.trans.z);

    ptrans->preMult(mT);
    ptrans->preMult(mR);
}

}

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/Camera>
#include <QtGui>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#include <osgManipulator/Dragger>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <osgText/Text>

namespace qtosgrave {

// Derive a class from NodeVisitor to find a node with a
//   specific name.
class FindNode : public osg::NodeVisitor
{
public:
    FindNode( osg::Node* node )
        : osg::NodeVisitor( // Traverse all children.
            osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _nodeToFind( node ),_node(NULL) {
    }
    // This method gets called for every node in the scene
    //   graph. Check each node to see if its name matches
    //   our target. If so, save the node's address.
    virtual void apply( osg::Node* node )
    {
        if (_nodeToFind == node)
            _node = node;
        // Keep traversing the rest of the scene graph.
        traverse( *node );
    }
    osg::Node* getNode() {
        return _node;
    }
protected:
    osg::Node*  _nodeToFind;
    osg::Node*  _node;
};

class QtOSGViewer;
typedef boost::shared_ptr<QtOSGViewer> QtOSGViewerPtr;
typedef boost::shared_ptr<QtOSGViewer const> QtOSGViewerConstPtr;

}

#include "Item.h"
#include "qtosgviewer.h"

#endif /* QTOSG_H_ */
