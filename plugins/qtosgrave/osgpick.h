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
#ifndef OPENRAVE_QTOSG_PICK_H_
#define OPENRAVE_QTOSG_PICK_H_

#include "qtosg.h"

#include <sstream>

const osg::Node::NodeMask OSG_IS_PICKABLE_MASK = 0x2;

namespace qtosgrave {

/// \brief class to handle events with a pick
class OSGPickHandler : public osgGA::GUIEventHandler
{
public:
    /// select(node, modkeymask) where node is the ray-picked node, and modkeymask is the modifier key mask currently pressed
    typedef boost::function<void (const osgUtil::LineSegmentIntersector::Intersection&, int, int)> HandleRayPickFn;
    typedef boost::function<void ()> DragFn;

    OSGPickHandler(const HandleRayPickFn& handleRayPickFn=HandleRayPickFn(), const DragFn& dragfn=DragFn());
    virtual ~OSGPickHandler();

    /// \brief override from base class
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    /// \brief Active joint selection
    //void ActivateSelection(bool active);
    //bool IsSelectionActive() const { return _select; }

protected:
    virtual void _Pick(osg::ref_ptr<osgViewer::View> view, const osgGA::GUIEventAdapter& ea, int buttonPressed);
    HandleRayPickFn _handleRayPickFn;
    DragFn _dragfn;
    //bool _select; ///< if true, then will call the _selectLinkFn with the raypicked node
    bool _bDoPickCallOnButtonRelease; ///< if true, then on button release can call _Pick
};

}

#endif /* OSGPICK_H_ */
