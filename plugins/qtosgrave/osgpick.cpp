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
#include "osgpick.h"

namespace qtosgrave {

OSGPickHandler::OSGPickHandler(const HandleRayPickFn& handleRayPickFn) : _handleRayPickFn(handleRayPickFn), _bDoPickCallOnButtonRelease(false)
{
}

OSGPickHandler::~OSGPickHandler()
{
}

bool OSGPickHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
    case osgGA::GUIEventAdapter::DOUBLECLICK:
    {
        //doubleClick();
        return false;
    }
    case osgGA::GUIEventAdapter::RELEASE:
    {
        if( (ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) && _bDoPickCallOnButtonRelease ) {
            _bDoPickCallOnButtonRelease = false;
            osg::ref_ptr<osgViewer::View> view(dynamic_cast<osgViewer::View*>(&aa));
            if (!!view) {
                _Pick(view, ea, 1);
            }
        }
        return false;
    }
    case osgGA::GUIEventAdapter::PUSH:
        if( ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) {
            _bDoPickCallOnButtonRelease = true;
        }
        else {
            _bDoPickCallOnButtonRelease = false;
        }
        return false;
    case osgGA::GUIEventAdapter::MOVE: {
        osg::ref_ptr<osgViewer::View> view(dynamic_cast<osgViewer::View*>(&aa));
        if (!!view) {
            _Pick(view, ea, 0);
        }
        return false;
    }
    case osgGA::GUIEventAdapter::DRAG:
        _bDoPickCallOnButtonRelease = false; // mouse moved, so cancel any pick calls
        return false;
        
    default:
        return false;
    }
}

void OSGPickHandler::_Pick(osg::ref_ptr<osgViewer::View> view, const osgGA::GUIEventAdapter& ea, int buttonPressed)
{
    try {
        float x = ea.getX();
        float y = ea.getY();
        osgUtil::LineSegmentIntersector::Intersections intersections;
        if (view->computeIntersections(x,y,intersections)) {
            for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin(); hitr != intersections.end(); ++hitr) {
                if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty())) {
                    _handleRayPickFn(*hitr, buttonPressed, ea.getModKeyMask());
                    return;
//                    // the geodes are identified by name.
//                    gdlist  = hitr->nodePath.back()->getName();
//                    node = hitr->drawable->getParent(0);
//
//                    break;

                }
                else if (hitr->drawable.valid()) {
                    //gdlist  = hitr->drawable->className();
                }
            }

            if( intersections.size() > 0 ) {
                // take the first intersection
                _handleRayPickFn(*intersections.begin(), buttonPressed, ea.getModKeyMask());
                return;
            }
        }

        // if still here, then no intersection
        _handleRayPickFn(osgUtil::LineSegmentIntersector::Intersection(), buttonPressed, ea.getModKeyMask());
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("exception in osg picker: %s", ex.what());
    }
}

}
