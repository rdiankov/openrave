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
#include "osgviewerQtContext.h"

namespace qtosgrave {

void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;

    osg::ref_ptr<osg::Node> node;
    std::string gdlist="";
    float x = ea.getX();
    float y = ea.getY();
#if 0
    osg::ref_ptr< osgUtil::LineSegmentIntersector > picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
    osgUtil::IntersectionVisitor iv(picker.get());
    view->getCamera()->accept(iv);
    if (picker->containsIntersections())
    {
        intersections = picker->getIntersections();
#else
    if (view->computeIntersections(x,y,intersections))
    {
#endif
        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
            hitr != intersections.end();
            ++hitr)
        {
            if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
            {
                // the geodes are identified by name.
                gdlist  = hitr->nodePath.back()->getName();
                node = hitr->drawable->getParent(0);
                break;
            }
            else if (hitr->drawable.valid())
            {
                gdlist  = hitr->drawable->className();


            }
        }
    }

    //  If selection is activated
    if (_select)
    {
        _viewer->select(node);
    }
}

void PickHandler::doubleClick()
{
    if (_viewer->isSimpleView)
    {
        _viewer->setMultipleView();
    }
    else
    {
        _viewer->setSimpleView();
    }
}

//  Active joint selection
void PickHandler::activeSelect(bool active)
{
    _select = active;
}

}
