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
#ifndef OPENRAVE_QTOSG_OBJECTTREE_H_
#define OPENRAVE_QTOSG_OBJECTTREE_H_

#include <QTreeView>

#include <osgGA/GUIActionAdapter>
#include <osgGA/EventQueue>
#include <osgGA/EventVisitor>
#include <osgUtil/SceneView>
#include <osg/Timer>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osg/observer_ptr>
#include <osg/Object>
#include <osg/Referenced>
#include <osg/ref_ptr>


namespace qtosgrave {

class ObjectTree : public QTreeView {
public:
    ObjectTree();
    virtual ~ObjectTree();

    void setRoot(osg::Object *obj);
protected:
    void on_selection_changed();
    void refresh();

private:
    class ModelColumns {
public:
        ModelColumns(){
        }
    };

    class OsgTreeModel {
public:
        OsgTreeModel();
        static OsgTreeModel *create();

        ModelColumns _columns;

    };
    OsgTreeModel *model;
};

}

#endif /* OBJECTTREE_H_ */
