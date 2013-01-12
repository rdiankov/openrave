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
#ifndef OPENRAVE_QTOSG_VIEWERCONTEXT_H
#define OPENRAVE_QTOSG_VIEWERCONTEXT_H

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/ShadeModel>
#include <osgDB/ReadFile>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <iostream>
//#include "GraphicsWindowQt.h"
#include <osgQt/GraphicsWindowQt>
#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <osg/PositionAttitudeTransform>

#include "osgpick.h"
#include "qtosg.h"

namespace qtosgrave {

using namespace OpenRAVE;
using namespace osgQt;

//  Class of the window viewer
class ViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
public:

    int uniqueLightNumber;
    int LIGHTS;

    osg::PositionAttitudeTransform **lightTransform;
    osg::StateSet *lightStateSet;

    bool isSimpleView;
//    bool doubleClickPressed;

    QList<QWidget*> widgetsList;
    osgViewer::View* view1;
    osgViewer::View* view2;
    osgViewer::View* view3;
    osgViewer::View* view4;

    QGridLayout* grid;
    ////////////////////////////////////////////////////////////////////////////
    /// Constructor of ViewerWidget
    ////////////////////////////////////////////////////////////////////////////
    ViewerWidget() : QWidget()
    {
        //  Sets light on
        _light_on = true;

        LIGHTS = 5;
        uniqueLightNumber = 0;

        //  Lights
        lightTransform = new osg::PositionAttitudeTransform*[LIGHTS];

        startup();

        QWidget* widget;
        isSimpleView = false;
//      doubleClickPressed = false;

        //  Reset acutal kinbody
        _actualKinbody = "";

        //  Initialize picker handler
        _picker = new PickHandler(this);

        view1 = new osgViewer::View;
        view2 = new osgViewer::View;
        view3 = new osgViewer::View;
        view4 = new osgViewer::View;

        //  Improve FPS to 60 per viewer
        setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);
//      setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

        widget = addViewWidget( createCamera(0,0,100,100), view1 );
        widgetsList << widget;
        widget = addViewWidget( createCamera(0,0,100,100), view2 );
        widgetsList << widget;
        widget = addViewWidget( createCamera(0,0,100,100), view3 );
        widgetsList << widget;
        widget = addViewWidget( createCamera(0,0,100,100), view4 );
        widgetsList << widget;

        grid = new QGridLayout;
        grid->addWidget( widgetsList[0], 0, 0 );
        grid->addWidget( widgetsList[1], 0, 1 );
        grid->addWidget( widgetsList[2], 1, 0 );
        grid->addWidget( widgetsList[3], 1, 1 );
        setLayout( grid );

        //  Sets pickhandler
        view1->addEventHandler(_picker);

        connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
        _timer.start( 10 );
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Sets environment of OpenRAVE
    ////////////////////////////////////////////////////////////////////////////
    void setEnv(EnvironmentBasePtr penv)
    {
        _penv = penv;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Draws bounding box
    ////////////////////////////////////////////////////////////////////////////
    void drawBoundingBox(bool pressed)
    {
        //  Gets camera transform
//      storeMatrixTransform();

        //  If boundingbox button is pressed
        if (pressed)
        {
            //  Debug
//        propagate();

            _draggerName = "Box";

            selectRobot(_actualKinbody);

            //view1->setSceneData(addDraggerToObject(_scene_lights.get(), "Box"));
//        view1->setSceneData(addDraggerToObject(view1->getSceneData()->asGroup()->getChild(0), "Box"));
        }

        //  Set camera transform
//      loadMatrixTransform();
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Clear dragger from the viewer
    ////////////////////////////////////////////////////////////////////////////
    void clearDragger()
    {
        if (!!_dragger)
        {
            _dragger->getParent(0)->removeChild(_dragger);
            _dragger.release();
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Active selection
    ////////////////////////////////////////////////////////////////////////////
    void select(bool active)
    {
        _picker->activeSelect(active);
    }

    ////////////////////////////////////////////////////////////////////////////
    ///  Draws trackball sphere
    ////////////////////////////////////////////////////////////////////////////
    void drawTrackball(bool pressed)
    {
        //  Gets camera transform
//      storeMatrixTransform();

        //  If boundingbox button is pressed
        if (pressed)
        {
            //  Debug
//        propagate();

            _draggerName = "TrackballDragger";

            selectRobot(_actualKinbody);

//        view1->setSceneData(addDraggerToObject(view1->getSceneData()->asGroup()->getChild(0),
//                                                 "TrackballDragger"));
        }

        //  Set camera transform
//      loadMatrixTransform();
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Draws translate axis
    ////////////////////////////////////////////////////////////////////////////
    void drawAxes(bool pressed)
    {
        //  Debug
//      qWarning("[drawAxes]");

//      osg::Node* node;

        //  Gets camera transform
//      storeMatrixTransform();

        if (pressed)
        {
            //  Debug
//        propagate();

            _draggerName = "TranslateAxisDragger";

            selectRobot(_actualKinbody);

//        node = view1->getSceneData()->asGroup()->getChild(0);

//        view1->setSceneData(addDraggerToObject(node,"TranslateAxisDragger"));

            //  Debug
//        showSceneGraph("",view1->getSceneData());
        }

        //  Set camera transform
//      loadMatrixTransform();
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Select robot or kinbody from screen
    ////////////////////////////////////////////////////////////////////////////
    void selectRobot(std::string name)
    {
        //  Gets camera transform
//      storeMatrixTransform();

        osg::Node* node;

        node = view1->getSceneData();

        node = findNamedNode(name,node);

        if (!!node)
        {
            propagate();

            //  Sets robot selected
            _actualKinbody = name;

            //  Debug
            qWarning("Node name %s",node->getName().c_str());

            addDraggerToObject(node,_draggerName);
        }
        //  Set camera transform
//      loadMatrixTransform();
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Sets scene data node in all viewers
    ////////////////////////////////////////////////////////////////////////////
    void setSceneData(osg::Node* scene)
    {
        //  Normalize object normals
        scene->getOrCreateStateSet()->setMode(GL_NORMALIZE,osg::StateAttribute::ON);

        _scene_lights->removeChild(_scene_lights_data.get());

        _scene_lights_data = scene->asGroup();

        _scene_lights->addChild(scene);

        if (_light_on)
        {
            view1->setSceneData(_scene_lights);
            view2->setSceneData(_scene_lights);
            view3->setSceneData(_scene_lights);
            view4->setSceneData(_scene_lights);
        }
        else
        {
            view1->setSceneData(_scene_lights_data);
            view2->setSceneData(_scene_lights_data);
            view3->setSceneData(_scene_lights_data);
            view4->setSceneData(_scene_lights_data);
        }

        //  Debug
//      showSceneGraph("",scene);
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Reset viewer to original position
    ////////////////////////////////////////////////////////////////////////////
    void home()
    {
        view1->home();
        view2->home();
        view3->home();
        view4->home();
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Reset viewer to original position
    ////////////////////////////////////////////////////////////////////////////
    void setHome()
    {
        //  If _scene_lights != NULL
        if (_scene_lights.valid())
        {
            const osg::BoundingSphere& bs = _scene_lights->getBound();

            view1->getCameraManipulator()->setHomePosition(osg::Vec3d(-4.0*bs.radius(),4.0*bs.radius(),0.0),bs.center(),osg::Vec3d(0.0,0.0,1.0));
            view2->getCameraManipulator()->setHomePosition(osg::Vec3d(-4.0*bs.radius(),0.0,0.0),bs.center(),osg::Vec3d(0.0,0.0,1.0));
            view3->getCameraManipulator()->setHomePosition(osg::Vec3d(0.0,4.0*bs.radius(),0.0),bs.center(),osg::Vec3d(0.0,0.0,1.0));
            view4->getCameraManipulator()->setHomePosition(osg::Vec3d(0.0,0.0,4.0*bs.radius()),bs.center(),osg::Vec3d(0.0,0.0,1.0));

            view1->home();
            view2->home();
            view3->home();
            view4->home();
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    //  Lighting
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    osg::Material *createSimpleMaterial(osg::Vec4 color)
    {
        osg::Material *material = new osg::Material();
        material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.0, 0.0, 0.0, 1.0));
        material->setEmission(osg::Material::FRONT, color);

        return material;
    }

    osg::Light* createLight(osg::Vec4 color)
    {
        osg::Light *light = new osg::Light();
        // each light must have a unique number
        light->setLightNum(uniqueLightNumber++);
        // we set the light's position via a PositionAttitudeTransform object
        light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
        light->setDiffuse(color);
        light->setSpecular(osg::Vec4(0.8, 0.8, 0.8, 1.0));
        light->setAmbient( osg::Vec4(0.2, 0.2, 0.2, 1.0));
        light->setConstantAttenuation(1);
        light->setQuadraticAttenuation(0.1);
        light->setSpotCutoff(70.0);

        return light;
    }

    osg::Light* createAmbientLight(osg::Vec4 color)
    {
        osg::Light *light = new osg::Light();
        // each light must have a unique number
        light->setLightNum(uniqueLightNumber++);
        // we set the light's position via a PositionAttitudeTransform object
        light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
        light->setDiffuse(color);
        light->setSpecular(osg::Vec4(0, 0, 0, 1.0));
        light->setAmbient( osg::Vec4(0.5, 0.5, 0.5, 1.0));

        //  Attenuation
        light->setConstantAttenuation(1);
        light->setQuadraticAttenuation(0.2);

        return light;
    }


    ////////////////////////////////////////////////////////////////////////////
    //  Initialize lighting
    ////////////////////////////////////////////////////////////////////////////
    void startup()
    {
        // we need the scene's state set to enable the light for the entire scene
        _scene_lights = new osg::Group();
        lightStateSet = _scene_lights->getOrCreateStateSet();

        // Create 3 Lights
        osg::Vec4 lightColors[] = { osg::Vec4(1.0, 1.0, 1.0, 1.0),
                                    osg::Vec4(1.0, 1.0, 1.0, 1.0), osg::Vec4(1.0, 1.0, 1.0, 1.0),
                                    osg::Vec4(1.0, 1.0, 1.0, 1.0), osg::Vec4(1.0, 1.0, 1.0, 1.0)};

        osg::Vec3 lightPosition[] = { osg::Vec3(0, 0, 3.5),
                                      osg::Vec3(2, -2.5, 2.5), osg::Vec3(-2, -2.5, 2.5),
                                      osg::Vec3(2, 2.5, 2.5),osg::Vec3(-2, 2.5, 2.5)};
        osg::Vec3 lightDirection[] = {osg::Vec3(0.0, 0.0, -1.0),
                                      osg::Vec3(0.0, 0.0, -1.0), osg::Vec3(0.0, 0.0, -1.0),
                                      osg::Vec3(0.0, 0.0, -1.0), osg::Vec3(0.0, 0.0, -1.0)};
        osg::Geode *lightMarker[LIGHTS];
        osg::LightSource *lightSource[LIGHTS];

        for (int i = 0; i < LIGHTS; i++)
        {
            lightMarker[i] = new osg::Geode();
            lightMarker[i]->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
            lightMarker[i]->getOrCreateStateSet()->setAttribute(createSimpleMaterial(lightColors[i]));

            lightSource[i] = new osg::LightSource();

            qWarning("Unique Light Number %d\n",uniqueLightNumber);

            if (i == 0)
            {
                lightSource[i]->setLight(createAmbientLight(lightColors[i]));
            }
            else
            {
                lightSource[i]->setLight(createLight(lightColors[i]));

            }

            lightSource[i]->getLight()->setDirection(lightDirection[i]);
            lightSource[i]->setLocalStateSetModes(osg::StateAttribute::ON);
            lightSource[i]->setStateSetModes(*lightStateSet, osg::StateAttribute::ON);

            lightTransform[i] = new osg::PositionAttitudeTransform();
            lightTransform[i]->addChild(lightSource[i]);
            lightTransform[i]->addChild(lightMarker[i]);
            lightTransform[i]->setPosition(lightPosition[i]);

            lightTransform[i]->setScale(osg::Vec3(0.1,0.1,0.1));

            _scene_lights->addChild(lightTransform[i]);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Light button
    ////////////////////////////////////////////////////////////////////////////
    void setLight(bool enabled)
    {
        if (enabled)
        {
            _light_on = true;
        }
        else
        {
            _light_on = false;
        }
    }

    //  Cull face
    void setFacesMode(bool enabled)
    {
        osg::StateSet* stateset = view1->getSceneData()->getOrCreateStateSet();
        if (enabled)
        {
            stateset->setAttribute(new osg::CullFace(osg::CullFace::FRONT));
            stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
            stateset->setAttributeAndModes(new osg::CullFace, osg::StateAttribute::OFF);
        }
        else
        {
            stateset->setAttribute(new osg::CullFace(osg::CullFace::FRONT));
            stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
            stateset->setAttributeAndModes(new osg::CullFace, osg::StateAttribute::ON);
        }



        view1->getSceneData()->setStateSet(stateset);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Sets poligon mode (SMOOTH, FLAT or WIRED)
    ////////////////////////////////////////////////////////////////////////////
    void setPolygonMode(int mode)
    {
        osg::PolygonMode *poly = new osg::PolygonMode();
        osg::ShadeModel *sm= new osg::ShadeModel();
        switch (mode)
        {
        case 0:
            poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::FILL);
            sm->setMode(osg::ShadeModel::SMOOTH);
            view1->getSceneData()->getOrCreateStateSet()->setAttribute(poly);
            view1->getSceneData()->getOrCreateStateSet()->setAttribute(sm);

            break;
        case 1:
            poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::FILL);
            sm->setMode(osg::ShadeModel::FLAT);
            view1->getSceneData()->getOrCreateStateSet()->setAttributeAndModes(poly,osg::StateAttribute::ON);
            view1->getSceneData()->getOrCreateStateSet()->setAttribute(sm);

            break;
        case 2:
            poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
            sm->setMode(osg::ShadeModel::SMOOTH);
            view1->getSceneData()->getOrCreateStateSet()->setAttribute(poly);
            view1->getSceneData()->getOrCreateStateSet()->setAttribute(sm);
            break;
        }

    }

    ////////////////////////////////////////////////////////////////////////////
    /// Set wire view to a node
    ////////////////////////////////////////////////////////////////////////////
    void setWire(osg::Node* node)
    {
        osg::PolygonMode *poly = new osg::PolygonMode();
        osg::ShadeModel *sm= new osg::ShadeModel();

        poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
        sm->setMode(osg::ShadeModel::SMOOTH);

        node->getOrCreateStateSet()->setAttribute(poly);
        node->getOrCreateStateSet()->setAttribute(sm);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Create a viewer widget
    ////////////////////////////////////////////////////////////////////////////
    QWidget* addViewWidget( osg::Camera* camera, osgViewer::View* view )
    {
        view->setCamera( camera );
        addView( view );
        view->addEventHandler( new osgViewer::StatsHandler );
        view->setCameraManipulator( new osgGA::TrackballManipulator );

        GraphicsWindowQt* gw = dynamic_cast<GraphicsWindowQt*>( camera->getGraphicsContext() );
        return gw ? gw->getGraphWidget() : NULL;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Create Open GL Context
    ////////////////////////////////////////////////////////////////////////////
    osg::Camera* createCamera( int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false )
    {
        osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();

        //  Anti aliasing
        ds->setNumMultiSamples(4);

        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->windowName = name;
        traits->windowDecoration = windowDecoration;
        traits->x = x;
        traits->y = y;
        traits->width = w;
        traits->height = h;
        traits->doubleBuffer = true;
        traits->alpha = ds->getMinimumNumAlphaBits();
        traits->stencil = ds->getMinimumNumStencilBits();
        traits->sampleBuffers = ds->getMultiSamples();
        traits->samples = ds->getNumMultiSamples();

        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext( new GraphicsWindowQt(traits.get()) );

        camera->setClearColor( osg::Vec4(0.66, 0.75, 0.85, 1.0) );
        camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
        camera->setProjectionMatrixAsPerspective(
            30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
        return camera.release();
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Find an OSG Node with the name given
    ////////////////////////////////////////////////////////////////////////////
    osg::Node* findNamedNode(const std::string& searchName,
                             osg::Node* currNode)
    {
        osg::ref_ptr<osg::Group> currGroup;
        osg::ref_ptr<osg::Node> foundNode;

        // check to see if we have a valid (non-NULL) node.
        // if we do have a null node, return NULL.
        if ( !currNode)
        {
            return NULL;
        }

        // We have a valid node, check to see if this is the node we
        // are looking for. If so, return the current node.
        if (currNode->getName() == searchName)
        {
            return currNode;
        }

        // We have a valid node, but not the one we are looking for.
        // Check to see if it has children (non-leaf node). If the node
        // has children, check each of the child nodes by recursive call.
        // If one of the recursive calls returns a non-null value we have
        // found the correct node, so return this node.
        // If we check all of the children and have not found the node,
        // return NULL
        currGroup = currNode->asGroup(); // returns NULL if not a group.
        if ( currGroup )
        {
            for (unsigned int i = 0; i < currGroup->getNumChildren(); i++)
            {
                foundNode = findNamedNode(searchName, currGroup->getChild(i));
                if (foundNode)
                    return foundNode.get();                                               // found a match!
            }
            return NULL; // We have checked each child node - no match found.
        }
        else
        {
            return NULL; // leaf node, no match
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Print nodes of scenegraph
    ////////////////////////////////////////////////////////////////////////////
    void showSceneGraph(const std::string& currLevel,osg::Node* currNode)
    {
        std::string level;
        osg::ref_ptr<osg::Group> currGroup;

        level = currLevel;

        // check to see if we have a valid (non-NULL) node.
        // if we do have a null node, return NULL.
        if ( !!currNode)
        {
            qWarning("|%sNode class:%s (%s)",currLevel.c_str(),currNode->className(),currNode->getName().c_str());

            level = level + "-";

            currGroup = currNode->asGroup(); // returns NULL if not a group.
            if ( currGroup )
            {
                for (unsigned int i = 0; i < currGroup->getNumChildren(); i++)
                {
                    showSceneGraph(level,currGroup->getChild(i));
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Get link children and store list in global variable _linkChildren
    ////////////////////////////////////////////////////////////////////////////
    void getLinkChildren( std::string & robotName, KinBody::LinkPtr link,
                          std::vector<KinBody::LinkPtr> vlinks)
    {
        osg::Node* robot;

        robot = findNamedNode(robotName,view1->getSceneData());

        osg::Node* transform;

        transform = findNamedNode("tg-"+link->GetName(),robot);

        _linkChildren.push_back(transform->asTransform()->asMatrixTransform());

        FOREACH(itlink,vlinks)
        {
            if ((*itlink)->IsParentLink(link))
            {
                getLinkChildren(robotName,(*itlink),vlinks);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Gets transform matrix of a given link
    ////////////////////////////////////////////////////////////////////////////
    osg::MatrixTransform* getLinkTransform(std::string& robotName,
                                           KinBody::LinkPtr link)
    {
        osg::Node* robot;
        osg::Node* node;
        osg::MatrixTransform* transform;

        robot = findNamedNode(robotName,view1->getSceneData());
        node = findNamedNode("tg-"+link->GetName(),robot);

        if (!!node)
        {
            transform = node->asTransform()->asMatrixTransform();
        }

        return transform;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Find node of Robot for the link picked
    ////////////////////////////////////////////////////////////////////////////
    osg::Node* findRobot(osg::Node* node)
    {
        if (!node)
        {
            //  Error robot not found
            qWarning("\n[findRobot] Robot not found!\n");
            return NULL;
        }

        if (string(node->className()) == "Switch" && node->getName().size() > 0)
        {
            //  Debug
//        qWarning("[findRobot] Robot found");

            //  Robot found
            return node;
        }
        else
        {
            if (string(node->className()) == "Geode")
            {
                //  Search robot in parent node
                return node = findRobot(node->asGeode()->getParent(0));
            }
            else
            {
                //  Search robot in parent node
                return node = findRobot(node->asGroup()->getParent(0));
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Select the link picked
    ////////////////////////////////////////////////////////////////////////////
    void select(osg::Node* node)
    {
        if (!node)
        {
            return;
        }

        string linkName;
        string robotName;
        osg::ref_ptr<osg::Node>   scene;
        osg::ref_ptr<osg::Node>   node_found;
        osg::ref_ptr<osg::Node>   selected;
        KinBody::JointPtr joint;

        linkName = node->getName();

        osg::ref_ptr<osg::Node> robot;

        //  Finds robot node
        robot = findRobot(node);

        if (!!robot)
        {
            robotName = robot->getName();

            //  Gets camera transform
            storeMatrixTransform();

            //  Copy scene node for modify it
            scene = view1->getSceneData();

            // Find joint of a link name given
            joint = findJoint(robotName,linkName);

            node_found = findNamedNode("tg-"+linkName,robot);
            node_found = node_found->getParent(0);

//        node_found = findLinkParent(node);

            if (!!node_found && !!joint)
            {
                selected = addDraggerToObject(robotName,node_found,
                                              "RotateCylinderDragger",joint);

                view1->setSceneData(scene);

                //  Set camera transform
                loadMatrixTransform();
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Find link initial node. Group node
    ////////////////////////////////////////////////////////////////////////////
    osg::Node* findLinkParent(osg::Node* node)
    {
        //  There is an error?
        if (!node)
        {
            return NULL;
        }

//      if (string(node->className()) == string(node->getParent(0)->className())
//          &&  string(node->className()) == string("Group"))
        if (string(node->className()) == string(node->getParent(0)->className())
            &&  string(node->className()) == string("Group"))
        {
            //  Node found
            return node;
        }
        else
        {
            //  Continue searching for parent
            return findLinkParent(node->getParent(0));
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Propagate transform to link children
    ////////////////////////////////////////////////////////////////////////////
    void propagate()
    {
        osg::ref_ptr<osg::Node>   robot;
        osg::ref_ptr<osg::Group>  parent;
        osg::MatrixTransform*     tglobal;
        osg::Matrix mR,mL;

        if (_linkChildren.size() == 0)
            return;

        if (!_root)
        {
            //  Clears childrens of link
            _linkChildren.clear();

            return;
        }

        // Get robot
        robot = findRobot(_selected);

        //  Gets parent of _root
        parent = _root->getParent(0);

        //  Restore parent of selected link
        parent->addChild(_selected);

        //  Clears object selection
        _selection->removeChild(_selected);

        //  Remove _root from scene graph
        parent->removeChild(_root);

        //  Clears memory of _root selection object
        _root.release();

        //  For each children of dragger recalculate his global transform
        for (size_t i = 0; i < _linkChildren.size(); i++)
        {
            tglobal = _linkChildren[i];

            //  Debug
//        qWarning("link: %s",tglobal->getName().c_str());

//        //  If link  does Not need anchor to place correctly dragger
//        if (_needAnchor.find(robot->getName() + tglobal->getName()) == _needAnchor.end())
//        {
//          //  If link transform demands anchor to place dragger
//          if (tglobal->getMatrix().getTrans().x() == 0.0 && tglobal->getMatrix().getTrans().y() == 0.0
//              && tglobal->getMatrix().getTrans().z() == 0.0)
//          {
//            _needAnchor[robot->getName() + tglobal->getName()] = true;
//          }
//        }

            mL = _selection->getMatrix();

            //  Modify transform
            tglobal->setMatrix(tglobal->getMatrix() * _selection->getMatrix());
        }

        // Clears list of link children
        _linkChildren.clear();

        //  Debug
//      showSceneGraph("",view1->getSceneData());

        //  Updates core from Viewer joint values
        updateCoreFromViewer();
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Find joint into OpenRAVE core
    ////////////////////////////////////////////////////////////////////////////
    KinBody::JointPtr findJoint(std::string & robotName,std::string &linkName)
    {
        KinBody::JointPtr joint;
        KinBody::LinkPtr link;

        std::vector<RobotBasePtr> robots;

        //  Gets robots
        _penv->GetRobots(robots);

        for (size_t i = 0; i < robots.size(); i++)
        {
            if (robots[i]->GetName() == robotName)
            {
                link = robots[i]->GetLink(linkName);

                if (!!link)
                {
                    //  Propagate transformations to child nodes
                    propagate();

                    //  Gets all childs of the link
                    getLinkChildren(robotName,link,robots[i]->GetLinks());

                    FOREACH(itjoint,robots[i]->GetJoints())
                    {
                        if ((*itjoint)->GetSecondAttached()==link)
                        {
                            return *itjoint;
                        }
                    }
                }
            }
        }

        return joint;
    }

public slots:

    ////////////////////////////////////////////////////////////////////////////
    /// Sets simple view
    ////////////////////////////////////////////////////////////////////////////
    void setSimpleView()
    {
        if (!isSimpleView)
        {
            //  Activate simple view flag
            isSimpleView = true;

            int count = grid->count();

            for (int i = 0; i < count; i++)
            {
                widgetsList[i]->close();
            }

            widgetsList[0]->show();
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Sets multiple view
    ////////////////////////////////////////////////////////////////////////////
    void setMultipleView()
    {
        if (isSimpleView)
        {
            //  Deactivate simple view flag
            isSimpleView = false;

            int count = grid->count();

            for (int i = 0; i < count; i++)
            {
                widgetsList[i]->show();
            }
        }
    }

protected:

    ////////////////////////////////////////////////////////////////////////////
    /// Updates joint values from viewer to OpenRAVE core
    ////////////////////////////////////////////////////////////////////////////
    void updateCoreFromViewer()
    {
        //  Debug
//      qWarning("ViewerWidget::UpdateCoreFromViewer");

        std::vector<KinBody::BodyState> vecbodies;

        _penv->GetPublishedBodies(vecbodies);

        FOREACH(itbody,vecbodies)
        {
            BOOST_ASSERT( !!itbody->pbody );
            KinBodyPtr pbody = itbody->pbody; // try to use only as an id, don't call any methods!
            KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData("qtosg"));

            //  Update KinBodyItem
            if (!!pitem)
            {
//          qWarning("Update item '%s'",pitem->GetName().c_str());
                pitem->UpdateFromIv();
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Stores matrix transform
    ////////////////////////////////////////////////////////////////////////////
    void storeMatrixTransform()
    {
        //  Gets camera transform
//      _matrix1 = view1->getCameraManipulator()->getMatrix();
        _matrix1 = view1->getCamera()->getViewMatrix();
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Loads matrix transform
    ////////////////////////////////////////////////////////////////////////////
    void loadMatrixTransform()
    {
//      view1->getCameraManipulator()->setByMatrix(_matrix1);
        view1->getCamera()->setViewMatrix(_matrix1);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Create a dragger with a name given
    ////////////////////////////////////////////////////////////////////////////
    osgManipulator::Dragger* createDragger(const std::string& name)
    {
        osgManipulator::Dragger* dragger = 0;
        if ("TabPlaneDragger" == name)
        {
            osgManipulator::TabPlaneDragger* d = new osgManipulator::TabPlaneDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else if ("TabPlaneTrackballDragger" == name)
        {
            osgManipulator::TabPlaneTrackballDragger* d = new osgManipulator::TabPlaneTrackballDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else if ("TrackballDragger" == name)
        {
            osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else if ("Translate1DDragger" == name)
        {
            osgManipulator::Translate1DDragger* d = new osgManipulator::Translate1DDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else if ("Translate2DDragger" == name)
        {
            osgManipulator::Translate2DDragger* d = new osgManipulator::Translate2DDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else if ("TranslateAxisDragger" == name)
        {
            osgManipulator::TranslateAxisDragger* d = new osgManipulator::TranslateAxisDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else if ("RotateCylinderDragger" == name)
        {
            osgManipulator::RotateCylinderDragger* d = new osgManipulator::RotateCylinderDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        else
        {
            osgManipulator::TabBoxDragger* d = new osgManipulator::TabBoxDragger();
            d->setupDefaultGeometry();
            dragger = d;
        }
        return dragger;
    }

    ////////////////////////////////////////////////////////////////////////////
    //  Create a manipulator over an object pased
    ////////////////////////////////////////////////////////////////////////////
    osg::Node* addDraggerToObject(osg::Node* object, const std::string& name)
    {
        std::string robotName;
        KinBody::JointPtr j;
        return addDraggerToObject(robotName,object,name,j);
    }

    osg::Node* addDraggerToObject(std::string& robotName,osg::Node* object,
                                  const std::string& name,
                                  KinBody::JointPtr joint)
    {
//      object->getOrCreateStateSet()->setMode(GL_NORMALIZE,
//                                              osg::StateAttribute::ON);

        // Clears dragger
        clearDragger();

        //  New selection
        _selection = new osg::MatrixTransform;

        //  Create a new dragger
        _dragger = createDragger(name);

        _root = new osg::Group;
        _root->addChild(_dragger.get());
        _root->addChild(_selection);

        //  Store object selected in global variable _selected
        _selected = object;

        if (name == "RotateCylinderDragger" && !!joint)
        {
            //  Change view of dragger
            setWire(_dragger);

            for (size_t i = 0; i < object->getParents().size(); i++)
            {
                osg::ref_ptr<osg::Group>  parent;
                parent = object->getParent(i);
                parent->removeChild(object);
                parent->addChild(_root);
            }
        }
        else if (name != "RotateCylinderDragger")
        {
            for (size_t i = 0; i < object->getParents().size(); i++)
            {
                osg::ref_ptr<osg::Group>  parent;
                parent = object->getParent(i);
                parent->replaceChild(object,_root);
            }
        }

        //  Adds object to selection
        _selection->addChild(object);

        float scale = object->getBound().radius() * 1.3;

        if (name == "RotateCylinderDragger" && !!joint)
        {
            Vector axis;
            Vector anchor;
            Vector dragger_direction;
            Vector dragger_rotation;
            osg::Matrix matrix;


            axis    = joint->GetAxis();
            anchor  = joint->GetAnchor();

            //  Debug
            qWarning("Anchor: %f %f %f",anchor.x,anchor.y,anchor.z);

            dragger_direction = Vector(0,0,1);
            dragger_rotation = quatRotateDirection(dragger_direction,axis);

            matrix.makeRotate(osg::Quat(dragger_rotation.y,dragger_rotation.z,dragger_rotation.w,dragger_rotation.x));

            _dragger->setMatrix(matrix *
                                osg::Matrix::scale(scale, scale, scale) *
                                osg::Matrix::translate(anchor.x,anchor.y,anchor.z));
        }
        else
        {
            _dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
                                osg::Matrix::translate(object->getBound().center()));
        }

        _dragger->addTransformUpdating(_selection);

        // we want the dragger to handle it's own events automatically
        _dragger->setHandleEvents(true);

        // if we don't set an activation key or mod mask then any mouse click on
        // the dragger will activate it, however if do define either of ActivationModKeyMask or
        // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
        // be able to activate the dragger when you mouse click on it.  Please note the follow allows
        // activation if either the ctrl key or the 'a' key is pressed and held down.
        _dragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
        _dragger->setActivationKeyEvent('a');

        //  Print scene graph
//      showSceneGraph("",view1->getSceneData());

        return _root;
    }

    //////////////////////////
    //    Event handling    //
    //////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    //  Paint event
    ////////////////////////////////////////////////////////////////////////////
    virtual void paintEvent( QPaintEvent* event )
    {
        frame();
    }

//    ////////////////////////////////////////////////////////////////////////////
//    //  Mouse release event
//    ////////////////////////////////////////////////////////////////////////////
//    virtual void mouseReleaseEvent(QMouseEvent *e)
//    {
//      if (doubleClickPressed)
//      {
//        doubleClickPressed = false;
//
//        if (isSimpleView)
//        {
//          setMultipleView();
//        }
//        else
//        {
//          setSimpleView();
//        }
//      }
//    }
//    ////////////////////////////////////////////////////////////////////////////
//    /// Mouse double click event handler
//    ////////////////////////////////////////////////////////////////////////////
//    virtual void mouseDoubleClickEvent(QMouseEvent *e)
//    {
//      doubleClickPressed = true;
//    }

    ////////////////////////
    //  Global variables  //
    ////////////////////////

    //  Light flag
    bool _light_on;

    //  Scene Node with lights
    osg::ref_ptr<osg::Group> _scene_lights;

    //  Scene Data to romove after each repaint
    osg::ref_ptr<osg::Group> _scene_lights_data;

    //  Parent of dragger and selection
    osg::ref_ptr<osg::Group> _root;

    //  There is only one dragger at the same time
    osg::ref_ptr<osgManipulator::Dragger> _dragger;

    //  Transform applied by dragger
    osg::ref_ptr<osg::MatrixTransform> _selection;

    //  Object selected by dragger
    osg::ref_ptr<osg::Node> _selected;

//    //  Flags to apply anchor (if true) to the dragger position
//    std::map<std::string,bool> _needAnchor;

    // Kinematic body selected or robot
    std::string _actualKinbody;

    //  Actual dragger selected
    std::string _draggerName;

    //  Pick handler for joint selection
    PickHandler *_picker;

    //  Matrix transform
    osg::Matrixd _matrix1;

    //  Timer for repaint
    QTimer _timer;

    //  Environment variable
    EnvironmentBasePtr _penv;

    //  List of link children
    std::vector<osg::MatrixTransform*> _linkChildren;
};

}

#endif
