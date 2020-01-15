// -*- coding: utf-8 -*-
// Copyright (C) 2012-2016 Rosen Diankov, Gustavo Puche, OpenGrasp Team
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
/*! --------------------------------------------------------------------
   \file   osgrenderitem.h
   \brief  Abstract base class for a render item
   -------------------------------------------------------------------- */

#ifndef OPENRAVE_OSG_ITEM_H
#define OPENRAVE_OSG_ITEM_H

namespace qtosgrave {

enum ViewGeometry {
    VG_RenderOnly = 0,
    VG_CollisionOnly = 1,
    VG_RenderCollision = 2,
};

/// \brief creates XYZ axes and returns their osg objects
OSGGroupPtr CreateOSGXYZAxes(double len, double axisthickness);

/// \brief Encapsulate the Inventor rendering of an Item
class Item : public boost::enable_shared_from_this<Item>, public OpenRAVE::UserData
{
public:
    Item(OSGGroupPtr osgSceneRoot, OSGGroupPtr osgFigureRoot);
    virtual ~Item();

    /// \brief called when OpenRAVE::Environment is locked and item is about to be removed
    virtual void PrepForDeletion() {
    }

    // general methods

    virtual const string& GetName() const {
        return _osgdata->getName();
    }

    /// \brief update underlying model from OSG's transformation
    virtual bool UpdateFromOSG() {
        return true;
    }
    virtual bool UpdateFromModel() {
        return true;
    }

    /// \brief update OSG nodes from model
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans) {
        return true;
    }

    inline RaveTransform<float> GetTransform() {
        return GetRaveTransform(*_osgWorldTransform);
    }

    // if false, then item isn't rendered through the sensors
    virtual bool InWorld() {
        return true;
    }
    virtual void SetGrab(bool bGrab, bool bUpdate=true) {
    }

    /// \brief sets how the item is visualized. item string means the default.
    virtual void SetVisualizationMode(const std::string& visualizationmode);

    OSGMatrixTransformPtr GetOSGRoot() const {
        return _osgWorldTransform;
    }

    OSGGroupPtr GetOSGGeom() const {
        return _osgdata;
    }

    /// \brief returns true if the given node is in the inventor hierarchy
    bool ContainsOSGNode(OSGNodePtr pNode);


    /// \brief Set the visibility of the geometry (ON = true).
//    void SetUnpickable();


protected:

    // Instance Data
    OSGGroupPtr _osgSceneRoot, _osgFigureRoot; ///< used for adding and removing itself from the viewer node
    string _name; ///< item name
    //OSGGroupPtr _osgitemroot; ///< root of this object's OSG data hierarchy
    OSGMatrixTransformPtr _osgWorldTransform; ///< Kinbody position
    OSGGroupPtr _osgdata; ///< item geometry hierarchy
    OSGGroupPtr _osgwireframe;
    std::string _visualizationmode; ///< current visualization mode that item is set to
};

typedef boost::shared_ptr<Item> ItemPtr;
typedef boost::weak_ptr<Item> ItemWeakPtr;
typedef boost::shared_ptr<Item const> ItemConstPtr;


/// \brief user data set to the OSG nodes to keep track of this item
class OSGItemUserData : public osg::Referenced
{
public:
    OSGItemUserData(ItemWeakPtr item) : _item(item) {
    }

    ItemPtr GetItem() const {
        return _item.lock();
    }
private:
    ItemWeakPtr _item;
};

/// \brief render item that  handles KinBodys
class KinBodyItem : public Item
{
public:
    KinBodyItem(OSGGroupPtr osgSceneRoot, OSGGroupPtr osgFigureRoot, KinBodyPtr, ViewGeometry viewmode);
    virtual ~KinBodyItem();

    virtual void PrepForDeletion() {
        _geometrycallback.reset();
        _drawcallback.reset();
    }

//    void SetName(const string& pNewName) {
//        _pbody->SetName(pNewName);
//    }

    virtual bool UpdateFromOSG();

    /// \brief updates from openrave model
    virtual bool UpdateFromModel();

    /// \brief updates from openrave model
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans);

    virtual void SetGrab(bool bGrab, bool bUpdate=true);

    inline KinBodyPtr GetBody() const {
        return _pbody;
    }

    /// \brief gets the link from osg node
    KinBody::LinkPtr GetLinkFromOSG(OSGNodePtr plinknode) const;

    /// \brief gets the link from the index
    OSGMatrixTransformPtr GetOSGLink(int index) const {
        return _veclinks.at(index).second;
    }

    /// \brief gets the geom from osg node
    KinBody::Link::GeometryPtr GetGeomFromOSG(OSGNodePtr pgeomnode) const;

    /// \brief gets the geom from the index
    OSGMatrixTransformPtr GetOSGGeom(int linkindex, int geomindex) const {
        return _vecgeoms.at(linkindex).at(geomindex).second;
    }

    void SetUserData(int userdata) {
        _userdata = userdata;
    }
    int GetUserData() {
        return _userdata;
    }

    virtual void GetDOFValues(vector<dReal>& vjoint) const;
    virtual void GetLinkTransformations(vector<Transform>& vtrans, std::vector<dReal>& vdofbranches) const;

    virtual Transform GetTransform() const {
        if( _vtrans.size() > 0 ) {
            return _vtrans.at(0);
        }
        else {
            return GetRaveTransform(*_osgWorldTransform);
        }
    }

    /// \brief loads the OSG nodes and also sets _osgWorldTransform's userdata to point to this item
    virtual void Load();

protected:
    /// \brief Calculate per-face normals from face vertices.
    //osg::ref_ptr<osg::Vec3Array> _GenerateNormals(const TriMesh&);

    virtual void _HandleGeometryChangedCallback();
    virtual void _HandleDrawChangedCallback();

    typedef std::pair<OSGGroupPtr, OSGMatrixTransformPtr> LinkNodes;
    typedef std::pair<OSGGroupPtr, OSGMatrixTransformPtr> GeomNodes;

    KinBodyPtr _pbody;
    int _environmentid;        ///< _pbody->GetEnvironmentId()
    std::vector<LinkNodes> _veclinks; ///< render items for each link, indexed same as links. The group's hierarchy mimics the kinematics hierarchy. For each pair, the first Group node is used for the hierarchy, the second node contains the transform with respect to the body's transform
    std::vector<std::vector<GeomNodes> > _vecgeoms; ///< render items for each link's geometries, indexed same as geometries.
    bool bEnabled;
    bool bGrabbed, _bReload, _bDrawStateChanged;
    ViewGeometry _viewmode;
    int _userdata;

    std::vector<dReal> _vjointvalues;
    vector<Transform> _vtrans;
    mutable boost::mutex _mutexjoints;
    UserDataPtr _geometrycallback, _drawcallback;

private:
    /// \brief Print matrix
    void _PrintMatrix(osg::Matrix& m);

    /// \brief  Print nodes of scenegraph
    void _PrintSceneGraph(const std::string& currLevel, OSGNodePtr currNode);

    /// \brief  Print the features of the OSG Node
    void _PrintNodeFeatures(OSGNodePtr node);
};

typedef boost::shared_ptr<KinBodyItem> KinBodyItemPtr;
typedef boost::shared_ptr<KinBodyItem const> KinBodyItemConstPtr;

class RobotItem : public KinBodyItem
{
public:
    /// objects that specify the end effector position
    /// first is the index link index
    struct EE
    {
        EE() {
        }
        EE(OSGMatrixTransformPtr ptrans, OSGSwitchPtr pswitch) : _ptrans(ptrans), _pswitch(pswitch) {
        }
        RobotBase::ManipulatorWeakPtr manip;
        RobotBase::AttachedSensorWeakPtr attsensor;
        OSGMatrixTransformPtr _ptrans;
        OSGSwitchPtr _pswitch;
    };

    RobotItem(OSGGroupPtr osgSceneRoot, OSGGroupPtr osgFigureRoot, RobotBasePtr robot, ViewGeometry viewmode);
    virtual ~RobotItem();

    virtual void Load();

    virtual bool UpdateFromOSG();
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans);

    RobotBasePtr GetRobot() const {
        return _probot;
    }

    virtual void SetGrab(bool bGrab, bool bUpdate=true);

private:
    std::vector< EE > _vEndEffectors, _vAttachedSensors;
    RobotBasePtr _probot;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(KinBodyItem)
BOOST_TYPEOF_REGISTER_TYPE(RobotItem)
BOOST_TYPEOF_REGISTER_TYPE(RobotItem::EE)
#endif

}

#endif   // RAVE_OSG_ITEM_H
