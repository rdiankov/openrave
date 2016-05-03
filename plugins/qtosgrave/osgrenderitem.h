// -*- coding: utf-8 -*-
// Copyright (C) 2012-2016 Gustavo Puche, Rosen Diankov, OpenGrasp Team
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

#define QTOSG_LOCALTRANSFORM_PREFIX "tl-"
#define QTOSG_GLOBALTRANSFORM_PREFIX "tg-"

namespace qtosgrave {

enum ViewGeometry {
    VG_RenderOnly = 0,
    VG_CollisionOnly = 1,
    VG_RenderCollision = 2,
};

/// Encapsulate the Inventor rendering of an Item
class Item : public boost::enable_shared_from_this<Item>, public OpenRAVE::UserData
{
public:
    Item(OSGGroupPtr osgViewerRoot);
    virtual ~Item();

    /// \brief called when OpenRAVE::Environment is locked and item is about to be removed
    virtual void PrepForDeletion() {
    }

    // general methods
    
    virtual const string& GetName() const {
        return _name;
    }
    virtual void SetName(const string& name) {
        _name = name;
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
    } ///< call when manipulating with the mouse, etc

    // Inventor related
    OSGGroupPtr GetOSGRoot() const {
        return _osgWorldTransform;
    }
    OSGMatrixTransformPtr GetIvTransform()    {
        return _osgWorldTransform;
    }
    OSGSwitchPtr GetOSGGeom() const {
        return _osgdata;
    }
//    SoTransparencyType* GetIvTransparency() const { return _ivTransparency; }

    /// \brief returns true if the given node is in the inventor hierarchy
    bool ContainsOSGNode(OSGNodePtr pNode);
//    bool ContainsOSGNode(osg::NodePath *pNodePath);

    /// \brief Set the visibility of the geometry (ON = true).
    void SetGeomVisibility(bool bFlag);
//    void SetUnpickable();

protected:

    // Instance Data
    OSGGroupPtr _osgViewerRoot; ///< used for adding and removing itself from the viewer node
    string _name; ///< item name
    //OSGGroupPtr _osgitemroot; ///< root of this object's OSG data hierarchy
    OSGMatrixTransformPtr _osgWorldTransform; ///< Kinbody position
    OSGSwitchPtr _osgdata; ///< item geometry hierarchy
    //osg::ref_ptr<osg::BlendColor> _ivTransparency;
};
typedef boost::shared_ptr<Item> ItemPtr;
typedef boost::shared_ptr<Item const> ItemConstPtr;

/// \brief render item that  handles KinBodys
class KinBodyItem : public Item
{
public:
    KinBodyItem(OSGGroupPtr osgViewerRoot, KinBodyPtr, ViewGeometry viewmode);
    virtual ~KinBodyItem();

    virtual void PrepForDeletion() {
        _geometrycallback.reset();
        _drawcallback.reset();
    }

    const string& GetName() const {
        return _pchain->GetName();
    }
    void SetName(const string& pNewName) {
        _pchain->SetName(pNewName);
    }

    virtual bool UpdateFromOSG();

    /// \brief updates from openrave model
    virtual bool UpdateFromModel();

    /// \brief updates from openrave model
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans);

    virtual void SetGrab(bool bGrab, bool bUpdate=true);

    inline KinBodyPtr GetBody() const {
        return _pchain;
    }

    /// \brief gets the link from osg node
    KinBody::LinkPtr GetLinkFromOSG(OSGNodePtr plinknode) const;

    /// \brief gets the link from the index
    OSGGroupPtr GetOSGLink(int index) const {
        return _veclinks[index].first;
    }

    void SetUserData(int userdata) {
        _userdata = userdata;
    }
    int GetUserData() {
        return _userdata;
    }

    virtual void GetDOFValues(vector<dReal>& vjoint) const;
    virtual void GetLinkTransformations(vector<Transform>& vtrans, std::vector<dReal>& vdofbranches) const;
    virtual void Load();

protected:
    /// \brief Calculate per-face normals from face vertices.
    osg::ref_ptr<osg::Vec3Array> _GenerateNormals(const TriMesh&);
    
    /// \brief Gets osg node with name 'name'
    OSGGroupPtr _FindNodeName(const string& name);

    virtual void _HandleGeometryChangedCallback();
    virtual void _HandleDrawChangedCallback();

    typedef std::pair<OSGGroupPtr, OSGMatrixTransformPtr> LINK;

    KinBodyPtr _pchain;
    int _environmentid;        ///< _pchain->GetEnvironmentId()
    std::vector< LINK > _veclinks; ///< render items for each link, indexed same as links
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
    void _SetNamedNode(const std::string&  name, OSGNodePtr currNode);
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
        EE(int index, OSGMatrixTransformPtr ptrans, OSGSwitchPtr pswitch) : _index(index), _ptrans(ptrans), _pswitch(pswitch) {
        }
        int _index;
        OSGMatrixTransformPtr _ptrans;
        OSGSwitchPtr _pswitch;
    };

    RobotItem(OSGGroupPtr osgViewerRoot, RobotBasePtr robot, ViewGeometry viewmode);

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
