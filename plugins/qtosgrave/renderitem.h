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
/*! --------------------------------------------------------------------
   \file   renderitem.h
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

/// Encapsulate the Inventor rendering of an Item
class Item : public boost::enable_shared_from_this<Item>, public OpenRAVE::UserData
{
public:
    Item(QtOSGViewerPtr viewer);
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
    virtual bool UpdateFromIv() {
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
        return GetRaveTransform(_ivXform);
    }

    // if false, then item isn't rendered through the sensors
    virtual bool InWorld() {
        return true;
    }
    virtual void SetGrab(bool bGrab, bool bUpdate=true) {
    } ///< call when manipulating with the mouse, etc

    // Inventor related
    osg::Group* GetIvRoot() const {
        return _ivRoot;
    }
    osg::MatrixTransform* GetIvTransform()    {
        return _ivXform;
    }
    osg::Switch*    GetIvGeom() const {
        return _ivGeom;
    }
//    SoTransparencyType* GetIvTransparency() const { return _ivTransparency; }

    /// \brief returns true if the given node is in the inventor hierarchy
    bool ContainsOSGNode(osg::Node *pNode);
//    bool ContainsOSGNode(osg::NodePath *pNodePath);

    /// \brief Set the visibility of the geometry (ON = true).
    void SetGeomVisibility(bool bFlag);
//    void SetUnpickable();

protected:

    // Instance Data
    QtOSGViewerPtr _viewer;
    string _name;           //!< item name
    osg::Group*                _ivRoot;           //!< root of OSG data hierarchy
    osg::MatrixTransform*  _ivXform;          //!< Kinbody position
    osg::Switch*             _ivGeom;           //!< item geometry hierarchy
    osg::BlendColor*           _ivTransparency;
};
typedef boost::shared_ptr<Item> ItemPtr;
typedef boost::shared_ptr<Item const> ItemConstPtr;

// handles KinBodys
class KinBodyItem : public Item
{
public:
    KinBodyItem(QtOSGViewerPtr viewer, KinBodyPtr, ViewGeometry viewmode);
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

    virtual bool UpdateFromIv();
    virtual bool UpdateFromModel();
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans);

    virtual void SetGrab(bool bGrab, bool bUpdate=true);

    inline KinBodyPtr GetBody() const {
        return _pchain;
    }

    // gets the link from IV
    KinBody::LinkPtr GetLinkFromIv(osg::Node* plinknode) const;

    // gets the link from the index
    osg::Group* GetIvLink(int index) const {
        return _veclinks[index].first;
    }

    void SetUserData(int userdata) {
        _userdata = userdata;
    }
    int GetUserData() {
        return _userdata;
    }

    int GetNetworkId() {
        return networkid;
    }

    virtual void GetDOFValues(vector<dReal>& vjoint) const;
    virtual void GetLinkTransformations(vector<Transform>& vtrans, std::vector<dReal>& vdofbranches) const;
    virtual void Load();

    osg::Vec3Array* generateNormals(osg::Vec3Array *vertices);

protected:
    /// \brief Gets osg node with name 'name'
    osg::Group* _FindNodeName(const string& name);

    virtual void GeometryChangedCallback();
    virtual void DrawChangedCallback();

    typedef std::pair<osg::Group*, osg::MatrixTransform*> LINK;

    KinBodyPtr _pchain;
    int networkid;        ///< _pchain->GetNetworkId()
    std::vector< LINK > _veclinks; ///< render items for each link, indexed same as links
    bool bEnabled;
    bool bGrabbed, _bReload, _bDrawStateChanged;
    ViewGeometry _viewmode;
    int _userdata;

    vector<dReal> _vjointvalues;
    vector<Transform> _vtrans;
    std::vector<dReal> _vdofbranches;
    mutable boost::mutex _mutexjoints;
    UserDataPtr _geometrycallback, _drawcallback;

private:
    /// \brief Print matrix
    void _PrintMatrix(osg::Matrix& m);

    /// \brief  Print nodes of scenegraph
    void _PrintSceneGraph(const std::string& currLevel,osg::Node* currNode);

    /// \brief  Print the features of the OSG Node
    void _PrintNodeFeatures(osg::Node *node);
    void setNamedNode(const std::string&  name, osg::Node*    currNode);
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
        EE(int index, osg::MatrixTransform* ptrans, osg::Switch* pswitch) : _index(index), _ptrans(ptrans), _pswitch(pswitch) {
        }
        int _index;
        osg::MatrixTransform* _ptrans;
        osg::Switch* _pswitch;
    };

    RobotItem(QtOSGViewerPtr viewer, RobotBasePtr robot, ViewGeometry viewmode);

    virtual bool UpdateFromIv();
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
