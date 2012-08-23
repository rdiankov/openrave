// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/*! --------------------------------------------------------------------
   \file   Item.h
   \brief  Abstract base class for a render item
   -------------------------------------------------------------------- */

#ifndef  RAVE_ITEM_H
#define  RAVE_ITEM_H

enum ViewGeometry {
    VG_RenderOnly = 0,
    VG_CollisionOnly = 1,
    VG_RenderCollision = 2,
};

/// Encapsulate the Inventor rendering of an Item
class Item : public boost::enable_shared_from_this<Item>, public OpenRAVE::UserData
{
public:
    Item(QtCoinViewerPtr viewer);
    virtual ~Item();

    /// \brief called when OpenRAVE::Environment is locked and item is about to be removed
    virtual void PrepForDeletion() {
    }

    // general methods
    virtual const string& GetName() const {
        return _name;
    }
    virtual void SetName(const string& newname) {
        _name = newname;
    }

    /// update underlying model from Inventor's transformation
    virtual bool UpdateFromIv() {
        return true;
    }
    virtual bool UpdateFromModel() {
        return true;
    }

    /// update Inventor nodes from model
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
    }                                                          ///< call when manipulating with the mouse, etc

    // Inventor related
    SoSeparator* GetIvRoot() const {
        return _ivRoot;
    }
    SoTransform* GetIvTransform()    {
        return _ivXform;
    }
    SoSwitch*    GetIvGeom() const {
        return _ivGeom;
    }
    SoTransparencyType* GetIvTransparency() const {
        return _ivTransparency;
    }

    /// \brief returns true if the given node is in the inventor hierarchy
    bool ContainsIvNode(SoNode *pNode);
    /// \brief returns true if the path the given node passes through the geometry root of this item (i.e. the node is part of the geometry of this item).
    bool ContainsIvNode(SoPath *pNodePath);
    /// \brief Set the visibility of the geometry (ON = true).
    void SetGeomVisibility(bool bFlag);
    /// \brief Set the pick style of the node to be unpickable
    void SetUnpickable();

    virtual SoSeparator* GetIvLink(int index) const {
        return _ivRoot;
    }
    virtual int GetNumIvLinks() const {
        return 1;
    }

protected:

    // Instance Data
    boost::weak_ptr<QtCoinViewer> _viewer;
    string _name;

    SoSeparator*   _ivRoot;               //!< root of Inventor data hierarchy
    SoTransform*   _ivXform;              //!< item Inventor transform
    SoSwitch*      _ivGeom;               //!< item geometry hierarchy
    SoTransparencyType* _ivTransparency;
};
typedef boost::shared_ptr<Item> ItemPtr;
typedef boost::weak_ptr<Item> ItemWeakPtr;
typedef boost::shared_ptr<Item const> ItemConstPtr;

class KinBodyItem : public Item
{
protected:
    struct LINK
    {
        SoSeparator* psep;
        SoTransform* ptrans;
        KinBody::LinkWeakPtr plink;
    };

    inline boost::shared_ptr<KinBodyItem> shared_kinbody() {
        return boost::dynamic_pointer_cast<KinBodyItem>(shared_from_this());
    }
    inline boost::weak_ptr<KinBodyItem> weak_kinbody() {
        return shared_kinbody();
    }

public:
    KinBodyItem(QtCoinViewerPtr viewer, KinBodyPtr, ViewGeometry viewmode);
    virtual ~KinBodyItem() {
    }

    const string& GetName() const {
        return _pchain->GetName();
    }
    void SetName(const string& pNewName) {
        _pchain->SetName(pNewName);
    }

    virtual void PrepForDeletion() {
        _geometrycallback.reset();
        _drawcallback.reset();
    }

    virtual bool UpdateFromIv();
    virtual bool UpdateFromModel();
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans);

    virtual void SetGrab(bool bGrab, bool bUpdate=true);

    inline KinBodyPtr GetBody() const {
        return _pchain;
    }

    /// gets the link from IV
    virtual KinBody::LinkPtr GetLinkFromIv(SoNode* plinknode) const;

    /// gets the link from the index
    virtual SoSeparator* GetIvLink(int index) const {
        return _veclinks.at(index).psep;
    }
    virtual int GetNumIvLinks() const {
        return _veclinks.size();
    }

    virtual void SetUserData(int userdata) {
        _userdata = userdata;
    }
    virtual int GetUserData() {
        return _userdata;
    }

    virtual int GetNetworkId() {
        return networkid;
    }

    virtual void GetDOFValues(vector<dReal>& vjoint) const;
    virtual void GetLinkTransformations(vector<Transform>& vtrans, std::vector<int>& vdofbranches) const;
    virtual void Load();
protected:
    virtual void GeometryChangedCallback();
    virtual void DrawChangedCallback();

    KinBodyPtr _pchain;
    int networkid;            ///< _pchain->GetNetworkId()
    std::vector< LINK > _veclinks;     ///< render items for each link, indexed same as links
    bool bGrabbed, _bReload, _bDrawStateChanged;
    ViewGeometry _viewmode;
    int _userdata;

    vector<dReal> _vjointvalues;
    vector<Transform> _vtrans;
    std::vector<int> _vdofbranches;
    mutable boost::mutex _mutexjoints;
    UserDataPtr _geometrycallback, _drawcallback;
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
        EE() : _index(-1), _ptrans(NULL), _pswitch(NULL) {
        }
        EE(int index, SoTransform* ptrans, SoSwitch* pswitch) : _index(index), _ptrans(ptrans), _pswitch(pswitch) {
        }
        int _index;
        SoTransform* _ptrans;
        SoSwitch* _pswitch;
    };

    RobotItem(QtCoinViewerPtr viewer, RobotBasePtr robot, ViewGeometry viewmode);

    virtual bool UpdateFromIv();
    virtual bool UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans);

    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }
    virtual void SetGrab(bool bGrab, bool bUpdate=true);
    virtual void Load();
private:
    void CreateAxis(EE& ee, const string& name, const Vector* pdirection=NULL);
    std::vector< EE > _vEndEffectors, _vAttachedSensors;
    RobotBasePtr _probot;
};
typedef boost::shared_ptr<RobotItem> RobotItemPtr;
typedef boost::shared_ptr<RobotItem const> RobotItemConstPtr;

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(KinBodyItem)
BOOST_TYPEOF_REGISTER_TYPE(RobotItem)
BOOST_TYPEOF_REGISTER_TYPE(RobotItem::EE)
#endif

#endif   // ITEM_H
