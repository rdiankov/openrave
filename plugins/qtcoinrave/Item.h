// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

// My classes
class Item;

/// Encapsulate the Inventor rendering of an Item
class Item
{
public:
    Item(QtCoinViewer* viewer);
    virtual ~Item();

    // general methods
    virtual const wchar_t* GetName() const        { return (_pname ? _pname : L"(NULL)"); }
    virtual void SetName(const wchar_t* pNewName);

    /// update underlying model from Inventor's transformation
    virtual bool UpdateFromIv() { return true; }
    virtual bool UpdateFromModel() { return true; }

    /// update Inventor nodes from model
    virtual bool UpdateFromModel(const vector<RaveTransform<dReal> >& vtrans) { return true; }

    inline RaveTransform<float> GetTransform() { return GetRaveTransform(_ivXform); }

    // if false, then item isn't rendered through the sensors
    virtual bool InWorld() { return true; }
    virtual void SetGrab(bool bGrab, bool bUpdate=true) {} ///< call when manipulating with the mouse, etc

    // Inventor related
    SoSeparator* GetIvRoot() const   { return _ivRoot; }
    SoTransform* GetIvTransform()    { return _ivXform; }
    SoSwitch*    GetIvGeom() const   { return _ivGeom; }
    SoTransparencyType* GetIvTransparency() const { return _ivTransparency; }
    bool ContainsIvNode(SoNode *pNode);
    bool ContainsIvNode(SoPath *pNodePath);
    void SetGeomVisibility(bool bFlag);
    void SetUnpickable();

protected:

    // Instance Data
    QtCoinViewer* _viewer;
    wchar_t* _pname;            //!< item name

    SoSeparator*   _ivRoot;           //!< root of Inventor data hierarchy
    SoTransform*   _ivXform;          //!< item Inventor transform
    SoSwitch*      _ivGeom;           //!< item geometry hierarchy
    SoTransparencyType* _ivTransparency;
};

// handles KinBodys
class KinBodyItem : public Item
{
public:
    KinBodyItem(QtCoinViewer* viewer, KinBody*, ViewGeometry viewmode);
    virtual ~KinBodyItem();

    const wchar_t* GetName() const        { return _pchain->GetName(); }
    void SetName(const wchar_t* pNewName) { _pchain->SetName(pNewName); }

    virtual bool UpdateFromIv();
    virtual bool UpdateFromModel();
    virtual bool UpdateFromModel(const vector<RaveTransform<dReal> >& vtrans);

    virtual void SetGrab(bool bGrab, bool bUpdate=true);

    inline KinBody* GetBody() const { return _pchain; }

    // gets the link from IV
    KinBody::Link* GetLinkFromIv(SoNode* plinknode) const;
    
    // gets the link from the index
    SoSeparator* GetIvLink(int index) const { return _veclinks[index].first; }

    void SetUserData(int userdata) { _userdata = userdata; }
    int GetUserData() { return _userdata; }
    
    int GetNetworkId() { return networkid; }

protected:
    typedef std::pair<SoSeparator*, SoTransform*> LINK;

    KinBody* _pchain;
    int networkid;        ///< _pchain->GetNetworkId()
    std::vector< LINK > _veclinks; ///< render items for each link, indexed same as links
    bool bGrabbed, bEnabled;
    ViewGeometry _viewmode;
    int _userdata;
};

class RobotItem : public KinBodyItem
{
public:
    /// objects that specify the end effector position
    /// first is the index link index
    struct EE
    {
        EE() {}
        EE(int index, SoTransform* ptrans, SoSwitch* pswitch) : _index(index), _ptrans(ptrans), _pswitch(pswitch) {}
        int _index;
        SoTransform* _ptrans;
        SoSwitch* _pswitch;
    };

    RobotItem(QtCoinViewer* viewer, RobotBase* robot, ViewGeometry viewmode);

    virtual bool UpdateFromIv();
    virtual bool UpdateFromModel(const vector<RaveTransform<dReal> >& vtrans);

    RobotBase* GetRobot() { return (RobotBase*)_pchain; }
    
    virtual void SetGrab(bool bGrab, bool bUpdate=true);

private:
    std::vector< EE > _vEndEffectors;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(KinBodyItem)
BOOST_TYPEOF_REGISTER_TYPE(RobotItem)
BOOST_TYPEOF_REGISTER_TYPE(RobotItem::EE)
#endif

#endif   // ITEM_H
