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
   \file   IvSelector.h
   \brief  OpenInventor selection class
   -------------------------------------------------------------------- */

#ifndef IV_SELECTOR_H
#define IV_SELECTOR_H

/// Construct a new dragger given the root of the scene graph and the selected node.
class IvDragger
{
public:
    IvDragger(QtCoinViewerPtr viewer, ItemPtr pItem, float draggerScale);
    virtual ~IvDragger();

    virtual void CheckCollision(bool flag) = 0;
    virtual void AddAnchor(Item *) {
    }

    virtual void UpdateSkeleton() = 0;
    virtual void UpdateDragger() = 0;

    virtual void GetMessage(ostream& sout) {
    }                                              // text message about the dragger

    virtual ItemPtr GetSelectedItem() {
        return _selectedItem.lock();
    }

protected:
    static const SbColor CHECK_COLOR, COLLISION_COLOR;

    static SoSeparator* _CreateAxes(float fSize=1.0f,float fColor=1.0f);

    /// Handler for Inventor motion callbacks.
    static void _MotionHandler(void *, SoDragger *);

    /// Get the bounding box of the given subtree.
    void        _GetBounds(SoSeparator *subtree, AABB& ab);

    /// Get the Inventor transformation matrix that describes the given node relative to the given root.
    void        _GetMatrix(SbMatrix &, SoNode *, SoNode *);

    bool _checkCollision;
    SbColor _normalColor;
    ItemWeakPtr _selectedItem;
    std::weak_ptr<QtCoinViewer> _viewer;
    EnvironmentBasePtr _penv;
    vector<SoSeparator*> _vlinkaxes;     // axes of the object's origin
    vector<float> vtransparency;
    float _scale;
    SoSFEnum _prevtransparency;
    //SoText2* _ptext; // text to display next to selected item
    AABB _ab;
};

/// Class to represent an object dragger. This allows general
/// translation and rotation, and checks for collision against
/// the rest of the world if requested.
class IvObjectDragger : public IvDragger
{
public:
    IvObjectDragger(QtCoinViewerPtr viewer, ItemPtr pItem, float draggerScale, bool bAllowRotation = true);
    ~IvObjectDragger();

    void CheckCollision(bool flag);
    void UpdateSkeleton();
    void UpdateDragger() {
    }

    virtual void GetMessage(ostream& sout);

protected:
    /// Set the color of the dragger.
    void _SetColor(const SbColor &);

    SoSeparator*           _draggerRoot;
    SoTransformBoxDragger* _transformBox;
    SoMaterial*            _draggerMaterial;
    RaveTransform<float> _toffset;
};

/// Class to represent an joint rotation dragger. This allows
/// rotation relative to the parent joint. It honors joint limits
/// and checks for collision between the world and the joint's subtree.
class IvJointDragger : public IvDragger {
public:
    IvJointDragger(QtCoinViewerPtr viewer, ItemPtr pItem, int iSelectedLink, float draggerScale, int iJointIndex, bool bHilitJoint);
    ~IvJointDragger();

    void CheckCollision(bool flag);
    void UpdateSkeleton();
    void UpdateDragger();

    virtual void GetMessage(ostream& sout);

protected:
    /// Set the color of the dragger.
    void _SetColor(const SbColor &);

    KinBody::JointType _jointtype;
    int _dofindex;
    vector<dReal> _vlower, _vupper;
    string _jointname;
    dReal _jointoffset;

    SoSeparator* _pLinkNode;     // node of the link of the target body
    SoSeparator*         _draggerRoot;
    SoMaterial*          _material;
    SoTrackballDragger*  _trackball;
    SoMaterial*          _draggerMaterial[3];
    int _iJointIndex, _iSelectedLink;
    bool _bHilitJoint;
};

#endif // IV_SELECTOR_H
