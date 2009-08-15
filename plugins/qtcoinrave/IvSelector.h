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
  \file   IvSelector.h
  \brief  OpenInventor selection class
 -------------------------------------------------------------------- */

#ifndef IV_SELECTOR_H
#define IV_SELECTOR_H

//! Abstract base class for all draggers.
class IvDragger
{
public:
    IvDragger(QtCoinViewer *viewer, Item *pItem, float draggerScale);
    virtual ~IvDragger();

    virtual void CheckCollision(bool flag) = 0;
    virtual void AddAnchor(Item *) {}

    virtual void UpdateSkeleton() = 0;
    virtual void UpdateDragger() = 0;

    virtual void GetMessage(ostream& sout) { } // text message about the dragger

protected:
    static const SbColor CHECK_COLOR, COLLISION_COLOR;

    static void _MotionHandler(void *, SoDragger *);
    void        _GetBounds(SoSeparator *subtree, AABB& ab);
    void        _GetMatrix(SbMatrix &, SoNode *, SoNode *);

    bool        _checkCollision;
    SbColor     _normalColor;
    Item*       _selectedItem;
    QtCoinViewer*    _viewer;
    SoSeparator* _axes; // axes of the object's origin
    vector<float> vtransparency;
    float       _scale;
    SoSFEnum _prevtransparency;
    //SoText2* _ptext; // text to display next to selected item
    AABB _ab;
};

//! Class to represent an object dragger.
class IvObjectDragger : public IvDragger
{
public:
    IvObjectDragger(QtCoinViewer *viewer, Item *pItem, float draggerScale, bool bAllowRotation = true);
    ~IvObjectDragger();

    void CheckCollision(bool flag);
    void UpdateSkeleton();
    void UpdateDragger() {}

    virtual void GetMessage(ostream& sout);

protected:
    void _SetColor(const SbColor &);

    SoSeparator*           _draggerRoot;
    SoTransformBoxDragger* _transformBox;
    SoMaterial*            _draggerMaterial;
    RaveTransform<float> _toffset;
};

/// Class to represent a joint rotation dragger.
class IvJointDragger : public IvDragger {
public:
    IvJointDragger(QtCoinViewer *viewer, Item *pItem, int iSelectedLink, float draggerScale, int iJointIndex, bool bHilitJoint);
    ~IvJointDragger();

    void CheckCollision(bool flag);
    void UpdateSkeleton();
    void UpdateDragger();

    virtual void GetMessage(ostream& sout);

protected:
    void _SetColor(const SbColor &);
    RaveVector<float> GetJointOffset();

    Vector vaxes[2];
    SoSeparator* _pLinkNode; // node of the link of the target body
    SoSeparator*         _draggerRoot;
    SoTranslation*       _offset;
    SoMaterial*          _material;
    SoTrackballDragger*  _trackball;
    SoMaterial*          _draggerMaterial[3];
    int _iJointIndex, _iSelectedLink;
    bool                _bHilitJoint;
};

////! Class to represent an IK dragger.
//
//class IvIKDragger : public IvDragger {
//public:
//  IvIKDragger(QtCoinViewer *viewer, Item *pItem, float draggerScale, LinkNode* pLink,
//	      bool bAxis[3]);
//  ~IvIKDragger();
//
//  void CheckCollision(bool flag);
//  void UpdateSkeleton();
//  void UpdateDragger();
//
//protected:
//  void _SetColor(const SbColor &);
//
//  SoSeparator*           _draggerRoot;
//  SoTransformBoxDragger* _transformBox;
//  SoMaterial*            _draggerMaterial;
//};

#endif // IV_SELECTOR_H
