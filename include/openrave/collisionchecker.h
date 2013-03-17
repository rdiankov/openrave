// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file collisionchecker.h
    \brief Collision checking related definitions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_COLLISIONCHECKER_H
#define OPENRAVE_COLLISIONCHECKER_H

namespace OpenRAVE {

/// options for collision checker
enum CollisionOptions
{
    CO_Distance = 1, ///< Compute distance measurements, this is usually slow and not all checkers support it.
    CO_UseTolerance = 2, ///< not used
    CO_Contacts = 4, ///< Return the contact points of the collision in the \ref CollisionReport. Note that this takes longer to compute.
    CO_RayAnyHit = 8, ///< When performing collision with rays, if this is set, algorithm just returns any hit instead of the closest (can be faster)

    /** Allows planners to greatly reduce redundant collision checks.
        If set and the target object is a robot, then only the links controlled by the currently set active DOFs and their attached bodies will be checked for collisions.

        The things that **will not be** checked for collision are:
        - links that do not remove with respect to each other as a result of moving the active dofs.
     */
    CO_ActiveDOFs = 16,
};

/// \brief action to perform whenever a collision is detected between objects
enum CollisionAction
{
    CA_DefaultAction = 0, ///< let the physics/collision engine resolve the action
    CA_Ignore = 1, ///< do nothing
};

/// \brief Holds information about a particular collision that occured.
class OPENRAVE_API CollisionReport
{
public:
    CollisionReport() {
        Reset();
    }

    class OPENRAVE_API CONTACT
    {
public:
        CONTACT() : depth(0) {
        }
        CONTACT(const Vector &p, const Vector &n, dReal d) : pos(p), norm(n) {
            depth = d;
        }

        Vector pos;  ///< where the contact occured
        Vector norm; ///< the normals of the faces
        dReal depth; ///< the penetration depth, positive means the surfaces are penetrating, negative means the surfaces are not colliding (used for distance queries)
    };

    int options; ///< the options that the CollisionReport was called with

    KinBody::LinkConstPtr plink1, plink2; ///< the colliding links if a collision involves a bodies. Collisions do not always occur with 2 bodies like ray collisions, so these fields can be empty.

    //KinBody::Link::GeomConstPtr pgeom1, pgeom2; ///< the specified geometries hit for the given links
    int numCols; ///< this is the number of objects that collide with the object of interest
    std::vector<KinBody::LinkConstPtr> vLinkColliding; ///< objects colliding with this object

    dReal minDistance; ///< minimum distance from last query, filled if CO_Distance option is set
    int numWithinTol; ///< number of objects within tolerance of this object, filled if CO_UseTolerance option is set

    std::vector<CONTACT> contacts; ///< the convention is that the normal will be "out" of plink1's surface. Filled if CO_UseContacts option is set.

    virtual void Reset(int coloptions = 0);
    virtual std::string __str__() const;
};

typedef CollisionReport COLLISIONREPORT RAVE_DEPRECATED;

/** \brief <b>[interface]</b> Responsible for all collision checking queries of the environment. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_collisionchecker.
    \ingroup interfaces
 */
class OPENRAVE_API CollisionCheckerBase : public InterfaceBase
{
public:
    CollisionCheckerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_CollisionChecker, penv) {
    }
    virtual ~CollisionCheckerBase() {
    }

    /// \brief return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_CollisionChecker;
    }

    /// \brief Set basic collision options using the CollisionOptions enum
    virtual bool SetCollisionOptions(int collisionoptions) = 0;

    /// \brief get the current collision options
    virtual int GetCollisionOptions() const = 0;

    virtual void SetTolerance(dReal tolerance) = 0;

    /// \brief initialize the checker with the current environment and gather all current bodies in the environment and put them in its collision space
    virtual bool InitEnvironment() = 0;

    /// \brief clear/deallocate any memory associated with tracking collision data for bodies
    virtual void DestroyEnvironment() = 0;

    /// \brief notified when a new body has been initialized in the environment
    virtual bool InitKinBody(KinBodyPtr pbody) = 0;

    /// \brief notified when a body has been removed from the environment
    virtual void RemoveKinBody(KinBodyPtr pbody) = 0;

    /// Each function takes an optional pointer to a CollisionReport structure and returns true if collision occurs.
    /// \name Collision specific functions.
    /// \anchor collision_checking
    //@{

    /// \brief checks collision of a body and a scene. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of the body.
    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief checks collision between two bodies. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of the pbody1.
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief checks collision of a link and a scene. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief checks collision of two links. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief checks collision of a link and a body. Attached bodies for pbody are respected. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief checks collision of a link and a scene. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief checks collision of a body and a scene. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of pbody.
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \brief Check collision with a link and a ray with a specified length. CO_ActiveDOFs option is ignored.
    ///
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param plink the link to collide with
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief Check collision with a link and a ray with a specified length.
    ///
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param pbody the link to collide with. If CO_ActiveDOFs is set, will only check affected links of the body.
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief Check collision with a body and a ray with a specified length. CO_ActiveDOFs option is ignored.
    ///
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param pbody the kinbody to look for collisions
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief Checks self collision only with the links of the passed in body.
    ///
    /// Links that are joined together are ignored.
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief Checks self collision of the link with the rest of the links with its parent
    ///
    /// Links that are joined together are ignored.
    virtual bool CheckSelfCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) = 0;

protected:
    /// \deprecated (12/12/11)
    virtual void SetCollisionData(KinBodyPtr pbody, UserDataPtr data) RAVE_DEPRECATED {
        pbody->SetUserData(GetXMLId(), data);
    }

    inline CollisionCheckerBasePtr shared_collisionchecker() {
        return boost::static_pointer_cast<CollisionCheckerBase>(shared_from_this());
    }
    inline CollisionCheckerBaseConstPtr shared_collisionchecker_const() const {
        return boost::static_pointer_cast<CollisionCheckerBase const>(shared_from_this());
    }

private:
    virtual const char* GetHash() const {
        return OPENRAVE_COLLISIONCHECKER_HASH;
    }

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
#else
    friend class ::Environment;
#endif
#endif
    friend class KinBody;
};

/// \brief Helper class to save and restore the collision options. If options are not supported and required is true, throws an exception.
class OPENRAVE_API CollisionOptionsStateSaver
{
public:
    CollisionOptionsStateSaver(CollisionCheckerBasePtr p, int newoptions, bool required=true);
    virtual ~CollisionOptionsStateSaver();
private:
    int _oldoptions;     ///< saved options
    CollisionCheckerBasePtr _p;
};

typedef boost::shared_ptr<CollisionOptionsStateSaver> CollisionOptionsStateSaverPtr;

} // end namespace OpenRAVE

#endif
