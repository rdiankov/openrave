// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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

#include <openrave/openravejson.h>

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
    CO_ActiveDOFs = 0x10,
    CO_AllLinkCollisions = 0x20, ///< if set then all the link collisions will be returned inside CollisionReport::vLinkColliding. Collision is slower because more pairs have to be checked.
    CO_AllGeometryCollisions = 0x40, ///< if set, then will return the collisions of all the colliding geometries. Do not need to explore all pairs of links once the first pair is found. This option can be slow.
    CO_AllGeometryContacts = 0x80, ///< if set, then will return the contact points of all the colliding geometries. Do not need to explore all pairs of links once the first pair is found. This option can be slow.

    CO_IgnoreCallbacks = 0x100 ///< if set, then will not use the registered collision callbacks.
};

/// \brief action to perform whenever a collision is detected between objects
enum CollisionAction
{
    CA_DefaultAction = 0, ///< let the physics/collision engine resolve the action
    CA_Ignore = 1, ///< do nothing
};

class OPENRAVE_API CONTACT
{
public:
    CONTACT() {
    }
    CONTACT(const Vector &p, const Vector &n, dReal d) : pos(p), norm(n), depth(d) {
    }

    Vector pos;  ///< where the contact occured
    Vector norm; ///< the normals of the faces
    dReal depth = 0; ///< the penetration depth, positive means the surfaces are penetrating, negative means the surfaces are not colliding (used for distance queries)

    void SaveToJson(rapidjson::Value& rContact, rapidjson::Document::AllocatorType& alloc) const;
    void LoadFromJson(const rapidjson::Value& rContact);
};

class OPENRAVE_API CollisionPairInfo
{
public:
    inline void Reset() {
        bodyLinkGeom1Name.clear();
        bodyLinkGeom2Name.clear();
        contacts.clear();
    }
    
    void SaveToJson(rapidjson::Value& rCollisionPair, rapidjson::Document::AllocatorType& alloc) const;
    void LoadFromJson(const rapidjson::Value& rCollisionPair);

    /// \brief swaps the contents
    void Swap(CollisionPairInfo& rhs);
    
    /// \brief swaps the first and second geometries, also changes contacts
    void SwapFirstSecond();
    
    void SetFirstCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname);
    void SetSecondCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname);

    /// \brief extracts the first body name
    void ExtractFirstBodyName(string_view& bodyname) const;
    void ExtractSecondBodyName(string_view& bodyname) const;
    /// \brief extracts the first link name
    void ExtractFirstLinkName(string_view& linkname) const;
    void ExtractSecondLinkName(string_view& linkname) const;
    /// \brief extracts the first body and link names
    void ExtractFirstBodyLinkNames(string_view& bodyname, string_view& linkname) const;
    void ExtractSecondBodyLinkNames(string_view& bodyname, string_view& linkname) const;
    /// \brief extracts the first body, link, and geom names
    void ExtractFirstBodyLinkGeomNames(string_view& bodyname, string_view& linkname, string_view& geomname) const;
    void ExtractSecondBodyLinkGeomNames(string_view& bodyname, string_view& linkname, string_view& geomname) const;
    
    /// \brief compares the first body name with the passed in bodyname. return value is same as strncmp
    int CompareFirstBodyName(const std::string& bodyname) const;
    int CompareSecondBodyName(const std::string& bodyname) const;

    /// \brief return true if the first collision matches the link
    int CompareFirstLink(const KinBody::Link& link) const;
    int CompareSecondLink(const KinBody::Link& link) const;

    /// \brief compares the first link name with the passed in linkname. return value is same as strncmp
    int CompareFirstLinkName(const std::string& linkname) const;
    int CompareSecondLinkName(const std::string& linkname) const;

    /// \brief if first linkname matches that of any of the links passed in, then return the index (also checks body name). Return -1 if no matching
    ///
    /// \param vlinks list of links coming from the same kinbody
    int FindFirstMatchingLinkIndex(const std::vector<KinBody::LinkPtr>& vlinks) const;
    int FindSecondMatchingLinkIndex(const std::vector<KinBody::LinkPtr>& vlinks) const;

    /// \brief returns a pointer to the first body
    KinBodyPtr ExtractFirstBody(EnvironmentBase& env) const;
    KinBodyPtr ExtractSecondBody(EnvironmentBase& env) const;
    
    // to save on dynamic allocation, only have one string per body/link/geom. Given that names cannot have spaces, can separate with spaces
    std::string bodyLinkGeom1Name; ///< "bodyname linkname geomname" for first collision
    std::string bodyLinkGeom2Name; ///< "bodyname linkname geomname" for second collision

    std::vector<CONTACT> contacts; ///< the convention is that the normal will be "out" of pgeom1's surface. Filled if CO_UseContacts option is set.
};

/// \brief Holds information about a particular collision that occured. Keep the class non-virtual to allow c++ to optimize more
class OPENRAVE_API CollisionReport
{
public:
    CollisionReport();
    CollisionReport(const CollisionReport& rhs);

    // should be careful not to copy all of vCollisionInfos, only the valid ones
    CollisionReport& operator=(const CollisionReport& rhs);

    /// \brief swaps the contents
    void Swap(CollisionReport& rhs);
    
    /// \brief resets the report structure for the next collision call
    ///
    /// depending on nKeepPrevious will keep previous data.
    void Reset(int coloptions = 0);
    std::string __str__() const;

    inline bool IsValid() const {
        return nNumValidCollisions > 0;
    }

    void SaveToJson(rapidjson::Value& rCollisionReport, rapidjson::Document::AllocatorType& alloc) const;
    void LoadFromJson(const rapidjson::Value& rCollisionReport);

    /// \brief return index of the collision
    int AddCollision();
    int AddLinkCollision(const KinBody::Link& link1);
    int AddLinkCollision(const KinBody::Link& link1, const KinBody::Link& link2);
    int AddLinkGeomCollision(const KinBody::LinkConstPtr& plink1, const std::string& geomname1, const KinBody::LinkConstPtr& plink2, const std::string& geomname2);

    // set only one collision
    int SetLinkGeomCollision(const KinBody::LinkConstPtr& plink1, const std::string& geomname1, const KinBody::LinkConstPtr& plink2, const std::string& geomname2);

    std::vector<CollisionPairInfo> vCollisionInfos; ///< all geometry collision pairs. Set when CO_AllGeometryCollisions or CO_AllLinkCollisions or CO_AllGeometryContacts is enabled. The size of the array is not indicative of how many valid collisions there are! See nNumValidCollisions instead. Due to caching and memory constraints, should not resize this vector, instead change nNumValidCollisions
    int nNumValidCollisions = 0; ///< how many infos are valid in vCollisionInfos
    int options = 0; ///< mix of CO_X, the options that the CollisionReport was called with. It is overwritten by the options set on the collision checker writing the report

    dReal minDistance = 1e20; ///< minimum distance from last query, filled if CO_Distance option is set
    int16_t numWithinTol = 0; ///< number of objects within tolerance of this object, filled if CO_UseTolerance option is set
    uint8_t nKeepPrevious = 0; ///< if 1, will keep all previous data when resetting the collision checker. otherwise will reset
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

    /// \brief Sets the geometry group that the collision checker will prefer to use (if present)
    ///
    /// Will also set the default geometry groups used for any new bodies added to the scene. groupname the geometry group name. If empty, will disable the groups and use the current geometries set on the link.
    virtual void SetGeometryGroup(const std::string& groupname) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Gets the geometry group this collision checker is tracking.
    ///
    /// If empty, collision checker is not tracking any specific groups.
    virtual const std::string& GetGeometryGroup() const OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Sets the geometry group that the collision checker will prefer to use for a body
    ///
    /// \param pbody the body to change the geometry group
    /// \param groupname the geometry group name. If empty, will disable the groups and use the current geometries set on the link.
    /// \return true if body geometry group with groupname exists. false otherwise
    virtual bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \biref Gets the geometry group that a body is currently using
    virtual const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const OPENRAVE_DUMMY_IMPLEMENTATION;

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
    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief checks collision between two bodies. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of the pbody1.
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief checks collision of a link and a scene. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief checks collision of two links. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief checks collision of a link and a body. Attached bodies for pbody are respected. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief checks collision of a link and a scene. Attached bodies are ignored. CO_ActiveDOFs option is ignored.
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief checks collision of a body and a scene. Attached bodies are respected. If CO_ActiveDOFs is set, will only check affected links of pbody.
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) = 0;

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

    /// \brief Check collision with a triangle mesh and a body in the scene.
    ///
    /// \param trimesh Holds a dynamic triangle mesh to check collision with the body.
    /// \param pbody the link to collide with. If CO_ActiveDOFs is set, will only check affected links of the body.
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Check collision with a triangle mesh and the entire scene
    ///
    /// \param trimesh Holds a dynamic triangle mesh to check collision with the body.
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const TriMesh& trimesh, CollisionReportPtr report = CollisionReportPtr()) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Check collision with a dummy box and the entire scene
    ///
    /// \param ab box to check collision with. The box is transformed by aabbPose
    /// \param aabbPose the pose of the box
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const AABB& ab, const Transform& aabbPose, CollisionReportPtr report = CollisionReportPtr()) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Check collision with a dummy box and a list of bodies
    ///
    /// \param ab box to check collision with. The box is transformed by aabbPose
    /// \param aabbPose the pose of the box
    /// \param bodies vector of bodies to check collision with the dummy AABB
    /// \param[out] report [optional] collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const AABB& ab, const Transform& aabbPose, const std::vector<KinBodyConstPtr>& vbodies, CollisionReportPtr report = CollisionReportPtr()) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Checks self collision only with the links of the passed in body.
    ///
    /// Only checks KinBody::GetNonAdjacentLinks(), Links that are joined together are ignored.
    /// \param[in] pbody The body to check self-collision for
    /// \param[out] report [optional] collision report to be filled with data about the collision.
    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \brief Checks self collision of the link with the rest of the links with its parent
    ///
    /// Only checks KinBody::GetNonAdjacentLinks(), Links that are joined together are ignored.
    /// \param[in] plink, vIncludedLinks : The pointer of target link and vector of included links. Among the link pairs in GetNonAdjacentLinks, this API only checks the collisions with the pairs so that one of the link of the pair is plink, and the other link of the pair is included in vIncludedLinks. If vIncludedLinks is empty, it's specially treated as no limitation for the included links, e.g. this API checks all possible pairs if one link of the pair is plink.
    /// \param[out] report [optional] collision report to be filled with data about the collision.
    virtual bool CheckStandaloneSelfCollision(KinBody::LinkConstPtr plink, const std::vector<KinBody::LinkConstPtr>& vIncludedLinks = std::vector<KinBody::LinkConstPtr>(), CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \deprecated (13/04/09)
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) RAVE_DEPRECATED
    {
        //RAVELOG_WARN("CollisionCheckerBase::CheckSelfCollision has been deprecated, please use CollisionCheckerBase::CheckStandaloneSelfCollision\n");
        return CheckStandaloneSelfCollision(pbody,report);
    }

    /// \deprecated (13/04/09)
    virtual bool CheckSelfCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) RAVE_DEPRECATED
    {
        //RAVELOG_WARN("CollisionCheckerBase::CheckSelfCollision has been deprecated, please use CollisionCheckerBase::CheckStandaloneSelfCollision\n");
        return CheckStandaloneSelfCollision(plink, std::vector<KinBody::LinkConstPtr>(), report);
    }

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

    /// \brief check if the caller should skip the given standalone self collision check pair.
    /// \param[in] targetLink, vIncludedLinks : target link to check and vector of included links.
    ///                                         - if index1 and index2 are not same as targetLink's index, should skip.
    ///                                         - if vIncludedLinks is empty, no check for another link in the pair and the given pair should be checked with collision check.
    ///                                         - if vIncludedLinks is not empty, if another link in the pair is not included in vIncludedLinks, such pair should be skipped with collision check.
    /// \param[in] index1, index2 : link indices of the pair.
    /// \param[in] pBody : kinbody to check self collision
    /// \return false if the given pair (index1 and index2) should be checked with standalone self collision checking. true if it should be skipped.
    static inline bool _ShouldSkipStandaloneSelfCollisionCheckPair(const KinBody::Link& taretLink, const size_t index1, const size_t index2, const KinBodyPtr& pBody, const std::vector<KinBody::LinkConstPtr>& vIncludedLinks)
    {
        const size_t targetLinkIndex = taretLink.GetIndex();
        if( targetLinkIndex == index1 ) {
            if( vIncludedLinks.size() > 0 && std::find(vIncludedLinks.begin(), vIncludedLinks.end(), pBody->GetLinks().at(index2)) == vIncludedLinks.end() ) {
                return true;
            }
        }
        else if( targetLinkIndex == index2 ) {
            if( vIncludedLinks.size() > 0 && std::find(vIncludedLinks.begin(), vIncludedLinks.end(), pBody->GetLinks().at(index1)) == vIncludedLinks.end() ) {
                return true;
            }
        }
        else {
            return true;
        }
        return false;
    }

private:
    virtual const char* GetHash() const override {
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

/** \brief Helper class to save and restore the nKeepPrevious variable in a collision report. Should be used by anyone using multiple CheckCollision calls and aggregating results.

    Sample code would look like:

    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
 */
class OPENRAVE_API CollisionReportKeepSaver
{
public:
    CollisionReportKeepSaver(CollisionReportPtr preport) : _report(preport) {
        if( !!preport ) {
            _nKeepPrevious = preport->nKeepPrevious;
        }
        else {
            _nKeepPrevious = 0; // keep here otherwise compiler warning
        }
    }
    virtual ~CollisionReportKeepSaver() {
        if( !!_report ) {
            _report->nKeepPrevious = _nKeepPrevious;
        }
    }

private:
    CollisionReportPtr _report;
    uint8_t _nKeepPrevious;
};

typedef boost::shared_ptr<CollisionReportKeepSaver> CollisionReportKeepSaverPtr;

} // end namespace OpenRAVE

#endif
