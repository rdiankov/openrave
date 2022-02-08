// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_SPACE
#define OPENRAVE_FCL_SPACE

#include <boost/shared_ptr.hpp>
#include <memory> // c++11
#include <vector>

namespace fclrave {

typedef KinBody::LinkConstPtr LinkConstPtr;
typedef std::pair<LinkConstPtr, LinkConstPtr> LinkPair;
typedef boost::weak_ptr<const KinBody> KinBodyConstWeakPtr;
typedef KinBody::GeometryConstPtr GeometryConstPtr;
typedef boost::weak_ptr<KinBody::Geometry> GeometryWeakPtr;
using OpenRAVE::ORE_Assert;

// Warning : this is the only place where we use std::shared_ptr (for compatibility with fcl)
typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef boost::function<CollisionGeometryPtr (std::vector<fcl::Vec3f> const &points, std::vector<fcl::Triangle> const &triangles) > MeshFactory;
typedef std::vector<fcl::CollisionObject *> CollisionGroup;
typedef boost::shared_ptr<CollisionGroup> CollisionGroupPtr;
typedef std::pair<Transform, CollisionObjectPtr> TransformCollisionPair;


// Helper functions for conversions from OpenRAVE to FCL

Vector ConvertVectorFromFCL(fcl::Vec3f const &v)
{
    return Vector(v[0], v[1], v[2]);
}

fcl::Vec3f ConvertVectorToFCL(Vector const &v)
{
    return fcl::Vec3f(v.x, v.y, v.z);
}
fcl::Quaternion3f ConvertQuaternionToFCL(Vector const &v)
{
    return fcl::Quaternion3f(v[0], v[1], v[2], v[3]);
}

Vector ConvertQuaternionFromFCL(fcl::Quaternion3f const &v) {
    return Vector(v.getW(), v.getX(), v.getY(), v.getZ());
}

fcl::AABB ConvertAABBToFcl(const OpenRAVE::AABB& bv) {
    return fcl::AABB(fcl::AABB(ConvertVectorToFCL(bv.pos)), ConvertVectorToFCL(bv.extents));
}


template <class T>
CollisionGeometryPtr ConvertMeshToFCL(std::vector<fcl::Vec3f> const &points,std::vector<fcl::Triangle> const &triangles)
{
    std::shared_ptr< fcl::BVHModel<T> > const model = make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}


/// \brief ensures vector size is at least size
template <typename T>
inline void EnsureVectorSize(std::vector<T>& vec, size_t size)
{
    if (vec.size() < size) {
        //RAVELOG_WARN_FORMAT("resizing 0x%x from %d to %d", &vec%(vec.size())%(index + 1));
        vec.resize(size);
    }
}

/// \brief fcl spaces manages the individual collision objects and sets up callbacks to track their changes.
///
/// It does not know or manage the broadphase manager
class FCLSpace : public boost::enable_shared_from_this<FCLSpace>
{
public:
    inline boost::weak_ptr<FCLSpace> weak_space() {
        return shared_from_this();
    }

    // corresponds to FCLUserData
    class FCLKinBodyInfo : public boost::enable_shared_from_this<FCLKinBodyInfo>, public OpenRAVE::UserData
    {
public:
        class FCLGeometryInfo
        {
public:
            FCLGeometryInfo() : bFromKinBodyGeometry(false) {
            }

            FCLGeometryInfo(KinBody::GeometryPtr pgeom) : _pgeom(pgeom), bFromKinBodyGeometry(true) {
            }

            virtual ~FCLGeometryInfo() {
            }

            inline KinBody::GeometryPtr GetGeometry() {
                return _pgeom.lock();
            }

            GeometryWeakPtr _pgeom;
            std::string bodylinkgeomname; // for debugging purposes
            bool bFromKinBodyGeometry; ///< if true, then from kinbodygeometry. Otherwise from standalone object that does not have any KinBody associations
        };

        class LinkInfo
        {
public:
            LinkInfo() : bFromKinBodyLink(false) {
            }
            LinkInfo(KinBody::LinkPtr plink) : _plink(plink), bFromKinBodyLink(true) {
            }

            virtual ~LinkInfo() {
                Reset();
            }

            void Reset() {
                if( !!linkBV.second ) {
                    if( !!GetLink() ) {
                        RAVELOG_VERBOSE_FORMAT("env=%s, resetting link %s:%s col=0x%x", GetLink()->GetParent()->GetEnv()->GetNameId()%GetLink()->GetParent()->GetName()%GetLink()->GetName()%(uint64_t)linkBV.second.get());
                    }
                    else {
                        RAVELOG_VERBOSE_FORMAT("resetting unknown link col=0x%x", (uint64_t)linkBV.second.get());
                    }
                    linkBV.second->setUserData(nullptr); // reset the user data since someone can hold a ref to the collision object and continue using it
                }
                linkBV.second.reset();

                FOREACH(itgeompair, vgeoms) {
                    (*itgeompair).second->setUserData(nullptr);
                    (*itgeompair).second.reset();
                }
                vgeoms.resize(0);

                // make sure to clear vgeominfos after vgeoms because the CollisionObject inside each vgeom element has a corresponding vgeominfo as a void pointer.
                vgeominfos.resize(0);
            }

            inline KinBody::LinkPtr GetLink() {
                return _plink.lock();
            }

            KinBody::LinkWeakPtr _plink;
            vector< boost::shared_ptr<FCLGeometryInfo> > vgeominfos; ///< info for every geometry of the link

            //int nLastStamp; ///< Tracks if the collision geometries are up to date wrt the body update stamp. This is for narrow phase collision
            TransformCollisionPair linkBV; ///< pair of the transformation and collision object corresponding to a bounding OBB for the link
            std::vector<TransformCollisionPair> vgeoms; ///< vector of transformations and collision object; one per geometries
            std::string bodylinkname; // for debugging purposes
            bool bFromKinBodyLink; ///< if true, then from kinbodylink. Otherwise from standalone object that does not have any KinBody associations
        };

        FCLKinBodyInfo() : nLastStamp(0), nLinkUpdateStamp(0), nGeometryUpdateStamp(0), nAttachedBodiesUpdateStamp(0), nActiveDOFUpdateStamp(0)
        {
        }

        virtual ~FCLKinBodyInfo() {
            Reset();
        }

        void Reset()
        {
            FOREACH(itlink, vlinks) {
                (*itlink)->Reset();
            }
            vlinks.resize(0);
            _geometrycallback.reset();
            _geometrygroupcallback.reset();
            _linkenablecallback.reset();
        }

        KinBodyPtr GetBody()
        {
            return _pbody.lock();
        }

        KinBodyWeakPtr _pbody;
        int nLastStamp;  ///< KinBody::GetUpdateStamp() when last synchronized ("is transform up to date")
        int nLinkUpdateStamp; ///< update stamp for link enable state (increases every time link enables change)
        int nGeometryUpdateStamp; ///< update stamp for geometry update state (increases every time geometry enables change)
        int nAttachedBodiesUpdateStamp; ///< update stamp for when attached bodies change of this body
        int nActiveDOFUpdateStamp; ///< update stamp for when active dofs change of this body

        vector< boost::shared_ptr<LinkInfo> > vlinks; ///< info for every link of the kinbody

        OpenRAVE::UserDataPtr _bodyAttachedCallback; ///< handle for the callback called when a body is attached or detached
        OpenRAVE::UserDataPtr _activeDOFsCallback; ///< handle for the callback called when a the activeDOFs have changed
        std::list<OpenRAVE::UserDataPtr> _linkEnabledCallbacks;

        OpenRAVE::UserDataPtr _geometrycallback; ///< handle for the callback called when the current geometry of the kinbody changed ( Prop_LinkGeometry )
        OpenRAVE::UserDataPtr _geometrygroupcallback; ///< handle for the callback called when some geometry group of one of the links of this kinbody changed ( Prop_LinkGeometryGroup )
        OpenRAVE::UserDataPtr _linkenablecallback; ///< handle for the callback called when some link enable status of this kinbody has changed so that the envManager is updated ( Prop_LinkEnable )
        OpenRAVE::UserDataPtr _bodyremovedcallback; ///< handle for the callback called when the kinbody is removed from the environment, used in self-collision checkers ( Prop_BodyRemoved )

        std::string _geometrygroup; ///< name of the geometry group tracked by this kinbody info ; if empty, tracks the current geometries
    };

    typedef boost::shared_ptr<FCLKinBodyInfo> FCLKinBodyInfoPtr;
    typedef boost::shared_ptr<FCLKinBodyInfo const> FCLKinBodyInfoConstPtr;
    typedef boost::weak_ptr<FCLKinBodyInfo> FCLKinBodyInfoWeakPtr;
    typedef boost::shared_ptr<FCLSpace::FCLKinBodyInfo::LinkInfo> LinkInfoPtr;
    typedef boost::function<void (FCLKinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : _penv(penv), _userdatakey(userdatakey),
        _currentpinfo(1, FCLKinBodyInfoPtr()), // initialize with one null pointer, this is a place holder for null pointer so that we can return by reference. env id 0 means invalid so it's consistent with the definition as well
        _bIsSelfCollisionChecker(true)
    {

        // After many test, OBB seems to be the only real option (followed by kIOS which is needed for distance checking)
        SetBVHRepresentation("OBB");
    }

    virtual ~FCLSpace()
    {
        DestroyEnvironment();
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE_FORMAT("destroying fcl collision environment (env %d) (userdatakey %s)", _penv->GetId()%_userdatakey);
        for (KinBodyConstPtr pbody : _vecInitializedBodies) {
            if (!pbody) {
                continue;
            }
            FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
        }
        // even after DestroyEnvironment is called, users of this class still try to access _currentpinfo
        // in that case, null pointer should be returned, instead of range error from _currentpinfo.at(0). for that purpose, keep the first element here.
        _currentpinfo.erase(_currentpinfo.begin() + 1, _currentpinfo.end());
        _cachedpinfo.clear();
        _vecInitializedBodies.clear();
    }

    FCLKinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, FCLKinBodyInfoPtr pinfo = FCLKinBodyInfoPtr(), bool bSetToCurrentPInfo=true)
    {
        if( !pinfo ) {
            pinfo.reset(new FCLKinBodyInfo());
            pinfo->_geometrygroup = _geometrygroup;
        }

        RAVELOG_VERBOSE_FORMAT("env=%s, self=%d, init body %s (%d)", _penv->GetNameId()%_bIsSelfCollisionChecker%pbody->GetName()%pbody->GetEnvironmentBodyIndex());
        pinfo->Reset();
        pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
        // make sure that synchronization do occur !
        pinfo->nLastStamp = pbody->GetUpdateStamp() - 1;

        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
            const KinBody::LinkPtr& plink = *itlink;
            boost::shared_ptr<FCLKinBodyInfo::LinkInfo> linkinfo(new FCLKinBodyInfo::LinkInfo(plink));


            fcl::AABB enclosingBV;

            // Glue code for a unified access to geometries
            if(pinfo->_geometrygroup.size() > 0 && plink->GetGroupNumGeometries(pinfo->_geometrygroup) >= 0) {
                const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = plink->GetGeometriesFromGroup(pinfo->_geometrygroup);
                FOREACH(itgeominfo, vgeometryinfos) {
                    const KinBody::GeometryInfoPtr& pgeominfo = *itgeominfo;
                    if( !pgeominfo ) {
                        int igeominfo = itgeominfo - vgeometryinfos.begin();
                        throw OpenRAVE::OpenRAVEException(str(boost::format("Failed to access geometry info %d for link %s:%s with geometrygroup %s")%igeominfo%plink->GetParent()->GetName()%plink->GetName()%pinfo->_geometrygroup), OpenRAVE::ORE_InvalidState);
                    }
                    const KinBody::GeometryInfo& geominfo = *pgeominfo;
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, geominfo);

                    if( !pfclgeom ) {
                        continue;
                    }
                    pfclgeom->setUserData(nullptr);

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                    pfclcoll->setUserData(linkinfo.get());
                    linkinfo->vgeoms.push_back(TransformCollisionPair(geominfo.GetTransform(), pfclcoll));

                    KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), geominfo);
                    if( itgeominfo == vgeometryinfos.begin() ) {
                        enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                    else {
                        enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                }
            }
            else {
                const std::vector<KinBody::Link::GeometryPtr> & vgeometries = plink->GetGeometries();
                FOREACH(itgeom, vgeometries) {
                    const KinBody::GeometryPtr& pgeom = *itgeom;
                    const KinBody::GeometryInfo& geominfo = pgeom->GetInfo();
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, geominfo);

                    if( !pfclgeom ) {
                        continue;
                    }
                    boost::shared_ptr<FCLKinBodyInfo::FCLGeometryInfo> pfclgeominfo(new FCLKinBodyInfo::FCLGeometryInfo(pgeom));
                    pfclgeominfo->bodylinkgeomname = pbody->GetName() + "/" + plink->GetName() + "/" + pgeom->GetName();
                    pfclgeom->setUserData(pfclgeominfo.get());
                    // save the pointers
                    linkinfo->vgeominfos.push_back(pfclgeominfo);

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                    pfclcoll->setUserData(linkinfo.get());

                    linkinfo->vgeoms.push_back(TransformCollisionPair(geominfo.GetTransform(), pfclcoll));

                    KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), geominfo);
                    if( itgeom == vgeometries.begin() ) {
                        enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                    else {
                        enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                }
            }

            if( linkinfo->vgeoms.size() == 0 ) {
                RAVELOG_DEBUG_FORMAT("env=%s, Initializing body '%s' (envBodyIndex=%d) link '%s' with 0 geometries (env %d) (userdatakey %s)", _penv->GetNameId()%pbody->GetName()%pbody->GetEnvironmentBodyIndex()%plink->GetName()%_penv->GetId()%_userdatakey);
            }
            else {
                CollisionGeometryPtr pfclgeomBV = std::make_shared<fcl::Box>(enclosingBV.max_ - enclosingBV.min_);
                pfclgeomBV->setUserData(nullptr);
                CollisionObjectPtr pfclcollBV = boost::make_shared<fcl::CollisionObject>(pfclgeomBV);
                Transform trans(Vector(1,0,0,0),ConvertVectorFromFCL(0.5 * (enclosingBV.min_ + enclosingBV.max_)));
                pfclcollBV->setUserData(linkinfo.get());
                linkinfo->linkBV = std::make_pair(trans, pfclcollBV);
            }

            //link->nLastStamp = pinfo->nLastStamp;
            linkinfo->bodylinkname = pbody->GetName() + "/" + plink->GetName();
            pinfo->vlinks.push_back(linkinfo);
#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
            RAVELOG_DEBUG_FORMAT("FCLSPACECOLLISIONOBJECT|%s|%s", linkinfo->linkBV.second.get()%linkinfo->bodylinkname);
#endif
        }

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetCurrentGeometryCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
        pinfo->_geometrygroupcallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometryGroup, boost::bind(&FCLSpace::_ResetGeometryGroupsCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
        pinfo->_linkenablecallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&FCLSpace::_ResetLinkEnableCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
        pinfo->_activeDOFsCallback = pbody->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs, boost::bind(&FCLSpace::_ResetActiveDOFsCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<FCLKinBodyInfo>(pinfo)));

        pinfo->_bodyAttachedCallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyAttached, boost::bind(&FCLSpace::_ResetAttachedBodyCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
        pinfo->_bodyremovedcallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyRemoved, boost::bind(&FCLSpace::RemoveUserData, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::bind(&OpenRAVE::utils::sptr_from<const KinBody>, boost::weak_ptr<const KinBody>(pbody))));

        const int envId = pbody->GetEnvironmentBodyIndex();
        const int maxEnvId = _penv->GetMaxEnvironmentBodyIndex();
        BOOST_ASSERT(envId != 0);
        if( bSetToCurrentPInfo ) {
            EnsureVectorSize(_currentpinfo, maxEnvId + 1);
            _currentpinfo.at(envId) = pinfo;
        }
        //_cachedpinfo[pbody->GetEnvironmentBodyIndex()] what to do with the cache?
        EnsureVectorSize(_vecInitializedBodies, maxEnvId + 1);
        _vecInitializedBodies.at(envId) = pbody;

        //Do I really need to synchronize anything at that point ?
        _Synchronize(*pinfo, *pbody);

        return pinfo;
    }

    bool HasNamedGeometry(const KinBody &body, const std::string& groupname) {
        // The empty string corresponds to current geometries so all kinbodies have it
        if( groupname.size() == 0 ) {
            return true;
        }
        FOREACH(itlink, body.GetLinks()) {
            if( (*itlink)->GetGroupNumGeometries(groupname) >= 0 ) {
                return true;
            }
        }
        return false;
    }


    void SetGeometryGroup(const std::string& groupname)
    {
        // should always do this since bodies can have different geometry groups set
        _geometrygroup = groupname;
        for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
            if (!pbody) {
                continue;
            }
            SetBodyGeometryGroup(pbody, groupname);
        }
    }

    const std::string& GetGeometryGroup() const
    {
        return _geometrygroup;
    }


    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
        const KinBody& body = *pbody;

        if (!HasNamedGeometry(body, groupname)) {
            return false;
        }

        // Save the already existing FCLKinBodyInfoPtr for the old geometry group
        FCLKinBodyInfoPtr& poldinfo = GetInfo(body);
        if( poldinfo->_geometrygroup == groupname ) {
            return true;
        }

        poldinfo->nGeometryUpdateStamp += 1;

        const int maxBodyIndex = _penv->GetMaxEnvironmentBodyIndex();
        EnsureVectorSize(_cachedpinfo, maxBodyIndex + 1);

        const int bodyIndex = body.GetEnvironmentBodyIndex();
        OPENRAVE_ASSERT_OP_FORMAT(bodyIndex, !=, 0, "env=%s, body %s", _penv->GetNameId()%body.GetName(), OpenRAVE::ORE_InvalidState);
        std::map< std::string, FCLKinBodyInfoPtr >& cache = _cachedpinfo.at(bodyIndex);
        cache[poldinfo->_geometrygroup] = poldinfo;

        FCLKinBodyInfoPtr& pinfo = cache[groupname];
        if(!pinfo) {
            RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%body.GetName()%bodyIndex%_penv->GetId());
            pinfo.reset(new FCLKinBodyInfo);
            pinfo->_geometrygroup = groupname;
            InitKinBody(pbody, pinfo);
        }
        else {
            RAVELOG_VERBOSE_FORMAT("env=%s, switching to geometry %s for kinbody %s (id = %d)", _penv->GetNameId()%groupname%body.GetName()%bodyIndex);
            // Set the current info to use the FCLKinBodyInfoPtr associated to groupname
            EnsureVectorSize(_currentpinfo, maxBodyIndex + 1);
            _currentpinfo.at(bodyIndex) = pinfo;

            // Revoke the information inside the cache so that a potentially outdated object does not survive
            cache.erase(groupname);
        }

        return true;
    }

    const std::string& GetBodyGeometryGroup(const KinBody &body) const {
        static const std::string empty;
        const FCLKinBodyInfoPtr& pinfo = GetInfo(body);
        if( !!pinfo ) {
            return pinfo->_geometrygroup;
        } else {
            return empty;
        }
    }

    // Set the current bvhRepresentation and reinitializes all the KinbodyInfo if needed
    void SetBVHRepresentation(std::string const &type)
    {
        if( type == _bvhRepresentation ) {
            return;
        }

        if (type == "AABB") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL<fcl::AABB>;
        } else if (type == "OBB") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL<fcl::OBB>;
        } else if (type == "RSS") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL<fcl::RSS>;
        } else if (type == "OBBRSS") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL<fcl::OBBRSS>;
        } else if (type == "kDOP16") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL< fcl::KDOP<16> >;
        } else if (type == "kDOP18") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL< fcl::KDOP<18> >;
        } else if (type == "kDOP24") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL< fcl::KDOP<24> >;
        } else if (type == "kIOS") {
            _bvhRepresentation = type;
            _meshFactory = &ConvertMeshToFCL<fcl::kIOS>;
        } else {
            RAVELOG_WARN(str(boost::format("Unknown BVH representation '%s', keeping '%s' representation") % type % _bvhRepresentation));
            return;
        }

        // reinitialize all the FCLKinBodyInfo

        for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
            if (!pbody) {
                continue;
            }
            const KinBody& body = *pbody;
            FCLKinBodyInfoPtr& pinfo = GetInfo(body);
            pinfo->nGeometryUpdateStamp++;
            InitKinBody(pbody, pinfo);
        }
        _cachedpinfo.clear();
    }

    std::string const& GetBVHRepresentation() const {
        return _bvhRepresentation;
    }


    void Synchronize()
    {
        // We synchronize only the initialized bodies, which differs from oderave
        for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
            if (!pbody) {
                continue;
            }
            Synchronize(*pbody);
        }
    }

    void Synchronize(const KinBody &body)
    {
        FCLKinBodyInfoPtr& pinfo = GetInfo(body);
        if( !pinfo ) {
            return;
        }
        // expensive, comment out for now
        //BOOST_ASSERT( pinfo->_pbody.lock().get() == &body);
        _Synchronize(*pinfo, body);
    }

    void SynchronizeWithAttached(const KinBody &body)
    {
        if( body.HasAttached() ) {
            std::vector<int>& vecAttachedEnvBodyIndices = _vecAttachedEnvBodyIndicesCache;
            body.GetAttachedEnvironmentBodyIndices(vecAttachedEnvBodyIndices);
            std::vector<KinBodyPtr>& attachedBodies = _vecAttachedBodiesCache;
            _penv->GetBodiesFromEnvironmentBodyIndices(vecAttachedEnvBodyIndices, attachedBodies);
            for (const KinBodyPtr& pattachedBody : attachedBodies) {
                if (!!pattachedBody) {
                    Synchronize(*pattachedBody);
                }
            }
        }
        else {
            Synchronize(body);
        }
    }

    inline FCLKinBodyInfoPtr& GetInfo(const KinBody &body)
    {
        int envId = body.GetEnvironmentBodyIndex();
        if ( envId <= 0 ) {
            RAVELOG_WARN_FORMAT("env=%s, body %s has invalid environment id %d", _penv->GetNameId()%body.GetName()%envId);
        }
        else {
            if (envId < (int) _currentpinfo.size()) {
                return _currentpinfo.at(envId);
            }
        }

        return _currentpinfo.at(0); // invalid
    }

    inline const FCLKinBodyInfoPtr& GetInfo(const KinBody &body) const
    {
        int envId = body.GetEnvironmentBodyIndex();
        if ( envId <= 0 ) {
            RAVELOG_WARN_FORMAT("env=%s, body %s has invalid environment id %d", _penv->GetNameId()%body.GetName()%envId);
        }
        else {
            if (envId < (int) _currentpinfo.size()) {
                return _currentpinfo.at(envId);
            }
        }

        return _currentpinfo.at(0);
    }


    void RemoveUserData(KinBodyConstPtr pbody) {
        if( !!pbody ) {
            RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d (userdatakey %s) : %s") % _penv->GetId() % _userdatakey % pbody->GetName()));
            const int envId = pbody->GetEnvironmentBodyIndex();
            if (envId < (int) _vecInitializedBodies.size()) {
                _vecInitializedBodies.at(envId).reset();
            }
            FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }

            if( envId == 0 ) {
                RAVELOG_WARN_FORMAT("env=%s, body '%s' has bodyIndex=0, so not adding to the environment!", _penv->GetNameId()%pbody->GetName());
            }

            if (envId < (int) _currentpinfo.size()) {
                _currentpinfo.at(envId).reset();
                //RAVELOG_INFO_FORMAT("erased %d but didn't pop back, size is %d", envId%_currentpinfo.size());
            }
            if (envId < (int) _cachedpinfo.size()) {
                _cachedpinfo.at(envId).clear();
            }
        }
    }


    /// \brief returns bodies initialized by this space. Note that some entries are null pointer.
    const std::vector<KinBodyConstPtr>& GetEnvBodies() const {
        return _vecInitializedBodies;
    }

    inline CollisionObjectPtr GetLinkBV(const KinBody::Link &link) {
        return GetLinkBV(*link.GetParent(), link.GetIndex());
    }

    inline CollisionObjectPtr GetLinkBV(const KinBody &body, int index) {
        FCLKinBodyInfoPtr& pinfo = GetInfo(body);
        if( !!pinfo ) {
            return GetLinkBV(*pinfo, index);
        } else {
            RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%body.GetName()%_userdatakey%_penv->GetId()));
            return CollisionObjectPtr();
        }
    }

    inline CollisionObjectPtr GetLinkBV(const FCLKinBodyInfo &info, int index) {
        return info.vlinks.at(index)->linkBV.second;
    }


    inline LinkInfoPtr GetLinkInfo(const KinBody::Link &link) {
        return GetInfo(*link.GetParent())->vlinks.at(link.GetIndex());
    }

    inline void SetIsSelfCollisionChecker(bool bIsSelfCollisionChecker)
    {
        _bIsSelfCollisionChecker = bIsSelfCollisionChecker;
    }

    inline bool IsSelfCollisionChecker() const
    {
        return _bIsSelfCollisionChecker;
    }

    inline const MeshFactory& GetMeshFactory() const {
        return _meshFactory;
    }

    inline int GetEnvironmentId() const {
        return _penv->GetId();
    }
private:
    static void _AddGeomInfoToBVHSubmodel(fcl::BVHModel<fcl::OBB>& model, KinBody::GeometryInfo const &info)
    {
        const OpenRAVE::TriMesh& mesh = info._meshcollision;
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            return;
        }

        OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
        size_t const num_points = mesh.vertices.size();
        size_t const num_triangles = mesh.indices.size() / 3;

        std::vector<fcl::Vec3f> fcl_points(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = info.GetTransform()*mesh.vertices[ipoint];
            fcl_points[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
        }

        std::vector<fcl::Triangle> fcl_triangles(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &mesh.indices[3 * itri];
            fcl_triangles[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }
        model.addSubModel(fcl_points, fcl_triangles);
    }

    static TransformCollisionPair _CreateTransformCollisionPairFromOBB(fcl::OBB const &bv) {
        CollisionGeometryPtr pbvGeom = make_shared<fcl::Box>(bv.extent[0]*2.0f, bv.extent[1]*2.0f, bv.extent[2]*2.0f);
        CollisionObjectPtr pbvColl = boost::make_shared<fcl::CollisionObject>(pbvGeom);
        fcl::Quaternion3f fclBvRot;
        fclBvRot.fromAxes(bv.axis);
        Vector bvRotation = ConvertQuaternionFromFCL(fclBvRot);
        Vector bvTranslation = ConvertVectorFromFCL(bv.center());

        return std::make_pair(Transform(bvRotation, bvTranslation), pbvColl);
    }

    // what about the tests on non-zero size (eg. box extents) ?
    static CollisionGeometryPtr _CreateFCLGeomFromGeometryInfo(const MeshFactory &mesh_factory, const KinBody::GeometryInfo &info)
    {
        switch(info._type) {

        case OpenRAVE::GT_None:
            return CollisionGeometryPtr();

        case OpenRAVE::GT_CalibrationBoard:
        case OpenRAVE::GT_Box:
            return make_shared<fcl::Box>(info._vGeomData.x*2.0f,info._vGeomData.y*2.0f,info._vGeomData.z*2.0f);

        case OpenRAVE::GT_Sphere:
            return make_shared<fcl::Sphere>(info._vGeomData.x);

        case OpenRAVE::GT_Cylinder:
            return make_shared<fcl::Cylinder>(info._vGeomData.x, info._vGeomData.y);

        case OpenRAVE::GT_Container:
        case OpenRAVE::GT_TriMesh:
        case OpenRAVE::GT_Cage:
        {
            const OpenRAVE::TriMesh& mesh = info._meshcollision;
            if (mesh.vertices.empty() || mesh.indices.empty()) {
                return CollisionGeometryPtr();
            }

            OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
            size_t const num_points = mesh.vertices.size();
            size_t const num_triangles = mesh.indices.size() / 3;

            std::vector<fcl::Vec3f> fcl_points(num_points);
            for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
                Vector v = mesh.vertices[ipoint];
                fcl_points[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
            }

            std::vector<fcl::Triangle> fcl_triangles(num_triangles);
            for (size_t itri = 0; itri < num_triangles; ++itri) {
                int const *const tri_indices = &mesh.indices[3 * itri];
                fcl_triangles[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
            }

            return mesh_factory(fcl_points, fcl_triangles);
        }

        default:
            RAVELOG_WARN(str(boost::format("FCL doesn't support geom type %d")%info._type));
            return CollisionGeometryPtr();
        }
    }

    /// \brief pass in info.GetBody() as a reference to avoid dereferencing the weak pointer in FCLKinBodyInfo
    void _Synchronize(FCLKinBodyInfo& info, const KinBody& body)
    {
        //KinBodyPtr pbody = info.GetBody();
        if( info.nLastStamp != body.GetUpdateStamp()) {
            info.nLastStamp = body.GetUpdateStamp();
            BOOST_ASSERT( body.GetLinks().size() == info.vlinks.size() );
            CollisionObjectPtr pcoll;
            for(size_t i = 0; i < body.GetLinks().size(); ++i) {
                const FCLSpace::FCLKinBodyInfo::LinkInfo& linkInfo = *info.vlinks[i];
                pcoll = linkInfo.linkBV.second;
                if( !pcoll ) {
                    continue;
                }
                const Transform& linkTransform = body.GetLinks()[i]->GetTransform();
                Transform pose = linkTransform * linkInfo.linkBV.first;
                fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                pcoll->setTranslation(newPosition);
                pcoll->setQuatRotation(newOrientation);
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                pcoll->computeAABB();

                //info.vlinks[i]->nLastStamp = info.nLastStamp;
                for (const TransformCollisionPair& pgeom : linkInfo.vgeoms) {
                    fcl::CollisionObject& coll = *pgeom.second;
                    Transform pose = linkTransform * pgeom.first;
                    fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                    fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                    coll.setTranslation(newPosition);
                    coll.setQuatRotation(newOrientation);
                    // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                    coll.computeAABB();
                }
            }

            // Does this have any use ?
            // if( !!_synccallback ) {
            //     _synccallback(pinfo);
            // }
        }
    }

    /// \brief controls whether the kinbody info is removed during the destructor
    class FCLKinBodyInfoRemover
    {
public:
        FCLKinBodyInfoRemover(const boost::function<void()>& fn) : _fn(fn) {
            _bDoRemove = true;
        }
        ~FCLKinBodyInfoRemover() {
            if( _bDoRemove ) {
                _fn();
            }
        }

        void ResetRemove() {
            _bDoRemove = false;
        }

private:
        boost::function<void()> _fn;
        bool _bDoRemove;
    };
    void _ResetCurrentGeometryCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo)
    {
        FCLKinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();
        const int bodyIndex = pbody->GetEnvironmentBodyIndex();
        if (0 < bodyIndex && bodyIndex < (int)_currentpinfo.size()) {
            const FCLKinBodyInfoPtr& pcurrentinfo = _currentpinfo.at(bodyIndex);

            if( !!pinfo && pinfo == pcurrentinfo ) {//pinfo->_geometrygroup.size() == 0 ) {
                // pinfo is current set to the current one, so should InitKinBody into _currentpinfo
                //RAVELOG_VERBOSE_FORMAT("env=%d, resetting current geometry for kinbody %s nGeometryUpdateStamp=%d, (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
                pinfo->nGeometryUpdateStamp++;
                FCLKinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
                InitKinBody(pbody, pinfo, false);
                remover.ResetRemove(); // succeeded
            }
            //_cachedpinfo[pbody->GetEnvironmentBodyIndex()].erase(std::string());
        }
    }

    void _ResetGeometryGroupsCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo)
    {
        FCLKinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();

        //FCLKinBodyInfoPtr pcurrentinfo = _currentpinfo.at(pbody->GetEnvironmentBodyIndex());

        if( !!pinfo ) {// && pinfo->_geometrygroup.size() > 0 ) {
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting geometry groups for kinbody %s, nGeometryUpdateStamp=%d (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
            pinfo->nGeometryUpdateStamp++;
            FCLKinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo, false);
            remover.ResetRemove(); // succeeded
        }
//        FCLKinBodyInfoPtr pinfoCurrentGeometry = _cachedpinfo[pbody->GetEnvironmentBodyIndex()][std::string()];
//        _cachedpinfo.erase(pbody->GetEnvironmentBodyIndex());
//        if( !!pinfoCurrentGeometry ) {
//            _cachedpinfo[pbody->GetEnvironmentBodyIndex()][std::string()] = pinfoCurrentGeometry;
//        }
    }

    void _ResetLinkEnableCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo) {
        FCLKinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->nLinkUpdateStamp++;
        }
    }

    void _ResetActiveDOFsCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo) {
        FCLKinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->nActiveDOFUpdateStamp++;
        }
    }

    void _ResetAttachedBodyCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo) {
        FCLKinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->nAttachedBodiesUpdateStamp++;
        }
    }


    EnvironmentBasePtr _penv;
    std::string _userdatakey;
    std::string _geometrygroup;
    //SynchronizeCallbackFn _synccallback;

    std::string _bvhRepresentation;
    MeshFactory _meshFactory;

    std::vector<KinBodyConstPtr> _vecInitializedBodies; ///< vector of the kinbody initialized in this space. index is the environment body index. nullptr means uninitialized.
    std::vector<std::map< std::string, FCLKinBodyInfoPtr> > _cachedpinfo; ///< Associates to each body id and geometry group name the corresponding kinbody info if already initialized and not currently set as user data. Index of vector is the environment id. index 0 holds null pointer because kin bodies in the env should have positive index.
    std::vector<FCLKinBodyInfoPtr> _currentpinfo; ///< maps kinbody environment id to the kinbodyinfo struct constaining fcl objects. Index of the vector is the environment id (id of the body in the env, not __nUniqueId of env) of the kinbody at that index. The index being environment id makes it easier to compare objects without getting a handle to their pointers. Whenever a FCLKinBodyInfoPtr goes into this map, it is removed from _cachedpinfo. Index of vector is the environment id. index 0 holds null pointer because kin bodies in the env should have positive index.

    std::vector<int> _vecAttachedEnvBodyIndicesCache; ///< cache
    std::vector<KinBodyPtr> _vecAttachedBodiesCache; ///< cache

    bool _bIsSelfCollisionChecker; // Currently not used
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::FCLKinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::FCLKinBodyInfo::LinkInfo)
#endif

}

#endif
