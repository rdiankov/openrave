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
    class KinBodyInfo : public boost::enable_shared_from_this<KinBodyInfo>, public OpenRAVE::UserData
    {
public:
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
                        RAVELOG_VERBOSE_FORMAT("env=%d, resetting link %s:%s col=0x%x", GetLink()->GetParent()->GetEnv()->GetId()%GetLink()->GetParent()->GetName()%GetLink()->GetName()%(uint64_t)linkBV.second.get());
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
            }

            KinBody::LinkPtr GetLink() {
                return _plink.lock();
            }

            KinBody::LinkWeakPtr _plink;

            //int nLastStamp; ///< Tracks if the collision geometries are up to date wrt the body update stamp. This is for narrow phase collision
            TransformCollisionPair linkBV; ///< pair of the transformation and collision object corresponding to a bounding OBB for the link
            std::vector<TransformCollisionPair> vgeoms; ///< vector of transformations and collision object; one per geometries
            std::string bodylinkname; // for debugging purposes
            bool bFromKinBodyLink; ///< if true, then from kinbodylink. Otherwise from standalone object that does not have any KinBody associations
        };

        KinBodyInfo() : nLastStamp(0), nLinkUpdateStamp(0), nGeometryUpdateStamp(0), nAttachedBodiesUpdateStamp(0), nActiveDOFUpdateStamp(0)
        {
        }

        virtual ~KinBodyInfo() {
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

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::weak_ptr<KinBodyInfo> KinBodyInfoWeakPtr;
    typedef boost::shared_ptr<FCLSpace::KinBodyInfo::LinkInfo> LinkInfoPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : _penv(penv), _userdatakey(userdatakey), _bIsSelfCollisionChecker(true)
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
        FOREACH(itbody, _setInitializedBodies) {
            KinBodyInfoPtr pinfo = GetInfo(**itbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
        }
        _currentpinfo.clear();
        _cachedpinfo.clear();
        _setInitializedBodies.clear();
    }

    KinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), bool bSetToCurrentPInfo=true)
    {
        if( !pinfo ) {
            pinfo.reset(new KinBodyInfo());
            pinfo->_geometrygroup = _geometrygroup;
        }

        RAVELOG_VERBOSE_FORMAT("env=%d, self=%d, init body %s (%d)", pbody->GetEnv()->GetId()%_bIsSelfCollisionChecker%pbody->GetName()%pbody->GetEnvironmentId());
        pinfo->Reset();
        pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
        // make sure that synchronization do occur !
        pinfo->nLastStamp = pbody->GetUpdateStamp() - 1;

        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
            const KinBody::LinkPtr& plink = *itlink;
            boost::shared_ptr<KinBodyInfo::LinkInfo> linkinfo(new KinBodyInfo::LinkInfo(plink));


            typedef boost::range_detail::any_iterator<KinBody::GeometryInfo, boost::forward_traversal_tag, KinBody::GeometryInfo const&, std::ptrdiff_t> GeometryInfoIterator;
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
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, *pgeominfo);

                    if( !pfclgeom ) {
                        continue;
                    }

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                    pfclcoll->setUserData(linkinfo.get());
                    linkinfo->vgeoms.push_back(TransformCollisionPair(pgeominfo->_t, pfclcoll));

                    KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), *pgeominfo);
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
                    const KinBody::GeometryInfo& geominfo = (*itgeom)->GetInfo();
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, geominfo);

                    if( !pfclgeom ) {
                        continue;
                    }

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                    pfclcoll->setUserData(linkinfo.get());

                    linkinfo->vgeoms.push_back(TransformCollisionPair(geominfo._t, pfclcoll));

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
                RAVELOG_DEBUG_FORMAT("Initializing link %s/%s with 0 geometries (env %d) (userdatakey %s)",pbody->GetName()%plink->GetName()%_penv->GetId()%_userdatakey);
            }
            else {
                CollisionGeometryPtr pfclgeomBV = std::make_shared<fcl::Box>(enclosingBV.max_ - enclosingBV.min_);
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

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetCurrentGeometryCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_geometrygroupcallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometryGroup, boost::bind(&FCLSpace::_ResetGeometryGroupsCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_linkenablecallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&FCLSpace::_ResetLinkEnableCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_activeDOFsCallback = pbody->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs, boost::bind(&FCLSpace::_ResetActiveDOFsCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBodyInfo>(pinfo)));

        pinfo->_bodyAttachedCallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyAttached, boost::bind(&FCLSpace::_ResetAttachedBodyCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_bodyremovedcallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyRemoved, boost::bind(&FCLSpace::RemoveUserData, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::bind(&OpenRAVE::utils::sptr_from<const KinBody>, boost::weak_ptr<const KinBody>(pbody))));

        BOOST_ASSERT(pbody->GetEnvironmentId() != 0);
        if( bSetToCurrentPInfo ) {
            _currentpinfo[pbody->GetEnvironmentId()] = pinfo;
        }
        //_cachedpinfo[pbody->GetEnvironmentId()] what to do with the cache?
        _setInitializedBodies.insert(pbody);

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
        FOREACHC(itbody, _setInitializedBodies) {
            SetBodyGeometryGroup(*itbody, groupname);
        }
    }

    const std::string& GetGeometryGroup() const
    {
        return _geometrygroup;
    }


    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
        if (!HasNamedGeometry(*pbody, groupname)) {
            return false;
        }

        // Save the already existing KinBodyInfoPtr for the old geometry group
        KinBodyInfoPtr poldinfo = GetInfo(*pbody);
        if( poldinfo->_geometrygroup == groupname ) {
            return true;
        }

        poldinfo->nGeometryUpdateStamp += 1;
        _cachedpinfo[(pbody)->GetEnvironmentId()][poldinfo->_geometrygroup] = poldinfo;

        BOOST_ASSERT(pbody->GetEnvironmentId() != 0);

        KinBodyInfoPtr pinfo = _cachedpinfo[pbody->GetEnvironmentId()][groupname];
        if(!pinfo) {
            RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%pbody->GetName()%pbody->GetEnvironmentId()%_penv->GetId());
            pinfo.reset(new KinBodyInfo);
            pinfo->_geometrygroup = groupname;
            InitKinBody(pbody, pinfo);
        }
        else {
            RAVELOG_VERBOSE_FORMAT("env=%d, switching to geometry %s for kinbody %s (id = %d)", _penv->GetId()%groupname%pbody->GetName()%pbody->GetEnvironmentId());
            // Set the current info to use the KinBodyInfoPtr associated to groupname
            _currentpinfo[pbody->GetEnvironmentId()] = pinfo;

            // Revoke the information inside the cache so that a potentially outdated object does not survive
            _cachedpinfo[(pbody)->GetEnvironmentId()].erase(groupname);
        }

        return true;
    }

    const std::string& GetBodyGeometryGroup(const KinBody &body) const {
        static const std::string empty;
        KinBodyInfoPtr pinfo = GetInfo(body);
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

        // reinitialize all the KinBodyInfo

        FOREACH(itbody, _setInitializedBodies) {
            KinBodyInfoPtr pinfo = GetInfo(**itbody);
            pinfo->nGeometryUpdateStamp++;
            InitKinBody(*itbody, pinfo);
        }
        _cachedpinfo.clear();
    }

    std::string const& GetBVHRepresentation() const {
        return _bvhRepresentation;
    }


    void Synchronize()
    {
        // We synchronize only the initialized bodies, which differs from oderave
        FOREACH(itbody, _setInitializedBodies) {
            Synchronize(**itbody);
        }
    }

    void Synchronize(const KinBody &body)
    {
        KinBodyInfoPtr pinfo = GetInfo(body);
        if( !pinfo ) {
            return;
        }
        BOOST_ASSERT( pinfo->GetBody().get() == &body);
        _Synchronize(*pinfo, body);
    }

    void SynchronizeWithAttached(const KinBody &body)
    {
        if( body.HasAttached() ) {
            std::set<KinBodyPtr> setAttachedpBodyTemp;
            body.GetAttached(setAttachedpBodyTemp);
            FOREACH(itbody, setAttachedpBodyTemp) {
                Synchronize(**itbody);
            }
        }
        else {
            Synchronize(body);
        }
    }

    KinBodyInfoPtr GetInfo(const KinBody &body) const
    {
        int envId = body.GetEnvironmentId();
        if ( envId == 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, body %s has invalid environment id 0", body.GetEnv()->GetId()%body.GetName());
            return KinBodyInfoPtr();
        }

        std::map< int, KinBodyInfoPtr >::const_iterator it = _currentpinfo.find(envId);
        if( it == _currentpinfo.end() ) {
            return KinBodyInfoPtr();
        }
        return it->second;
    }


    void RemoveUserData(KinBodyConstPtr pbody) {
        if( !!pbody ) {
            RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d (userdatakey %s) : %s") % _penv->GetId() % _userdatakey % pbody->GetName()));
            _setInitializedBodies.erase(pbody);
            KinBodyInfoPtr pinfo = GetInfo(*pbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
            BOOST_ASSERT(pbody->GetEnvironmentId() != 0);

            _currentpinfo.erase(pbody->GetEnvironmentId());
            _cachedpinfo.erase(pbody->GetEnvironmentId());
        }
    }


    const std::set<KinBodyConstPtr>& GetEnvBodies() const {
        return _setInitializedBodies;
    }

    inline CollisionObjectPtr GetLinkBV(const KinBody::Link &link) {
        return GetLinkBV(*link.GetParent(), link.GetIndex());
    }

    inline CollisionObjectPtr GetLinkBV(const KinBody &body, int index) {
        KinBodyInfoPtr pinfo = GetInfo(body);
        if( !!pinfo ) {
            return GetLinkBV(*pinfo, index);
        } else {
            RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%body.GetName()%_userdatakey%_penv->GetId()));
            return CollisionObjectPtr();
        }
    }

    inline CollisionObjectPtr GetLinkBV(const KinBodyInfo &info, int index) {
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
            Vector v = info._t*mesh.vertices[ipoint];
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

    /// \brief pass in info.GetBody() as a reference to avoid dereferencing the weak pointer in KinBodyInfo
    void _Synchronize(KinBodyInfo& info, const KinBody& body)
    {
        //KinBodyPtr pbody = info.GetBody();
        if( info.nLastStamp != body.GetUpdateStamp()) {
            vector<Transform> vtrans;
            body.GetLinkTransformations(vtrans);
            info.nLastStamp = body.GetUpdateStamp();
            BOOST_ASSERT( body.GetLinks().size() == info.vlinks.size() );
            BOOST_ASSERT( vtrans.size() == info.vlinks.size() );
            for(size_t i = 0; i < vtrans.size(); ++i) {
                CollisionObjectPtr pcoll = info.vlinks[i]->linkBV.second;
                if( !pcoll ) {
                    continue;
                }
                Transform pose = vtrans[i] * info.vlinks[i]->linkBV.first;
                fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                pcoll->setTranslation(newPosition);
                pcoll->setQuatRotation(newOrientation);
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                pcoll->computeAABB();

                //info.vlinks[i]->nLastStamp = info.nLastStamp;
                FOREACHC(itgeomcoll, info.vlinks[i]->vgeoms) {
                    CollisionObjectPtr pcoll = (*itgeomcoll).second;
                    Transform pose = vtrans[i] * (*itgeomcoll).first;
                    fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                    fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                    pcoll->setTranslation(newPosition);
                    pcoll->setQuatRotation(newOrientation);
                    // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                    pcoll->computeAABB();
                }
            }

            // Does this have any use ?
            // if( !!_synccallback ) {
            //     _synccallback(pinfo);
            // }
        }
    }

    /// \brief controls whether the kinbody info is removed during the destructor
    class KinBodyInfoRemover
    {
public:
        KinBodyInfoRemover(const boost::function<void()>& fn) : _fn(fn) {
            _bDoRemove = true;
        }
        ~KinBodyInfoRemover() {
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
    void _ResetCurrentGeometryCallback(boost::weak_ptr<KinBodyInfo> _pinfo)
    {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();
        KinBodyInfoPtr pcurrentinfo = _currentpinfo[pbody->GetEnvironmentId()];

        if( !!pinfo && pinfo == pcurrentinfo ) {//pinfo->_geometrygroup.size() == 0 ) {
            // pinfo is current set to the current one, so should InitKinBody into _currentpinfo
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting current geometry for kinbody %s nGeometryUpdateStamp=%d, (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
            pinfo->nGeometryUpdateStamp++;
            KinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo, false);
            remover.ResetRemove(); // succeeded
        }
        //_cachedpinfo[pbody->GetEnvironmentId()].erase(std::string());
    }

    void _ResetGeometryGroupsCallback(boost::weak_ptr<KinBodyInfo> _pinfo)
    {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();

        //KinBodyInfoPtr pcurrentinfo = _currentpinfo[pbody->GetEnvironmentId()];

        if( !!pinfo ) {// && pinfo->_geometrygroup.size() > 0 ) {
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting geometry groups for kinbody %s, nGeometryUpdateStamp=%d (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
            pinfo->nGeometryUpdateStamp++;
            KinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo, false);
            remover.ResetRemove(); // succeeded
        }
//        KinBodyInfoPtr pinfoCurrentGeometry = _cachedpinfo[pbody->GetEnvironmentId()][std::string()];
//        _cachedpinfo.erase(pbody->GetEnvironmentId());
//        if( !!pinfoCurrentGeometry ) {
//            _cachedpinfo[pbody->GetEnvironmentId()][std::string()] = pinfoCurrentGeometry;
//        }
    }

    void _ResetLinkEnableCallback(boost::weak_ptr<KinBodyInfo> _pinfo) {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->nLinkUpdateStamp++;
        }
    }

    void _ResetActiveDOFsCallback(boost::weak_ptr<KinBodyInfo> _pinfo) {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->nActiveDOFUpdateStamp++;
        }
    }

    void _ResetAttachedBodyCallback(boost::weak_ptr<KinBodyInfo> _pinfo) {
        KinBodyInfoPtr pinfo = _pinfo.lock();
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

    std::set<KinBodyConstPtr> _setInitializedBodies; ///< Set of the kinbody initialized in this space
    std::map< int, std::map< std::string, KinBodyInfoPtr > > _cachedpinfo; ///< Associates to each body id and geometry group name the corresponding kinbody info if already initialized and not currently set as user data
    std::map< int, KinBodyInfoPtr> _currentpinfo; ///< maps kinbody environment id to the kinbodyinfo struct constaining fcl objects. The key being environment id makes it easier to compare objects without getting a handle to their pointers. Whenever a KinBodyInfoPtr goes into this map, it is removed from _cachedpinfo

    bool _bIsSelfCollisionChecker; // Currently not used
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo::LinkInfo)
#endif

}

#endif
