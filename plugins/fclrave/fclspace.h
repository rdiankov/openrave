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
        struct LINK
        {
            LINK(KinBody::LinkPtr plink) : _plink(plink) {
            }

            virtual ~LINK() {
                Reset();
            }

            void Reset() {
                if( !!linkBV.second ) {
                    if( !!GetLink() ) {
                        RAVELOG_VERBOSE_FORMAT("resetting link %s:%s col=0x%x (env %d)", GetLink()->GetParent()->GetName()%GetLink()->GetName()%(uint64_t)linkBV.second.get()%GetLink()->GetParent()->GetEnv()->GetId());
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

        vector< boost::shared_ptr<LINK> > vlinks; ///< info for every link of the kinbody

        OpenRAVE::UserDataPtr _bodyAttachedCallback; ///< handle for the callback called when a body is attached or detached
        OpenRAVE::UserDataPtr _activeDOFsCallback; ///< handle for the callback called when a the activeDOFs have changed
        std::list<OpenRAVE::UserDataPtr> _linkEnabledCallbacks;

        OpenRAVE::UserDataPtr _geometrycallback; ///< handle for the callback called when the current geometry of the kinbody changed ( Prop_LinkGeometry )
        OpenRAVE::UserDataPtr _geometrygroupcallback; ///< handle for the callback called when some geometry group of one of the links of this kinbody changed ( Prop_LinkGeometryGroup )
        OpenRAVE::UserDataPtr _linkenablecallback; ///< handle for the callback called when some link enable status of this kinbody has changed so that the envManager is updated ( Prop_LinkEnable )

        std::string _geometrygroup; ///< name of the geometry group tracked by this kinbody info ; if empty, tracks the current geometries
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::weak_ptr<KinBodyInfo> KinBodyInfoWeakPtr;
    typedef boost::shared_ptr<FCLSpace::KinBodyInfo::LINK> LinkInfoPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : _penv(penv), _userdatakey(userdatakey)
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
        RAVELOG_VERBOSE_FORMAT("destroying fcl collision environment (env %d) (userdatakey %s)\n", _penv->GetId()%_userdatakey);
        FOREACH(itbody, _setInitializedBodies) {
            KinBodyInfoPtr pinfo = GetInfo(*itbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
            bool is_consistent = (*itbody)->RemoveUserData(_userdatakey);
            if( !is_consistent ) {
                RAVELOG_WARN_FORMAT("inconsistency detected with fclspace user data (kinbody %s) (env %d) (userdatakey %s)\n", (*itbody)->GetName()%_penv->GetId()%_userdatakey);
            }
        }
        _setInitializedBodies.clear();
    }

    KinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr())
    {

        if( !pinfo ) {
            pinfo.reset(new KinBodyInfo());
            pinfo->_geometrygroup = _geometrygroup;
        }

        pinfo->Reset();
        pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
        // make sure that synchronization do occur !
        pinfo->nLastStamp = pbody->GetUpdateStamp() - 1;

        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {

            boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK(*itlink));


            typedef boost::range_detail::any_iterator<KinBody::GeometryInfo, boost::forward_traversal_tag, KinBody::GeometryInfo const&, std::ptrdiff_t> GeometryInfoIterator;
            GeometryInfoIterator begingeom, endgeom;

            // Glue code for a unified access to geometries
            if(pinfo->_geometrygroup.size() > 0 && (*itlink)->GetGroupNumGeometries(pinfo->_geometrygroup) >= 0) {
                const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = (*itlink)->GetGeometriesFromGroup(pinfo->_geometrygroup);
                typedef boost::function<KinBody::GeometryInfo const& (KinBody::GeometryInfoPtr const&)> Func;
                typedef boost::transform_iterator<Func, std::vector<KinBody::GeometryInfoPtr>::const_iterator> PtrGeomInfoIterator;
                Func deref = boost::mem_fn(&KinBody::GeometryInfoPtr::operator*);
                begingeom = GeometryInfoIterator(PtrGeomInfoIterator(vgeometryinfos.begin(), deref));
                endgeom = GeometryInfoIterator(PtrGeomInfoIterator(vgeometryinfos.end(), deref));
            }
            else {
                std::vector<KinBody::Link::GeometryPtr> const &geoms = (*itlink)->GetGeometries();
                typedef boost::function<KinBody::GeometryInfo const& (KinBody::Link::GeometryPtr const&)> Func;
                typedef boost::transform_iterator<Func, std::vector<KinBody::Link::GeometryPtr>::const_iterator> PtrGeomInfoIterator;
                Func getInfo = [] (KinBody::Link::GeometryPtr const &itgeom) -> KinBody::GeometryInfo const& {
                                   return itgeom->GetInfo();
                               };
                begingeom = GeometryInfoIterator(PtrGeomInfoIterator(geoms.begin(), getInfo));
                endgeom = GeometryInfoIterator(PtrGeomInfoIterator(geoms.end(), getInfo));
            }

            for(GeometryInfoIterator itgeominfo = begingeom; itgeominfo != endgeom; ++itgeominfo) {
                const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, *itgeominfo);

                if( !pfclgeom ) {
                    continue;
                }

                // We do not set the transformation here and leave it to _Synchronize
                CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                pfclcoll->setUserData(link.get());

                link->vgeoms.push_back(TransformCollisionPair(itgeominfo->_t, pfclcoll));
            }

            if( link->vgeoms.size() == 0 ) {
                RAVELOG_WARN_FORMAT("Initializing link %s/%s with 0 geometries (env %d) (userdatakey %s)",pbody->GetName()%(*itlink)->GetName()%_penv->GetId()%_userdatakey);
            } else {
                // create the bounding volume for the link
                KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), *begingeom);
                fcl::AABB enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                for(GeometryInfoIterator it = ++begingeom; it != endgeom; ++it) {
                    KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), *it);
                    enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                }
                CollisionGeometryPtr pfclgeomBV = std::make_shared<fcl::Box>(enclosingBV.max_ - enclosingBV.min_);
                CollisionObjectPtr pfclcollBV = boost::make_shared<fcl::CollisionObject>(pfclgeomBV);
                Transform trans(Vector(1,0,0,0),ConvertVectorFromFCL(0.5 * (enclosingBV.min_ + enclosingBV.max_)));
                pfclcollBV->setUserData(link.get());
                link->linkBV = std::make_pair(trans, pfclcollBV);
            }

            //link->nLastStamp = pinfo->nLastStamp;
            link->bodylinkname = pbody->GetName() + "/" + (*itlink)->GetName();
            pinfo->vlinks.push_back(link);
#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
            RAVELOG_DEBUG_FORMAT("FCLSPACECOLLISIONOBJECT|%s|%s", link->linkBV.second.get()%link->bodylinkname);
#endif
        }

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetCurrentGeometryCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_geometrygroupcallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometryGroup, boost::bind(&FCLSpace::_ResetGeometryGroupsCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_linkenablecallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&FCLSpace::_ResetLinkEnableCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBodyInfo>(pinfo)));
        pinfo->_activeDOFsCallback = pbody->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs, boost::bind(&FCLSpace::_ResetActiveDOFsCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBodyInfo>(pinfo)));

        // if the attachedBodies callback is not set, we set it
        pinfo->_bodyAttachedCallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyAttached, boost::bind(&FCLSpace::_ResetAttachedBodyCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBodyInfo>(pinfo)));

        pbody->SetUserData(_userdatakey, pinfo);
        _setInitializedBodies.insert(pbody);

        //Do I really need to synchronize anything at that point ?
        _Synchronize(pinfo);

        return pinfo;
    }

    bool HasNamedGeometry(KinBodyConstPtr pbody, const std::string& groupname) {
        // The empty string corresponds to current geometries so all kinbodies have it
        if( groupname.size() == 0 ) {
            return true;
        }
        FOREACH(itlink, pbody->GetLinks()) {
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


    void SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
        if( HasNamedGeometry(pbody, groupname) ) {
            // Save the already existing KinBodyInfoPtr for the old geometry group
            KinBodyInfoPtr poldinfo = GetInfo(pbody);
            if( poldinfo->_geometrygroup == groupname ) {
                return;
            }

            poldinfo->nGeometryUpdateStamp += 1;
            _cachedpinfo[(pbody)->GetEnvironmentId()][poldinfo->_geometrygroup] = poldinfo;

            KinBodyInfoPtr pinfo = _cachedpinfo[pbody->GetEnvironmentId()][groupname];
            if(!pinfo) {
                RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%pbody->GetName()%pbody->GetEnvironmentId()%_penv->GetId());
                pinfo.reset(new KinBodyInfo);
                pinfo->_geometrygroup = groupname;
                InitKinBody(pbody, pinfo);
            }
            else {
                RAVELOG_VERBOSE_FORMAT("FCLSpace : switching to geometry %s for kinbody %s (id = %d) (env = %d)", groupname%pbody->GetName()%pbody->GetEnvironmentId()%_penv->GetId());
                // Set the current user data to use the KinBodyInfoPtr associated to groupname
                pbody->SetUserData(_userdatakey, pinfo);

                // Revoke the information inside the cache so that a potentially outdated object does not survive
                _cachedpinfo[(pbody)->GetEnvironmentId()].erase(groupname);
            }
        }
    }

    const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const {
        static const std::string empty;
        KinBodyInfoPtr pinfo = GetInfo(pbody);
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
            KinBodyInfoPtr pinfo = GetInfo(*itbody);
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
            Synchronize(*itbody);
        }
    }

    void Synchronize(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        if( !pinfo ) {
            return;
        }
        BOOST_ASSERT( pinfo->GetBody() == pbody);
        _Synchronize(pinfo);
    }

    void SetSynchronizationCallback(const SynchronizeCallbackFn& synccallback) {
        _synccallback = synccallback;
    }

    KinBodyInfoPtr GetInfo(KinBodyConstPtr pbody) const
    {
        return boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData(_userdatakey));
    }


    void RemoveUserData(KinBodyConstPtr pbody) {
        if( !!pbody ) {
            RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d (userdatakey %s) : %s") % _penv->GetId() % _userdatakey % pbody->GetName()));
            _setInitializedBodies.erase(pbody);
            KinBodyInfoPtr pinfo = GetInfo(pbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
            _cachedpinfo.erase(pbody->GetEnvironmentId());
            bool is_consistent = pbody->RemoveUserData(_userdatakey);
            if( !is_consistent ) {
                RAVELOG_WARN_FORMAT("inconsistency detected with fclspace user data (body %s) (env %d) (userdatakey %s)", pbody->GetName()%_penv->GetId()%_userdatakey);
            }
        }
    }

//    void SynchronizeGeometries(LinkConstPtr plink, boost::shared_ptr<KinBodyInfo::LINK> pLINK)
//    {
//        if( pLINK->nLastStamp != plink->GetParent()->GetUpdateStamp() ) {
//            pLINK->nLastStamp = plink->GetParent()->GetUpdateStamp();
//            FOREACHC(itgeomcoll, pLINK->vgeoms) {
//                CollisionObjectPtr pcoll = (*itgeomcoll).second;
//                Transform pose = plink->GetTransform() * (*itgeomcoll).first;
//                fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
//                fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);
//
//                pcoll->setTranslation(newPosition);
//                pcoll->setQuatRotation(newOrientation);
//                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
//                pcoll->computeAABB();
//            }
//        }
//    }


    const std::set<KinBodyConstPtr>& GetEnvBodies() const {
        return _setInitializedBodies;
    }

    inline CollisionObjectPtr GetLinkBV(LinkConstPtr plink) {
        return GetLinkBV(plink->GetParent(), plink->GetIndex());
    }

    inline CollisionObjectPtr GetLinkBV(KinBodyConstPtr pbody, int index) {
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        if( !!pinfo ) {
            return GetLinkBV(pinfo, index);
        } else {
            RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%pbody->GetName()%_userdatakey%_penv->GetId()));
            return CollisionObjectPtr();
        }
    }

    inline CollisionObjectPtr GetLinkBV(KinBodyInfoPtr pinfo, int index) {
        return pinfo->vlinks.at(index)->linkBV.second;
    }


    inline LinkInfoPtr GetLinkInfo(LinkConstPtr plink) {
        return GetInfo(plink->GetParent())->vlinks.at(plink->GetIndex());
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

    void _Synchronize(KinBodyInfoPtr pinfo)
    {
        KinBodyPtr pbody = pinfo->GetBody();
        if( pinfo->nLastStamp != pbody->GetUpdateStamp()) {
            vector<Transform> vtrans;
            pbody->GetLinkTransformations(vtrans);
            pinfo->nLastStamp = pbody->GetUpdateStamp();
            BOOST_ASSERT( pbody->GetLinks().size() == pinfo->vlinks.size() );
            BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
            for(size_t i = 0; i < vtrans.size(); ++i) {
                CollisionObjectPtr pcoll = pinfo->vlinks[i]->linkBV.second;
                if( !pcoll ) {
                    continue;
                }
                Transform pose = vtrans[i] * pinfo->vlinks[i]->linkBV.first;
                fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                pcoll->setTranslation(newPosition);
                pcoll->setQuatRotation(newOrientation);
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                pcoll->computeAABB();

                //pinfo->vlinks[i]->nLastStamp = pinfo->nLastStamp;
                FOREACHC(itgeomcoll, pinfo->vlinks[i]->vgeoms) {
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
            if( !!_synccallback ) {
                _synccallback(pinfo);
            }
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
        //RAVELOG_VERBOSE_FORMAT("Resetting current geometry for kinbody %s (in env %d, key %s)", pbody->GetName()%_penv->GetId()%_userdatakey);
        if( !!pinfo && pinfo->_geometrygroup.size() == 0 ) {
            pinfo->nGeometryUpdateStamp++;
            KinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo);
            remover.ResetRemove(); // succeeded
        }
        _cachedpinfo[pbody->GetEnvironmentId()].erase(std::string());
    }

    void _ResetGeometryGroupsCallback(boost::weak_ptr<KinBodyInfo> _pinfo)
    {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();
        //RAVELOG_VERBOSE_FORMAT("Resetting geometry groups for kinbody %s (in env %d, key %s)", pbody->GetName()%_penv->GetId()%_userdatakey);
        if( !!pinfo && pinfo->_geometrygroup.size() > 0 ) {
            pinfo->nGeometryUpdateStamp++;
            KinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo);
            remover.ResetRemove(); // succeeded
        }
        KinBodyInfoPtr pinfoCurrentGeometry = _cachedpinfo[pbody->GetEnvironmentId()][std::string()];
        _cachedpinfo.erase(pbody->GetEnvironmentId());
        if( !!pinfoCurrentGeometry ) {
            _cachedpinfo[pbody->GetEnvironmentId()][std::string()] = pinfoCurrentGeometry;
        }
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
    SynchronizeCallbackFn _synccallback;

    std::string _bvhRepresentation;
    MeshFactory _meshFactory;

    std::set<KinBodyConstPtr> _setInitializedBodies; ///< Set of the kinbody initialized in this space
    std::map< int, std::map< std::string, KinBodyInfoPtr > > _cachedpinfo; ///< Associates to each body id and geometry group name the corresponding kinbody info if already initialized and not currently set as user data
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo::LINK)
#endif

}

#endif
