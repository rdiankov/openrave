#ifndef OPENRAVE_FCL_SPACE
#define OPENRAVE_FCL_SPACE


#include <boost/shared_ptr.hpp>
#include <memory> // c++11
#include <vector>

// TODO : I should put these in some namespace...

typedef KinBody::LinkConstPtr LinkConstPtr;
typedef std::pair<LinkConstPtr, LinkConstPtr> LinkPair;
typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
// Warning : this is the only place where we use std::shared_ptr (compatibility with fcl)
typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef boost::function<CollisionGeometryPtr (std::vector<fcl::Vec3f> const &points, std::vector<fcl::Triangle> const &triangles) > MeshFactory;
typedef std::vector<fcl::CollisionObject *> CollisionGroup;
typedef boost::shared_ptr<CollisionGroup> CollisionGroupPtr;
using OpenRAVE::ORE_Assert;

/* Helper functions for conversions from OpenRAVE to FCL */

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

template <class T>
CollisionGeometryPtr ConvertMeshToFCL(std::vector<fcl::Vec3f> const &points,std::vector<fcl::Triangle> const &triangles)
{
    shared_ptr< fcl::BVHModel<T> > const model = make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}



class FCLSpace : public boost::enable_shared_from_this<FCLSpace>
{


public:
    typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
    typedef boost::shared_ptr<Transform> TransformPtr;
    // TODO : is it okay to leave a plain transform there instead of a pointer
    typedef pair<Transform, CollisionObjectPtr> TransformCollisionPair;



    inline boost::weak_ptr<FCLSpace> weak_space() {
        return shared_from_this();
    }



    // corresponds to FCLUserData
    class KinBodyInfo : public boost::enable_shared_from_this<KinBodyInfo>, public OpenRAVE::UserData
    {
public:

        typedef boost::weak_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerWeakPtr;

        struct LINK
        {
        LINK(KinBody::LinkPtr plink) : _plink(plink), nLastStamp(0) {
            }

            virtual ~LINK() {
                Reset();
            }

            void Reset() {
                if( !!_linkManager ) {
                    _linkManager->clear();
                    _linkManager.reset();
                }

                if( !!plinkBV ) {
                    FOREACH(itwpmanager, _vmanagers) {
                        BroadPhaseCollisionManagerPtr pmanager = itwpmanager->lock();
                        if( pmanager ) {
                            pmanager->unregisterObject(plinkBV->second.get());
                        }
                    }
                    plinkBV.reset();
                }

                FOREACH(itgeompair, vgeoms) {
                    (*itgeompair).second.reset();
                }
                vgeoms.resize(0);
            }

            void Register(BroadPhaseCollisionManagerPtr pmanager) {
                // We cannot use find here because eqaulity is not well-defined between 2 weak pointers
                FOREACH(itmanager, _vmanagers) {
                    if ( itmanager->lock() == pmanager ) {
                        return;
                    }
                }
                pmanager->registerObject(plinkBV->second.get());
                _vmanagers.push_back(BroadPhaseCollisionManagerWeakPtr(pmanager));
            }

            KinBody::LinkPtr GetLink() {
                return _plink.lock();
            }

            KinBody::LinkWeakPtr _plink;

          int nLastStamp;
            // TODO : What about a dynamic collection of managers containing this link ?
            std::list<BroadPhaseCollisionManagerWeakPtr> _vmanagers; ///< Broad phase managers containing this link's BV
            BroadPhaseCollisionManagerPtr _linkManager;
            boost::shared_ptr<TransformCollisionPair> plinkBV;
            std::vector<TransformCollisionPair> vgeoms; // fcl variant of ODE dBodyID
            std::string bodylinkname; // for debugging purposes
        };

        KinBodyInfo() : nLastStamp(0), _bactiveDOFsDirty(true)
        {
        }

        virtual ~KinBodyInfo() {
            Reset();
        }

        void Reset()
        {
            if( !!_bodyManager ) {
                _bodyManager->clear();
                _bodyManager.reset();
            }
            if( !!_bodyManagerActiveDOFs ) {
                _bodyManagerActiveDOFs->clear();
                _bodyManagerActiveDOFs.reset();
            }

            // The LINKs should not unregister themselves from _bodyManager or _bodyManagerActiveDOFs anymore so it should be safe to reset them after these
            FOREACH(itlink, vlinks) {
                (*itlink)->Reset();
            }
            vlinks.resize(0);
            _geometrycallback.reset();
        }

        void _ResetBodyManagers() {
          // we invalidate both managers
          _bodyManager.reset();
          // TODO : this might not be always necessary, i.e. when the attached or detached body is not on an active link...
          _bodyManagerActiveDOFs.reset();
          // we reinitialize the link-enabled callbacks
          _linkEnabledCallbacks.clear();
        }

        void _ChangeActiveDOFsFlag() {
            _bactiveDOFsDirty = true;
            _bodyManagerActiveDOFs.reset();
        }

        // TODO : Is this used ?
        KinBodyPtr GetBody()
        {
            return _pbody.lock();
        }

        KinBodyWeakPtr _pbody;
        int nLastStamp;  // used during synchronization ("is transform up to date")
        vector< boost::shared_ptr<LINK> > vlinks;
        OpenRAVE::UserDataPtr _geometrycallback;

        bool _bactiveDOFsDirty; ///< true if some active link has been added or removed since the last construction of _bodyManagerActiveDOFs
        BroadPhaseCollisionManagerPtr _bodyManager; ///< Broad phase manager containing all the enabled links of the kinbody (does not contain attached kinbodies' links)
        BroadPhaseCollisionManagerPtr _bodyManagerActiveDOFs; ///< Broad phase manager containing all the active links of the kinbody (does not contain attached kinbodies' links)
        int nBodyManagerStamp; // sum of the update stamps of all attached bodies; TODO : consider a vector
        int nBodyManagerActiveDOFsStamp;
        std::vector<int> _vactiveLinks; ///< ith element is 1 if the ith link of the kinbody is active, 0 otherwise ; ensured to be correct only after a call to GetBodyManager(true)
        OpenRAVE::UserDataPtr _bodyAttachedCallback; ///< handle for the callback called when a body is attached or detached
        OpenRAVE::UserDataPtr _activeDOFsCallback; ///< handle for the callback called when a the activeDOFs have changed
      std::list<OpenRAVE::UserDataPtr> _linkEnabledCallbacks;
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : _penv(penv), _userdatakey(userdatakey)
    {
        // TODO : test best default choice
        SetBVHRepresentation("OBB");
    }

    virtual ~FCLSpace()
    {
        DestroyEnvironment();
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE("destroying fcl collision environment\n");
        if( !!_envManager ) {
            _envManager.reset();
        }
        FOREACH(itbody, _setInitializedBodies) {
            KinBodyInfoPtr pinfo = GetInfo(*itbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
            bool is_consistent = (*itbody)->RemoveUserData(_userdatakey);
            if( !is_consistent ) {
                RAVELOG_WARN("inconsistency detected with fclspace user data\n");
            }
        }
        _setInitializedBodies.clear();
    }


    KinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr())
    {
        EnvironmentMutex::scoped_lock lock(pbody->GetEnv()->GetMutex());

        if( !pinfo ) {
            pinfo.reset(new KinBodyInfo());
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
            if(_geometrygroup.size() > 0 && (*itlink)->GetGroupNumGeometries(_geometrygroup) >= 0) {
                const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = (*itlink)->GetGeometriesFromGroup(_geometrygroup);
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
                Func getInfo = [] (KinBody::Link::GeometryPtr const &itgeom)->KinBody::GeometryInfo const& { return itgeom->GetInfo(); };
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
                continue;
            }

            if( link->vgeoms.size() == 1) {
                // set the unique geometry as its own bounding volume
                link->plinkBV = boost::make_shared<TransformCollisionPair>(link->vgeoms[0]);
            } else {
                // create the bounding volume for the link
                fcl::BVHModel<fcl::OBB> model;
                model.beginModel();
                // This could be costly
                FOREACH(itgeom, (*itlink)->GetGeometries()) {
                    (*itgeom)->InitCollisionMesh(0.1f);
                }
                for(GeometryInfoIterator it = begingeom; it != endgeom; ++it) {
                    _bvAddSubmodelFromGeomInfo(model, *it);
                }
                model.endModel();
                OPENRAVE_ASSERT_OP( model.getNumBVs(), !=, 0);
                link->plinkBV = _CreateTransformCollisionPairFromOBB(model.getBV(0).bv);
                link->plinkBV->second->setUserData(link.get());
            }

            link->nLastStamp = pinfo->nLastStamp;
            link->bodylinkname = pbody->GetName() + "/" + (*itlink)->GetName();
            pinfo->vlinks.push_back(link);
        }
        if( !!_envManager ) {
            // If the _envManager does exists, we register all the links BV of the kinbody
            FOREACH(itpLINK, pinfo->vlinks) {
                (*itpLINK)->Register(_envManager);
            }
        }

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetKinBodyCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBody const>(pbody)));

        pbody->SetUserData(_userdatakey, pinfo);
        _setInitializedBodies.insert(pbody);

        //Do I really need to synchronize anything at that point ?
        _Synchronize(pinfo);


        return pinfo;
    }




    void SetEnvManager(BroadPhaseCollisionManagerPtr envManager) {
        _envManager = envManager;
    }


    void SetGeometryGroup(const std::string& groupname)
    {
        if(groupname != _geometrygroup) {
            _geometrygroup = groupname;

            FOREACHC(itbody, _setInitializedBodies) {
                KinBodyInfoPtr pinfo = this->GetInfo(*itbody);
                if( !!pinfo ) {
                    InitKinBody(*itbody, pinfo);
                }
            }
        }
    }

    const std::string& GetGeometryGroup() const
    {
        return _geometrygroup;
    }


    // Already existing geometry not updated !
    // Is that the behaviour we want ?
    void SetBVHRepresentation(std::string const &type)
    {
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
            RAVELOG_WARN(str(boost::format("Unknown BVH representation '%s'.") % type));
        }
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
        KinBodyInfoPtr pinfo = GetCreateInfo(pbody).first;
        BOOST_ASSERT( pinfo->GetBody() == pbody);
        _Synchronize(pinfo);
    }

    void SetSynchronizationCallback(const SynchronizeCallbackFn& synccallback) {
        _synccallback = synccallback;
    }




    KinBodyInfoPtr GetInfo(KinBodyConstPtr pbody)
    {
        return boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData(_userdatakey));
    }

    std::pair<KinBodyInfoPtr, bool> GetCreateInfo(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = this->GetInfo(pbody);
        bool bcreated = false;
        if( !pinfo ) {
            pinfo = InitKinBody(pbody, KinBodyInfoPtr());
            pbody->SetUserData(_userdatakey, pinfo);
            bcreated = true;
        }
        return std::make_pair(pinfo, bcreated);
    }

    void RemoveUserData(KinBodyConstPtr pbody) {
        if( !!pbody ) {
            RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d : %s") % _penv->GetId() % pbody->GetName()));
            _setInitializedBodies.erase(pbody);
            KinBodyInfoPtr pinfo = GetInfo(pbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
            bool is_consistent = pbody->RemoveUserData(_userdatakey);
            if( !is_consistent ) {
                RAVELOG_WARN("inconsistency detected with fclspace user data\n");
            }
        }
    }

    void InvalidateCachedManagers() {
        FOREACH(itbody, _setInitializedBodies) {
            KinBodyInfoPtr pinfo = GetInfo(*itbody);
            BOOST_ASSERT( !!pinfo );
            pinfo->_bodyManager.reset();
            pinfo->_bodyManagerActiveDOFs.reset();
            FOREACH(itlink, pinfo->vlinks) {
                (*itlink)->_linkManager.reset();
                (*itlink)->_vmanagers.resize(0);
            }
        }
    }

    bool HasMultipleGeometries(LinkConstPtr plink) {
        KinBodyInfoPtr pinfo = GetInfo(plink->GetParent());
        return !pinfo->vlinks[plink->GetIndex()]->vgeoms.empty();
    }

    void SynchronizeGeometries(LinkConstPtr plink, boost::shared_ptr<KinBodyInfo::LINK> pLINK) {
      if( pLINK->nLastStamp < plink->GetParent()->GetUpdateStamp() ) {
        pLINK->nLastStamp = plink->GetParent()->GetUpdateStamp();
        FOREACHC(itgeomcoll, pLINK->vgeoms) {
          CollisionObjectPtr pcoll = (*itgeomcoll).second;
          Transform pose = plink->GetTransform() * (*itgeomcoll).first;
          fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
          fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

          pcoll->setTranslation(newPosition);
          pcoll->setQuatRotation(newOrientation);
          // Why do we compute the AABB ?
          pcoll->computeAABB();
        }
      }
    }


private:

    static void _bvAddSubmodelFromGeomInfo(fcl::BVHModel<fcl::OBB>& model, KinBody::GeometryInfo const &info) {

        OpenRAVE::TriMesh mesh = info._meshcollision;
        mesh.ApplyTransform(info._t);
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            return;
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
        model.addSubModel(fcl_points, fcl_triangles);
    }

    static boost::shared_ptr<TransformCollisionPair> _CreateTransformCollisionPairFromOBB(fcl::OBB const &bv) {
        CollisionGeometryPtr pbvGeom = make_shared<fcl::Box>(bv.extent[0]*2.0f, bv.extent[1]*2.0f, bv.extent[2]*2.0f);
        CollisionObjectPtr pbvColl = boost::make_shared<fcl::CollisionObject>(pbvGeom);
        fcl::Quaternion3f fclBvRot;
        fclBvRot.fromAxes(bv.axis);
        Vector bvRotation = ConvertQuaternionFromFCL(fclBvRot);
        Vector bvTranslation = ConvertVectorFromFCL(bv.center());

        return boost::make_shared<TransformCollisionPair>(Transform(bvRotation, bvTranslation), pbvColl);
    }

    // couldn't we make this static / what about the tests on non-zero size (eg. box extents) ?
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
            // TODO double check this
            return make_shared<fcl::Cylinder>(info._vGeomData.x, info._vGeomData.y);

        // TODO : Is that ok ? (consider testing the transformatiom with boxes)
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
              BOOST_ASSERT( !!pinfo->vlinks[i]->plinkBV);
                CollisionObjectPtr pcoll = pinfo->vlinks[i]->plinkBV->second;
                Transform pose = vtrans[i] * pinfo->vlinks[i]->plinkBV->first;
                fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                pcoll->setTranslation(newPosition);
                pcoll->setQuatRotation(newOrientation);
                // Why do we compute the AABB ?
                pcoll->computeAABB();
            }

//            if( !!_envManager ) {
//              CollisionGroup vupdatedObjects(pbody->GetLinks().size());
//              FOREACH(itLINK, pinfo->vlinks) {
//                vupdatedObjects.push_back((*itLINK)->plinkBV->second.get());
//              }
//              _envManager->update(vupdatedObjects);
//            }

            if( !!_synccallback ) {
                _synccallback(pinfo);
            }
        }
    }


    void _ResetKinBodyCallback(boost::weak_ptr<KinBody const> _pbody)
    {
        KinBodyConstPtr pbody(_pbody);
        std::pair<KinBodyInfoPtr, bool> infocreated = GetCreateInfo(pbody);
        if( !infocreated.second ) {
            BOOST_ASSERT( infocreated.first->GetBody() == pbody );
            InitKinBody(pbody, infocreated.first);
        }
    }



    EnvironmentBasePtr _penv;
    std::string _userdatakey;
    std::string _geometrygroup;
    SynchronizeCallbackFn _synccallback;

    std::string _bvhRepresentation;
    MeshFactory _meshFactory;

    std::set<KinBodyConstPtr> _setInitializedBodies;
    BroadPhaseCollisionManagerPtr _envManager;
};




#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo::LINK)
#endif


#endif
