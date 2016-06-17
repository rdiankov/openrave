#ifndef OPENRAVE_FCL_SPACE
#define OPENRAVE_FCL_SPACE

namespace fclrave {

#include <boost/shared_ptr.hpp>
#include <memory> // c++11
#include <vector>


typedef KinBody::LinkConstPtr LinkConstPtr;
typedef std::pair<LinkConstPtr, LinkConstPtr> LinkPair;
typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
typedef boost::weak_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerWeakPtr;
typedef boost::weak_ptr<const KinBody> KinBodyConstWeakPtr;
 using OpenRAVE::ORE_Assert;

/// \brief A broadphase collision manager together with the current update stamps of the bodies it contains
struct ManagerInstance {
    BroadPhaseCollisionManagerPtr pmanager;
    std::vector< std::pair<KinBodyConstWeakPtr, int> > vUpdateStamps;
};
typedef boost::shared_ptr<ManagerInstance> ManagerInstancePtr;
typedef boost::weak_ptr<ManagerInstance> ManagerInstanceWeakPtr;

// Warning : this is the only place where we use std::shared_ptr (for compatibility with fcl)
typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef boost::function<CollisionGeometryPtr (std::vector<fcl::Vec3f> const &points, std::vector<fcl::Triangle> const &triangles) > MeshFactory;
typedef std::vector<fcl::CollisionObject *> CollisionGroup;
typedef boost::shared_ptr<CollisionGroup> CollisionGroupPtr;
typedef std::pair<Transform, CollisionObjectPtr> TransformCollisionPair;


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
    std::shared_ptr< fcl::BVHModel<T> > const model = make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}



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
            LINK(KinBody::LinkPtr plink) : _plink(plink), nLastStamp(0) {
            }

            virtual ~LINK() {
                Reset();
            }

            void Reset() {

              BroadPhaseCollisionManagerPtr localEnvManager = _envManager.lock();
              if( !!localEnvManager && !!linkBV.second ) {
                localEnvManager->unregisterObject(linkBV.second.get());
              }

              linkBV.second.reset();

              FOREACH(itgeompair, vgeoms) {
                (*itgeompair).second->setUserData(nullptr);
                (*itgeompair).second.reset();
              }
              vgeoms.resize(0);
            }



            CollisionObjectPtr PrepareEnvManagerRegistering(BroadPhaseCollisionManagerPtr envManager) {
                _envManager = BroadPhaseCollisionManagerWeakPtr(envManager);
                return linkBV.second;
            }

            void Register(BroadPhaseCollisionManagerPtr envManager) {
                BroadPhaseCollisionManagerPtr localEnvManager = _envManager.lock();
                // if the localEnvManager is already set then the link is already registered
                if( !localEnvManager ) {
                    _envManager = BroadPhaseCollisionManagerWeakPtr(envManager);
                    envManager->registerObject(linkBV.second.get());
                } else {
                    // debug only
                    BOOST_ASSERT( localEnvManager == envManager );
                    envManager->update(linkBV.second.get(), false);
                }
            }

            void Unregister() {
                BroadPhaseCollisionManagerPtr localEnvManager = _envManager.lock();
                // if the localEnvManager does not exists we don't need to unregister
                if( !!localEnvManager ) {
                    localEnvManager->unregisterObject(linkBV.second.get());
                    _envManager.reset();
                }
            }

            KinBody::LinkPtr GetLink() {
                return _plink.lock();
            }

            KinBody::LinkWeakPtr _plink;

          int nLastStamp; ///< Tracks if the collision geometries are up to date wrt the body update stamp
          BroadPhaseCollisionManagerWeakPtr _envManager; ///< If set the linkBV is part of this collision manager
          TransformCollisionPair linkBV; ///< pair of the transformation and collision object corresponding to a bounding OBB for the link
          std::vector<TransformCollisionPair> vgeoms; ///< vector of transformations and collision object; one per geometries
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
            _bodyManager.reset();
            _bodyManagerActiveDOFs.reset();

            // The LINKs should not unregister themselves from _bodyManager or _bodyManagerActiveDOFs anymore so it should be safe to reset them after these
            FOREACH(itlink, vlinks) {
                (*itlink)->Reset();
            }
            vlinks.resize(0);
            _geometrycallback.reset();
            _geometrygroupcallback.reset();
            _excludecallback.reset();
        }

        void UnregisterAllLinks() {
            FOREACH(itpLINK, vlinks) {
                (*itpLINK)->Unregister();
            }
        }

        bool UpdateLinksRegisterStatus(BroadPhaseCollisionManagerPtr envManager) {
            bool bhasRegisteredLink = false;
            FOREACH(itpLINK, vlinks) {
                if( (*itpLINK)->GetLink()->IsEnabled() ) {
                    bhasRegisteredLink = true;
                    (*itpLINK)->Register(envManager);
                } else {
                    (*itpLINK)->Unregister();
                }
            }
            return bhasRegisteredLink;
        }

        void _ResetBodyManagers() {
            // we invalidate both managers
            _bodyManager.reset();
            _bodyManagerActiveDOFs.reset();
            // we reinitialize the link-enabled callbacks
            _linkEnabledCallbacks.clear();
        }

        void _ChangeActiveDOFsFlag() {
            _bactiveDOFsDirty = true;
            _bodyManagerActiveDOFs.reset();
        }

        KinBodyPtr GetBody()
        {
            return _pbody.lock();
        }

        KinBodyWeakPtr _pbody;
        int nLastStamp;  // used during synchronization ("is transform up to date")
        vector< boost::shared_ptr<LINK> > vlinks;

        ManagerInstancePtr _bodyManager; ///< Broad phase manager containing all the enabled links of the kinbody (does not contain attached kinbodies' links)

        bool _bactiveDOFsDirty; ///< true if some active link has been added or removed since the last construction of _bodyManagerActiveDOFs
        ManagerInstancePtr _bodyManagerActiveDOFs; ///< Broad phase manager containing all the active links of the kinbody (does not contain attached kinbodies' links)
        std::vector<int> _vactiveLinks; ///< ith element is 1 if the ith link of the kinbody is active, 0 otherwise ; ensured to be correct only after a call to GetBodyManager(true)

        OpenRAVE::UserDataPtr _bodyAttachedCallback; ///< handle for the callback called when a body is attached or detached
        OpenRAVE::UserDataPtr _activeDOFsCallback; ///< handle for the callback called when a the activeDOFs have changed
        std::list<OpenRAVE::UserDataPtr> _linkEnabledCallbacks;

        OpenRAVE::UserDataPtr _geometrycallback; ///< handle for the callback called when the current geometry of the kinbody changed ( Prop_LinkGeometry )
        OpenRAVE::UserDataPtr _geometrygroupcallback; ///< handle for the callback called when some geometry group of one of the links of this kinbody changed ( Prop_LinkGeometryGroup )
        OpenRAVE::UserDataPtr _excludecallback; ///< handle for the callback called when some link enable status of this kinbody has changed so that the envManager is updated ( Prop_LinkEnable )

        std::string _geometrygroup; ///< name of the geometry group tracked by this kinbody info ; if empty, tracks the current geometries
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::shared_ptr<FCLSpace::KinBodyInfo::LINK> LinkInfoPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : _penv(penv), _userdatakey(userdatakey)
    {
      // After many test, OBB seems to be the only real option (followed by kIOS)
        SetBVHRepresentation("OBB");
    }

    virtual ~FCLSpace()
    {
        DestroyEnvironment();
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE("destroying fcl collision environment\n");
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
        _unstableBodies.clear();
        _setInitializedBodies.clear();
    }


    KinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr())
    {
        EnvironmentMutex::scoped_lock lock(pbody->GetEnv()->GetMutex());

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
                RAVELOG_ERROR("Found a link with 0 geometries, this may create some bugs...");
                continue;
            }

            if( link->vgeoms.size() == 1) {
                // set the unique geometry as its own bounding volume
                link->linkBV = link->vgeoms[0];
            } else {
                // create the bounding volume for the link
                fcl::BVHModel<fcl::OBB> model;
                model.beginModel();
                // TODO : Check if I can assume that the collision mesh are already initialized
                for(GeometryInfoIterator it = begingeom; it != endgeom; ++it) {
                    _bvAddSubmodelFromGeomInfo(model, *it);
                }
                model.endModel();
                OPENRAVE_ASSERT_OP( model.getNumBVs(), !=, 0);
                link->linkBV = _CreateTransformCollisionPairFromOBB(model.getBV(0).bv);
                link->linkBV.second->setUserData(link.get());
            }

            link->nLastStamp = pinfo->nLastStamp;
            link->bodylinkname = pbody->GetName() + "/" + (*itlink)->GetName();
            pinfo->vlinks.push_back(link);
#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
            RAVELOG_DEBUG_FORMAT("FCLSPACECOLLISIONOBJECT|%s|%s", link->linkBV.second.get()%link->bodylinkname);
#endif
        }

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetCurrentGeometryCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBody const>(pbody)));
        pinfo->_geometrygroupcallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometryGroup, boost::bind(&FCLSpace::_ResetGeometryGroupsCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBody const>(pbody)));
        pinfo->_excludecallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&FCLSpace::_ExcludeWkBodyFromEnv, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<KinBody const>(pbody)));

        pbody->SetUserData(_userdatakey, pinfo);
        _ExcludeBodyFromEnv(pbody);
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
        if(groupname != _geometrygroup) {
            _geometrygroup = groupname;

            FOREACHC(itbody, _setInitializedBodies) {
                SetBodyGeometryGroup(*itbody, groupname);
            }
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

            if( !!_envManagerInstance ) {
                BOOST_ASSERT( !!_envManagerInstance->pmanager );
                FOREACH(itWkbodyStampPair, _envManagerInstance->vUpdateStamps) {
                    if( itWkbodyStampPair->first.lock() == pbody ) {
                        _envManagerInstance->vUpdateStamps.erase(itWkbodyStampPair);
                        break;
                    }
                }
                poldinfo->UnregisterAllLinks();
            }
            _cachedpinfo[(pbody)->GetEnvironmentId()][poldinfo->_geometrygroup] = poldinfo;

            KinBodyInfoPtr& pinfo = _cachedpinfo[pbody->GetEnvironmentId()][groupname];


            if(!pinfo) {
                RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%pbody->GetName()%pbody->GetEnvironmentId()%_penv->GetId());
                pinfo.reset(new KinBodyInfo);
                pinfo->_geometrygroup = groupname;
                InitKinBody(pbody, pinfo);
            } else {
                RAVELOG_VERBOSE_FORMAT("FCLSpace : switching to geometry %s for kinbody %s (id = %d) (env = %d)", groupname%pbody->GetName()%pbody->GetEnvironmentId()%_penv->GetId());
                // Set the current user data to use the KinBodyInfoPtr associated to groupname
                pbody->SetUserData(_userdatakey, pinfo);
                // Revoke the information inside the cache so that a potentially outdated object does not survive
                pinfo.reset();
            }
            // Notify to the environment manager that this kinbody must be added
            _ExcludeBodyFromEnv(pbody);
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

    void SetBodyUnstable(KinBodyConstPtr pbody, bool bsetUnstable) {
      RAVELOG_DEBUG_FORMAT("FCL User : Setting unstable status %d to body %s (env = %d, %s)", bsetUnstable%pbody->GetName()%_penv->GetId()%_userdatakey);
      if( bsetUnstable ) {
        _unstableBodies.insert(pbody);
      } else {
        _unstableBodies.erase(pbody);
      }
      if( !!GetInfo(pbody) ) {
        // excluding the body from env will either remove it from current environment or add it during the next update
        _ExcludeBodyFromEnv(pbody);
      }
    }

    bool IsBodyUnstable(KinBodyConstPtr pbody) const {
      return _unstableBodies.count(pbody);
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

    std::pair<KinBodyInfoPtr, bool> GetCreateInfo(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = this->GetInfo(pbody);
        bool bcreated = false;
        if( !pinfo ) {
            pinfo = InitKinBody(pbody, KinBodyInfoPtr());
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
            _ExcludeBodyFromEnv(pbody);
            _envExcludedBodies.erase(pbody);
            _unstableBodies.erase(pbody);
            _cachedpinfo.erase(pbody->GetEnvironmentId());
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
            // We may not need this anymore...
            pinfo->_ResetBodyManagers();
        }
        _envManagerInstance.reset();
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
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                pcoll->computeAABB();
            }
        }
    }


    const std::set<KinBodyConstPtr>& GetEnvBodies() const {
        return _setInitializedBodies;
    }

    const std::set<KinBodyConstPtr>& GetUnstableBodies() const {
      return _unstableBodies;
    }

    void SetEnvManagerInstance(ManagerInstancePtr envManagerInstance) {
        BOOST_ASSERT( !_envManagerInstance );
        _envManagerInstance = envManagerInstance;
    }

    ManagerInstancePtr GetEnvManagerInstance() const {
        return _envManagerInstance;
    }

    void SetEnvExcludiedBodiesId(const std::set<KinBodyConstPtr>& envExcludedBodiesId) {
        _envExcludedBodies = envExcludedBodiesId;
    }

    const std::set<KinBodyConstPtr>& GetEnvExcludiedBodies() const {
        return _envExcludedBodies;
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
                Transform pose = vtrans[i] * pinfo->vlinks[i]->linkBV.first;
                fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                pcoll->setTranslation(newPosition);
                pcoll->setQuatRotation(newOrientation);
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                pcoll->computeAABB();


                pinfo->vlinks[i]->nLastStamp = pinfo->nLastStamp;
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

            if( !!_synccallback ) {
                _synccallback(pinfo);
            }
        }
    }


    void _ResetCurrentGeometryCallback(boost::weak_ptr<KinBody const> _pbody)
    {
        KinBodyConstPtr pbody(_pbody);
        RAVELOG_DEBUG_FORMAT("Resetting current geometry for kinbody %s (in env %d, collision manager %s)", pbody->GetName()%_penv->GetId()%_userdatakey);
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        if( !!pinfo && pinfo->_geometrygroup == "" ) {
            InitKinBody(pbody, pinfo);
        }
        _cachedpinfo[pbody->GetEnvironmentId()].erase("");
    }

    void _ResetGeometryGroupsCallback(boost::weak_ptr<KinBody const> _pbody)
    {
        KinBodyConstPtr pbody(_pbody);
        RAVELOG_DEBUG_FORMAT("Resetting geometry groups for kinbody %s (in env %d, collision manager %s)", pbody->GetName()%_penv->GetId()%_userdatakey);
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        if( !!pinfo && pinfo->_geometrygroup != "" ) {
            InitKinBody(pbody, pinfo);
        }
        KinBodyInfoPtr pinfoCurrentGeometry = _cachedpinfo[pbody->GetEnvironmentId()][""];
        _cachedpinfo.erase(pbody->GetEnvironmentId());
        if( !!pinfoCurrentGeometry ) {
            _cachedpinfo[pbody->GetEnvironmentId()][""] = pinfoCurrentGeometry;
        }
    }

    void _ExcludeWkBodyFromEnv(boost::weak_ptr<KinBody const> _pbody) {
        KinBodyConstPtr pbody = _pbody.lock();
        if( !!pbody ) {
            _ExcludeBodyFromEnv(pbody);
        }
    }

    void _ExcludeBodyFromEnv(KinBodyConstPtr pbody) {
      if( _unstableBodies.count(pbody) ) {
            return;
        }
        _envExcludedBodies.insert(pbody);
        if( !!_envManagerInstance ) {
            FOREACH(itWkbodyStampPair, _envManagerInstance->vUpdateStamps) {
                if( itWkbodyStampPair->first.lock() == pbody ) {
                    _envManagerInstance->vUpdateStamps.erase(itWkbodyStampPair);
                    break;
                }
            }
        }
    }



    EnvironmentBasePtr _penv;
    std::string _userdatakey;
    std::string _geometrygroup;
    SynchronizeCallbackFn _synccallback;

    std::string _bvhRepresentation;
    MeshFactory _meshFactory;

    ManagerInstancePtr _envManagerInstance;
    std::set<KinBodyConstPtr> _envExcludedBodies; ///< Set of the kinbodies which are temporarily excluded of the environment

    std::set<KinBodyConstPtr> _unstableBodies; ///< Set of the kinbodies which should NOT be managed by the _envManagerInstance
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
