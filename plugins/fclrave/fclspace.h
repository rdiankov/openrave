#ifndef OPENRAVE_FCL_SPACE
#define OPENRAVE_FCL_SPACE

// TODO : I should put these in some namespace...

typedef KinBody::LinkConstPtr LinkConstPtr;
typedef std::pair<LinkConstPtr, LinkConstPtr> LinkPair;
typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
// Warning : this is the only place where we use std::shared_ptr (compatibility with fcl)
typedef shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
typedef boost::function<CollisionGeometryPtr (std::vector<fcl::Vec3f> const &points, std::vector<fcl::Triangle> const &triangles) > MeshFactory;
typedef std::vector<fcl::CollisionObject *> CollisionGroup;
typedef boost::shared_ptr<CollisionGroup> CollisionGroupPtr;

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

template <class T>
CollisionGeometryPtr ConvertMeshToFCL(std::vector<fcl::Vec3f> const &points,std::vector<fcl::Triangle> const &triangles)
{
    shared_ptr< fcl::BVHModel<T> > const model = make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}


// TODO : Erase me
bool testValidity(BroadPhaseCollisionManagerPtr m);

void UnregisterObjects(BroadPhaseCollisionManagerPtr manager, CollisionGroup& group)
{
  FOREACH(itcollobj, group) {
    manager->unregisterObject(*itcollobj);
  }
}

struct RestoreObjects {
  RestoreObjects(CollisionGroupPtr pgroup, BroadPhaseCollisionManagerPtr envManager) : _pgroup(pgroup), _penvManager(envManager) {
  }

  void operator()(fcl::BroadPhaseCollisionManager* pmanager) {
    BroadPhaseCollisionManagerPtr envManager = _penvManager.lock();
    if( !envManager ) {
      return;
    }
    if( pmanager ) {
      envManager->registerObjects(*_pgroup);
    } else {
      RAVELOG_WARN(str(boost::format("broadphase manager invalid : %d objects seems to have been lost") % _pgroup->size()));
    }
  }

  CollisionGroupPtr _pgroup;
  boost::weak_ptr<fcl::BroadPhaseCollisionManager> _penvManager;
};


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
        struct LINK
        {
          LINK(KinBody::LinkPtr plink) : _plink(plink) {
          }

            virtual ~LINK() {
              Reset(BroadPhaseCollisionManagerPtr());
            }

          void Reset(BroadPhaseCollisionManagerPtr pmanager) {
            FOREACH(itgeompair, vgeoms) {
              if( pmanager ) {
                pmanager->unregisterObject((*itgeompair).second.get());
              }
              (*itgeompair).second.reset();
            }
            vgeoms.resize(0);
          }

          /*
          KinBody::LinkPtr GetLink(EnvironmentBasePtr penv)
          {
            BOOST_ASSERT(penv);
            KinBodyPtr pbody = penv->GetBodyFromEnvironmentId(_parentEnvId);
            if( !pbody ) {
              return KinBody::LinkPtr();
            }
            return pbody->GetLinks()[_linkIndex];
            }

            int _parentEnvId;
            int _linkIndex;
          */
          KinBody::LinkPtr GetLink() {
            return _plink.lock();
          }

          KinBody::LinkWeakPtr _plink;

            // TODO : consider the possibility of having one broadPhaseCollisionManager per LINK

            std::vector<TransformCollisionPair> vgeoms; // fcl variant of ODE dBodyID
            std::string bodylinkname; // for debugging purposes
        };

        KinBodyInfo() : nLastStamp(0)
        {
        }

        virtual ~KinBodyInfo() {
          Reset(BroadPhaseCollisionManagerPtr());
        }

        void Reset(BroadPhaseCollisionManagerPtr pmanager)
        {
            FOREACH(itlink, vlinks) {
              (*itlink)->Reset(pmanager);
            }
            vlinks.resize(0);
            _geometrycallback.reset();
            // should-I reinitialize nLastStamp ?
        }

        KinBodyPtr GetBody()
        {
            return _pbody.lock();
        }

        KinBodyWeakPtr _pbody;  // could we make this const ?
        int nLastStamp;  // used during synchronization ("is transform up to date")
        vector< boost::shared_ptr<LINK> > vlinks;
        OpenRAVE::UserDataPtr _geometrycallback;
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : _penv(penv), _userdatakey(userdatakey), _allEnvInitialized(false)
    {
      _manager = _CreateManagerFromBroadphaseAlgorithm("Naive");
        // TODO : test best default choice
        SetBVHRepresentation("AABB");
        // Naive : initialization working
        // SaP : not working infinite loop at line 352 of Broadphase_SaP.cpp
        // SSaP : initialization working
        // IntervalTree : not working received SIGSEV at line 427 of interval_tree.cpp
        // DynamicAABBTree : initialization working
        // DynamicAABBTree_Array : initialization working
        SetBroadphaseAlgorithm("SSaP");
    }

    virtual ~FCLSpace()
    {
        DestroyEnvironment();
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE("destroying ode collision environment\n");
        _EraseAll(_userdatakey);
    }


    KinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr())
    {
      BOOST_ASSERT(testValidity(_manager));
        if( !pinfo ) {
            pinfo.reset(new KinBodyInfo());
        }

        pinfo->Reset(_manager);
        pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);

        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
          BOOST_ASSERT(testValidity(_manager));
            boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK(*itlink));
            pinfo->vlinks.push_back(link);
            link->bodylinkname = pbody->GetName() + "/" + (*itlink)->GetName();

            BOOST_ASSERT( link->GetLink() );

            if(_geometrygroup.size() > 0 && (*itlink)->GetGroupNumGeometries(_geometrygroup) >= 0) {
                const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = (*itlink)->GetGeometriesFromGroup(_geometrygroup);
                FOREACHC(itgeominfo, vgeometryinfos) {
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, **itgeominfo);
                    if( !pfclgeom ) {
                        continue;
                    }

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                    // why is there a const cast ?
                    pfclcoll->setUserData(link.get());
                    link->vgeoms.push_back(TransformCollisionPair((*itgeominfo)->_t, pfclcoll));
                    _manager->registerObject(pfclcoll.get());
                    BOOST_ASSERT(testValidity(_manager));
                }
            } else {
                FOREACHC(itgeom, (*itlink)->GetGeometries()) {
                    const KinBody::GeometryInfo& geominfo = (*itgeom)->GetInfo();
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(_meshFactory, geominfo);
                    if( !pfclgeom ) {
                        continue;
                    }

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                    pfclcoll->setUserData(link.get());
                    link->vgeoms.push_back(TransformCollisionPair(geominfo._t, pfclcoll));
                    _manager->registerObject(pfclcoll.get());
                    BOOST_ASSERT( link->GetLink() );
                    BOOST_ASSERT(testValidity(_manager));
                }
            }
        }

        BOOST_ASSERT( pinfo->vlinks.size() == pbody->GetLinks().size());

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetKinBodyCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<KinBody const>(pbody)));

        pbody->SetUserData(_userdatakey, pinfo);
        _Insert(pbody);

        // make sure that synchronization do occur !
        pinfo->nLastStamp = pbody->GetUpdateStamp() - 1;
        _Synchronize(pinfo);

        return pinfo;
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        if(groupname != _geometrygroup) {
            _geometrygroup = groupname;

            // _initializedBodies could be used here
            std::vector<KinBodyPtr> vbodies;
            _penv->GetBodies(vbodies);
            FOREACHC(itbody, vbodies) {
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

    void GetCollisionObjects(KinBodyConstPtr pbody, CollisionGroup& group)
    {
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        BOOST_ASSERT( pinfo->GetBody() == pbody );
        FOREACHC(itlink, pinfo->vlinks) {
            _GetCollisionObjects(*itlink, group);
        }
    }

    void GetCollisionObjects(LinkConstPtr plink, CollisionGroup &group)
    {
        KinBodyInfoPtr pinfo = GetInfo(plink->GetParent());
        BOOST_ASSERT( pinfo->GetBody() == plink->GetParent() );
        BOOST_ASSERT( plink->GetIndex() >= 0 && plink->GetIndex() < (int)pinfo->vlinks.size());
        return _GetCollisionObjects(pinfo->vlinks[plink->GetIndex()], group);
    }

  // Warning : the managers need to be set up before usage
  void GetManagers(CollisionGroupPtr pgroup1, CollisionGroupPtr pgroup2, BroadPhaseCollisionManagerPtr& manager1, BroadPhaseCollisionManagerPtr& manager2, BroadPhaseCollisionManagerPtr& envManager, bool bagainstEnv) {
    _CreateTemporaryManagerFromBroadphaseAlgorithm(manager1, pgroup1, true);
    _CreateTemporaryManagerFromBroadphaseAlgorithm(manager2, pgroup2, !bagainstEnv);
    envManager.reset(_manager, _manager.get());
  }

    BroadPhaseCollisionManagerPtr GetEnvManager() const {
        return _manager;
    }

    BroadPhaseCollisionManagerPtr CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }


    // Already existing geometry not updated !
    // Is that the behaviour we want ?
    void SetBVHRepresentation(std::string const &type)
    {
        if (type == "AABB") {
            _meshFactory = &ConvertMeshToFCL<fcl::AABB>;
        } else if (type == "OBB") {
            _meshFactory = &ConvertMeshToFCL<fcl::OBB>;
        } else if (type == "RSS") {
            _meshFactory = &ConvertMeshToFCL<fcl::RSS>;
        } else if (type == "OBBRSS") {
            _meshFactory = &ConvertMeshToFCL<fcl::OBBRSS>;
        } else if (type == "kIDS") {
            _meshFactory = &ConvertMeshToFCL<fcl::AABB>;
        } else {
            RAVELOG_WARN(str(boost::format("Unknown BVH representation '%s'.") % type));
        }
    }

    void SetBroadphaseAlgorithm(std::string const &algorithm)
    {
      if(_broadPhaseCollisionManagerAlgorithm == algorithm) {
        return;
      }
      _broadPhaseCollisionManagerAlgorithm = algorithm;

      CollisionGroup vcollObjects;
      _manager->getObjects(vcollObjects);
      _manager = _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
      _manager->registerObjects(vcollObjects);
    }


    void RemoveUserData(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            bool is_consistent = _Erase(pbody, _userdatakey);
            if( !is_consistent ) {
                RAVELOG_WARN("inconsistency detected with odespace user data\n");
            }
        }
    }


    void Synchronize()
    {
        _InsertAll();
        std::vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        FOREACH(itbody, vbodies) {
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



private:

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
            return make_shared<fcl::Cylinder>(info._vGeomData.x, info._vGeomData.y);

        case OpenRAVE::GT_TriMesh:
        {
            const OpenRAVE::TriMesh& mesh = info._meshcollision;
            if (mesh.vertices.empty() || mesh.indices.empty()) {
                return CollisionGeometryPtr();
            }

            BOOST_ASSERT(mesh.indices.size() % 3 == 0);
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
                FOREACHC(itgeomcoll, pinfo->vlinks[i]->vgeoms) {
                    CollisionObjectPtr collObj = (*itgeomcoll).second;
                    Transform pose = vtrans[i] * (*itgeomcoll).first;
                    fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
                    fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

                    collObj->setTranslation(newPosition);
                    collObj->setQuatRotation(newOrientation);
                    collObj->computeAABB();
                    _manager->update(collObj.get());
                }
            }

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

    static BroadPhaseCollisionManagerPtr _CreateManagerFromBroadphaseAlgorithm(std::string const &algorithm)
    {
        if(algorithm == "Naive") {
            return boost::make_shared<fcl::NaiveCollisionManager>();
        } else if(algorithm == "SaP") {
            return boost::make_shared<fcl::SaPCollisionManager>();
        } else if(algorithm == "SSaP") {
            return boost::make_shared<fcl::SSaPCollisionManager>();
        } else if(algorithm == "IntervalTree") {
            return boost::make_shared<fcl::IntervalTreeCollisionManager>();
        } else if(algorithm == "DynamicAABBTree") {
          return boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        } else if(algorithm == "DynamicAABBTree_Array") {
          return boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        } else {
            throw OpenRAVE::openrave_exception(str(boost::format("Unknown broad-phase algorithm '%s'.") % algorithm), OpenRAVE::ORE_InvalidArguments);
        }
    }

  void _CreateTemporaryManagerFromBroadphaseAlgorithm(BroadPhaseCollisionManagerPtr& pmanager, CollisionGroupPtr pgroup, bool binitializeManager)
    {
      RestoreObjects deleter(pgroup, _manager);
      if( binitializeManager ) {
        if(_broadPhaseCollisionManagerAlgorithm == "Naive") {
          pmanager.reset(new fcl::NaiveCollisionManager(), deleter);
        } else if(_broadPhaseCollisionManagerAlgorithm == "SaP") {
          pmanager.reset(new fcl::SaPCollisionManager(), deleter);
        } else if(_broadPhaseCollisionManagerAlgorithm == "SSaP") {
          pmanager.reset(new fcl::SSaPCollisionManager(), deleter);
        } else if(_broadPhaseCollisionManagerAlgorithm == "IntervalTree") {
          pmanager.reset(new fcl::IntervalTreeCollisionManager(), deleter);
        } else if(_broadPhaseCollisionManagerAlgorithm == "DynamicAABBTree") {
          pmanager.reset(new fcl::DynamicAABBTreeCollisionManager(), deleter);
        } else if(_broadPhaseCollisionManagerAlgorithm == "DynamicAABBTree_Array") {
          pmanager.reset(new fcl::DynamicAABBTreeCollisionManager_Array(), deleter);
        } else {
          throw OpenRAVE::openrave_exception(str(boost::format("Unknown broadphase algorithm '%s'.") % _broadPhaseCollisionManagerAlgorithm), OpenRAVE::ORE_InvalidArguments);
        }
        pmanager->registerObjects(*pgroup);
        UnregisterObjects(_manager, *pgroup);
      } else if ( pgroup->size() > 0 ) {
        pmanager = boost::shared_ptr<fcl::BroadPhaseCollisionManager>(static_cast<fcl::BroadPhaseCollisionManager*>(NULL), deleter);
        UnregisterObjects(_manager, *pgroup);
      }
    }



    // TODO : consider changing vgeoms so that we can just add them in one go
  void _GetCollisionObjects(boost::shared_ptr<KinBodyInfo::LINK> pLink, CollisionGroup &group)
    {
        group.reserve(group.size()+pLink->vgeoms.size());
        FOREACH(itgeomcoll, pLink->vgeoms) {
          BOOST_ASSERT( !!(*itgeomcoll).second );
          BOOST_ASSERT( static_cast<KinBodyInfo::LINK *>((*itgeomcoll).second->getUserData()) );
          BOOST_ASSERT( static_cast<KinBodyInfo::LINK *>((*itgeomcoll).second->getUserData())->GetLink() );
          group.push_back((*itgeomcoll).second.get());
        }
    }

    void _Insert(KinBodyConstPtr pbody)
    {
        if( !_allEnvInitialized ) {
            _setInitializedBodies.insert(pbody);
        }
    }

    void _InsertAll()
    {
        _setInitializedBodies.clear();
        _allEnvInitialized = true;
    }

    bool _Erase(KinBodyConstPtr pbody, const std::string& userdatakey)
    {
        if( _allEnvInitialized ) {
            std::vector<KinBodyPtr> vbodies;
            _penv->GetBodies(vbodies);
            std::copy(vbodies.begin(), vbodies.end(), std::inserter(_setInitializedBodies, _setInitializedBodies.begin()));
        }
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        FOREACH(itlink, pinfo->vlinks) {
            FOREACH(itgeomcoll, (*itlink)->vgeoms) {
                _manager->unregisterObject((*itgeomcoll).second.get());
            }
        }
        bool bremoved = pbody->RemoveUserData(userdatakey);
        size_t numerased = _setInitializedBodies.erase(pbody);
        return ( bremoved == numerased );
    }

    void _EraseAll(const std::string& userdatakey)
    {
        _manager->clear();
        if( _allEnvInitialized ) {
            std::vector<KinBodyPtr> vbodies;
            _penv->GetBodies(vbodies);
            FOREACH(itbody, vbodies) {
                (*itbody)->RemoveUserData(userdatakey);
            }
        } else {
            FOREACH(itbody, _setInitializedBodies) {
                (*itbody)->RemoveUserData(userdatakey);
            }
        }
    }

    EnvironmentBasePtr _penv;
    std::string _userdatakey;
    std::string _geometrygroup;
    SynchronizeCallbackFn _synccallback;

    std::string _broadPhaseCollisionManagerAlgorithm;
    BroadPhaseCollisionManagerPtr _manager;
    MeshFactory _meshFactory;

    bool _allEnvInitialized;
    std::set<KinBodyConstPtr> _setInitializedBodies;

  // TODO : erase me 
  bool testValidity(BroadPhaseCollisionManagerPtr m) {
    CollisionGroup group;
    m->getObjects(group);
    FOREACH(ito, group) {
      BOOST_ASSERT(*ito != NULL);

      FCLSpace::KinBodyInfo::LINK *link_raw = static_cast<FCLSpace::KinBodyInfo::LINK *>((*ito)->getUserData());
      BOOST_ASSERT( link_raw != NULL );
      return !!link_raw->GetLink();
    }
    return true;
  }
};





#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(FCLSpace::KinBodyInfo::LINK)
#endif


#endif
