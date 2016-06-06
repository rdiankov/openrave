#ifndef OPENRAVE_FCL_COLLISION
#define OPENRAVE_FCL_COLLISION

#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>
#include <boost/function_output_iterator.hpp>

#include "fclspace.h"

//#define FCLUSESTATISTICS 1
#include "fclstatistics.h"

namespace fclrave {

#define START_TIMING_OPT(statistics, label, options, isRobot);           \
    START_TIMING(statistics, boost::str(boost::format("%s,%x,%d")%label%options%isRobot))

typedef FCLSpace::KinBodyInfoConstPtr KinBodyInfoConstPtr;
typedef FCLSpace::KinBodyInfoPtr KinBodyInfoPtr;

template<typename T>
inline bool IsIn(T const& x, std::vector<T> const &collection) {
    return std::find(collection.begin(), collection.end(), x) != collection.end();
}


typedef std::pair<fcl::CollisionObject*, fcl::CollisionObject*> CollisionPair;
} // fclrave

namespace std {
template<>
struct hash<fclrave::CollisionPair>
{
    size_t operator()(const fclrave::CollisionPair& collpair) const {
        static const size_t shift = (size_t)log2(1 + sizeof(fcl::CollisionObject*));
        size_t seed = (size_t)(collpair.first) >> shift;
        boost::hash_combine(seed, (size_t)(collpair.second) >> shift);
        return seed;
    }

};
} // std


namespace fclrave {

typedef std::unordered_map<CollisionPair, fcl::Vec3f> NarrowCollisionCache;



class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:

    struct SpatialHashData {
        SpatialHashData(fcl::FCL_REAL cellSize, const fcl::Vec3f& sceneMin, const fcl::Vec3f& sceneMax) : _cellSize(cellSize), _sceneMin(sceneMin), _sceneMax(sceneMax) {
        }
        fcl::FCL_REAL _cellSize;
        fcl::Vec3f _sceneMin, _sceneMax;
    };


    class CollisionCallbackData {
public:
        CollisionCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker, CollisionReportPtr report, std::vector<KinBodyConstPtr> vbodyexcluded = std::vector<KinBodyConstPtr>(), std::vector<LinkConstPtr> vlinkexcluded = std::vector<LinkConstPtr>()) : _pchecker(pchecker), _report(report), _vbodyexcluded(vbodyexcluded), _vlinkexcluded(vlinkexcluded), bselfCollision(false), _bStopChecking(false), _bCollision(false)
        {
            _bHasCallbacks = _pchecker->GetEnv()->HasRegisteredCollisionCallbacks();
            if( _bHasCallbacks && !_report ) {
                _report.reset(new CollisionReport());
            }

            // TODO : What happens if we have CO_AllGeometryContacts set and not CO_Contacts ?
            // TODO not sure what's happening with FCL's contact computation. is it really disabled?
            if( !!report && !!(_pchecker->GetCollisionOptions() & OpenRAVE::CO_Contacts) ) {
                _request.num_max_contacts = _pchecker->GetNumMaxContacts();
                _request.enable_contact = true;
            } else {
                _request.enable_contact = false; // explicitly disable
            }

            // set the gjk solver (collision checking between convex bodies) so that we can use hints
            _request.gjk_solver_type = fcl::GST_INDEP;

            if( !!_report ) {
                _report->Reset(_pchecker->GetCollisionOptions());
            }
        }

        const std::list<EnvironmentBase::CollisionCallbackFn>& GetCallbacks() {
            if( _bHasCallbacks &&( _listcallbacks.size() == 0) ) {
                _pchecker->GetEnv()->GetRegisteredCollisionCallbacks(_listcallbacks);
            }
            return _listcallbacks;
        }

        boost::shared_ptr<FCLCollisionChecker> _pchecker;
        fcl::CollisionRequest _request;
        fcl::CollisionResult _result;
        CollisionReportPtr _report;
        std::vector<KinBodyConstPtr> const& _vbodyexcluded;
        std::vector<LinkConstPtr> const& _vlinkexcluded;
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        bool bselfCollision;  ///< true if currently checking for self collision.
        bool _bStopChecking;  ///< if true, then stop the collision checking loop
        bool _bCollision;  ///< result of the collision

        bool _bHasCallbacks; ///< true if there's callbacks registered in the environment
        std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    };

    typedef boost::shared_ptr<CollisionCallbackData> CollisionCallbackDataPtr;

    typedef boost::shared_ptr<FCLSpace::KinBodyInfo::LINK> LinkInfoPtr;

    // initialization of _broadPhaseCollisionManagerAlgorithm parameter :
    // Naive : initialization working
    // SaP : not working infinite loop at line 352 of Broadphase_SaP.cpp
    // SSaP : initialization working
    // IntervalTree : not working received SIGSEV at line 427 of interval_tree.cpp
    // DynamicAABBTree : initialization working
    // DynamicAABBTree_Array : initialization working, problems with unregister
    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
        : OpenRAVE::CollisionCheckerBase(penv), _bIsSelfCollisionChecker(true), _broadPhaseCollisionManagerAlgorithm("DynamicAABBTree2_Array")
    {

        _userdatakey = std::string("fclcollision") + boost::lexical_cast<std::string>(this);
        _fclspace.reset(new FCLSpace(penv, _userdatakey));
        _options = 0;
        _numMaxContacts = std::numeric_limits<int>::max(); // TODO
        __description = ":Interface Author: Kenji Maillard\n\nFlexible Collision Library collision checker";
        SETUP_STATISTICS(_statistics, _userdatakey, GetEnv()->GetId());

        RegisterCommand("SetBroadphaseAlgorithm", boost::bind(&FCLCollisionChecker::_SetBroadphaseAlgorithm, this, _1, _2), "sets the broadphase algorithm (Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTree_Array)");

        RegisterCommand("SetBVHRepresentation", boost::bind(&FCLCollisionChecker::_SetBVHRepresentation, this, _1, _2), "sets the Bouding Volume Hierarchy representation for meshes (AABB, OBB, OBBRSS, RSS, kIDS)");
        // TODO : check that the coordinate are in the right order
        RegisterCommand("SetSpatialHashingBroadPhaseAlgorithm", boost::bind(&FCLCollisionChecker::SetSpatialHashingBroadPhaseAlgorithm, this, _1, _2), "sets the broadphase algorithm to spatial hashing with (cell size) (scene min x) (scene min y) (scene min z) (scene max x) (scene max y) (scene max z)");
        RAVELOG_VERBOSE_FORMAT("FCLCollisionChecker %s created in env %d", _userdatakey%penv->GetId());

        std::string broadphasealg, bvhrepresentation;
        sinput >> broadphasealg >> bvhrepresentation;
        if( broadphasealg != "" ) {
            if( broadphasealg == "SpatialHashing" ) {
                std::ostream nullout(nullptr);
                SetSpatialHashingBroadPhaseAlgorithm(nullout, sinput);
            } else {
                SetBroadphaseAlgorithm(broadphasealg);
            }
        }
        if( bvhrepresentation != "" ) {
            _fclspace->SetBVHRepresentation(bvhrepresentation);
        }
    }

    virtual ~FCLCollisionChecker() {
        RAVELOG_VERBOSE_FORMAT("FCLCollisionChecker %s destroyed in env %d", _userdatakey%GetEnv()->GetId());
        DestroyEnvironment();
    }

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<FCLCollisionChecker const> r = boost::dynamic_pointer_cast<FCLCollisionChecker const>(preference);
        _fclspace->SetGeometryGroup(r->GetGeometryGroup());
        _fclspace->SetBVHRepresentation(r->GetBVHRepresentation());

        const std::string &broadphaseAlgorithm = r->GetBroadphaseAlgorithm();
        if( broadphaseAlgorithm == "SpatialHashing" ) {
            SetSpatialHashingBroadPhaseAlgorithm(*r->GetSpatialHashData());
        } else {
            SetBroadphaseAlgorithm(broadphaseAlgorithm);
        }
        // We don't want to clone _bIsSelfCollisionChecker since a self collision checker can be created by cloning a environment collision checker
        _options = r->_options;
        _numMaxContacts = r->_numMaxContacts;
        RAVELOG_VERBOSE(str(boost::format("FCL User data cloning env %d into env %d") % r->GetEnv()->GetId() % GetEnv()->GetId()));
    }



    void SetNumMaxContacts(int numMaxContacts) {
        _numMaxContacts = numMaxContacts;
    }

    int GetNumMaxContacts() const {
        return _numMaxContacts;
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        _fclspace->SetGeometryGroup(groupname);
    }

    const std::string& GetGeometryGroup() const
    {
        return _fclspace->GetGeometryGroup();
    }

    void SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
        _fclspace->SetBodyGeometryGroup(pbody, groupname);
    }


    virtual bool SetCollisionOptions(int collision_options)
    {
        _options = collision_options;

        // TODO : remove when distance is implemented
        if( _options & OpenRAVE::CO_Distance ) {
            return false;
        }

        if( _options & OpenRAVE::CO_RayAnyHit ) {
            return false;
        }

        return true;
    }

    virtual int GetCollisionOptions() const
    {
        return _options;
    }

    virtual void SetTolerance(OpenRAVE::dReal tolerance)
    {
    }


    /// Sets the broadphase algorithm for collision checking
    /// The input algorithm can be one of : Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTree_Array, SpatialHashing
    /// e.g. "SetBroadPhaseAlgorithm DynamicAABBTree"
    /// In order to use SpatialHashing, the spatial hashing data must be set beforehand with SetSpatialHashingBroadPhaseAlgorithm
    bool _SetBroadphaseAlgorithm(ostream& sout, istream& sinput)
    {
        std::string algorithm;
        sinput >> algorithm;
        SetBroadphaseAlgorithm(algorithm);
        return !!sinput;
    }

    void SetBroadphaseAlgorithm(const std::string &algorithm)
    {
        if(_broadPhaseCollisionManagerAlgorithm == algorithm) {
            return;
        }
        _broadPhaseCollisionManagerAlgorithm = algorithm;

        _fclspace->InvalidateCachedManagers();
    }

    const std::string & GetBroadphaseAlgorithm() const {
        return _broadPhaseCollisionManagerAlgorithm;
    }


    /// Sets the spatial hashing data and switch to the spatial hashing broadphase algorithm
    /// e.g. SetSpatialHashingBroadPhaseAlgorithm cell_size scene_min_x scene_min_y scene_min_z scene_max_x scene_max_y scene_max_z
    bool SetSpatialHashingBroadPhaseAlgorithm(ostream& sout, istream& sinput)
    {
        fcl::FCL_REAL cell_size;
        fcl::Vec3f scene_min, scene_max;
        sinput >> cell_size >> scene_min[0] >> scene_min[1] >> scene_min[2] >> scene_max[0] >> scene_max[1] >> scene_max[2];
        SetSpatialHashingBroadPhaseAlgorithm(SpatialHashData(cell_size, scene_min, scene_max));
        return !!sinput;
    }

    void SetSpatialHashingBroadPhaseAlgorithm(const SpatialHashData & data) {
        _spatialHashData = boost::make_shared<SpatialHashData>(data);
        SetBroadphaseAlgorithm("SpatialHashing");
    }

    boost::shared_ptr<const SpatialHashData> GetSpatialHashData() const {
        return boost::const_pointer_cast<SpatialHashData>(_spatialHashData);
    }


    /// Sets the bounding volume hierarchy representation which can be one of
    /// AABB, OBB, RSS, OBBRSS, kDOP16, kDOP18, kDOP24, kIOS
    /// e.g. "SetBVHRepresentation OBB"
    bool _SetBVHRepresentation(ostream& sout, istream& sinput)
    {
        std::string type;
        sinput >> type;
        _fclspace->SetBVHRepresentation(type);
        return !!sinput;
    }

    std::string const& GetBVHRepresentation() const {
        return _fclspace->GetBVHRepresentation();
    }


    virtual bool InitEnvironment()
    {
        RAVELOG_VERBOSE(str(boost::format("FCL User data initializing %s in env %d") % _userdatakey % GetEnv()->GetId()));
        _bIsSelfCollisionChecker = false;
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            InitKinBody(*itbody);
        }
        return true;
    }

    virtual void DestroyEnvironment()
    {
        RAVELOG_VERBOSE(str(boost::format("FCL User data destroying %s in env %d") % _userdatakey % GetEnv()->GetId()));
        _fclspace->DestroyEnvironment();
    }

    virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        FCLSpace::KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<FCLSpace::KinBodyInfo>(pbody->GetUserData(_userdatakey));
        if( !pinfo || pinfo->GetBody() != pbody ) {
            pinfo = _fclspace->InitKinBody(pbody);
        }
        return !pinfo;
    }

    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        _fclspace->RemoveUserData(pbody);
    }





    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "Body/Env",_options,pbody1->IsRobot());
        // TODO : tailor this case when stuff become stable enough
        return CheckCollision(pbody1, std::vector<KinBodyConstPtr>(), std::vector<LinkConstPtr>(), report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "Body/Body",_options,(pbody1->IsRobot() || pbody2->IsRobot()));
        if( !!report ) {
            report->Reset(_options);
        }

        if( pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled() ) {
            return false;
        }

        if( pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled() ) {
            return false;
        }

        if( pbody1->IsAttached(pbody2) ) {
            return false;
        }

        // Do we really want to synchronize everything ?
        // We could put the synchronization directly inside GetBodyManager
        //_fclspace->Synchronize();

        BroadPhaseCollisionManagerPtr body1Manager = GetBodyManager(pbody1, _options & OpenRAVE::CO_ActiveDOFs), body2Manager = GetBodyManager(pbody2, _options & OpenRAVE::CO_ActiveDOFs);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; //TODO
        } else {
            CollisionCallbackData query(shared_checker(), report);
            ADD_TIMING(_statistics);
            body1Manager->collide(body2Manager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
            return query._bCollision;
        }

    }

    virtual bool CheckCollision(LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "Link/Env",_options,false);
        // TODO : tailor this case when stuff become stable enough
        return CheckCollision(plink, std::vector<KinBodyConstPtr>(), std::vector<LinkConstPtr>(), report);
    }

    virtual bool CheckCollision(LinkConstPtr plink1, LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "Link/Link",_options,false);
        if( !!report ) {
            report->Reset(_options);
        }

        if( !plink1->IsEnabled() || !plink2->IsEnabled() ) {
            return false;
        }

        _fclspace->Synchronize(plink1->GetParent());
        _fclspace->Synchronize(plink2->GetParent());

        CollisionObjectPtr pcollLink1 = GetLinkBV(plink1), pcollLink2 = GetLinkBV(plink2);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            if( !pcollLink1->getAABB().overlap(pcollLink2->getAABB()) ) {
                return false;
            }
            CollisionCallbackData query(shared_checker(), report);
            ADD_TIMING(_statistics);
            CheckNarrowPhaseCollision(pcollLink1.get(), pcollLink2.get(), &query);
            return query._bCollision;
        }
    }

    virtual bool CheckCollision(LinkConstPtr plink, KinBodyConstPtr pbody,CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "Link/Body",_options,pbody->IsRobot());

        if( !!report ) {
            report->Reset(_options);
        }

        if( !plink->IsEnabled() ) {
            return false;
        }

        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
            return false;
        }

        if( pbody->IsAttached(plink->GetParent()) ) {
            return false;
        }

        _fclspace->Synchronize(plink->GetParent());

        CollisionObjectPtr pcollLink = GetLinkBV(plink);
        // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
        BroadPhaseCollisionManagerPtr bodyManager = GetBodyManager(pbody, false);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            CollisionCallbackData query(shared_checker(), report);
            ADD_TIMING(_statistics);
            bodyManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
            return query._bCollision;
        }
    }

    virtual bool CheckCollision(LinkConstPtr plink, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())
    {

        if( !!report ) {
            report->Reset(_options);
        }

        if( !plink->IsEnabled() || find(vlinkexcluded.begin(), vlinkexcluded.end(), plink) != vlinkexcluded.end() ) {
            return false;
        }

        _fclspace->Synchronize();


        CollisionObjectPtr pcollLink = GetLinkBV(plink);

        std::set<KinBodyConstPtr> attachedBodies;
        plink->GetParent()->GetAttached(attachedBodies);
        BroadPhaseCollisionManagerPtr envManager = GetEnvManager(attachedBodies);
//      BOOST_ASSERT(static_cast<fcl::DynamicAABBTreeCollisionManager_Array*>(envManager.get())->isValid());

        if( _options & OpenRAVE::CO_Distance ) {
            return false;
        } else {
            CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
            ADD_TIMING(_statistics);
            envManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

            // TODO : consider testing this before the env
            // The robots are not contained in the envManager, we need to consider them separately
            if( !query._bStopChecking ) {
                // TODO : precompute the list of robots
                FOREACH(itbody, _fclspace->GetEnvBodies() ) {
                    if( (*itbody)->IsRobot() ) {

                        if( (*itbody)->IsAttached(plink->GetParent()) ) {
                            continue;
                        }
                        // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
                        BroadPhaseCollisionManagerPtr robotManager = GetBodyManager(*itbody, false);
                        robotManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

                        if( query._bStopChecking ) {
                            return query._bCollision;
                        }
                    }
                }
            }

            return query._bCollision;
        }
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())
    {
        if( !!report ) {
            report->Reset(_options);
        }

        if( (pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            return false;
        }

        _fclspace->Synchronize();

        BroadPhaseCollisionManagerPtr bodyManager = GetBodyManager(pbody, _options & OpenRAVE::CO_ActiveDOFs);

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        BroadPhaseCollisionManagerPtr envManager = GetEnvManager(attachedBodies);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
            ADD_TIMING(_statistics);
            envManager->collide(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

            // The robots are not contained in the envManager, we need to consider them separately
            if( !query._bStopChecking ) {
                FOREACH(itbody, _fclspace->GetEnvBodies() ) {
                    if( (*itbody)->IsRobot() ) {

                        if( (*itbody)->IsAttached(pbody) ) {
                            continue;
                        }
                        // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
                        BroadPhaseCollisionManagerPtr robotManager = GetBodyManager(*itbody, false);
                        robotManager->collide(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

                        if( query._bStopChecking ) {
                            return query._bCollision;
                        }
                    }
                }
            }

            return query._bCollision;
        }
    }

    virtual bool CheckCollision(RAY const &ray, LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr())
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(RAY const &ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(RAY const &ray, CollisionReportPtr report = CollisionReportPtr())
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "BodySelf",_options,pbody->IsRobot());
        if( !!report ) {
            report->Reset(_options);
        }

        if( pbody->GetLinks().size() <= 1 ) {
            return false;
        }

        // We only want to consider the enabled links
        int adjacentOptions = KinBody::AO_Enabled;
        if( (_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentOptions |= KinBody::AO_ActiveDOFs;
        }

        const std::set<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
        // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody even if it is const
        //_fclspace->Synchronize(pbody);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            CollisionCallbackData query(shared_checker(), report);
            query.bselfCollision = true;

            KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
            FOREACH(itset, nonadjacent) {
                size_t index1 = *itset&0xffff, index2 = *itset>>16;
                // We don't need to check if the links are enabled since we got adjacency information with AO_Enabled
                LinkInfoPtr pLINK1 = pinfo->vlinks[index1], pLINK2 = pinfo->vlinks[index2];
                _fclspace->SynchronizeGeometries(pbody->GetLinks()[index1], pLINK1);
                _fclspace->SynchronizeGeometries(pbody->GetLinks()[index2], pLINK2);
                FOREACH(itgeom1, pLINK1->vgeoms) {
                    FOREACH(itgeom2, pLINK2->vgeoms) {
                        CheckNarrowPhaseGeomCollision((*itgeom1).second.get(), (*itgeom2).second.get(), &query);
                        if( query._bStopChecking ) {
                            return query._bCollision;
                        }
                    }
                }
            }
            return query._bCollision;
        }
    }

    virtual bool CheckStandaloneSelfCollision(LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())
    {
        START_TIMING_OPT(_statistics, "LinkSelf",_options,false);
        if( !!report ) {
            report->Reset(_options);
        }

        KinBodyPtr pbody = plink->GetParent();
        if( pbody->GetLinks().size() <= 1 ) {
            return false;
        }

        // We only want to consider the enabled links
        int adjacentOptions = KinBody::AO_Enabled;
        if( (_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentOptions |= KinBody::AO_ActiveDOFs;
        }

        const std::set<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
        // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody evn if it is const
        //_fclspace->Synchronize(pbody);


        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; //TODO
        } else {
            CollisionCallbackData query(shared_checker(), report);
            query.bselfCollision = true;
            KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
            FOREACH(itset, nonadjacent) {
                int index1 = *itset&0xffff, index2 = *itset>>16;
                if( plink->GetIndex() == index1 || plink->GetIndex() == index2 ) {
                    LinkInfoPtr pLINK1 = pinfo->vlinks[index1], pLINK2 = pinfo->vlinks[index2];
                    _fclspace->SynchronizeGeometries(pbody->GetLinks()[index1], pLINK1);
                    _fclspace->SynchronizeGeometries(pbody->GetLinks()[index2], pLINK2);
                    FOREACH(itgeom1, pLINK1->vgeoms) {
                        FOREACH(itgeom2, pLINK2->vgeoms) {
                            CheckNarrowPhaseGeomCollision((*itgeom1).second.get(), (*itgeom2).second.get(), &query);
                            if( query._bStopChecking ) {
                                return query._bCollision;
                            }
                        }
                    }
                }
            }
            return query._bCollision;
        }
    }

    CollisionObjectPtr GetLinkBV(LinkConstPtr plink) {
        return GetLinkBV(plink->GetParent(), plink->GetIndex());
    }

    CollisionObjectPtr GetLinkBV(KinBodyConstPtr pbody, int index) {
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
        if( !!pinfo ) {
            return GetLinkBV(pinfo, index);
        } else {
            RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%pbody->GetName()%_userdatakey%GetEnv()->GetId()));
            return CollisionObjectPtr();
        }
    }

    CollisionObjectPtr GetLinkBV(KinBodyInfoPtr pinfo, int index) {
        return pinfo->vlinks.at(index)->plinkBV->second;
    }

    BroadPhaseCollisionManagerPtr CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }

private:
    inline boost::shared_ptr<FCLCollisionChecker> shared_checker() {
        return boost::dynamic_pointer_cast<FCLCollisionChecker>(shared_from_this());
    }


    inline std::vector<int> GetEnabledLinks(KinBodyConstPtr pbody) {
        std::vector<int> venabledLinks;
        venabledLinks.reserve(pbody->GetLinks().size());
        for(size_t i = 0; i < pbody->GetLinks().size(); ++i) {
            if( pbody->GetLinks()[i]->IsEnabled() ) {
                venabledLinks.push_back(i);
            }
        }
        return venabledLinks;
    }

    void CollectEnabledLinkBVs(KinBodyConstPtr pbody, CollisionGroup& group) {
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
        group.reserve(group.size() + pbody->GetLinks().size());
        FOREACH(itlink, pbody->GetLinks()) {
            if( ((*itlink)->IsEnabled()) ) {
                // not a very good idea to access it directly for code maintenance
                group.push_back(GetLinkBV(pinfo, (*itlink)->GetIndex()).get());
            }
        }
    }

    ///< \param pkey signature characterizing part of the environment
    ///< \return a ManagerInstancePtr containing the part of the environment indicated by pkey
    ManagerInstancePtr GetManagerInstance(const CollisionGroup &collisionGroup) {
        RAVELOG_VERBOSE_FORMAT("FCL COLLISION : Rebuilding manager (env = %d)", GetEnv()->GetId());
        ManagerInstancePtr managerInstance = boost::make_shared<ManagerInstance>();
        managerInstance->pmanager = CreateManager();
        managerInstance->pmanager->registerObjects(collisionGroup);
        managerInstance->pmanager->setup();
        return managerInstance;
    }

    /// \brief Updates the element of the manager instance whose update stamp is outdated
    void UpdateManagerInstance(ManagerInstancePtr pmanagerinstance) {
        if( _broadPhaseCollisionManagerAlgorithm == "Naive" ) {
            return;
        }
        FOREACH(itWkbodyStampPair, pmanagerinstance->vUpdateStamps) {
            KinBodyConstPtr pbody = itWkbodyStampPair->first.lock();
            if( !!pbody ) {
                KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
                if( !!pinfo && itWkbodyStampPair->second < pbody->GetUpdateStamp() ) {
                    itWkbodyStampPair->second = pbody->GetUpdateStamp();
                    FOREACH(itlink, pbody->GetLinks()) {
                        if( (*itlink)->IsEnabled() ) {
                            // Do not forget to setup the manager at the end since we are deactivating the automatic setup there
                            pmanagerinstance->pmanager->update(GetLinkBV(pinfo, (*itlink)->GetIndex()).get(), false);
                        }
                    }
                }
            }
        }
        pmanagerinstance->pmanager->setup();
    }

    BroadPhaseCollisionManagerPtr SetupManagerAllDOFs(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo) {

        if( !pinfo->_bodyManager ) {

            pinfo->_bodyManager = boost::make_shared<ManagerInstance>();
            pinfo->_bodyManager->pmanager = CreateManager();

            std::set<KinBodyConstPtr> attachedBodies;
            pbody->GetAttached(attachedBodies);

            // Compute current collision objects
            _tmpbuffer.resize(0);
            FOREACH(itbody, attachedBodies) {
                CollectEnabledLinkBVs(*itbody, _tmpbuffer);
                pinfo->_bodyManager->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(*itbody), _fclspace->GetInfo(*itbody)->nLastStamp));
            }
            pinfo->_bodyManager->pmanager->registerObjects(_tmpbuffer);
            pinfo->_bodyManager->pmanager->setup();

            // Set the callbacks to invalidate the manager

            // if the bodyAttachedCallback is not set, we set it
            if( !pinfo->_bodyAttachedCallback ) {
                const boost::function<void()>& attachedBodiesChangedCallback = boost::bind(&FCLSpace::KinBodyInfo::_ResetBodyManagers, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace::KinBodyInfo>, boost::weak_ptr<FCLSpace::KinBodyInfo>(pinfo)));
                pinfo->_bodyAttachedCallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyAttached, attachedBodiesChangedCallback);
            }

            const boost::function<void()>& linkEnabledChangedCallback = boost::bind(&FCLSpace::KinBodyInfo::_ResetBodyManagers, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace::KinBodyInfo>, boost::weak_ptr<FCLSpace::KinBodyInfo>(pinfo)));
            FOREACH(itbody, attachedBodies) {
                pinfo->_linkEnabledCallbacks.push_back((*itbody)->RegisterChangeCallback(KinBody::Prop_LinkEnable, linkEnabledChangedCallback));
            }

        } else {
            // if the pinfo->_bodyManager has not been created just now, we may need to update its content
            UpdateManagerInstance(pinfo->_bodyManager);
        }
        return pinfo->_bodyManager->pmanager;
    }

    BroadPhaseCollisionManagerPtr SetupManagerActiveDOFs(RobotBaseConstPtr probot, KinBodyInfoPtr pinfo) {
        if( pinfo->_bactiveDOFsDirty ) {
            // if the activeDOFs have changed the _pinfo->_bodyManagerActiveDOFsActiveDOFs must be invalid
            BOOST_ASSERT( !pinfo->_bodyManagerActiveDOFs );
            // (re)compute _vactiveLinks
            pinfo->_vactiveLinks.resize(0);
            pinfo->_vactiveLinks.resize(probot->GetLinks().size(), 0);
            for(size_t i = 0; i < probot->GetLinks().size(); ++i) {
                int isLinkActive = 0;
                FOREACH(itindex, probot->GetActiveDOFIndices()) {
                    if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(), i) ) {
                        isLinkActive = 1;
                        break;
                    }
                }
                pinfo->_vactiveLinks[i] = isLinkActive;
            }
            pinfo->_bactiveDOFsDirty = false;
        }

        if( !pinfo->_bodyManagerActiveDOFs ) {

            pinfo->_bodyManagerActiveDOFs = boost::make_shared<ManagerInstance>();
            pinfo->_bodyManagerActiveDOFs->pmanager = CreateManager();

            // Compute current set of collision objects
            std::set<KinBodyConstPtr> attachedBodies;
            probot->GetAttached(attachedBodies);
            attachedBodies.erase(probot);

            _tmpbuffer.resize(0);
            _tmpbuffer.reserve(probot->GetLinks().size());
            FOREACH(itlink, probot->GetLinks()) {
                int index = (*itlink)->GetIndex();
                if( (*itlink)->IsEnabled() && pinfo->_vactiveLinks[index] ) {
                    _tmpbuffer.push_back(GetLinkBV(pinfo, index).get());
                }
            }
            pinfo->_bodyManagerActiveDOFs->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(probot), pinfo->nLastStamp));

            FOREACH(itbody, attachedBodies) {
                BOOST_ASSERT(*itbody != probot);
                LinkConstPtr pgrabbinglink = probot->IsGrabbing(*itbody);
                if( !!pgrabbinglink && pinfo->_vactiveLinks[pgrabbinglink->GetIndex()] ) {
                    CollectEnabledLinkBVs(*itbody, _tmpbuffer);
                    pinfo->_bodyManagerActiveDOFs->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(*itbody), _fclspace->GetInfo(*itbody)->nLastStamp));
                }
            }


            pinfo->_bodyManagerActiveDOFs->pmanager->registerObjects(_tmpbuffer);
            pinfo->_bodyManagerActiveDOFs->pmanager->setup();



            // Set the callbacks to invalidate the manager
            // if the activeDOFs callback is not set we set it
            if( !pinfo->_activeDOFsCallback ) {
                const boost::function<void()>& activeDOFsChangeCallback = boost::bind(&FCLSpace::KinBodyInfo::_ChangeActiveDOFsFlag,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace::KinBodyInfo>, boost::weak_ptr<FCLSpace::KinBodyInfo>(pinfo)));
                pinfo->_activeDOFsCallback = probot->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs, activeDOFsChangeCallback);
            }

            // if the attachedBodies callback is not set, we set it
            if( !pinfo->_bodyAttachedCallback ) {
                const boost::function<void()>& attachedBodiesChangedCallback = boost::bind(&FCLSpace::KinBodyInfo::_ResetBodyManagers, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace::KinBodyInfo>, boost::weak_ptr<FCLSpace::KinBodyInfo>(pinfo)));
                pinfo->_bodyAttachedCallback = probot->RegisterChangeCallback(KinBody::Prop_BodyAttached, attachedBodiesChangedCallback);
            }

            const boost::function<void()>& linkEnabledChangedCallback = boost::bind(&FCLSpace::KinBodyInfo::_ResetBodyManagers, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace::KinBodyInfo>, boost::weak_ptr<FCLSpace::KinBodyInfo>(pinfo)));
            FOREACH(itbody, attachedBodies) {
                pinfo->_linkEnabledCallbacks.push_back((*itbody)->RegisterChangeCallback(KinBody::Prop_LinkEnable, linkEnabledChangedCallback));
            }

        } else {
            // if the pinfo->_bodyManagerActiveDOFs has not been created just now, we may need to update its content
            UpdateManagerInstance(pinfo->_bodyManagerActiveDOFs);
        }

        return pinfo->_bodyManagerActiveDOFs->pmanager;
    }

    /// \param pbody The broadphase manager of this kinbody is (re)computed if needed
    /// \param bactiveDOFs true if we only need to consider active DOFs
    /// \return vmanagers vector filled with the relevant broadphase managers wrt pbody upon return
    BroadPhaseCollisionManagerPtr GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs) {
        FCLSpace::KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
        if( !pinfo ) {
            RAVELOG_ERROR_FORMAT("The kinbody %s has no info in checker %s, env %d", pbody->GetName()%_userdatakey%GetEnv()->GetId());
            return BroadPhaseCollisionManagerPtr();
        }

        bool ballDOFs = true; ///< true if we consider all DOFs for the manager
        RobotBaseConstPtr probot;
        // if bactiveDOFs is false or pbody is not a robot we check all DOFs
        if( bactiveDOFs && pbody->IsRobot() ) {
            probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(pbody);
            // if the robot has affine DOF we check all DOFs
            ballDOFs = probot->GetAffineDOF();
        }

        // TODO : we may recompute the attached bodies twice per query if we need to reconstruct the manager
        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        // we need to synchronize all the involved bodies before setting up the manager
        FOREACH(itbody, attachedBodies) {
            _fclspace->Synchronize(*itbody);
        }

        if( ballDOFs ) {
            return SetupManagerAllDOFs(pbody, pinfo);
        } else {
            return SetupManagerActiveDOFs(probot, pinfo);
        }
    }


    ///< \param _sexcludedbodies set of bodies which can be safely omitted from the collision checking
    ///< \return an up to date and setup broadphase collision manager containing at least the collision object of the enviroment which are relevant
    BroadPhaseCollisionManagerPtr GetEnvManager(const std::set<KinBodyConstPtr>&sexcludedbodies) {

        _fclspace->Synchronize();

        ManagerInstancePtr envManagerInstance = _fclspace->GetEnvManagerInstance();

        if( !envManagerInstance ) {

            RAVELOG_VERBOSE_FORMAT("FCL COLLISION : Rebuilding env manager (env = %d)", GetEnv()->GetId());

            _tmpbuffer.resize(0);
            envManagerInstance = boost::make_shared<ManagerInstance>();
            envManagerInstance->pmanager = CreateManager();
            FOREACH(itbody, _fclspace->GetEnvBodies()) {
                // we treat the case of robots separately
                if( (*itbody)->IsRobot() ) {
                    continue;
                }
                if( sexcludedbodies.count(*itbody) ) {
                    continue;
                }
                KinBodyInfoPtr pitinfo = _fclspace->GetInfo(*itbody);
                bool bsetUpdateStamp = false;
                FOREACH(itlink, (*itbody)->GetLinks()) {
                    if( (*itlink)->IsEnabled() ) {
                        // the link is set "as if" it was registered, but the actual registering is delayed to the end of the loop
                        _tmpbuffer.push_back(pitinfo->vlinks[(*itlink)->GetIndex()]->PrepareEnvManagerRegistering(envManagerInstance->pmanager).get());
                        bsetUpdateStamp = true;
                    }
                }
                if( bsetUpdateStamp ) {
                    envManagerInstance->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(*itbody), pitinfo->nLastStamp));
                }
            }
            // do not forget to register the objects which have been registered
            envManagerInstance->pmanager->registerObjects(_tmpbuffer);

            _fclspace->SetEnvExcludiedBodiesId(sexcludedbodies);
            _fclspace->SetEnvManagerInstance(envManagerInstance);

            // Set the callbacks to update the manager
            // For now we update the manager when a link is Reset (FCLSpace::KinBodyInfo::LINK::Reset)
            // And "exclude" an object from the envManager :
            // - when an object is added (FCLSpace::InitKinBody)
            // - when a geometry group is modified (FCLSpace::SetBodyGeometryGroup)
            // - when a link is enabled/disabled : (FCLSpace::InitKinBody)

        } else {

            std::set_difference(_fclspace->GetEnvExcludiedBodies().begin(),
                                _fclspace->GetEnvExcludiedBodies().end(),
                                sexcludedbodies.begin(),
                                sexcludedbodies.end(),
                                boost::make_function_output_iterator([this, &envManagerInstance](KinBodyConstPtr pbody) {
                                                                         KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
                                                                         if( !!pinfo && pinfo->UpdateLinksRegisterStatus(envManagerInstance->pmanager) ) {
                                                                             envManagerInstance->vUpdateStamps.push_back(std::make_pair(pinfo->_pbody, pinfo->nLastStamp));
                                                                         }
                                                                     }));
            _fclspace->SetEnvExcludiedBodiesId(sexcludedbodies);

            UpdateManagerInstance(envManagerInstance);
        }

        return envManagerInstance->pmanager;
    }

    LinkInfoPtr GetLinkInfo(LinkConstPtr plink) {
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(plink->GetParent());
        return pinfo->vlinks[plink->GetIndex()];
    }

    inline bool HasMultipleGeometries(LinkInfoPtr pLINK) {
        return pLINK->vgeoms.size() > 1;
    }

    BroadPhaseCollisionManagerPtr GetLinkGeometriesManager(LinkConstPtr plink) {
        LinkInfoPtr pLINK = GetLinkInfo(plink);
        // why not synchronize geometries at this stage only ?
        _fclspace->SynchronizeGeometries(plink, pLINK);
        if( !pLINK->_linkManager && HasMultipleGeometries(pLINK) ) {
            pLINK->_linkManager = CreateManager();
            FOREACH(itgeompair, pLINK->vgeoms) {
                pLINK->_linkManager->registerObject(itgeompair->second.get());
            }
        }
        if( !!pLINK->_linkManager ) {
            pLINK->_linkManager->update();
        }
        return pLINK->_linkManager;
    }

    static bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseCollision(o1, o2, pcb);
    }

    bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb) {

        ADD_TIMING(_statistics);
        if( pcb->_bStopChecking ) {
            return true;     // don't test anymore
        }

        LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

        if( !plink1 || !plink2 ) {
            return false;
        }

        if( !plink1->IsEnabled() || !plink2->IsEnabled() ) {
            return false;
        }

        if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
            return false;
        }

        // Proceed to the next if the links are attached and not enabled
        if( !pcb->bselfCollision && plink1->GetParent()->IsAttached(KinBodyConstPtr(plink2->GetParent())) ) {
            return false;
        }

        BroadPhaseCollisionManagerPtr plink1Manager = GetLinkGeometriesManager(plink1), plink2Manager = GetLinkGeometriesManager(plink2);

        if( !!plink1Manager ) {
            if( !!plink2Manager ) {
                plink1Manager->setup();
                plink2Manager->setup();
                plink1Manager->collide(plink2Manager.get(), pcb, &FCLCollisionChecker::CheckNarrowPhaseGeomCollision);
            } else {
                plink1Manager->setup();
                plink1Manager->collide(o2, pcb, &FCLCollisionChecker::CheckNarrowPhaseGeomCollision);
            }
        } else {
            if( !!plink2Manager ) {
                plink2Manager->setup();
                plink2Manager->collide(o1, pcb, &FCLCollisionChecker::CheckNarrowPhaseGeomCollision);
            } else {
                CheckNarrowPhaseGeomCollision(o1, o2, pcb);
            }
        }

        if( pcb->_bCollision && !(_options & OpenRAVE::CO_AllLinkCollisions) ) {
            pcb->_bStopChecking = true; // stop checking collision
        }

        return pcb->_bStopChecking;
    }




    static bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseGeomCollision(o1, o2, pcb);
    }

    bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
    {
        if( pcb->_bStopChecking ) {
            return true; // don't test anymore
        }

        pcb->_result.clear();

        CollisionPair collpair = MakeCollisionPair(o1, o2);
        NarrowCollisionCache::iterator it = mCollisionCachedGuesses.find(collpair);
        if( it != mCollisionCachedGuesses.end() ) {
            pcb->_request.cached_gjk_guess = it->second;
        } else {
            // Is there anything more intelligent we could do there with the collision objects AABB ?
            pcb->_request.cached_gjk_guess = fcl::Vec3f(1,0,0);
        }

        size_t numContacts = fcl::collide(o1, o2, pcb->_request, pcb->_result);

        mCollisionCachedGuesses[collpair] = pcb->_result.cached_gjk_guess;

        if( numContacts > 0 ) {

            if( !!pcb->_report ) {
                LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

                // these should be useless, just to make sure I haven't messed up
                BOOST_ASSERT( plink1 && plink2 );
                BOOST_ASSERT( plink1->IsEnabled() && plink2->IsEnabled() );
                BOOST_ASSERT( pcb->bselfCollision || !plink1->GetParent()->IsAttached(KinBodyConstPtr(plink2->GetParent())) );

                _reportcache.Reset(_options);
                _reportcache.plink1 = plink1;
                _reportcache.plink2 = plink2;

                // TODO : eliminate the contacts points (insertion sort (std::lower) + binary_search ?) duplicated
                // TODO : Collision points are really different from their ode variant
                if( _options & (OpenRAVE::CO_Contacts | OpenRAVE::CO_AllGeometryContacts) ) {
                    _reportcache.contacts.resize(numContacts);
                    for(size_t i = 0; i < numContacts; ++i) {
                        fcl::Contact const &c = pcb->_result.getContact(i);
                        _reportcache.contacts[i] = CollisionReport::CONTACT(ConvertVectorFromFCL(c.pos), ConvertVectorFromFCL(c.normal), c.penetration_depth);
                    }
                }


                if( pcb->_bHasCallbacks ) {
                    OpenRAVE::CollisionAction action = OpenRAVE::CA_DefaultAction;
                    CollisionReportPtr preport(&_reportcache, OpenRAVE::utils::null_deleter());
                    FOREACH(callback, pcb->GetCallbacks()) {
                        action = (*callback)(preport, false);
                        if( action == OpenRAVE::CA_Ignore ) {
                            return false;
                        }
                    }
                }

                pcb->_report->plink1 = _reportcache.plink1;
                pcb->_report->plink2 = _reportcache.plink2;
                if(pcb->_report->contacts.size() + numContacts < pcb->_report->contacts.capacity()) {
                    pcb->_report->contacts.reserve(pcb->_report->contacts.capacity() + numContacts); // why capacity instead of size ?
                }
                copy(_reportcache.contacts.begin(),_reportcache.contacts.end(), back_inserter(pcb->_report->contacts));
                // pcb->_report->contacts.swap(_report.contacts); // would be faster but seems just wrong...

                if( _options & OpenRAVE::CO_AllLinkCollisions ) {
                    // We maintain vLinkColliding ordered
                    LinkPair linkPair = MakeLinkPair(plink1, plink2);
                    typedef std::vector< std::pair< LinkConstPtr, LinkConstPtr > >::iterator PairIterator;
                    PairIterator end = pcb->_report->vLinkColliding.end(), first = std::lower_bound(pcb->_report->vLinkColliding.begin(), end, linkPair);
                    if( first == end || *first != linkPair ) {
                        pcb->_report->vLinkColliding.insert(first, linkPair);
                    }
                }

                pcb->_bCollision = true;
                if( !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts)) ) {
                    pcb->_bStopChecking = true; // stop checking collision
                }
                return pcb->_bStopChecking;
            }

            pcb->_bCollision = true;
            pcb->_bStopChecking = true; // since the report is NULL, there is no reason to continue
            return pcb->_bStopChecking;
        }

        return false; // keep checking collision
    }

    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data)
    {
        // TODO
        return false;
    }

    static CollisionPair MakeCollisionPair(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
    {
        if( o1 < o2 ) {
            return make_pair(o1, o2);
        } else {
            return make_pair(o2, o1);
        }
    }

    static LinkPair MakeLinkPair(LinkConstPtr plink1, LinkConstPtr plink2)
    {
        if( plink1.get() < plink2.get() ) {
            return make_pair(plink1, plink2);
        } else {
            return make_pair(plink2, plink1);
        }
    }


    LinkConstPtr GetCollisionLink(const fcl::CollisionObject &collObj) {
        FCLSpace::KinBodyInfo::LINK *link_raw = static_cast<FCLSpace::KinBodyInfo::LINK *>(collObj.getUserData());
        if( link_raw != NULL ) {
            LinkConstPtr plink = link_raw->GetLink();
            if( !plink ) {
                RAVELOG_WARN(str(boost::format("The link %s was lost from fclspace")%link_raw->bodylinkname));
            }
            return plink;
        }
        RAVELOG_WARN("fcl collision object does not have a link attached");
        return LinkConstPtr();
    }

    // TODO : This is becoming really stupid, I should just add optional additional data for DynamicAABBTree
    BroadPhaseCollisionManagerPtr _CreateManagerFromBroadphaseAlgorithm(std::string const &algorithm)
    {
        if(algorithm == "Naive") {
            return boost::make_shared<fcl::NaiveCollisionManager>();
        } else if(algorithm == "SaP") {
            return boost::make_shared<fcl::SaPCollisionManager>();
        } else if(algorithm == "SSaP") {
            return boost::make_shared<fcl::SSaPCollisionManager>();
        } else if(algorithm == "SpatialHashing") {
            if( _spatialHashData ) {
                // TODO : add other HashTables (GoogleDenseHashTable ?)
                return boost::make_shared< fcl::SpatialHashingCollisionManager< fcl::SparseHashTable<fcl::AABB, fcl::CollisionObject*, fcl::SpatialHash> > >(_spatialHashData->_cellSize, _spatialHashData->_sceneMin, _spatialHashData->_sceneMax);
            } else {
                throw OPENRAVE_EXCEPTION_FORMAT0("No spatial data provided, spatial hashing needs to be set up  with SetSpatialHashingBroadPhaseAlgorithm", OpenRAVE::ORE_InvalidArguments);
            }
        } else if(algorithm == "IntervalTree") {
            return boost::make_shared<fcl::IntervalTreeCollisionManager>();
        } else if(algorithm == "DynamicAABBTree") {
            return boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        } else if(algorithm == "DynamicAABBTree1") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
            pmanager->tree_init_level = 1;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree2") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
            pmanager->tree_init_level = 2;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree3") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager>();
            pmanager->tree_init_level = 3;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree_Array") {
            return boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        } else if(algorithm == "DynamicAABBTree1_Array") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager_Array> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
            pmanager->tree_init_level = 1;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree2_Array") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager_Array> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
            pmanager->tree_init_level = 2;
            return pmanager;
        } else if(algorithm == "DynamicAABBTree3_Array") {
            boost::shared_ptr<fcl::DynamicAABBTreeCollisionManager_Array> pmanager = boost::make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
            pmanager->tree_init_level = 3;
            return pmanager;
        } else {
            throw OPENRAVE_EXCEPTION_FORMAT("Unknown broad-phase algorithm '%s'.", algorithm, OpenRAVE::ORE_InvalidArguments);
        }
    }




    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
    bool _bIsSelfCollisionChecker;
    CollisionReport _reportcache;

    std::string _broadPhaseCollisionManagerAlgorithm;
    boost::shared_ptr<SpatialHashData> _spatialHashData;

    NarrowCollisionCache mCollisionCachedGuesses;

#ifdef FCLUSESTATISTICS
    FCLStatisticsPtr _statistics;
#endif
    CollisionGroup _tmpbuffer;
};

} // fclrave

#endif
