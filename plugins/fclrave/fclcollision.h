// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_COLLISION
#define OPENRAVE_FCL_COLLISION

#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>
#include <boost/function_output_iterator.hpp>

#include "fclspace.h"
#include "fclmanagercache.h"

#include "fclstatistics.h"

#define FCLRAVE_CHECKPARENTLESS

namespace fclrave {

#define START_TIMING_OPT(statistics, label, options, isRobot);           \
    START_TIMING(statistics, boost::str(boost::format("%s,%x,%d")%label%options%isRobot))

#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
static EnvironmentMutex log_collision_use_mutex;
#endif // FCLRAVE_COLLISION_OBJECTS_STATISTIC

#ifdef NARROW_COLLISION_CACHING
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
#endif // NARROW_COLLISION_CACHING

typedef FCLSpace::KinBodyInfoConstPtr KinBodyInfoConstPtr;
typedef FCLSpace::KinBodyInfoPtr KinBodyInfoPtr;
typedef FCLSpace::LinkInfoPtr LinkInfoPtr;

template<typename T>
inline bool IsIn(T const& x, std::vector<T> const &collection) {
    return std::find(collection.begin(), collection.end(), x) != collection.end();
}

class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:
    class CollisionCallbackData {
public:
        CollisionCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker, CollisionReportPtr report, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<LinkConstPtr>& vlinkexcluded) : _pchecker(pchecker), _report(report), _vbodyexcluded(vbodyexcluded), _vlinkexcluded(vlinkexcluded), bselfCollision(false), _bStopChecking(false), _bCollision(false)
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
            _distanceRequest.gjk_solver_type = fcl::GST_LIBCCD;

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
        fcl::DistanceRequest _distanceRequest;
        fcl::DistanceResult _distanceResult;
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

    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
        : OpenRAVE::CollisionCheckerBase(penv), _broadPhaseCollisionManagerAlgorithm("DynamicAABBTree2"), _bIsSelfCollisionChecker(true) // DynamicAABBTree2 should be slightly faster than Naive
    {
        _bParentlessCollisionObject = false;
        _userdatakey = std::string("fclcollision") + boost::lexical_cast<std::string>(this);
        _fclspace.reset(new FCLSpace(penv, _userdatakey));
        _options = 0;
        // TODO : Should we put a more reasonable arbitrary value ?
        _numMaxContacts = std::numeric_limits<int>::max();
        _nGetEnvManagerCacheClearCount = 100000;
        __description = ":Interface Author: Kenji Maillard\n\nFlexible Collision Library collision checker";

        SETUP_STATISTICS(_statistics, _userdatakey, GetEnv()->GetId());

        // TODO : Consider removing these which could be more harmful than anything else
        RegisterCommand("SetBroadphaseAlgorithm", boost::bind(&FCLCollisionChecker::SetBroadphaseAlgorithmCommand, this, _1, _2), "sets the broadphase algorithm (Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTree_Array)");
        RegisterCommand("SetBVHRepresentation", boost::bind(&FCLCollisionChecker::_SetBVHRepresentation, this, _1, _2), "sets the Bouding Volume Hierarchy representation for meshes (AABB, OBB, OBBRSS, RSS, kIDS)");

        RAVELOG_VERBOSE_FORMAT("FCLCollisionChecker %s created in env %d", _userdatakey%penv->GetId());

        std::string broadphasealg, bvhrepresentation;
        sinput >> broadphasealg >> bvhrepresentation;
        if( broadphasealg != "" ) {
            _SetBroadphaseAlgorithm(broadphasealg);
        }
        if( bvhrepresentation != "" ) {
            _fclspace->SetBVHRepresentation(bvhrepresentation);
        }
    }

    virtual ~FCLCollisionChecker() {
        RAVELOG_VERBOSE_FORMAT("FCLCollisionChecker %s destroyed in env %d", _userdatakey%GetEnv()->GetId());
        DestroyEnvironment();

#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
        EnvironmentMutex::scoped_lock lock(log_collision_use_mutex);

        FOREACH(itpair, _currentlyused) {
            if(itpair->second > 0) {
                _usestatistics[itpair->first][itpair->second]++;
            }
        }
        std::fstream f("fclrave_collision_use.log", std::fstream::app | std::fstream::out);
        FOREACH(itpair, _usestatistics) {
            f << GetEnv()->GetId() << "|" << _userdatakey << "|" << itpair->first;
            FOREACH(itintpair, itpair->second) {
                f << "|" << itintpair->first << ";" << itintpair->second;
            }
            f << std::endl;
        }
        f.close();
#endif
    }

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<FCLCollisionChecker const> r = boost::dynamic_pointer_cast<FCLCollisionChecker const>(preference);
        // We don't clone Kinbody's specific geometry group
        _fclspace->SetGeometryGroup(r->GetGeometryGroup());
        _fclspace->SetBVHRepresentation(r->GetBVHRepresentation());
        _SetBroadphaseAlgorithm(r->GetBroadphaseAlgorithm());

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

    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname)
    {
        return _fclspace->SetBodyGeometryGroup(pbody, groupname);
    }

    const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const
    {
        return _fclspace->GetBodyGeometryGroup(*pbody);
    }

    virtual bool SetCollisionOptions(int collision_options)
    {
        _options = collision_options;

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
    /// The input algorithm can be one of : Naive, SaP, SSaP, IntervalTree, DynamicAABBTree{,1,2,3}, DynamicAABBTree_Array{,1,2,3}, SpatialHashing
    /// e.g. "SetBroadPhaseAlgorithm DynamicAABBTree"
    bool SetBroadphaseAlgorithmCommand(ostream& sout, istream& sinput)
    {
        std::string algorithm;
        sinput >> algorithm;
        _SetBroadphaseAlgorithm(algorithm);
        return !!sinput;
    }

    void _SetBroadphaseAlgorithm(const std::string &algorithm)
    {
        if(_broadPhaseCollisionManagerAlgorithm == algorithm) {
            return;
        }
        _broadPhaseCollisionManagerAlgorithm = algorithm;

        // clear all the current cached managers
        _bodymanagers.clear();
        _envmanagers.clear();
    }

    const std::string & GetBroadphaseAlgorithm() const {
        return _broadPhaseCollisionManagerAlgorithm;
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
            throw OPENRAVE_EXCEPTION_FORMAT0("No spatial data provided, spatial hashing needs to be set up  with SetSpatialHashingBroadPhaseAlgorithm", OpenRAVE::ORE_InvalidArguments);
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
        _fclspace->SetIsSelfCollisionChecker(false);
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
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        FCLSpace::KinBodyInfoPtr pinfo = _fclspace->GetInfo(*pbody);
        if( !pinfo || pinfo->GetBody() != pbody ) {
            pinfo = _fclspace->InitKinBody(pbody);
        }
        return !pinfo;
    }

    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        // remove body from all the managers
        _bodymanagers.erase(std::make_pair(pbody.get(), (int)0));
        _bodymanagers.erase(std::make_pair(pbody.get(), (int)1));
        FOREACH(itmanager, _envmanagers) {
            itmanager->second->RemoveBody(*pbody);
        }
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

        if( pbody1->GetLinks().size() == 0 || !_IsEnabled(*pbody1) ) {
            return false;
        }

        if( pbody2->GetLinks().size() == 0 || !_IsEnabled(*pbody2) ) {
            return false;
        }

        if( pbody1->IsAttached(*pbody2) ) {
            return false;
        }

        _fclspace->SynchronizeWithAttached(*pbody1);
        _fclspace->SynchronizeWithAttached(*pbody2);

        // Do we really want to synchronize everything ?
        // We could put the synchronization directly inside GetBodyManager
        FCLCollisionManagerInstance& body1Manager = _GetBodyManager(pbody1, !!(_options & OpenRAVE::CO_ActiveDOFs));
        FCLCollisionManagerInstance& body2Manager = _GetBodyManager(pbody2, false);
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstance, this, boost::ref(*pbody1), boost::ref(body1Manager), boost::ref(*pbody2), boost::ref(body2Manager)));
#endif

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        if( _options & OpenRAVE::CO_Distance ) {
            if(!report) {
                throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
            }
            body1Manager.GetManager()->distance(body2Manager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
        }
        ADD_TIMING(_statistics);
        body1Manager.GetManager()->collide(body2Manager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
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

        KinBodyPtr plink1parent = plink1->GetParent(true);
        if( !plink1parent ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Failed to get link %s parent", plink1parent->GetName(), OpenRAVE::ORE_InvalidArguments);
        }
        KinBodyPtr plink2parent = plink2->GetParent(true);
        if( !plink2parent ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Failed to get link %s parent", plink2parent->GetName(), OpenRAVE::ORE_InvalidArguments);
        }

        _fclspace->SynchronizeWithAttached(*plink1parent);
        if( plink1parent != plink2parent ) {
            _fclspace->SynchronizeWithAttached(*plink2parent);
        }

        CollisionObjectPtr pcollLink1 = _fclspace->GetLinkBV(*plink1), pcollLink2 = _fclspace->GetLinkBV(*plink2);

        if( !pcollLink1 || !pcollLink2 ) {
            return false;
        }

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        if( _options & OpenRAVE::CO_Distance ) {
            if(!report) {
                throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
            }
            fcl::FCL_REAL dist = -1.0;
            CheckNarrowPhaseDistance(pcollLink1.get(), pcollLink2.get(), &query, dist);
        }
        if( !pcollLink1->getAABB().overlap(pcollLink2->getAABB()) ) {
            return false;
        }
        ADD_TIMING(_statistics);
        query.bselfCollision = true;  // for ignoring attached information!
        CheckNarrowPhaseCollision(pcollLink1.get(), pcollLink2.get(), &query);
        return query._bCollision;
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

        if( pbody->GetLinks().size() == 0 || !_IsEnabled(*pbody) ) {
            return false;
        }

        if( pbody->IsAttached(*plink->GetParent()) ) {
            return false;
        }

        _fclspace->SynchronizeWithAttached(*plink->GetParent());

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        FOREACH(itbody, attachedBodies) {
            if( (*itbody)->GetEnvironmentId() ) { // for now GetAttached can hold bodies that are not initialized
                _fclspace->SynchronizeWithAttached(**itbody);
            }
        }

        CollisionObjectPtr pcollLink = _fclspace->GetLinkBV(*plink);

        if( !pcollLink ) {
            return false;
        }

        FCLCollisionManagerInstance& bodyManager = _GetBodyManager(pbody, false);

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        if( _options & OpenRAVE::CO_Distance ) {
            if(!report) {
                throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
            }
            bodyManager.GetManager()->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
        }
        ADD_TIMING(_statistics);
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceBL, this, boost::ref(*pbody), boost::ref(bodyManager), boost::ref(*plink)));
#endif
        bodyManager.GetManager()->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
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
        CollisionObjectPtr pcollLink = _fclspace->GetLinkBV(*plink);

        if( !pcollLink ) {
            return false;
        }

        std::set<KinBodyConstPtr> attachedBodies;
        plink->GetParent()->GetAttached(attachedBodies);
        FCLCollisionManagerInstance& envManager = _GetEnvManager(attachedBodies);

        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        if( _options & OpenRAVE::CO_Distance ) {
            if(!report) {
                throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
            }
            envManager.GetManager()->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
        }
        ADD_TIMING(_statistics);
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceLE, this, boost::ref(*plink), boost::ref(envManager)));
#endif
        envManager.GetManager()->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())
    {
        if( !!report ) {
            report->Reset(_options);
        }

        if( (pbody->GetLinks().size() == 0) || !_IsEnabled(*pbody) ) {
            return false;
        }

        _fclspace->Synchronize();
        FCLCollisionManagerInstance& bodyManager = _GetBodyManager(pbody, !!(_options & OpenRAVE::CO_ActiveDOFs));

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        FCLCollisionManagerInstance& envManager = _GetEnvManager(attachedBodies);

        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        if( _options & OpenRAVE::CO_Distance ) {
            if(!report) {
                throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
            }
            envManager.GetManager()->distance(bodyManager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
        }
        ADD_TIMING(_statistics);
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceBE, this, boost::ref(*pbody), boost::ref(bodyManager), boost::ref(envManager)));
#endif
        envManager.GetManager()->collide(bodyManager.GetManager().get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

        return query._bCollision;
    }

    virtual bool CheckCollision(const RAY& ray, LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr())
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr())
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    virtual bool CheckCollision(const OpenRAVE::TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) override
    {
        if( !!report ) {
            report->Reset(_options);
        }

        if( (pbody->GetLinks().size() == 0) || !_IsEnabled(*pbody) ) {
            return false;
        }

        _fclspace->SynchronizeWithAttached(*pbody);
        FCLCollisionManagerInstance& bodyManager = _GetBodyManager(pbody, !!(_options & OpenRAVE::CO_ActiveDOFs));

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        OPENRAVE_ASSERT_OP(trimesh.indices.size() % 3, ==, 0);
        size_t const num_points = trimesh.vertices.size();
        size_t const num_triangles = trimesh.indices.size() / 3;

        _fclPointsCache.resize(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = trimesh.vertices[ipoint];
            _fclPointsCache[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
        }

        _fclTrianglesCache.resize(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &trimesh.indices[3 * itri];
            _fclTrianglesCache[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }

        FCLSpace::KinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr ctrigeom = _fclspace->GetMeshFactory()(_fclPointsCache, _fclTrianglesCache);
        fcl::CollisionObject ctriobj(ctrigeom);
        //ctriobj.computeAABB(); // necessary?
        ctriobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(bodyManager)));
#endif
        bodyManager.GetManager()->collide(&ctriobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(const OpenRAVE::TriMesh& trimesh, CollisionReportPtr report = CollisionReportPtr()) override
    {
        if( !!report ) {
            report->Reset(_options);
        }

        _fclspace->Synchronize();
        std::set<KinBodyConstPtr> attachedBodies;
        FCLCollisionManagerInstance& envManager = _GetEnvManager(attachedBodies);
        
        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        OPENRAVE_ASSERT_OP(trimesh.indices.size() % 3, ==, 0);
        size_t const num_points = trimesh.vertices.size();
        size_t const num_triangles = trimesh.indices.size() / 3;

        _fclPointsCache.resize(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = trimesh.vertices[ipoint];
            _fclPointsCache[ipoint] = fcl::Vec3f(v.x, v.y, v.z);
        }

        _fclTrianglesCache.resize(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &trimesh.indices[3 * itri];
            _fclTrianglesCache[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }

        FCLSpace::KinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr ctrigeom = _fclspace->GetMeshFactory()(_fclPointsCache, _fclTrianglesCache);
        fcl::CollisionObject ctriobj(ctrigeom);
        //ctriobj.computeAABB(); // necessary?
        ctriobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        //boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(bodyManager)));
#endif
        envManager.GetManager()->collide(&ctriobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }
    
    virtual bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, CollisionReportPtr report = CollisionReportPtr()) override
    {
        if( !!report ) {
            report->Reset(_options);
        }
        
        _fclspace->Synchronize();
        std::set<KinBodyConstPtr> attachedBodies;
        FCLCollisionManagerInstance& envManager = _GetEnvManager(attachedBodies);
        
        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        FCLSpace::KinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr cboxgeom = make_shared<fcl::Box>(ab.extents.x*2,ab.extents.y*2,ab.extents.z*2);
        fcl::CollisionObject cboxobj(cboxgeom);

        fcl::Vec3f newPosition = ConvertVectorToFCL(aabbPose * ab.pos);
        fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(aabbPose.rot);
        cboxobj.setTranslation(newPosition);
        cboxobj.setQuatRotation(newOrientation);
        cboxobj.computeAABB(); // probably necessary since translation changed
        cboxobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        //boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(envManager)));
#endif
        envManager.GetManager()->collide(&cboxobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, const std::vector<OpenRAVE::KinBodyConstPtr>& vIncludedBodies, OpenRAVE::CollisionReportPtr report) override
    {
        if( !!report ) {
            report->Reset(_options);
        }

        _fclspace->Synchronize();
        std::set<KinBodyConstPtr> excludedBodies;
        FOREACH(itbody, _fclspace->GetEnvBodies()) {
            if( find(vIncludedBodies.begin(), vIncludedBodies.end(), *itbody) == vIncludedBodies.end() ) {
                excludedBodies.insert(*itbody);
            }
        }
        FCLCollisionManagerInstance& envManager = _GetEnvManager(excludedBodies);

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);

        FCLSpace::KinBodyInfo::LinkInfo objUserData;

        CollisionGeometryPtr cboxgeom = make_shared<fcl::Box>(ab.extents.x*2,ab.extents.y*2,ab.extents.z*2);
        fcl::CollisionObject cboxobj(cboxgeom);

        fcl::Vec3f newPosition = ConvertVectorToFCL(aabbPose * ab.pos);
        fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(aabbPose.rot);
        cboxobj.setTranslation(newPosition);
        cboxobj.setQuatRotation(newOrientation);
        cboxobj.computeAABB(); // probably necessary since translation changed
        cboxobj.setUserData(&objUserData);
#ifdef FCLRAVE_CHECKPARENTLESS
        //boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceB, this, boost::ref(*pbody), boost::ref(envManager)));
#endif
        envManager.GetManager()->collide(&cboxobj, &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        return query._bCollision;
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

        const std::vector<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
        // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody even if it is const
        _fclspace->SynchronizeWithAttached(*pbody);

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);
        query.bselfCollision = true;
#ifdef FCLRAVE_CHECKPARENTLESS
        boost::shared_ptr<void> onexit((void*) 0, boost::bind(&FCLCollisionChecker::_PrintCollisionManagerInstanceSelf, this, boost::ref(*pbody)));
#endif            
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(*pbody);
        FOREACH(itset, nonadjacent) {
            size_t index1 = *itset&0xffff, index2 = *itset>>16;
            // We don't need to check if the links are enabled since we got adjacency information with AO_Enabled
            const FCLSpace::KinBodyInfo::LinkInfo& pLINK1 = *pinfo->vlinks.at(index1);
            const FCLSpace::KinBodyInfo::LinkInfo& pLINK2 = *pinfo->vlinks.at(index2);
            if( !pLINK1.linkBV.second->getAABB().overlap(pLINK2.linkBV.second->getAABB()) ) {
                continue;
            }
            FOREACH(itgeom1, pLINK1.vgeoms) {
                FOREACH(itgeom2, pLINK2.vgeoms) {
                    if ( _options & OpenRAVE::CO_Distance ) {
                        if(!report) {
                            throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
                        }
                        fcl::FCL_REAL dist = -1.0;
                        CheckNarrowPhaseGeomDistance((*itgeom1).second.get(), (*itgeom2).second.get(), &query, dist);
                    }
                    if( !(*itgeom1).second->getAABB().overlap((*itgeom2).second->getAABB()) ) {
                        continue;
                    }
                    CheckNarrowPhaseGeomCollision((*itgeom1).second.get(), (*itgeom2).second.get(), &query);
                    if( !(_options & OpenRAVE::CO_Distance) && query._bStopChecking ) {
                        return query._bCollision;
                    }
                }
            }
        }
        return query._bCollision;
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

        const std::vector<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
        // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody evn if it is const
        _fclspace->SynchronizeWithAttached(*pbody);

        const std::vector<KinBodyConstPtr> vbodyexcluded;
        const std::vector<LinkConstPtr> vlinkexcluded;
        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);
        query.bselfCollision = true;
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(*pbody);
        FOREACH(itset, nonadjacent) {
            int index1 = *itset&0xffff, index2 = *itset>>16;
            if( plink->GetIndex() == index1 || plink->GetIndex() == index2 ) {
                const FCLSpace::KinBodyInfo::LinkInfo& pLINK1 = *pinfo->vlinks.at(index1);
                const FCLSpace::KinBodyInfo::LinkInfo& pLINK2 = *pinfo->vlinks.at(index2);
                if( !pLINK1.linkBV.second->getAABB().overlap(pLINK2.linkBV.second->getAABB()) ) {
                    continue;
                }
                FOREACH(itgeom1, pLINK1.vgeoms) {
                    FOREACH(itgeom2, pLINK2.vgeoms) {
                        if ( _options & OpenRAVE::CO_Distance ) {
                            if(!report) {
                                throw openrave_exception("FCLCollision - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
                            }
                            fcl::FCL_REAL dist = -1.0;
                            CheckNarrowPhaseGeomDistance((*itgeom1).second.get(), (*itgeom2).second.get(), &query, dist);
                        }
                        if( !(*itgeom1).second->getAABB().overlap((*itgeom2).second->getAABB()) ) {
                            continue;
                        }
                        CheckNarrowPhaseGeomCollision((*itgeom1).second.get(), (*itgeom2).second.get(), &query);
                        if( !(_options & OpenRAVE::CO_Distance) && query._bStopChecking ) {
                            return query._bCollision;
                        }
                    }
                }
            }
        }
        return query._bCollision;
    }


private:
    inline boost::shared_ptr<FCLCollisionChecker> shared_checker() {
        return boost::static_pointer_cast<FCLCollisionChecker>(shared_from_this());
    }

    static bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseCollision(o1, o2, pcb);
    }

    bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb)
    {
        if( pcb->_bStopChecking ) {
            return true;     // don't test anymore
        }

//        _o1 = o1;
//        _o2 = o2;
        std::pair<FCLSpace::KinBodyInfo::LinkInfo*, LinkConstPtr> o1info = GetCollisionLink(*o1), o2info = GetCollisionLink(*o2);

        if( !o1info.second ) {
            if( !o1info.first ) {
                if( _bParentlessCollisionObject ) {
                    if( !!o2info.second ) {
                        RAVELOG_WARN_FORMAT("env=%d, fcl::CollisionObject o1 %x collides with link2 %s:%s, but collision ignored", GetEnv()->GetId()%o1%o2info.second->GetParent()->GetName()%o2info.second->GetName());
                    }
                }
                return false;
            }
            // o1 is standalone object
        }
        if( !o2info.second ) {
            if( !o2info.first ) {
                if( _bParentlessCollisionObject ) {
                    if( !!o1info.second ) {
                        RAVELOG_WARN_FORMAT("env=%d, link1 %s:%s collides with fcl::CollisionObject o2 %x, but collision ignored", GetEnv()->GetId()%o1info.second->GetParent()->GetName()%o1info.second->GetName()%o2);
                    }
                }
                return false;
            }
            // o2 is standalone object
        }

        LinkConstPtr& plink1 = o1info.second;
        LinkConstPtr& plink2 = o2info.second;

        if( !!plink1 ) {
            if( !plink1->IsEnabled() ) {
                return false;
            }
            if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) ) {
                return false;
            }
        }

        if( !!plink2 ) {
            if( !plink2->IsEnabled() ) {
                return false;
            }
            if( IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
                return false;
            }
        }

        if( !!plink1 && !!plink2 ) {
            if( !pcb->bselfCollision && plink1->GetParent()->IsAttached(*plink2->GetParent())) {
                return false;
            }

            LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1), pLINK2 = _fclspace->GetLinkInfo(*plink2);

            //RAVELOG_VERBOSE_FORMAT("env=%d, link %s:%s with %s:%s", GetEnv()->GetId()%plink1->GetParent()->GetName()%plink1->GetName()%plink2->GetParent()->GetName()%plink2->GetName());
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                FOREACH(itgeompair2, pLINK2->vgeoms) {
                    if( itgeompair1->second->getAABB().overlap(itgeompair2->second->getAABB()) ) {
                        CheckNarrowPhaseGeomCollision(itgeompair1->second.get(), itgeompair2->second.get(), pcb);
                        if( pcb->_bStopChecking ) {
                            return true;
                        }
                    }
                }
            }
        }
        else if( !!plink1 ) {
            LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1);
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                if( itgeompair1->second->getAABB().overlap(o2->getAABB()) ) {
                    CheckNarrowPhaseGeomCollision(itgeompair1->second.get(), o2, pcb);
                    if( pcb->_bStopChecking ) {
                        return true;
                    }
                }
            }
        }
        else if( !!plink2 ) {
            LinkInfoPtr pLINK2 = _fclspace->GetLinkInfo(*plink2);
            FOREACH(itgeompair2, pLINK2->vgeoms) {
                if( itgeompair2->second->getAABB().overlap(o1->getAABB()) ) {
                    CheckNarrowPhaseGeomCollision(o1, itgeompair2->second.get(), pcb);
                    if( pcb->_bStopChecking ) {
                        return true;
                    }
                }
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

#ifdef NARROW_COLLISION_CACHING
        CollisionPair collpair = MakeCollisionPair(o1, o2);
        NarrowCollisionCache::iterator it = mCollisionCachedGuesses.find(collpair);
        if( it != mCollisionCachedGuesses.end() ) {
            pcb->_request.cached_gjk_guess = it->second;
        } else {
            // Is there anything more intelligent we could do there with the collision objects AABB ?
            pcb->_request.cached_gjk_guess = fcl::Vec3f(1,0,0);
        }
#endif

        size_t numContacts = fcl::collide(o1, o2, pcb->_request, pcb->_result);

#ifdef NARROW_COLLISION_CACHING
        mCollisionCachedGuesses[collpair] = pcb->_result.cached_gjk_guess;
#endif

        if( numContacts > 0 ) {
            if( !!pcb->_report ) {
                std::pair<FCLSpace::KinBodyInfo::LinkInfo*, LinkConstPtr> o1info = GetCollisionLink(*o1), o2info = GetCollisionLink(*o2);
                LinkConstPtr& plink1 = o1info.second;
                LinkConstPtr& plink2 = o2info.second;

                // plink1 or plink2 can be None if object is standalone (ie coming from trimesh)

                //LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

                // these should be useless, just to make sure I haven't messed up
                //BOOST_ASSERT( plink1 && plink2 );
                //BOOST_ASSERT( plink1->IsEnabled() && plink2->IsEnabled() );
                if( !!plink1 && !!plink2 ) {
                    BOOST_ASSERT( pcb->bselfCollision || !plink1->GetParent()->IsAttached(*plink2->GetParent()));
                }

                _reportcache.Reset(_options);
                _reportcache.plink1 = plink1;
                _reportcache.plink2 = plink2;

                // TODO : eliminate the contacts points (insertion sort (std::lower) + binary_search ?) duplicated
                // How comes that there are duplicated contacts points ?
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
                if( pcb->_report->contacts.size() == 0) {
                    pcb->_report->contacts.swap(_reportcache.contacts);
                } else {
                    pcb->_report->contacts.reserve(pcb->_report->contacts.size() + numContacts);
                    copy(_reportcache.contacts.begin(),_reportcache.contacts.end(), back_inserter(pcb->_report->contacts));
                }

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

    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist)
    {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseDistance(o1, o2, pcb, dist);
    }

    bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist) {
        std::pair<FCLSpace::KinBodyInfo::LinkInfo*, LinkConstPtr> o1info = GetCollisionLink(*o1), o2info = GetCollisionLink(*o2);

        if( !o1info.second && !o1info.first ) {
            // o1 is standalone object
            if( _bParentlessCollisionObject && !!o2info.second ) {
                RAVELOG_WARN_FORMAT("env=%d, fcl::CollisionObject o1 %x collides with link2 %s:%s, but is ignored for distance computation", GetEnv()->GetId()%o1%o2info.second->GetParent()->GetName()%o2info.second->GetName());
            }
            return false;
        }
        if( !o2info.second && !o2info.first ) {
            // o2 is standalone object
            if( _bParentlessCollisionObject && !!o1info.second ) {
                RAVELOG_WARN_FORMAT("env=%d, link1 %s:%s collides with fcl::CollisionObject o2 %x, but is ignored for distance computation", GetEnv()->GetId()%o1info.second->GetParent()->GetName()%o1info.second->GetName()%o2);
            }
            return false;
        }

        LinkConstPtr& plink1 = o1info.second;
        LinkConstPtr& plink2 = o2info.second;

        if( !!plink1 ) {
            if( !plink1->IsEnabled() ) {
                return false;
            }
            if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) ) {
                return false;
            }
        }

        if( !!plink2 ) {
            if( !plink2->IsEnabled() ) {
                return false;
            }
            if( IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
                return false;
            }
        }

        if( !!plink1 && !!plink2 ) {

            LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1), pLINK2 = _fclspace->GetLinkInfo(*plink2);

            //RAVELOG_VERBOSE_FORMAT("env=%d, link %s:%s with %s:%s", GetEnv()->GetId()%plink1->GetParent()->GetName()%plink1->GetName()%plink2->GetParent()->GetName()%plink2->GetName());
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                FOREACH(itgeompair2, pLINK2->vgeoms) {
-                   CheckNarrowPhaseGeomDistance(itgeompair1->second.get(), itgeompair2->second.get(), pcb, dist);
                }
            }
        }
        else if( !!plink1 ) {
            LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(*plink1);
            FOREACH(itgeompair1, pLINK1->vgeoms) {
                CheckNarrowPhaseGeomDistance(itgeompair1->second.get(), o2, pcb, dist);
            }
        }
        else if( !!plink2 ) {
            LinkInfoPtr pLINK2 = _fclspace->GetLinkInfo(*plink2);
            FOREACH(itgeompair2, pLINK2->vgeoms) {
                CheckNarrowPhaseGeomDistance(o1, itgeompair2->second.get(), pcb, dist);
            }
        }

        return false;
    }

    static bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseGeomDistance(o1, o2, pcb, dist);
    }


    bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb, fcl::FCL_REAL& dist) {
        // Compute the min distance between the objects.
        fcl::distance(o1, o2, pcb->_distanceRequest, pcb->_distanceResult);

        // If the min distance between these two objects is smaller than the min distance found so far, store it as the new min distance.
        if (pcb->_report->minDistance > pcb->_distanceResult.min_distance) {
            pcb->_report->minDistance = pcb->_distanceResult.min_distance;
        }

        // Store the current min distance.
        dist = pcb->_distanceResult.min_distance;

        // Can we ever stop without testing all objects?
        return false;
    }

#ifdef NARROW_COLLISION_CACHING
    static CollisionPair MakeCollisionPair(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
    {
        if( o1 < o2 ) {
            return make_pair(o1, o2);
        } else {
            return make_pair(o2, o1);
        }
    }
#endif

    static LinkPair MakeLinkPair(LinkConstPtr plink1, LinkConstPtr plink2)
    {
        if( plink1.get() < plink2.get() ) {
            return make_pair(plink1, plink2);
        } else {
            return make_pair(plink2, plink1);
        }
    }

    std::pair<FCLSpace::KinBodyInfo::LinkInfo*, LinkConstPtr> GetCollisionLink(const fcl::CollisionObject &collObj)
    {
        FCLSpace::KinBodyInfo::LinkInfo* link_raw = static_cast<FCLSpace::KinBodyInfo::LinkInfo *>(collObj.getUserData());
        if( !!link_raw ) {
            LinkConstPtr plink = link_raw->GetLink();
            if( !plink ) {
                if( link_raw->bFromKinBodyLink ) {
                    RAVELOG_WARN_FORMAT("The link %s was lost from fclspace (env %d) (userdatakey %s)", GetEnv()->GetId()%link_raw->bodylinkname%_userdatakey);
                }
            }
            return std::make_pair(link_raw, plink);
        }
        RAVELOG_WARN_FORMAT("env=%d, fcl collision object %x does not have a link attached (userdatakey %s)", GetEnv()->GetId()%(&collObj)%_userdatakey);
        _bParentlessCollisionObject = true;
        return std::make_pair(link_raw, LinkConstPtr());
    }

    inline BroadPhaseCollisionManagerPtr _CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }

    FCLCollisionManagerInstance& _GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs)
    {
        _bParentlessCollisionObject = false;
        BODYMANAGERSMAP::iterator it = _bodymanagers.find(std::make_pair(pbody.get(), (int)bactiveDOFs));
        if( it == _bodymanagers.end() ) {
            FCLCollisionManagerInstancePtr p(new FCLCollisionManagerInstance(*_fclspace, _CreateManager()));
            p->InitBodyManager(pbody, bactiveDOFs);
            it = _bodymanagers.insert(BODYMANAGERSMAP::value_type(std::make_pair(pbody.get(), (int)bactiveDOFs), p)).first;
        }

        it->second->Synchronize();
        //RAVELOG_VERBOSE_FORMAT("env=%d, returning body manager cache %x (self=%d)", GetEnv()->GetId()%it->second.get()%_bIsSelfCollisionChecker);
        //it->second->PrintStatus(OpenRAVE::Level_Info);
        return *it->second;
    }

    FCLCollisionManagerInstance& _GetEnvManager(const std::set<KinBodyConstPtr>& excludedbodies)
    {
        _bParentlessCollisionObject = false;
        std::set<int> setExcludeBodyIds; ///< any
        FOREACH(itbody, excludedbodies) {
            setExcludeBodyIds.insert((*itbody)->GetEnvironmentId());
        }

        // check the cache and cleanup any unused environments
        if( --_nGetEnvManagerCacheClearCount < 0 ) {
            uint32_t curtime = OpenRAVE::utils::GetMilliTime();
            _nGetEnvManagerCacheClearCount = 100000;
            std::map<std::set<int>, FCLCollisionManagerInstancePtr>::iterator it = _envmanagers.begin();
            while(it != _envmanagers.end()) {
                if( (it->second->GetLastSyncTimeStamp() - curtime) > 10000 ) {
                    //RAVELOG_VERBOSE_FORMAT("env=%d erasing manager at %u", GetEnv()->GetId()%it->second->GetLastSyncTimeStamp());
                    _envmanagers.erase(it++);
                }
                else {
                    ++it;
                }
            }
        }

        std::map<std::set<int>, FCLCollisionManagerInstancePtr>::iterator it = _envmanagers.find(setExcludeBodyIds);
        if( it == _envmanagers.end() ) {
            FCLCollisionManagerInstancePtr p(new FCLCollisionManagerInstance(*_fclspace, _CreateManager()));
            p->InitEnvironment(excludedbodies);
            it = _envmanagers.insert(std::map<std::set<int>, FCLCollisionManagerInstancePtr>::value_type(setExcludeBodyIds, p)).first;
        }
        it->second->EnsureBodies(_fclspace->GetEnvBodies());
        it->second->Synchronize();
        //it->second->PrintStatus(OpenRAVE::Level_Info);
        //RAVELOG_VERBOSE_FORMAT("env=%d, returning env manager cache %x (self=%d)", GetEnv()->GetId()%it->second.get()%_bIsSelfCollisionChecker);
        return *it->second;
    }

    void _PrintCollisionManagerInstanceB(const KinBody& body, FCLCollisionManagerInstance& manager)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%d, self=%d, body %s ", GetEnv()->GetId()%_bIsSelfCollisionChecker%body.GetName());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceSelf(const KinBody& body)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%d, self=%d, body %s ", GetEnv()->GetId()%_bIsSelfCollisionChecker%body.GetName());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceBL(const KinBody& body, FCLCollisionManagerInstance& manager, const KinBody::Link& link)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%d, self=%d, body %s with link %s:%s (enabled=%d) ", GetEnv()->GetId()%_bIsSelfCollisionChecker%body.GetName()%link.GetParent()->GetName()%link.GetName()%link.IsEnabled());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceBE(const KinBody& body, FCLCollisionManagerInstance& manager, FCLCollisionManagerInstance& envManager)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%d, self=%d, body %s ", GetEnv()->GetId()%_bIsSelfCollisionChecker%body.GetName());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstance(const KinBody& body1, FCLCollisionManagerInstance& manager1, const KinBody& body2, FCLCollisionManagerInstance& manager2)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%d, self=%d, body1 %s (enabled=%d) body2 %s (enabled=%d) ", GetEnv()->GetId()%_bIsSelfCollisionChecker%body1.GetName()%body1.IsEnabled()%body2.GetName()%body2.IsEnabled());
            _bParentlessCollisionObject = false;
        }
    }

    void _PrintCollisionManagerInstanceLE(const KinBody::Link& link, FCLCollisionManagerInstance& envManager)
    {
        if( _bParentlessCollisionObject ) {
            RAVELOG_WARN_FORMAT("env=%d, self=%d, link %s:%s (enabled=%d) ", GetEnv()->GetId()%_bIsSelfCollisionChecker%link.GetParent()->GetName()%link.GetName()%link.IsEnabled());
            _bParentlessCollisionObject = false;
        }
    }

    inline bool _IsEnabled(const KinBody& body)
    {
        if( body.IsEnabled() ) {
            return true;
        }

        // check if body has any enabled bodies
        body.GetGrabbed(_vCachedGrabbedBodies);
        FOREACH(itbody, _vCachedGrabbedBodies) {
            if( (*itbody)->IsEnabled() ) {
                return true;
            }
        }

        return false;
    }
    
    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
    std::string _broadPhaseCollisionManagerAlgorithm; ///< broadphase algorithm to use to create a manager. tested: Naive, DynamicAABBTree2

    typedef std::map< std::pair<const void*, int>, FCLCollisionManagerInstancePtr> BODYMANAGERSMAP; ///< Maps pairs of (body, bactiveDOFs) to oits manager
    BODYMANAGERSMAP _bodymanagers; ///< managers for each of the individual bodies. each manager should be called with InitBodyManager. Cannot use KinBodyPtr here since that will maintain a reference to the body!
    std::map< std::set<int>, FCLCollisionManagerInstancePtr> _envmanagers;
    int _nGetEnvManagerCacheClearCount; ///< count down until cache can be cleared

#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
    std::map<fcl::CollisionObject*, int> _currentlyused;
    std::map<fcl::CollisionObject*, std::map<int, int> > _usestatistics;
#endif

#ifdef NARROW_COLLISION_CACHING
    NarrowCollisionCache mCollisionCachedGuesses;
#endif

#ifdef FCLUSESTATISTICS
    FCLStatisticsPtr _statistics;
#endif

    // In order to reduce allocations during collision checking

    CollisionReport _reportcache;
    std::vector<fcl::Vec3f> _fclPointsCache;
    std::vector<fcl::Triangle> _fclTrianglesCache;
    std::vector<KinBodyPtr> _vCachedGrabbedBodies;

    bool _bIsSelfCollisionChecker; // Currently not used
    bool _bParentlessCollisionObject; ///< if set to true, the last collision command ran into colliding with an unknown object
};

} // fclrave

#endif
