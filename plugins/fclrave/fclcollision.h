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
        CollisionCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker, CollisionReportPtr report, const std::vector<KinBodyConstPtr>& vbodyexcluded = std::vector<KinBodyConstPtr>(), const std::vector<LinkConstPtr>& vlinkexcluded = std::vector<LinkConstPtr>()) : _pchecker(pchecker), _report(report), _vbodyexcluded(vbodyexcluded), _vlinkexcluded(vlinkexcluded), bselfCollision(false), _bStopChecking(false), _bCollision(false)
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

    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
        : OpenRAVE::CollisionCheckerBase(penv), _broadPhaseCollisionManagerAlgorithm("DynamicAABBTree2"), _bIsSelfCollisionChecker(true) // DynamicAABBTree2 should be slightly faster than Naive
    {
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

    void SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname)
    {
        _fclspace->SetBodyGeometryGroup(pbody, groupname);
    }

    const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const
    {
        return _fclspace->GetBodyGeometryGroup(pbody);
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
        FCLSpace::KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<FCLSpace::KinBodyInfo>(pbody->GetUserData(_userdatakey));
        if( !pinfo || pinfo->GetBody() != pbody ) {
            pinfo = _fclspace->InitKinBody(pbody);
        }
        return !pinfo;
    }

    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        // remove body from all the managers
        _bodymanagers.erase(std::make_pair(pbody, (int)0));
        _bodymanagers.erase(std::make_pair(pbody, (int)1));
        FOREACH(itmanager, _envmanagers) {
            itmanager->second->RemoveBody(pbody);
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

        if( pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled() ) {
            return false;
        }

        if( pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled() ) {
            return false;
        }

        if( pbody1->IsAttached(pbody2) ) {
            return false;
        }

        _fclspace->Synchronize(pbody1);
        _fclspace->Synchronize(pbody2);

        // Do we really want to synchronize everything ?
        // We could put the synchronization directly inside GetBodyManager
        BroadPhaseCollisionManagerPtr body1Manager = _GetBodyManager(pbody1, !!(_options & OpenRAVE::CO_ActiveDOFs)), body2Manager = _GetBodyManager(pbody2, false);

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
        if( plink1->GetParent() != plink2->GetParent() ) {
            _fclspace->Synchronize(plink2->GetParent());
        }

        CollisionObjectPtr pcollLink1 = _fclspace->GetLinkBV(plink1), pcollLink2 = _fclspace->GetLinkBV(plink2);

        if( !pcollLink1 || !pcollLink2 ) {
            return false;
        }

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            if( !pcollLink1->getAABB().overlap(pcollLink2->getAABB()) ) {
                return false;
            }
            CollisionCallbackData query(shared_checker(), report);
            ADD_TIMING(_statistics);
            query.bselfCollision = true;  // for ignoring attached information!
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

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        FOREACH(itbody, attachedBodies) {
            if( (*itbody)->GetEnvironmentId() ) { // for now GetAttached can hold bodies that are not initialized
                _fclspace->Synchronize(*itbody);
            }
        }

        CollisionObjectPtr pcollLink = _fclspace->GetLinkBV(plink);

        if( !pcollLink ) {
            return false;
        }

        BroadPhaseCollisionManagerPtr bodyManager = _GetBodyManager(pbody, false);

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
        CollisionObjectPtr pcollLink = _fclspace->GetLinkBV(plink);

        if( !pcollLink ) {
            return false;
        }

        std::set<KinBodyConstPtr> attachedBodies;
        plink->GetParent()->GetAttached(attachedBodies);
        BroadPhaseCollisionManagerPtr envManager = _GetEnvManager(attachedBodies);

        if( _options & OpenRAVE::CO_Distance ) {
            return false;
        }
        else {
            CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
            ADD_TIMING(_statistics);
            envManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
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
        BroadPhaseCollisionManagerPtr bodyManager = _GetBodyManager(pbody, !!(_options & OpenRAVE::CO_ActiveDOFs));

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        BroadPhaseCollisionManagerPtr envManager = _GetEnvManager(attachedBodies);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
            ADD_TIMING(_statistics);
//            BODYMANAGERSMAP::iterator it0 = _bodymanagers.find(std::make_pair(pbody, (int)!!(_options & OpenRAVE::CO_ActiveDOFs)));
//            BOOST_ASSERT(it0 != _bodymanagers.end());
//
//            std::set<int> setExcludeBodyIds; ///< any
//            FOREACH(itbody, attachedBodies) {
//                setExcludeBodyIds.insert((*itbody)->GetEnvironmentId());
//            }
//
//            std::map<std::set<int>, FCLCollisionManagerInstancePtr>::iterator it1 = _envmanagers.find(setExcludeBodyIds);
//            BOOST_ASSERT(it1 != _envmanagers.end());
//            _bodymanager = it0->second;
//            _envmanager = it1->second;
            envManager->collide(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
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
        _fclspace->Synchronize(pbody);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; // TODO
        } else {
            CollisionCallbackData query(shared_checker(), report);
            ADD_TIMING(_statistics);
            query.bselfCollision = true;

            KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
            FOREACH(itset, nonadjacent) {
                size_t index1 = *itset&0xffff, index2 = *itset>>16;
                // We don't need to check if the links are enabled since we got adjacency information with AO_Enabled
                LinkInfoPtr pLINK1 = pinfo->vlinks[index1], pLINK2 = pinfo->vlinks[index2];
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
        _fclspace->Synchronize(pbody);

        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("fcl doesn't support CO_Distance yet\n");
            return false; //TODO
        } else {
            CollisionCallbackData query(shared_checker(), report);
            ADD_TIMING(_statistics);
            query.bselfCollision = true;
            KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
            FOREACH(itset, nonadjacent) {
                int index1 = *itset&0xffff, index2 = *itset>>16;
                if( plink->GetIndex() == index1 || plink->GetIndex() == index2 ) {
                    LinkInfoPtr pLINK1 = pinfo->vlinks[index1], pLINK2 = pinfo->vlinks[index2];
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


private:
    inline boost::shared_ptr<FCLCollisionChecker> shared_checker() {
        return boost::dynamic_pointer_cast<FCLCollisionChecker>(shared_from_this());
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
        LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

        if( !plink1 || !plink2 ) {
            return false;
        }

        // Proceed to the next if the links are attached or not enabled
        if( !plink1->IsEnabled() || !plink2->IsEnabled() ) {
            return false;
        }

        if( !pcb->bselfCollision && plink1->GetParent()->IsAttached(KinBodyConstPtr(plink2->GetParent())) ) {
            return false;
        }

        if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) ||
            IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) ||
            IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) ||
            IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
            return false;
        }

        LinkInfoPtr pLINK1 = _fclspace->GetLinkInfo(plink1), pLINK2 = _fclspace->GetLinkInfo(plink2);

        //RAVELOG_INFO_FORMAT("link %s:%s with %s:%s", plink1->GetParent()->GetName()%plink1->GetName()%plink2->GetParent()->GetName()%plink2->GetName());
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
                LinkConstPtr plink1 = GetCollisionLink(*o1), plink2 = GetCollisionLink(*o2);

                // these should be useless, just to make sure I haven't messed up
                BOOST_ASSERT( plink1 && plink2 );
                BOOST_ASSERT( plink1->IsEnabled() && plink2->IsEnabled() );
                BOOST_ASSERT( pcb->bselfCollision || !plink1->GetParent()->IsAttached(KinBodyConstPtr(plink2->GetParent())) );

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

    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data)
    {
        // TODO
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

    LinkConstPtr GetCollisionLink(const fcl::CollisionObject &collObj) {
        FCLSpace::KinBodyInfo::LINK *link_raw = static_cast<FCLSpace::KinBodyInfo::LINK *>(collObj.getUserData());
        if( !!link_raw ) {
            LinkConstPtr plink = link_raw->GetLink();
            if( !plink ) {
                RAVELOG_WARN_FORMAT("The link %s was lost from fclspace (env %d) (userdatakey %s)", link_raw->bodylinkname%GetEnv()->GetId()%_userdatakey);
            }
            return plink;
        }
        RAVELOG_WARN_FORMAT("fcl collision object does not have a link attached (env %d) (userdatakey %s)", GetEnv()->GetId()%_userdatakey);
        return LinkConstPtr();
    }

    inline BroadPhaseCollisionManagerPtr _CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }

    BroadPhaseCollisionManagerPtr _GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs)
    {
        BODYMANAGERSMAP::iterator it = _bodymanagers.find(std::make_pair(pbody, (int)bactiveDOFs));
        if( it == _bodymanagers.end() ) {
            FCLCollisionManagerInstancePtr p(new FCLCollisionManagerInstance(*_fclspace, _CreateManager()));
            p->InitBodyManager(pbody, bactiveDOFs);
            it = _bodymanagers.insert(BODYMANAGERSMAP::value_type(std::make_pair(pbody, (int)bactiveDOFs), p)).first;
        }

        it->second->Synchronize();
        //it->second->PrintStatus(OpenRAVE::Level_Info);
        return it->second->GetManager();
    }

    BroadPhaseCollisionManagerPtr _GetEnvManager(const std::set<KinBodyConstPtr>& excludedbodies)
    {
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
        return it->second->GetManager();
    }

    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
    std::string _broadPhaseCollisionManagerAlgorithm; ///< broadphase algorithm to use to create a manager. tested: Naive, DynamicAABBTree2

    typedef std::map< std::pair<KinBodyConstPtr, int>, FCLCollisionManagerInstancePtr> BODYMANAGERSMAP; ///< Maps pairs of (body, bactiveDOFs) to oits manager
    BODYMANAGERSMAP _bodymanagers; ///< managers for each of the individual bodies. each manager should be called with InitBodyManager.
    //std::map<KinBodyPtr, FCLCollisionManagerInstancePtr> _activedofbodymanagers; ///< managers for each of the individual bodies specifically when active DOF is used. each manager should be called with InitBodyManager
    std::map< std::set<int>, FCLCollisionManagerInstancePtr> _envmanagers;
    int _nGetEnvManagerCacheClearCount; ///< count down until cache can be cleared

//    FCLCollisionManagerInstancePtr _bodymanager, _envmanager;
//    fcl::CollisionObject* _o1, *_o2;
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

    bool _bIsSelfCollisionChecker; // Currently not used
};

} // fclrave

#endif
