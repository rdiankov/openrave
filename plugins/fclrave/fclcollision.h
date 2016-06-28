#ifndef OPENRAVE_FCL_COLLISION
#define OPENRAVE_FCL_COLLISION

#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>
#include <boost/function_output_iterator.hpp>

#include "fclspace.h"

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


#ifdef FCLRAVE_PERMANENT_UNSTABLE
std::vector<std::string> globalUnstableBodynames;
#endif // FCLRAVE_PERMANENT_UNSTABLE



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

    struct SpatialHashData {
        SpatialHashData(fcl::FCL_REAL cellSize, const fcl::Vec3f& sceneMin, const fcl::Vec3f& sceneMax) : _cellSize(cellSize), _sceneMin(sceneMin), _sceneMax(sceneMax) {
        }
        fcl::FCL_REAL _cellSize;
        fcl::Vec3f _sceneMin, _sceneMax;
    };


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



    class DistanceCallbackData {
public:
        DistanceCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker, CollisionReportPtr report, const std::vector<KinBodyConstPtr>& vbodyexcluded = std::vector<KinBodyConstPtr>(), const std::vector<LinkConstPtr>& vlinkexcluded = std::vector<LinkConstPtr>()) : _pchecker(pchecker), _report(report), _vbodyexcluded(vbodyexcluded), _vlinkexcluded(vlinkexcluded), bselfCollision(false), _bStopChecking(false)
        {
            // TODO : Should we left minDistance to max when queried the distance between 2 attached objects ?
            _minDistance = std::numeric_limits<fcl::FCL_REAL>::max();

            _bHasCallbacks = _pchecker->GetEnv()->HasRegisteredCollisionCallbacks();
            if( _bHasCallbacks && !_report ) {
                _report.reset(new CollisionReport());
            }

            // Interpreting CO_Contacts as closest pair of points
            if( !!report && !!(_pchecker->GetCollisionOptions() & OpenRAVE::CO_Contacts) ) {
                _request.enable_nearest_points = true;
            } else {
                _request.enable_nearest_points = false; // explicitly disable
            }

            // TODO : consider setting the two following paraneters
            //_request.rel_err
            //_request.abs_err

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
        fcl::DistanceRequest _request;
        fcl::DistanceResult _result;
        CollisionReportPtr _report;
        std::vector<KinBodyConstPtr> const& _vbodyexcluded;
        std::vector<LinkConstPtr> const& _vlinkexcluded;
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        bool bselfCollision;  ///< true if currently checking for self collision.
        bool _bStopChecking;  ///< if true, then stop the collision checking loop
        fcl::FCL_REAL _minDistance;   ///< result of the distance query

        bool _bHasCallbacks; ///< true if there's callbacks registered in the environment
        std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    };

    class RayCollisionCallbackData {
public:
        RayCollisionCallbackData(boost::shared_ptr<FCLCollisionChecker> pchecker, CollisionReportPtr report,  Vector origin) : _pchecker(pchecker), _report(report), _bStopChecking(false), _bCollision(false)
        {
            _origin = origin;
            _currentSquaredDistance = std::numeric_limits<dReal>::max();

            _bHasCallbacks = _pchecker->GetEnv()->HasRegisteredCollisionCallbacks();
            if( _bHasCallbacks && !_report ) {
                _report.reset(new CollisionReport());
            }

            // We need to enable the contacts if we want to return the closest hit
            if( !!report && !(_pchecker->GetCollisionOptions() & OpenRAVE::CO_RayAnyHit) ) {
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
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        Vector _origin;
        dReal _currentSquaredDistance;

        bool _bStopChecking;  ///< if true, then stop the collision checking loop
        bool _bCollision;  ///< result of the collision

        bool _bHasCallbacks; ///< true if there's callbacks registered in the environment
        std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    };

    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
        : OpenRAVE::CollisionCheckerBase(penv), _bIsSelfCollisionChecker(true), _ray(CollisionGeometryPtr()), _broadPhaseCollisionManagerAlgorithm("DynamicAABBTree2")
    {
        _userdatakey = std::string("fclcollision") + boost::lexical_cast<std::string>(this);
        _fclspace.reset(new FCLSpace(penv, _userdatakey));
        _options = 0;
        // TODO : Should we put a more reasonable arbitrary value ?
        _numMaxContacts = std::numeric_limits<int>::max();
        _rayRadius = 1e-5; //std::numeric_limits<fcl::FCL_REAL>::epsilon();
        __description = ":Interface Author: Kenji Maillard\n\nFlexible Collision Library collision checker";

        SETUP_STATISTICS(_statistics, _userdatakey, GetEnv()->GetId());


#ifdef FCLRAVE_PERMANENT_UNSTABLE
        // TODO : temporary command to test unstability
        RegisterCommand("SetUnstable", boost::bind(&FCLCollisionChecker::_SetUnstable, this, _1, _2), "register a body as unstable for all the collision checkers");
#endif

        // TODO : Consider removing these which could be more harmful than anything else
        RegisterCommand("SetBroadphaseAlgorithm", boost::bind(&FCLCollisionChecker::_SetBroadphaseAlgorithm, this, _1, _2), "sets the broadphase algorithm (Naive, SaP, SSaP, IntervalTree, DynamicAABBTree, DynamicAABBTree_Array)");
        RegisterCommand("SetBVHRepresentation", boost::bind(&FCLCollisionChecker::_SetBVHRepresentation, this, _1, _2), "sets the Bouding Volume Hierarchy representation for meshes (AABB, OBB, OBBRSS, RSS, kIDS)");
        RegisterCommand("SetSpatialHashingBroadPhaseAlgorithm", boost::bind(&FCLCollisionChecker::SetSpatialHashingBroadPhaseAlgorithm, this, _1, _2), "sets the broadphase algorithm to spatial hashing with (cell size) (scene min x) (scene min y) (scene min z) (scene max x) (scene max y) (scene max z)");

        RegisterCommand("PrepareRay", boost::bind(&FCLCollisionChecker::_DebugPrepareRay, this, _1, _2), "");
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

    bool _DebugPrepareRay(ostream& sout, istream& sinput) {
        RAY ray;
        sinput >> ray.pos.x >> ray.pos.y >> ray.pos.z >> ray.dir.x >> ray.dir.y >> ray.dir.z;
        PrepareRay(ray);

        //sout << _ray.getAABB().min_ << _ray.getAABB().max_ << std::endl;
        sout << TransformMatrix(Transform(ConvertQuaternionFromFCL(_ray.getQuatRotation()), ConvertVectorFromFCL(_ray.getTranslation())));
        return true;
    }

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<FCLCollisionChecker const> r = boost::dynamic_pointer_cast<FCLCollisionChecker const>(preference);
        // We don't clone Kinbody's specific geometry group nor unstable bodies
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

#ifdef FCLRAVE_PERMANENT_UNSTABLE
        FOREACH(itbodyname, globalUnstableBodynames) {
            RAVELOG_DEBUG_FORMAT("FCL User : Trying to set unstable status %s (env %d, %s)", *itbodyname%GetEnv()->GetId()%_userdatakey);
            KinBodyConstPtr pbody = GetEnv()->GetKinBody(*itbodyname);
            if( !!pbody ) {
                SetBodyUnstable(pbody);
            }
        }
#endif
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

#ifdef FCLRAVE_PERMANENT_UNSTABLE
    bool _SetUnstable(ostream& out, istream& sinput) {
        std::string bodyname;
        sinput >> bodyname;
        globalUnstableBodynames.push_back(bodyname);
        KinBodyConstPtr pbody = GetEnv()->GetKinBody(bodyname);
        if( !!pbody ) {
            SetBodyUnstable(pbody);
        }
        return !!pbody;
    }
#endif

    virtual void SetBodyUnstable(KinBodyConstPtr pbody, bool bsetUnstable = true)
    {
        _fclspace->SetBodyUnstable(pbody, bsetUnstable);
    }

    virtual bool IsBodyUnstable(KinBodyConstPtr pbody) const
    {
        return _fclspace->IsBodyUnstable(pbody);
    }

    virtual bool SetCollisionOptions(int collision_options)
    {
        _options = collision_options;

        // fcl does not support distance checking for non-spherical BV
        // if distance checking is necessary, kIOS seems to give the best results for collision checking
        if( !!(_options & OpenRAVE::CO_Distance) ) {
            const std::string& bvhrepr = _fclspace->GetBVHRepresentation();
            return ( bvhrepr == "RSS" || bvhrepr == "OBBRSS" || bvhrepr == "kIOS" );
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
#ifdef FCLRAVE_PERMANENT_UNSTABLE
        FOREACH(itbodyname, globalUnstableBodynames) {
            RAVELOG_DEBUG_FORMAT("FCL User : Trying to set unstable status %s (env %d, %s)", *itbodyname%GetEnv()->GetId()%_userdatakey);
            KinBodyConstPtr pbody = GetEnv()->GetKinBody(*itbodyname);
            if( !!pbody ) {
                SetBodyUnstable(pbody);
            }
        }
#endif
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
#ifdef FCLRAVE_PERMANENT_UNSTABLE
        auto it = std::find(globalUnstableBodynames.begin(), globalUnstableBodynames.end(), pbody->GetName());
        if( it != globalUnstableBodynames.end() ) {
            SetBodyUnstable(pbody);
        }
#endif
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

        BroadPhaseCollisionManagerPtr body1Manager = GetBodyManager(pbody1, _options & OpenRAVE::CO_ActiveDOFs), body2Manager = GetBodyManager(pbody2, false);

        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report);
            body1Manager->distance(body2Manager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);

            report->minDistance = (dReal)query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }

        CollisionCallbackData query(shared_checker(), report);
        ADD_TIMING(_statistics);
        body1Manager->collide(body2Manager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
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

        _fclspace->Synchronize(plink1->GetParent());
        _fclspace->Synchronize(plink2->GetParent());

        CollisionObjectPtr pcollLink1 = GetLinkBV(plink1), pcollLink2 = GetLinkBV(plink2);

        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report);
            query.bselfCollision = true;
            CheckNarrowPhaseDistance(pcollLink1.get(), pcollLink2.get(), &query, query._minDistance);

            report->minDistance = (dReal)query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }

        if( !pcollLink1->getAABB().overlap(pcollLink2->getAABB()) ) {
            return false;
        }
        CollisionCallbackData query(shared_checker(), report);
        query.bselfCollision = true;
        ADD_TIMING(_statistics);
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

        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
            return false;
        }

        if( pbody->IsAttached(plink->GetParent()) ) {
            return false;
        }

        _fclspace->Synchronize(plink->GetParent());

        CollisionObjectPtr pcollLink = GetLinkBV(plink);
        BroadPhaseCollisionManagerPtr bodyManager = GetBodyManager(pbody, false);

        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report);
            bodyManager->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);

            report->minDistance = (dReal) query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }


        CollisionCallbackData query(shared_checker(), report);
        ADD_TIMING(_statistics);
        bodyManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
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

        CollisionObjectPtr pcollLink = GetLinkBV(plink);

        std::set<KinBodyConstPtr> attachedBodies;
        plink->GetParent()->GetAttached(attachedBodies);
        BroadPhaseCollisionManagerPtr envManager = GetEnvManager(attachedBodies);

        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
            envManager->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);

            // The unstable bodies are not contained in the envManager, we need to consider them separately
            FOREACH(itbody, _fclspace->GetUnstableBodies()) {
                if( query._bStopChecking ) {
                    break;
                }

                if( !_fclspace->GetInfo(*itbody) || (*itbody)->IsAttached(plink->GetParent()) ) {
                    continue;
                }
                // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
                BroadPhaseCollisionManagerPtr itbodyManager = GetBodyManager(*itbody, false);
                itbodyManager->distance(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
            }

            report->minDistance = (dReal) query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }

        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);
        envManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

        // The unstable bodies are not contained in the envManager, we need to consider them separately
        FOREACH(itbody, _fclspace->GetUnstableBodies()) {
            if( query._bStopChecking ) {
                return query._bCollision;
            }

            if( !_fclspace->GetInfo(*itbody) || (*itbody)->IsAttached(plink->GetParent()) ) {
                continue;
            }
            // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
            BroadPhaseCollisionManagerPtr itbodyManager = GetBodyManager(*itbody, false);
            itbodyManager->collide(pcollLink.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

        }

        return query._bCollision;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())
    {
        if( !!report ) {
            report->Reset(_options);
        }

        if( (pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            return false;
        }


        BroadPhaseCollisionManagerPtr bodyManager = GetBodyManager(pbody, _options & OpenRAVE::CO_ActiveDOFs);

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        BroadPhaseCollisionManagerPtr envManager = GetEnvManager(attachedBodies);

        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
            envManager->distance(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);

            // The unstable bodies are not contained in the envManager, we need to consider them separately
            FOREACH(itbody, _fclspace->GetUnstableBodies()) {
                if( query._bStopChecking ) {
                    break;
                }

                if( !_fclspace->GetInfo(*itbody) || (pbody)->IsAttached(*itbody) ) {
                    continue;
                }
                // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
                BroadPhaseCollisionManagerPtr itbodyManager = GetBodyManager(*itbody, false);
                itbodyManager->distance(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseDistance);
            }

            report->minDistance = (dReal) query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }

        CollisionCallbackData query(shared_checker(), report, vbodyexcluded, vlinkexcluded);
        ADD_TIMING(_statistics);
        envManager->collide(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);

        // The robots are not contained in the envManager, we need to consider them separately
        FOREACH(itbody, _fclspace->GetUnstableBodies()) {
            if( query._bStopChecking ) {
                return query._bCollision;
            }

            if( !_fclspace->GetInfo(*itbody) || (*itbody)->IsAttached(pbody) ) {
                continue;
            }
            // seems that activeDOFs are not considered in oderave : the check is done but it will always return true
            BroadPhaseCollisionManagerPtr itbodyManager = GetBodyManager(*itbody, false);
            itbodyManager->collide(bodyManager.get(), &query, &FCLCollisionChecker::CheckNarrowPhaseCollision);
        }

        return query._bCollision;
    }

    virtual bool CheckCollision(RAY const &ray, LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr())
    {

        if( !!report ) {
            report->Reset(_options);
        }

        if( !plink->IsEnabled() ) {
            return false;
        }

        PrepareRay(ray);
        RayCollisionCallbackData query(shared_checker(), report, ray.pos);
        CheckNarrowPhaseRayCollision(GetLinkBV(plink).get(), &_ray, &query);
        return query._bCollision;
    }

    virtual bool CheckCollision(RAY const &ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {

        if( !!report ) {
            report->Reset(_options);
        }

        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
            return false;
        }

        PrepareRay(ray);
        BroadPhaseCollisionManagerPtr bodyManager = GetBodyManager(pbody, _options & OpenRAVE::CO_ActiveDOFs);
        RayCollisionCallbackData query(shared_checker(), report, ray.pos);
        bodyManager->collide(&_ray, &query, &FCLCollisionChecker::CheckNarrowPhaseRayCollision);
        return query._bCollision;
    }

    virtual bool CheckCollision(RAY const &ray, CollisionReportPtr report = CollisionReportPtr())
    {
        if( !!report ) {
            report->Reset(_options);
        }

        PrepareRay(ray);
        std::set<KinBodyConstPtr> empty;
        BroadPhaseCollisionManagerPtr envManager = GetEnvManager(empty);
        RayCollisionCallbackData query(shared_checker(), report, ray.pos);
        envManager->collide(&_ray, &query, &FCLCollisionChecker::CheckNarrowPhaseRayCollision);

        // The unstable bodies are not contained in the envManager, we need to consider them separately
        FOREACH(itbody, _fclspace->GetUnstableBodies()) {
            if( query._bStopChecking ) {
                return query._bCollision;
            }

            if( !_fclspace->GetInfo(*itbody) || !(*itbody)->IsEnabled() ) {
                continue;
            }
            BroadPhaseCollisionManagerPtr itbodyManager = GetBodyManager(*itbody, false);
            itbodyManager->collide(&_ray, &query, &FCLCollisionChecker::CheckNarrowPhaseRayCollision);

        }

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

        const std::set<int> &nonadjacent = pbody->GetNonAdjacentLinks(adjacentOptions);
        // We need to synchronize after calling GetNonAdjacentLinks since it can move pbody even if it is const

        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report);
            query.bselfCollision = true;
            KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
            FOREACH(itset, nonadjacent) {
                size_t index1 = *itset&0xffff, index2 = *itset>>16;
                // We don't need to check if the links are enabled since we got adjacency information with AO_Enabled
                LinkInfoPtr pLINK1 = pinfo->vlinks[index1], pLINK2 = pinfo->vlinks[index2];
                CheckNarrowPhaseDistance(pLINK1->linkBV.second.get(), pLINK2->linkBV.second.get(), &query, query._minDistance);
                if( query._bStopChecking ) {
                    break;
                }
            }

            report->minDistance = (dReal) query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }

        CollisionCallbackData query(shared_checker(), report);
        ADD_TIMING(_statistics);
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


        if( !!report && (_options & OpenRAVE::CO_Distance) ) {
            DistanceCallbackData query(shared_checker(), report);
            query.bselfCollision = true;
            KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
            FOREACH(itset, nonadjacent) {
                size_t index1 = *itset&0xffff, index2 = *itset>>16;
                if( plink->GetIndex() != index1 && plink->GetIndex() != index2 ) {
                    continue;
                }
                // We don't need to check if the links are enabled since we got adjacency information with AO_Enabled
                LinkInfoPtr pLINK1 = pinfo->vlinks[index1], pLINK2 = pinfo->vlinks[index2];
                CheckNarrowPhaseDistance(pLINK1->linkBV.second.get(), pLINK2->linkBV.second.get(), &query, query._minDistance);
                if( query._bStopChecking ) {
                    break;
                }
            }

            report->minDistance = (dReal) query._minDistance;
            bool bCollision = report->minDistance <= std::numeric_limits<dReal>::epsilon();

            // if there is no collision or no need to gather more data, just return
            if( !bCollision || !(_options & (OpenRAVE::CO_AllLinkCollisions | OpenRAVE::CO_AllGeometryContacts))) {
                return bCollision;
            }
        }

        CollisionCallbackData query(shared_checker(), report);
        ADD_TIMING(_statistics);
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


private:

    inline CollisionObjectPtr GetLinkBV(LinkConstPtr plink) {
        return GetLinkBV(plink->GetParent(), plink->GetIndex());
    }

    inline CollisionObjectPtr GetLinkBV(KinBodyConstPtr pbody, int index) {
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
        if( !!pinfo ) {
            return GetLinkBV(pinfo, index);
        } else {
            RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%pbody->GetName()%_userdatakey%GetEnv()->GetId()));
            return CollisionObjectPtr();
        }
    }

    inline CollisionObjectPtr GetLinkBV(KinBodyInfoPtr pinfo, int index) {
        return pinfo->vlinks.at(index)->linkBV.second;
    }

    inline BroadPhaseCollisionManagerPtr CreateManager() {
        return _CreateManagerFromBroadphaseAlgorithm(_broadPhaseCollisionManagerAlgorithm);
    }

    inline boost::shared_ptr<FCLCollisionChecker> shared_checker() {
        return boost::dynamic_pointer_cast<FCLCollisionChecker>(shared_from_this());
    }


    void CollectEnabledLinkBVs(KinBodyConstPtr pbody, CollisionGroup& group) {
        KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
        // not necessary at the moment
        // group.reserve(group.size() + pbody->GetLinks().size());
        FOREACH(itlink, pbody->GetLinks()) {
            if( ((*itlink)->IsEnabled()) ) {
                // not a very good idea to access it directly for code maintenance
                group.push_back(GetLinkBV(pinfo, (*itlink)->GetIndex()).get());
            }
        }
    }

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

    BroadPhaseCollisionManagerPtr SetupManagerAllDOFs(KinBodyConstPtr pbody, KinBodyInfoPtr pinfo, const std::set<KinBodyConstPtr>& attachedBodies) {

        if( !pinfo->_bodyManager ) {

            pinfo->_bodyManager = boost::make_shared<ManagerInstance>();
            pinfo->_bodyManager->pmanager = CreateManager();

            // Compute current collision objects
            _tmpbuffer.resize(0);
            FOREACH(itbody, attachedBodies) {
                // The attached bodies might not be in the environment (Strange feature of current OpenRAVE)
                if( !!_fclspace->GetInfo(*itbody) ) {
                    CollectEnabledLinkBVs(*itbody, _tmpbuffer);
                    pinfo->_bodyManager->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(*itbody), _fclspace->GetInfo(*itbody)->nLastStamp));
                }
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

    BroadPhaseCollisionManagerPtr SetupManagerActiveDOFs(RobotBaseConstPtr probot, KinBodyInfoPtr pinfo, const std::set<KinBodyConstPtr>& attachedBodies) {
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

            _tmpbuffer.resize(0);

            FOREACH(itbody, attachedBodies) {
                if( *itbody == probot ) {
                    _tmpbuffer.reserve(probot->GetLinks().size());
                    FOREACH(itlink, probot->GetLinks()) {
                        int index = (*itlink)->GetIndex();
                        if( (*itlink)->IsEnabled() && pinfo->_vactiveLinks[index] ) {
                            _tmpbuffer.push_back(GetLinkBV(pinfo, index).get());
                        }
                    }
                    pinfo->_bodyManagerActiveDOFs->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(probot), pinfo->nLastStamp));
                } else if( !!_fclspace->GetInfo(*itbody) ) {
                    LinkConstPtr pgrabbinglink = probot->IsGrabbing(*itbody);
                    if( !!pgrabbinglink && pinfo->_vactiveLinks[pgrabbinglink->GetIndex()] ) {
                        CollectEnabledLinkBVs(*itbody, _tmpbuffer);
                        pinfo->_bodyManagerActiveDOFs->vUpdateStamps.push_back(std::make_pair(KinBodyConstWeakPtr(*itbody), _fclspace->GetInfo(*itbody)->nLastStamp));
                    }
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

        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        // we need to synchronize all the involved bodies before setting up the manager
        FOREACH(itbody, attachedBodies) {
            _fclspace->Synchronize(*itbody);
        }

        if( ballDOFs ) {
            return SetupManagerAllDOFs(pbody, pinfo, attachedBodies);
        } else {
            return SetupManagerActiveDOFs(probot, pinfo, attachedBodies);
        }
    }


    ///< \param _sexcludedbodies set of bodies which can be safely omitted from the collision checking
    ///< \return an up to date and setup broadphase collision manager containing at least the collision object of the enviroment which are relevant
    BroadPhaseCollisionManagerPtr GetEnvManager(const std::set<KinBodyConstPtr>&sexcludedbodies) {

        _fclspace->Synchronize();

        ManagerInstancePtr envManagerInstance = _fclspace->GetEnvManagerInstance();

#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
        FOREACH(itbody, _fclspace->GetEnvExcludiedBodies()) {
            FOREACH(itpLINK, _fclspace->GetInfo(*itbody)->vlinks) {
                fcl::CollisionObject* pcoll = (*itpLINK)->linkBV.second.get();
                int& n = _currentlyused[pcoll];
                if( n > 0 ) {
                    _usestatistics[pcoll][n]++;
                }
                n = 0;
            }
        }
#endif

        if( !envManagerInstance ) {

            RAVELOG_VERBOSE_FORMAT("FCL COLLISION : Rebuilding env manager (env = %d)", GetEnv()->GetId());

            _tmpbuffer.resize(0);
            envManagerInstance = boost::make_shared<ManagerInstance>();
            envManagerInstance->pmanager = CreateManager();
            FOREACH(itbody, _fclspace->GetEnvBodies()) {
                // we treat the case of unstable kinbodies separately
                if( _fclspace->IsBodyUnstable(*itbody) || sexcludedbodies.count(*itbody) ) {
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
            // - when an object is added or rebuilt from scratch (FCLSpace::InitKinBody)
            // - when a geometry group is modified (FCLSpace::SetBodyGeometryGroup)
            // - when a link is enabled/disabled : (FCLSpace::InitKinBody)

        } else {

            // We need to add to the env manager all the previously excluded bodies which are not excluded anymore
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

#ifndef FCLRAVE_COLLISION_OBJECTS_STATISTICS
            UpdateManagerInstance(envManagerInstance);
#else
            FOREACH(itWkbodyStampPair, envManagerInstance->vUpdateStamps) {
                KinBodyConstPtr pbody = itWkbodyStampPair->first.lock();
                if( !!pbody ) {
                    KinBodyInfoPtr pinfo = _fclspace->GetInfo(pbody);
                    if( !!pinfo && itWkbodyStampPair->second < pbody->GetUpdateStamp() ) {
                        itWkbodyStampPair->second = pbody->GetUpdateStamp();
                        FOREACH(itlink, pbody->GetLinks()) {
                            if( (*itlink)->IsEnabled() ) {
                                // Do not forget to setup the manager at the end since we are deactivating the automatic setup there
                                envManagerInstance->pmanager->update(GetLinkBV(pinfo, (*itlink)->GetIndex()).get(), false);
                                fcl::CollisionObject* pcoll = GetLinkBV(pinfo, (*itlink)->GetIndex()).get();
                                int& n = _currentlyused[pcoll];
                                if( n > 0 ) {
                                    _usestatistics[pcoll][n]++;
                                }
                                n = 0;

                            }
                        }
                    }
                }
            }
            envManagerInstance->pmanager->setup();
#endif
        }

#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
        CollisionGroup tmpgroup;
        envManagerInstance->pmanager->getObjects(tmpgroup);
        FOREACH(itcoll, tmpgroup) {
            _currentlyused[*itcoll]++;
        }
#endif

        return envManagerInstance->pmanager;
    }

    inline LinkInfoPtr GetLinkInfo(LinkConstPtr plink) {
        return _fclspace->GetInfo(plink->GetParent())->vlinks[plink->GetIndex()];
    }

    static bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data) {
        CollisionCallbackData* pcb = static_cast<CollisionCallbackData *>(data);
        return pcb->_pchecker->CheckNarrowPhaseCollision(o1, o2, pcb);
    }

    bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, CollisionCallbackData* pcb) {

        if( pcb->_bStopChecking ) {
            return true;     // don't test anymore
        }

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

        if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
            return false;
        }

        LinkInfoPtr pLINK1 = GetLinkInfo(plink1), pLINK2 = GetLinkInfo(plink2);

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

    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& minDistance)
    {
        DistanceCallbackData* pcb = static_cast<DistanceCallbackData*>(data);
        return pcb->_pchecker->CheckNarrowPhaseDistance(o1, o2, pcb, minDistance);
    }

    bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, DistanceCallbackData* pcb, fcl::FCL_REAL& minDistance)
    {
        if( pcb->_bStopChecking ) {
            return true;     // don't test anymore
        }

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

        if( IsIn<KinBodyConstPtr>(plink1->GetParent(), pcb->_vbodyexcluded) || IsIn<KinBodyConstPtr>(plink2->GetParent(), pcb->_vbodyexcluded) || IsIn<LinkConstPtr>(plink1, pcb->_vlinkexcluded) || IsIn<LinkConstPtr>(plink2, pcb->_vlinkexcluded) ) {
            return false;
        }

        LinkInfoPtr pLINK1 = GetLinkInfo(plink1), pLINK2 = GetLinkInfo(plink2);

        FOREACH(itgeompair1, pLINK1->vgeoms) {
            FOREACH(itgeompair2, pLINK2->vgeoms) {
                CheckNarrowPhaseGeomDistance(itgeompair1->second.get(), itgeompair2->second.get(), pcb);
                if( pcb->_bStopChecking ) {
                    minDistance = pcb->_minDistance;
                    return true;
                }
            }
        }

        minDistance = pcb->_minDistance;
        return pcb->_bStopChecking;
    }


    void CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2,DistanceCallbackData* pcb) {
        pcb->_result.clear();
        fcl::FCL_REAL dist = fcl::distance(o1, o2, pcb->_request, pcb->_result);
        if( dist < pcb->_minDistance ) {
            pcb->_minDistance = dist;

            pcb->_report->plink1 = GetCollisionLink(*o1);
            pcb->_report->plink2 = GetCollisionLink(*o2);

            //It would almost make sense to use the CONTACT structure to report nearest points but it not really hygenic...
            //if( pcb->_report->_options & OpenRAVE::CO_Contacts ) {
            //  CollisionReport::CONTACT(ConvertVectorFromFCL(c.pos), ConvertVectorFromFCL(c.normal), c.penetration_depth);
            //}

            if( dist <= std::numeric_limits<fcl::FCL_REAL>::epsilon() ) {
                // Should we call the collision callbacks here ?
                pcb->_bStopChecking = true;
            }
        }
    }


    static bool CheckNarrowPhaseRayCollision(fcl::CollisionObject* o1, fcl::CollisionObject* ray, void* data) {
        RayCollisionCallbackData* pcb = static_cast<RayCollisionCallbackData*>(data);
        return pcb->_pchecker->CheckNarrowPhaseRayCollision(o1, ray, pcb);
    }

    bool CheckNarrowPhaseRayCollision(fcl::CollisionObject* o1, fcl::CollisionObject* ray, RayCollisionCallbackData* pcb) {
        BOOST_ASSERT(ray == &_ray);

        if( pcb->_bStopChecking ) {
            return true;
        }

        LinkConstPtr plink = GetCollisionLink(*o1);

        if( !plink || !plink->IsEnabled() ) {
            return false;
        }

        LinkInfoPtr pLINK = GetLinkInfo(plink);

        FOREACH(itgeompair, pLINK->vgeoms) {
            if(itgeompair->second->getAABB().overlap(_ray.getAABB())) {
                pcb->_result.clear();
                std::size_t numContacts = fcl::collide(itgeompair->second.get(), ray, pcb->_request, pcb->_result);
                if( numContacts > 0 ) {
                    // TODO : Should I call the collision callbacks ?
                    pcb->_bCollision = true;

                    if( _options & OpenRAVE::CO_RayAnyHit ) {
                        pcb->_bStopChecking = true;
                        if( !!pcb->_report ) {
                            pcb->_report->plink1 = plink;
                        }
                        return true;
                    } else {
                        for( size_t i = 0; i < numContacts; ++i) {
                            const fcl::Contact& c = pcb->_result.getContact(i);
                            dReal squaredDistance = (ConvertVectorFromFCL(c.pos) - pcb->_origin).lengthsqr3();
                            if( squaredDistance < pcb->_currentSquaredDistance ) {
                                pcb->_currentSquaredDistance = squaredDistance;
                                if( !!pcb->_report ) {
                                    pcb->_report->plink1 = plink;
                                    pcb->_report->contacts.resize(1);
                                    pcb->_report->contacts[0] = CollisionReport::CONTACT(ConvertVectorFromFCL(c.pos), ConvertVectorFromFCL(c.normal), c.penetration_depth);
                                }
                            }
                        }
                    }
                }
            }
        }

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

    void PrepareRay(const RAY& ray) {
        dReal dirNorm = MATH_SQRT(ray.dir.lengthsqr3());
        _pRayGeometry = std::make_shared<fcl::Capsule>(_rayRadius, (fcl::FCL_REAL)dirNorm);
        _ray = fcl::CollisionObject(_pRayGeometry);
        _ray.setTranslation(ConvertVectorToFCL(ray.pos + 0.5f*ray.dir));

        // If the ray is not aligned along the z-axis, we need to compute its rotation wrt the z-axis
        if( MATH_FABS(ray.dir.x) > _rayRadius || MATH_FABS(ray.dir.y) > _rayRadius ) {
            dReal epsilon = std::numeric_limits<dReal>::epsilon();
            Vector dir = ray.dir, up(0,0,1);
            if ( dirNorm < 1 - epsilon || dirNorm > 1 + epsilon ) {
                dir = (1/dirNorm) * ray.dir;
            }
            Vector s = dir.cross(up).normalize3();
            Vector upn = dir.cross(s).normalize3();
            _ray.setRotation(fcl::Matrix3f(ConvertVectorToFCL(s), ConvertVectorToFCL(upn), ConvertVectorToFCL(dir)).transpose());
        }
        _ray.computeAABB();
    }



    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
    bool _bIsSelfCollisionChecker; // Currently not used

    fcl::FCL_REAL _rayRadius;
    CollisionGeometryPtr _pRayGeometry;
    fcl::CollisionObject _ray;

    std::string _broadPhaseCollisionManagerAlgorithm;
    boost::shared_ptr<SpatialHashData> _spatialHashData;

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
    CollisionGroup _tmpbuffer;
};

} // fclrave

#endif
