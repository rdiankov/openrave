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

inline Vector ConvertVectorFromFCL(fcl::Vec3f const &v)
{
    return Vector(v[0], v[1], v[2]);
}

inline fcl::Vec3f ConvertVectorToFCL(Vector const &v)
{
    return fcl::Vec3f(v.x, v.y, v.z);
}
inline fcl::Quaternion3f ConvertQuaternionToFCL(Vector const &v)
{
    return fcl::Quaternion3f(v[0], v[1], v[2], v[3]);
}

inline Vector ConvertQuaternionFromFCL(fcl::Quaternion3f const &v) {
    return Vector(v.getW(), v.getX(), v.getY(), v.getZ());
}

inline fcl::AABB ConvertAABBToFcl(const OpenRAVE::AABB& bv) {
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

        FCLKinBodyInfo();

        virtual ~FCLKinBodyInfo() {
            Reset();
        }

        void Reset();

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

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey);

    virtual ~FCLSpace()
    {
        DestroyEnvironment();
    }

    void DestroyEnvironment();

    FCLKinBodyInfoPtr InitKinBody(KinBodyConstPtr pbody, FCLKinBodyInfoPtr pinfo = FCLKinBodyInfoPtr(), bool bSetToCurrentPInfo=true);

    bool HasNamedGeometry(const KinBody &body, const std::string& groupname);


    void SetGeometryGroup(const std::string& groupname);

    const std::string& GetGeometryGroup() const;

    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname);

    const std::string& GetBodyGeometryGroup(const KinBody &body) const;

    // Set the current bvhRepresentation and reinitializes all the KinbodyInfo if needed
    void SetBVHRepresentation(std::string const &type);

    std::string const& GetBVHRepresentation() const;

    void Synchronize();

    void Synchronize(const KinBody &body);

    void SynchronizeWithAttached(const KinBody &body);

    FCLKinBodyInfoPtr& GetInfo(const KinBody &body);

    const FCLKinBodyInfoPtr& GetInfo(const KinBody &body) const;

    void RemoveUserData(KinBodyConstPtr pbody);


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
            RAVELOG_WARN_FORMAT("env=%s, KinBody '%s' is not initialized in fclspace %s (self=%d)", _penv->GetNameId()%body.GetName()%_userdatakey%_bIsSelfCollisionChecker);
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

    // what about the tests on non-zero size (eg. box extents) ?
    CollisionGeometryPtr _CreateFCLGeomFromGeometryInfo(const KinBody::GeometryInfo &info);

    /// \brief pass in info.GetBody() as a reference to avoid dereferencing the weak pointer in FCLKinBodyInfo
    void _Synchronize(FCLKinBodyInfo& info, const KinBody& body);

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
    void _ResetCurrentGeometryCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

    void _ResetGeometryGroupsCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo);

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
