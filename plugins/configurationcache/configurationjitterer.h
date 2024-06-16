#include <openrave/openravejson.h>
#include "configurationcachetree.h"

namespace configurationcache {

/// \brief holds parameters for threshing the direction. if dot(manipdir, tooldir) > cosanglethresh, then ok
class ManipDirectionThresh
{
public:
    ManipDirectionThresh() : vManipDir(0,0,1), vGlobalDir(0,0,1), fCosAngleThresh(0.9999999) {
    }
    ManipDirectionThresh(const ManipDirectionThresh &r) : vManipDir(r.vManipDir), vGlobalDir(r.vGlobalDir), fCosAngleThresh(r.fCosAngleThresh) {
    }

    void SaveToJson(rapidjson::Value& rManipDirectionThresh, rapidjson::Document::AllocatorType& alloc) const
    {
        rManipDirectionThresh.SetObject();
        orjson::SetJsonValueByKey(rManipDirectionThresh, "manipDir", vManipDir, alloc);
        orjson::SetJsonValueByKey(rManipDirectionThresh, "globalDir", vGlobalDir, alloc);
        orjson::SetJsonValueByKey(rManipDirectionThresh, "cosAngleThresh", fCosAngleThresh, alloc);
    }

    inline bool IsInConstraints(const Transform& tmanip) const
    {
        return tmanip.rotate(vManipDir).dot3(vGlobalDir) >= fCosAngleThresh;
    }

    /// \return the cos of the angle between current tmanip and the global dir
    inline dReal ComputeCosAngle(const Transform& tmanip) const {
        return tmanip.rotate(vManipDir).dot3(vGlobalDir);
    }

    Vector vManipDir; ///< direction on the manipulator
    Vector vGlobalDir; ///< direction in world coordinates
    dReal fCosAngleThresh; ///< the cos angle threshold
};

typedef OPENRAVE_SHARED_PTR<ManipDirectionThresh> ManipDirectionThreshPtr;

/// \brief holds parameters for threshing the position with respect to a bounding box.
class ManipPositionConstraints
{
public:
    ManipPositionConstraints() {
    }
    ManipPositionConstraints(const ManipPositionConstraints &r) : obb(r.obb) {
    }

    void SaveToJson(rapidjson::Value& rManipPositionConstraints, rapidjson::Document::AllocatorType& alloc) const
    {
        rManipPositionConstraints.SetObject();
        orjson::SetJsonValueByKey(rManipPositionConstraints, "right", obb.right, alloc);
        orjson::SetJsonValueByKey(rManipPositionConstraints, "up", obb.up, alloc);
        orjson::SetJsonValueByKey(rManipPositionConstraints, "dir", obb.dir, alloc);
        orjson::SetJsonValueByKey(rManipPositionConstraints, "pos", obb.pos, alloc);
        orjson::SetJsonValueByKey(rManipPositionConstraints, "extents", obb.extents, alloc);
    }

    inline bool IsInConstraints(const Transform& tmanip) const
    {
        // transform tmanip.trans in obb coordinate system
        Vector vdelta = tmanip.trans - obb.pos;
        dReal fright = obb.right.dot(vdelta);
        if( RaveFabs(fright) > obb.extents.x ) {
            return false;
        }
        dReal fup = obb.up.dot(vdelta);
        if( RaveFabs(fup) > obb.extents.y ) {
            return false;
        }
        dReal fdir = obb.dir.dot(vdelta);
        if( RaveFabs(fdir) > obb.extents.z ) {
            return false;
        }

        return true;
    }

    OBB obb;
};

typedef OPENRAVE_SHARED_PTR<ManipPositionConstraints> ManipPositionConstraintsPtr;

struct FailureCounter
{
    FailureCounter()
    {
    }

    inline void Reset() {
        nNeighStateFailure = 0;
        nConstraintToolDirFailure = 0;
        nConstraintToolPositionFailure = 0;
        nEnvCollisionFailure = 0;
        nSelfCollisionFailure = 0;
        nSameSamples = 0;
        nCacheHitSamples = 0;
        nLinkDistThreshRejections = 0;
    }

    void SaveToJson(rapidjson::Value& rFailureCounter, rapidjson::Document::AllocatorType& alloc) const
    {
        rFailureCounter.SetObject();
        orjson::SetJsonValueByKey(rFailureCounter, "neighStateFailure", nNeighStateFailure, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "constraintToolDirFailure", nConstraintToolDirFailure, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "constraintToolPositionFailure", nConstraintToolPositionFailure, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "envCollisionFailure", nEnvCollisionFailure, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "selfCollisionFailure", nSelfCollisionFailure, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "sameSamples", nSameSamples, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "cacheHitSamples", nCacheHitSamples, alloc);
        orjson::SetJsonValueByKey(rFailureCounter, "linkDistThreshRejections", nLinkDistThreshRejections, alloc);
    }

    int nNeighStateFailure = 0;
    int nConstraintToolDirFailure = 0;
    int nConstraintToolPositionFailure = 0;
    int nEnvCollisionFailure = 0;
    int nSelfCollisionFailure = 0;
    int nSameSamples = 0;
    int nCacheHitSamples = 0;
    int nLinkDistThreshRejections = 0;
}; // end struct FailureCounter

} // end namespace configurationcache
