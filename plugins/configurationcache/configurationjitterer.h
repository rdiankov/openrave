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

} // end namespace configurationcache
