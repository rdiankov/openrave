#include "plugindefs.h"

#include "fclspace.h"
#include <fcl/container.h>

namespace fclrave {

template <class T>
CollisionGeometryPtr ConvertMeshToFCL(std::vector<fcl::Vec3f> const &points,std::vector<fcl::Triangle> const &triangles)
{
    std::shared_ptr< fcl::BVHModel<T> > const model = make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}

void FCLSpace::FCLKinBodyInfo::Reset()
{
    FOREACH(itlink, vlinks) {
        (*itlink)->Reset();
    }
    vlinks.resize(0);
    _geometrycallback.reset();
    _geometrygroupcallback.reset();
    _linkenablecallback.reset();
}

FCLSpace::FCLKinBodyInfo::FCLGeometryInfo::FCLGeometryInfo()
{
}

FCLSpace::FCLKinBodyInfo::LinkInfo::LinkInfo() : bFromKinBodyLink(false)
{
}

FCLSpace::FCLKinBodyInfo::LinkInfo::LinkInfo(KinBody::LinkPtr plink) : _plink(plink), bFromKinBodyLink(true)
{
}

FCLSpace::FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
    : _penv(penv)
    , _userdatakey(userdatakey)
    , _currentpinfo(1, FCLKinBodyInfoPtr()) // initialize with one null pointer, this is a place holder for null pointer so that we can return by reference. env id 0 means invalid so it's consistent with the definition as well
    , _bIsSelfCollisionChecker(true)
{
    // After many test, OBB seems to be the only real option (followed by kIOS which is needed for distance checking)
    SetBVHRepresentation("OBB");
}

void FCLSpace::DestroyEnvironment()
{
    RAVELOG_VERBOSE_FORMAT("destroying fcl collision environment (env %d) (userdatakey %s)", _penv->GetId()%_userdatakey);
    for (KinBodyConstPtr pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
        if( !!pinfo ) {
            pinfo->Reset();
        }
    }
    // even after DestroyEnvironment is called, users of this class still try to access _currentpinfo
    // in that case, null pointer should be returned, instead of range error from _currentpinfo.at(0). for that purpose, keep the first element here.
    _currentpinfo.erase(_currentpinfo.begin() + 1, _currentpinfo.end());
    _cachedpinfo.clear();
    _vecInitializedBodies.clear();
}

void FCLSpace::ReloadKinBodyLinks(KinBodyConstPtr pbody, FCLKinBodyInfoPtr pinfo) {
    // If the body hasn't changed, don't reload the links.
    if (pbody->GetUpdateStamp() == pinfo->nLastLinkReloadStamp) {
        return;
    }
    pinfo->nLastLinkReloadStamp = pbody->GetUpdateStamp();

    pinfo->vlinks.clear();
    pinfo->vlinks.reserve(pbody->GetLinks().size());
    FOREACHC(itlink, pbody->GetLinks()) {
        const KinBody::LinkPtr& plink = *itlink;
        boost::shared_ptr<FCLKinBodyInfo::LinkInfo> linkinfo(new FCLKinBodyInfo::LinkInfo(plink));

        fcl::AABB enclosingBV;

        // Glue code for a unified access to geometries
        if(pinfo->_geometrygroup.size() > 0 && plink->GetGroupNumGeometries(pinfo->_geometrygroup) >= 0) {
            const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos = plink->GetGeometriesFromGroup(pinfo->_geometrygroup);
            FOREACH(itgeominfo, vgeometryinfos) {
                const KinBody::GeometryInfoPtr& pgeominfo = *itgeominfo;
                if( !pgeominfo ) {
                    int igeominfo = itgeominfo - vgeometryinfos.begin();
                    throw OpenRAVE::OpenRAVEException(str(boost::format("Failed to access geometry info %d for link %s:%s with geometrygroup %s")%igeominfo%plink->GetParent()->GetName()%plink->GetName()%pinfo->_geometrygroup), OpenRAVE::ORE_InvalidState);
                }
                const KinBody::GeometryInfo& geominfo = *pgeominfo;
                const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(geominfo);

                if( !pfclgeom ) {
                    continue;
                }
                // currently there is no information about which geometry group was used for collision checking.
                // It's usually obvious immediately after CheckCollision is called, but later on, it it is not that obvious collision report is computed with which geometry group.
                boost::shared_ptr<FCLKinBodyInfo::FCLGeometryInfo> pfclgeominfo(new FCLKinBodyInfo::FCLGeometryInfo());
                pfclgeominfo->geomname = geominfo._name;
                pfclgeom->setUserData(pfclgeominfo.get());
                // save the pointers
                linkinfo->vgeominfos.push_back(pfclgeominfo);

                // We do not set the transformation here and leave it to _Synchronize
                CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                pfclcoll->setUserData(linkinfo.get());
                linkinfo->vgeoms.push_back(TransformCollisionPair(geominfo.GetTransform(), pfclcoll));

                KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), geominfo);
                if( itgeominfo == vgeometryinfos.begin() ) {
                    enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                }
                else {
                    enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                }
            }
            linkinfo->bFromExtraGeometries = true;
        }
        else {
            const std::vector<KinBody::Link::GeometryPtr> & vgeometries = plink->GetGeometries();
            FOREACH(itgeom, vgeometries) {
                const KinBody::GeometryPtr& pgeom = *itgeom;
                const KinBody::GeometryInfo& geominfo = pgeom->GetInfo();
                const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(geominfo);

                if( !pfclgeom ) {
                    continue;
                }
                boost::shared_ptr<FCLKinBodyInfo::FCLGeometryInfo> pfclgeominfo(new FCLKinBodyInfo::FCLGeometryInfo());
                pfclgeominfo->geomname = geominfo._name;
                pfclgeom->setUserData(pfclgeominfo.get());
                // save the pointers
                linkinfo->vgeominfos.push_back(pfclgeominfo);

                // We do not set the transformation here and leave it to _Synchronize
                CollisionObjectPtr pfclcoll = boost::make_shared<fcl::CollisionObject>(pfclgeom);
                pfclcoll->setUserData(linkinfo.get());

                linkinfo->vgeoms.push_back(TransformCollisionPair(geominfo.GetTransform(), pfclcoll));

                KinBody::Link::Geometry _tmpgeometry(boost::shared_ptr<KinBody::Link>(), geominfo);
                if( itgeom == vgeometries.begin() ) {
                    enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                }
                else {
                    enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                }
            }
            linkinfo->bFromExtraGeometries = false;
        }

        if( linkinfo->vgeoms.size() == 0 ) {
            RAVELOG_DEBUG_FORMAT("env=%s, Initializing body '%s' (index=%d) link '%s' with 0 geometries (env %d) (userdatakey %s, group=%s, bodygroup=%s)", _penv->GetNameId()%pbody->GetName()%pbody->GetEnvironmentBodyIndex()%plink->GetName()%_penv->GetId()%_userdatakey%_geometrygroup%pinfo->_geometrygroup);
        }
        else {
            CollisionGeometryPtr pfclgeomBV = std::make_shared<fcl::Box>(enclosingBV.max_ - enclosingBV.min_);
            pfclgeomBV->setUserData(nullptr);
            CollisionObjectPtr pfclcollBV = boost::make_shared<fcl::CollisionObject>(pfclgeomBV);
            const Vector trans = ConvertVectorFromFCL(0.5 * (enclosingBV.min_ + enclosingBV.max_));
            pfclcollBV->setUserData(linkinfo.get());
            linkinfo->linkBV = std::make_pair(trans, pfclcollBV);
        }

        linkinfo->nLastStamp = pinfo->nLastStamp;
        linkinfo->bodylinkname = pbody->GetName() + "/" + plink->GetName();
        pinfo->vlinks.push_back(linkinfo);
#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
        RAVELOG_DEBUG_FORMAT("FCLSPACECOLLISIONOBJECT|%s|%s", linkinfo->linkBV.second.get()%linkinfo->bodylinkname);
#endif
    }

}

FCLSpace::FCLKinBodyInfoPtr FCLSpace::InitKinBody(KinBodyConstPtr pbody, FCLKinBodyInfoPtr pinfo, bool bSetToCurrentPInfo)
{
    if( !pinfo ) {
        pinfo.reset(new FCLKinBodyInfo());
        pinfo->_geometrygroup = _geometrygroup;
    }

    RAVELOG_VERBOSE_FORMAT("env=%s, self=%d, init body %s (%d)", _penv->GetNameId()%_bIsSelfCollisionChecker%pbody->GetName()%pbody->GetEnvironmentBodyIndex());
    pinfo->Reset();
    pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
    // make sure that synchronization do occur !
    pinfo->nLastStamp = pbody->GetUpdateStamp() - 1;
    pinfo->nLastLinkReloadStamp = pbody->GetUpdateStamp() - 1;

    ReloadKinBodyLinks(pbody, pinfo);

    pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&FCLSpace::_ResetCurrentGeometryCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
    pinfo->_geometrygroupcallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometryGroup, boost::bind(&FCLSpace::_ResetGeometryGroupsCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
    pinfo->_linkenablecallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&FCLSpace::_ResetLinkEnableCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
    pinfo->_activeDOFsCallback = pbody->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs, boost::bind(&FCLSpace::_ResetActiveDOFsCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<FCLKinBodyInfo>(pinfo)));

    pinfo->_bodyAttachedCallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyAttached, boost::bind(&FCLSpace::_ResetAttachedBodyCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::weak_ptr<FCLKinBodyInfo>(pinfo)));
    pinfo->_bodyremovedcallback = pbody->RegisterChangeCallback(KinBody::Prop_BodyRemoved, boost::bind(&FCLSpace::RemoveUserData, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::bind(&OpenRAVE::utils::sptr_from<const KinBody>, boost::weak_ptr<const KinBody>(pbody))));

    const int envId = pbody->GetEnvironmentBodyIndex();
    const int maxEnvId = _penv->GetMaxEnvironmentBodyIndex();
    BOOST_ASSERT(envId != 0);
    if( bSetToCurrentPInfo ) {
        EnsureVectorSize(_currentpinfo, maxEnvId + 1);
        _currentpinfo.at(envId) = pinfo;
    }
    //_cachedpinfo[pbody->GetEnvironmentBodyIndex()] what to do with the cache?
    EnsureVectorSize(_vecInitializedBodies, maxEnvId + 1);
    _vecInitializedBodies.at(envId) = pbody;

    //Do I really need to synchronize anything at that point ?
    _Synchronize(*pinfo, *pbody);

    return pinfo;
}

bool FCLSpace::HasNamedGeometry(const KinBody &body, const std::string& groupname) {
    // The empty string corresponds to current geometries so all kinbodies have it
    if( groupname.size() == 0 ) {
        return true;
    }
    FOREACH(itlink, body.GetLinks()) {
        if( (*itlink)->GetGroupNumGeometries(groupname) >= 0 ) {
            return true;
        }
    }
    return false;
}

void FCLSpace::SetGeometryGroup(const std::string& groupname)
{
    // should always do this since bodies can have different geometry groups set
    _geometrygroup = groupname;
    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        SetBodyGeometryGroup(pbody, groupname);
    }
}

const std::string& FCLSpace::GetGeometryGroup() const
{
    return _geometrygroup;
}

bool FCLSpace::SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
    const KinBody& body = *pbody;

    if (!HasNamedGeometry(body, groupname)) {
        return false;
    }

    // Save the already existing FCLKinBodyInfoPtr for the old geometry group
    FCLKinBodyInfoPtr& poldinfo = GetInfo(body);
    if( poldinfo->_geometrygroup == groupname ) {
        return true;
    }

    poldinfo->nGeometryUpdateStamp += 1;

    const int maxBodyIndex = _penv->GetMaxEnvironmentBodyIndex();
    EnsureVectorSize(_cachedpinfo, maxBodyIndex + 1);

    const int bodyIndex = body.GetEnvironmentBodyIndex();
    OPENRAVE_ASSERT_OP_FORMAT(bodyIndex, !=, 0, "env=%s, body %s", _penv->GetNameId()%body.GetName(), OpenRAVE::ORE_InvalidState);
    std::map< std::string, FCLKinBodyInfoPtr >& cache = _cachedpinfo.at(bodyIndex);
    cache[poldinfo->_geometrygroup] = poldinfo;

    FCLKinBodyInfoPtr& pinfo = cache[groupname];
    if(!pinfo) {
        RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%body.GetName()%bodyIndex%_penv->GetId());
        pinfo.reset(new FCLKinBodyInfo);
        pinfo->_geometrygroup = groupname;
        InitKinBody(pbody, pinfo);
    }
    else {
        RAVELOG_VERBOSE_FORMAT("env=%s, switching to geometry %s for kinbody %s (id = %d)", _penv->GetNameId()%groupname%body.GetName()%bodyIndex);
        // Set the current info to use the FCLKinBodyInfoPtr associated to groupname
        EnsureVectorSize(_currentpinfo, maxBodyIndex + 1);
        _currentpinfo.at(bodyIndex) = pinfo;

        // Revoke the information inside the cache so that a potentially outdated object does not survive
        cache.erase(groupname);
    }

    return true;
}

const std::string& FCLSpace::GetBodyGeometryGroup(const KinBody &body) const {
    static const std::string empty;
    const FCLKinBodyInfoPtr& pinfo = GetInfo(body);
    if( !!pinfo ) {
        return pinfo->_geometrygroup;
    } else {
        return empty;
    }
}

// Set the current bvhRepresentation and reinitializes all the KinbodyInfo if needed
void FCLSpace::SetBVHRepresentation(std::string const &type)
{
    if( type == _bvhRepresentation ) {
        return;
    }

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
        RAVELOG_WARN(str(boost::format("Unknown BVH representation '%s', keeping '%s' representation") % type % _bvhRepresentation));
        return;
    }

    // reinitialize all the FCLKinBodyInfo

    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
        pinfo->nGeometryUpdateStamp++;
        InitKinBody(pbody, pinfo);
    }
    _cachedpinfo.clear();
}

std::string const& FCLSpace::GetBVHRepresentation() const {
    return _bvhRepresentation;
}

void FCLSpace::Synchronize()
{
    // We synchronize only the initialized bodies, which differs from oderave
    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        Synchronize(*pbody);
    }
}

void FCLSpace::Synchronize(const KinBody &body)
{
    FCLKinBodyInfoPtr& pinfo = GetInfo(body);
    if( !pinfo ) {
        return;
    }
    // expensive, comment out for now
    //BOOST_ASSERT( pinfo->_pbody.lock().get() == &body);
    _Synchronize(*pinfo, body);
}

void FCLSpace::SynchronizeWithAttached(const KinBody &body)
{
    if( body.HasAttached() ) {
        std::vector<int>& vecAttachedEnvBodyIndices = _vecAttachedEnvBodyIndicesCache;
        body.GetAttachedEnvironmentBodyIndices(vecAttachedEnvBodyIndices);
        std::vector<KinBodyPtr>& attachedBodies = _vecAttachedBodiesCache;
        _penv->GetBodiesFromEnvironmentBodyIndices(vecAttachedEnvBodyIndices, attachedBodies);
        for (const KinBodyPtr& pattachedBody : attachedBodies) {
            if (!!pattachedBody) {
                Synchronize(*pattachedBody);
            }
        }
    }
    else {
        Synchronize(body);
    }
}

void FCLSpace::SynchronizeExcluded(const KinBodyConstPtr& pbodyexcluded)
{
    // We synchronize only the initialized bodies, which differs from oderave
    for (const KinBodyConstPtr& pbody : _vecInitializedBodies) {
        if (!pbody) {
            continue;
        }
        if (pbody == pbodyexcluded) {
            continue;
        }
        Synchronize(*pbody);
    }
}

void FCLSpace::SynchronizeLink(const KinBody::Link &link)
{
    const KinBody &body = *link.GetParent();
    FCLKinBodyInfoPtr& pinfo = GetInfo(body);
    if( !pinfo ) {
        return;
    }
    _SynchronizeLink(*pinfo, body, link.GetIndex());
}

FCLSpace::FCLKinBodyInfoPtr& FCLSpace::GetInfo(const KinBody &body)
{
    int envId = body.GetEnvironmentBodyIndex();
    if ( envId <= 0 ) {
        RAVELOG_WARN_FORMAT("env=%s, body %s has invalid environment id %d", _penv->GetNameId()%body.GetName()%envId);
    }
    else {
        if (envId < (int) _currentpinfo.size()) {
            return _currentpinfo.at(envId);
        }
    }

    return _currentpinfo.at(0); // invalid
}

const FCLSpace::FCLKinBodyInfoPtr& FCLSpace::GetInfo(const KinBody &body) const
{
    int envId = body.GetEnvironmentBodyIndex();
    if ( envId <= 0 ) {
        RAVELOG_WARN_FORMAT("env=%s, body %s has invalid environment id %d", _penv->GetNameId()%body.GetName()%envId);
    }
    else {
        if (envId < (int) _currentpinfo.size()) {
            return _currentpinfo.at(envId);
        }
    }

    return _currentpinfo.at(0);
}

void FCLSpace::RemoveUserData(KinBodyConstPtr pbody) {
    if( !!pbody ) {
        RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d (userdatakey %s) : %s") % _penv->GetId() % _userdatakey % pbody->GetName()));
        const int envId = pbody->GetEnvironmentBodyIndex();
        if (envId < (int) _vecInitializedBodies.size()) {
            _vecInitializedBodies.at(envId).reset();
        }
        FCLKinBodyInfoPtr& pinfo = GetInfo(*pbody);
        if( !!pinfo ) {
            pinfo->Reset();
        }

        if( envId == 0 ) {
            RAVELOG_WARN_FORMAT("env=%s, body '%s' has bodyIndex=0, so not adding to the environment!", _penv->GetNameId()%pbody->GetName());
        }

        if (envId < (int) _currentpinfo.size()) {
            _currentpinfo.at(envId).reset();
            //RAVELOG_INFO_FORMAT("erased %d but didn't pop back, size is %d", envId%_currentpinfo.size());
        }
        if (envId < (int) _cachedpinfo.size()) {
            _cachedpinfo.at(envId).clear();
        }
    }
}

/// \brief helper function to initialize fcl::Container
void _AppendFclBoxCollsionObject(const OpenRAVE::Vector& fullExtents, const OpenRAVE::Vector& pos, std::vector<std::shared_ptr<fcl::CollisionObject>>& contents)
{
    std::shared_ptr<fcl::CollisionGeometry> fclGeom = std::make_shared<fcl::Box>(fullExtents.x, fullExtents.y, fullExtents.z);
    contents.emplace_back(std::make_shared<fcl::CollisionObject>(fclGeom, fcl::Transform3f(fcl::Vec3f(pos.x, pos.y, pos.z))));
}

/// \brief helper function to initialize fcl::Container
void _AppendFclBoxCollsionObject(const OpenRAVE::Vector& fullExtents, const OpenRAVE::Transform& trans, std::vector<std::shared_ptr<fcl::CollisionObject>>& contents)
{
    std::shared_ptr<fcl::CollisionGeometry> fclGeom = std::make_shared<fcl::Box>(fullExtents.x, fullExtents.y, fullExtents.z);
    const fcl::Transform3f fclTrans(ConvertQuaternionToFCL(trans.rot), ConvertVectorToFCL(trans.trans));
    contents.emplace_back(std::make_shared<fcl::CollisionObject>(fclGeom, fclTrans));
}

/// \brief helper function to initialize fcl::Container
void _AppendFclHalfspaceCollsionObject(const OpenRAVE::Transform& trans, std::vector<std::shared_ptr<fcl::CollisionObject> >& contents)
{
    std::shared_ptr<fcl::CollisionGeometry> fclGeom = std::make_shared<fcl::Halfspace>(fcl::Vec3f(0, 0, 1), 0);
    const fcl::Transform3f fclTrans(ConvertQuaternionToFCL(trans.rot), ConvertVectorToFCL(trans.trans));
    contents.emplace_back(std::make_shared<fcl::CollisionObject>(fclGeom, fclTrans));
}

CollisionGeometryPtr FCLSpace::_CreateFCLGeomFromGeometryInfo(const KinBody::GeometryInfo &info)
{
    switch(info._type) {

    case OpenRAVE::GT_None:
        return CollisionGeometryPtr();

    case OpenRAVE::GT_CalibrationBoard:
    case OpenRAVE::GT_Box:
        return std::make_shared<fcl::Box>(info._vGeomData.x*2.0f,info._vGeomData.y*2.0f,info._vGeomData.z*2.0f);

    case OpenRAVE::GT_Sphere:
        return std::make_shared<fcl::Sphere>(info._vGeomData.x);

    case OpenRAVE::GT_Cylinder:
        return std::make_shared<fcl::Cylinder>(info._vGeomData.x, info._vGeomData.y);

    case OpenRAVE::GT_Capsule:
        return std::make_shared<fcl::Capsule>(info._vGeomData.x, info._vGeomData.y);

    case OpenRAVE::GT_Container:
    {
        const Vector& outerextents = info._vGeomData;
        const Vector& innerextents = info._vGeomData2;
        const Vector& bottomcross = info._vGeomData3;
        const Vector& bottom = info._vGeomData4;
        OpenRAVE::dReal zoffset = 0;
        if( bottom[2] > 0 ) {
            if( bottom[0] > 0 && bottom[1] > 0 ) {
                zoffset = bottom[2];
            }
        }
        std::vector<std::shared_ptr<fcl::CollisionObject>> contents;
        // +x wall
        _AppendFclBoxCollsionObject(Vector((outerextents[0] - innerextents[0]) / 2.0, outerextents[1], outerextents[2]), Vector(+(outerextents[0] + innerextents[0]) / 4.0, 0.0, outerextents[2] / 2.0 + zoffset), contents);
        // -x wall
        _AppendFclBoxCollsionObject(Vector((outerextents[0] - innerextents[0]) / 2.0, outerextents[1], outerextents[2]), Vector(-(outerextents[0] + innerextents[0]) / 4.0, 0.0, outerextents[2] / 2.0 + zoffset), contents);
        // +y wall
        _AppendFclBoxCollsionObject(Vector(outerextents[0], (outerextents[1] - innerextents[1]) / 2.0, outerextents[2]), Vector(0.0, +(outerextents[1] + innerextents[1]) / 4.0, outerextents[2] / 2.0 + zoffset), contents);
        // -y wall
        _AppendFclBoxCollsionObject(Vector(outerextents[0], (outerextents[1] - innerextents[1]) / 2.0, outerextents[2]), Vector(0.0, -(outerextents[1] + innerextents[1]) / 4.0, outerextents[2] / 2.0 + zoffset), contents);
        // bottom
        if( outerextents[2] - innerextents[2] >= 1e-6 ) { // small epsilon error can make thin triangles appear, so test with a reasonable threshold
            _AppendFclBoxCollsionObject(Vector(outerextents[0], outerextents[1], outerextents[2]-innerextents[2]), Vector(0.0, 0.0, (outerextents[2] - innerextents[2]) / 2.0 + zoffset), contents);
        }
        // cross
        if( bottomcross[2] > 0 ) {
            if( bottomcross[0] > 0 ) {
                _AppendFclBoxCollsionObject(Vector(bottomcross[0], innerextents[1], bottomcross[2]), Vector(0.0, 0.0, bottomcross[2] / 2.0 + outerextents[2] - innerextents[2] + zoffset), contents);
            }
            if( bottomcross[1] > 0 ) {
                _AppendFclBoxCollsionObject(Vector(innerextents[0], bottomcross[1], bottomcross[2]), Vector(0.0, 0.0, bottomcross[2] / 2.0 + outerextents[2] - innerextents[2] + zoffset), contents);
            }
        }
        // bottom
        if( bottom[2] > 0 ) {
            if( bottom[0] > 0 && bottom[1] > 0 ) {
                _AppendFclBoxCollsionObject(bottom, Vector(0.0, 0.0, bottom[2] / 2.0), contents);
            }
        }
        return std::make_shared<fcl::Container>(contents);
    }
    case OpenRAVE::GT_Cage:
    {
        const Vector& vCageBaseExtents = info._vGeomData;
        std::vector<std::shared_ptr<fcl::CollisionObject>> contents;
        for( const KinBody::GeometryInfo::SideWall& sideWall : info._vSideWalls ) {
            Transform sideWallBoxPose;
            sideWallBoxPose.rot = sideWall.transf.rot;
            sideWallBoxPose.trans = sideWall.transf * Vector(0, 0, sideWall.vExtents.z);
            _AppendFclBoxCollsionObject(2.0*sideWall.vExtents, sideWallBoxPose, contents);
        }
        // finally add the base
        _AppendFclBoxCollsionObject(2.0*vCageBaseExtents, Vector(0, 0, vCageBaseExtents.z), contents);
        return std::make_shared<fcl::Container>(contents);
    }
    case OpenRAVE::GT_Prism:
    {
        std::vector<std::shared_ptr<fcl::CollisionObject> > contents;
        const OpenRAVE::TriMesh& mesh = info._meshcollision;
        const size_t nPoints = mesh.vertices.size();
        for( size_t ipoint = 0; ipoint < nPoints; ipoint += 2 ) {
            const OpenRAVE::Vector p0(mesh.vertices[ipoint].x, mesh.vertices[ipoint].y, 0);
            const OpenRAVE::Vector p1(mesh.vertices[(ipoint + 2) % nPoints].x, mesh.vertices[(ipoint + 2) % nPoints].y, 0);
            if( (p1 - p0).lengthsqr2() < g_fEpsilon ) {
                continue; // ipoint
            }
            OpenRAVE::Transform trans(OpenRAVE::geometry::quatRotateDirection(OpenRAVE::Vector(1, 0, 0), (p1 - p0).normalize()), (p0 + p1) * 0.5); // Y pointing to the left side of the directed segment (p0, p1)
            trans *= OpenRAVE::Transform(OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(1, 0, 0), -M_PI * 0.5), OpenRAVE::Vector()); // Z pointing to the left side of the directed segment (p0, p1)
            _AppendFclHalfspaceCollsionObject(trans, contents);
        }
        _AppendFclHalfspaceCollsionObject(OpenRAVE::Transform(OpenRAVE::Vector(1, 0, 0, 0), OpenRAVE::Vector(0, 0, -info._vGeomData.y * 0.5)), contents);
        _AppendFclHalfspaceCollsionObject(OpenRAVE::Transform(OpenRAVE::Vector(0, 1, 0, 0), OpenRAVE::Vector(0, 0, info._vGeomData.y * 0.5)), contents);
        return std::make_shared<fcl::Container>(contents);
    }
    case OpenRAVE::GT_ConicalFrustum:
    case OpenRAVE::GT_Axial:
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

        return _meshFactory(fcl_points, fcl_triangles);
    }

    default:
        RAVELOG_WARN(str(boost::format("FCL doesn't support geom type %d")%info._type));
        return CollisionGeometryPtr();
    }
}

void FCLSpace::_SynchronizeLink(FCLKinBodyInfo& info, const KinBody& body, int linkIndex)
{
    FCLSpace::FCLKinBodyInfo::LinkInfo& linkInfo = *info.vlinks[linkIndex];
    if( linkInfo.nLastStamp != body.GetUpdateStamp() ) {
        linkInfo.nLastStamp = body.GetUpdateStamp();
        CollisionObjectPtr& pcoll = linkInfo.linkBV.second; // avoid copying shared pointer for performance
        if( !pcoll ) {
            return;
        }
        const Transform& linkTransform = body.GetLinks()[linkIndex]->GetTransform();
        Transform pose = linkTransform;
        pose.trans += pose.rotate(linkInfo.linkBV.first);
        const fcl::Vec3f newPosition = ConvertVectorToFCL(pose.trans);
        const fcl::Quaternion3f newOrientation = ConvertQuaternionToFCL(pose.rot);

        pcoll->setTranslation(newPosition);
        pcoll->setQuatRotation(newOrientation);
        // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
        pcoll->computeAABB();

        for (const TransformCollisionPair& pgeom : linkInfo.vgeoms) {
            fcl::CollisionObject& coll = *pgeom.second;
            const Transform pose1 = linkTransform * pgeom.first;
            const fcl::Vec3f newPosition1 = ConvertVectorToFCL(pose1.trans);
            const fcl::Quaternion3f newOrientation1 = ConvertQuaternionToFCL(pose1.rot);

            coll.setTranslation(newPosition1);
            coll.setQuatRotation(newOrientation1);
            // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
            coll.computeAABB();
        }
    }
}

void FCLSpace::_Synchronize(FCLKinBodyInfo& info, const KinBody& body)
{
    //KinBodyPtr pbody = info.GetBody();
    if( info.nLastStamp != body.GetUpdateStamp()) {
        info.nLastStamp = body.GetUpdateStamp();
        if( body.GetLinks().size() != info.vlinks.size() ) {
            throw OpenRAVE::OpenRAVEException(str(boost::format("env=%s, the current number of links in body '%s' are %d, and are not the same as the number cached links %d")%_penv->GetNameId()%body.GetName()%body.GetLinks().size()%info.vlinks.size()), OpenRAVE::ORE_InvalidState);
        }

        for(size_t i = 0; i < body.GetLinks().size(); ++i) {
            _SynchronizeLink(info, body, i);
        }

        // Does this have any use ?
        // if( !!_synccallback ) {
        //     _synccallback(pinfo);
        // }
    }
}

/// Scope guard to ensure that user data for a kinbody is cleared on scope exit
/// May be reset to 'disarm' the guard if the data should be kept on scope exit after all.
struct ScopedUserDataRemover
{
    ScopedUserDataRemover(FCLSpace& space, const KinBodyPtr& ptr)
        : _space(space), _ptr(ptr)
    {
    }
    ~ScopedUserDataRemover()
    {
        if (!!_ptr) {
            _space.RemoveUserData(_ptr);
        }
    }
    void Reset() {
        _ptr.reset();
    }

private:
    FCLSpace& _space;
    KinBodyPtr _ptr;
};

void FCLSpace::_ResetCurrentGeometryCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo)
{
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    KinBodyPtr pbody = pinfo->GetBody();
    const int bodyIndex = pbody->GetEnvironmentBodyIndex();
    if (0 < bodyIndex && bodyIndex < (int)_currentpinfo.size()) {
        const FCLKinBodyInfoPtr& pcurrentinfo = _currentpinfo.at(bodyIndex);

        if (!!pinfo && pinfo == pcurrentinfo) { //pinfo->_geometrygroup.size() == 0 ) {
            // pinfo is current set to the current one, so should InitKinBody into _currentpinfo
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting current geometry for kinbody %s nGeometryUpdateStamp=%d, (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
            pinfo->nGeometryUpdateStamp++;

            // In order to ensure that the body is removed from the FCL space if something goes wrong during the reload process,
            // create a scoped remover that will clear our user data for this body on scope exit. If the reload succeeds, we
            // reset the scoped remover to prevent it clearing the data.
            ScopedUserDataRemover userDataGuard{*this, pbody};
            ReloadKinBodyLinks(pbody, pinfo);
            userDataGuard.Reset();
        }
        //_cachedpinfo[pbody->GetEnvironmentBodyIndex()].erase(std::string());
    }
}

void FCLSpace::_ResetGeometryGroupsCallback(boost::weak_ptr<FCLKinBodyInfo> _pinfo)
{
    FCLKinBodyInfoPtr pinfo = _pinfo.lock();
    KinBodyPtr pbody = pinfo->GetBody();

    //FCLKinBodyInfoPtr pcurrentinfo = _currentpinfo.at(pbody->GetEnvironmentBodyIndex());

    if( !!pinfo ) {// && pinfo->_geometrygroup.size() > 0 ) {
        //RAVELOG_VERBOSE_FORMAT("env=%d, resetting geometry groups for kinbody %s, nGeometryUpdateStamp=%d (key %s, self=%d)", _penv->GetId()%pbody->GetName()%pinfo->nGeometryUpdateStamp%_userdatakey%_bIsSelfCollisionChecker);
        pinfo->nGeometryUpdateStamp++;

        // In order to ensure that the body is removed from the FCL space if something goes wrong during the reload process,
        // create a scoped remover that will clear our user data for this body on scope exit. If the reload succeeds, we
        // reset the scoped remover to prevent it clearing the data.
        ScopedUserDataRemover userDataGuard{*this, pbody};
        ReloadKinBodyLinks(pbody, pinfo);
        userDataGuard.Reset();
    }

//   FCLKinBodyInfoPtr pinfoCurrentGeometry = _cachedpinfo[pbody->GetEnvironmentBodyIndex()][std::string()];
//   _cachedpinfo.erase(pbody->GetEnvironmentBodyIndex());
//   if( !!pinfoCurrentGeometry ) {
//       _cachedpinfo[pbody->GetEnvironmentBodyIndex()][std::string()] = pinfoCurrentGeometry;
//   }
}

} // namespace fclrave
