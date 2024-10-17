// -*- coding: utf-8 -*-
// Copyright (C) 2006-2019 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "libopenrave.h"
namespace OpenRAVE {

/// \brief Get independent links from robot.
/// \param[out] vlinks : vector of independent links.
/// \param[in] proot, varmdofindices, vgripperdofindices : used to compute the independent links.
template<typename LinkPtrT>
void _GetIndependentLinks(std::vector<LinkPtrT>& vlinks, const RobotBasePtr& probot, const std::vector<int>& varmdofindices, const std::vector<int>& vgripperdofindices)
{
    vlinks.clear();
    FOREACHC(itlink, probot->GetLinks()) {
        bool bAffected = false;
        FOREACHC(itindex,varmdofindices) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        FOREACHC(itindex,vgripperdofindices) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }

        if( !bAffected ) {
            vlinks.push_back(*itlink);
        }
    }
}

/// \brief update grabbedBody1 transform and return saver.
static KinBody::KinBodyStateSaverPtr _UpdateGrabbedBodyTransformWithSaver(KinBodyPtr& pGrabbedBody, const Transform& tRelative, const Transform& tLinkTrans)
{
    KinBody::KinBodyStateSaverPtr saver(new KinBody::KinBodyStateSaver(pGrabbedBody, KinBody::Save_LinkTransformation));
    pGrabbedBody->SetTransform(tLinkTrans * tRelative);
    return saver;
}

void RobotBase::ManipulatorInfo::Reset()
{
    _id.clear();
    _name.clear();
    _sBaseLinkName.clear();
    _sIkChainEndLinkName.clear();
    _sEffectorLinkName.clear();
    _tLocalTool = Transform();
    _vChuckingDirection.clear();
    _vdirection = Vector(0,0,1);
    _sIkSolverXMLId.clear();
    _vGripperJointNames.clear();
    _grippername.clear();
    _toolChangerConnectedBodyToolName.clear();
    _toolChangerLinkName.clear();
    _vRestrictGraspSetNames.clear();
}

void RobotBase::ManipulatorInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    orjson::SetJsonValueByKey(value, "id", _id, allocator);
    orjson::SetJsonValueByKey(value, "name", _name, allocator);
    orjson::SetJsonValueByKey(value, "transform", _tLocalTool, allocator);
    if( _vChuckingDirection.size() > 0 ) {
        orjson::SetJsonValueByKey(value, "chuckingDirections", _vChuckingDirection, allocator);
    }
    orjson::SetJsonValueByKey(value, "direction", _vdirection, allocator);
    orjson::SetJsonValueByKey(value, "baseLinkName", _sBaseLinkName, allocator);
    orjson::SetJsonValueByKey(value, "ikChainEndLinkName", _sIkChainEndLinkName, allocator); //optional;
    orjson::SetJsonValueByKey(value, "effectorLinkName", _sEffectorLinkName, allocator);
    orjson::SetJsonValueByKey(value, "ikSolverType", _sIkSolverXMLId, allocator);
    if( _vGripperJointNames.size() > 0 ) {
        orjson::SetJsonValueByKey(value, "gripperJointNames", _vGripperJointNames, allocator);
    }
    orjson::SetJsonValueByKey(value, "grippername", _grippername, allocator);
    orjson::SetJsonValueByKey(value, "toolChangerConnectedBodyToolName", _toolChangerConnectedBodyToolName, allocator);
    orjson::SetJsonValueByKey(value, "toolChangerLinkName", _toolChangerLinkName, allocator);
    orjson::SetJsonValueByKey(value, "restrictGraspSetNames", _vRestrictGraspSetNames, allocator);
}

void RobotBase::ManipulatorInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "id", _id);
    orjson::LoadJsonValueByKey(value, "name", _name);
    orjson::LoadJsonValueByKey(value, "transform", _tLocalTool);

    // Now recommended to use chuckingDirection in GripperInfo. However, we still read it from ManipulatorInfo for backward compatibility, e.g. the saved scene is not migrated.
    rapidjson::Value::ConstMemberIterator itChuckingDirections = value.FindMember("chuckingDirections");
    if( itChuckingDirections != value.MemberEnd() ) {
        const rapidjson::Value& rChuckingDirections = itChuckingDirections->value;
        if( !rChuckingDirections.IsArray() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("When loading tool '%s' with id '%s', 'chuckingDirections' needs to be an array, currently it is '%s'"), _name%_id%orjson::DumpJson(rChuckingDirections), ORE_InvalidArguments);
        }

        _vChuckingDirection.resize(rChuckingDirections.Size());
        if( rChuckingDirections.Size() > 0 ) {
            RAVELOG_WARN_FORMAT("For manipulator '%s', read 'chuckingDirections' (size=%d) in tools' setting. But, since it's deprecated, using 'gripperInfos' is recommended.", _name%_vChuckingDirection.size());
        }
        for(int index = 0; index < (int)_vChuckingDirection.size(); ++index) {
            int direction = 0;
            if( rChuckingDirections[index].IsFloat() || rChuckingDirections[index].IsDouble() ) {
                double fdirection = 0;
                orjson::LoadJsonValue(rChuckingDirections[index], fdirection);
                if( fdirection > 0 ) {
                    direction = 1;
                }
                else if( fdirection < 0 ) {
                    direction = -1;
                }
            }
            else {
                orjson::LoadJsonValue(rChuckingDirections[index], direction);
                if (direction > 0) {
                    direction = 1;
                }
                else if (direction < 0) {
                    direction = -1;
                }
            }
            _vChuckingDirection[index] = direction;
        }
    }

    orjson::LoadJsonValueByKey(value, "direction", _vdirection);
    orjson::LoadJsonValueByKey(value, "baseLinkName", _sBaseLinkName);
    orjson::LoadJsonValueByKey(value, "ikChainEndLinkName", _sIkChainEndLinkName); //optional;
    orjson::LoadJsonValueByKey(value, "effectorLinkName", _sEffectorLinkName);
    orjson::LoadJsonValueByKey(value, "ikSolverType", _sIkSolverXMLId);
    orjson::LoadJsonValueByKey(value, "gripperJointNames", _vGripperJointNames);
    // Now recommended to use gripperJointNames in GripperInfo. However, we still read it from ManipulatorInfo for backward compatibility, e.g. the saved scene is not migrated.
    rapidjson::Value::ConstMemberIterator itGripperJointNames = value.FindMember("gripperJointNames");
    if( (itGripperJointNames != value.MemberEnd()) && itGripperJointNames->value.IsArray() && (itGripperJointNames->value.Size() > 0) ) {
        RAVELOG_WARN_FORMAT("For manipulator '%s', read 'gripperJointNames' (size=%d) in 'tools' setting. But, since it's deprecated, using 'gripperInfos is recommended.", _name%_vGripperJointNames.size());
    }
    orjson::LoadJsonValueByKey(value, "grippername", _grippername);
    orjson::LoadJsonValueByKey(value, "toolChangerConnectedBodyToolName", _toolChangerConnectedBodyToolName);
    orjson::LoadJsonValueByKey(value, "toolChangerLinkName", _toolChangerLinkName);
    orjson::LoadJsonValueByKey(value, "restrictGraspSetNames", _vRestrictGraspSetNames);
}

bool RobotBase::ManipulatorInfo::operator==(const ManipulatorInfo& other) const
{
    return _name == other._name
        && _sBaseLinkName == other._sBaseLinkName
        && _sIkChainEndLinkName == other._sIkChainEndLinkName
        && _sEffectorLinkName == other._sEffectorLinkName
        && TransformDistanceFast(_tLocalTool, other._tLocalTool) <= g_fEpsilonLinear
        && _vChuckingDirection == other._vChuckingDirection
        && _vdirection == other._vdirection
        && _sIkSolverXMLId == other._sIkSolverXMLId
        && _vGripperJointNames == other._vGripperJointNames
        && _grippername == other._grippername
        && _toolChangerConnectedBodyToolName == other._toolChangerConnectedBodyToolName
        && _toolChangerLinkName == other._toolChangerLinkName
        && _vRestrictGraspSetNames == other._vRestrictGraspSetNames
        && _id == other._id;
}

RobotBase::Manipulator::Manipulator(RobotBasePtr probot, const RobotBase::ManipulatorInfo& info) : _info(info), __probot(probot) {
}
RobotBase::Manipulator::~Manipulator() {
}

RobotBase::Manipulator::Manipulator(const RobotBase::Manipulator& r)
{
    *this = r;
    __pIkSolver.reset();
    if( _info._sIkSolverXMLId.size() > 0 ) {
        __pIkSolver = RaveCreateIkSolver(GetRobot()->GetEnv(), _info._sIkSolverXMLId);
    }
}

RobotBase::Manipulator::Manipulator(RobotBasePtr probot, boost::shared_ptr<RobotBase::Manipulator const> r)
{
    *this = *r.get();
    __probot = probot;
    if( !!r->GetBase() ) {
        __pBase = probot->GetLinks().at(r->GetBase()->GetIndex());
    }
    if( !!r->GetIkChainEndLink() ) {
        __pIkChainEndLink = probot->GetLinks().at(r->GetIkChainEndLink()->GetIndex());
    }
    if( !!r->GetEndEffector() ) {
        __pEffector = probot->GetLinks().at(r->GetEndEffector()->GetIndex());
    }
    __pIkSolver.reset(); // will be initialized when needed. Problem: the freeinc parameters will not get transferred to the new iksolver
//    if( _info._sIkSolverXMLId.size() > 0 ) {
//        //__pIkSolver = RaveCreateIkSolver(probot->GetEnv(), _info._sIkSolverXMLId);
//        // cannot call __pIkSolver->Init since this is the constructor...
//    }
}

void RobotBase::Manipulator::UpdateInfo()
{
    // TODO: update _info
    RAVELOG_WARN("Manipulator UpdateInfo is not implemented");
}

void RobotBase::Manipulator::ExtractInfo(RobotBase::ManipulatorInfo& info) const
{
    // TODO
    info = _info;
}

UpdateFromInfoResult RobotBase::Manipulator::UpdateFromInfo(const RobotBase::ManipulatorInfo& info)
{
    BOOST_ASSERT(info._id == _info._id);

    if (_info._sBaseLinkName != info._sBaseLinkName) {
        RAVELOG_VERBOSE_FORMAT("manipulator %s base link name changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if (_info._sIkChainEndLinkName != info._sIkChainEndLinkName) {
        RAVELOG_VERBOSE_FORMAT("manipulator %s end link name changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if (_info._sEffectorLinkName != info._sEffectorLinkName) {
        RAVELOG_VERBOSE_FORMAT("manipulator %s effector link name changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if (_info._grippername != info._grippername) {
        RAVELOG_VERBOSE_FORMAT("manipulator %s gripper name changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if (_info._vGripperJointNames != info._vGripperJointNames) {
        RAVELOG_VERBOSE_FORMAT("manipulator %s gripper joint names changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if (_info._sIkSolverXMLId != info._sIkSolverXMLId) {
        RAVELOG_VERBOSE_FORMAT("manipulator %s ik solver xml id changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    UpdateFromInfoResult updateFromInfoResult = UFIR_NoChange;
    if (GetName() != info._name) {
        SetName(info._name);
        RAVELOG_VERBOSE_FORMAT("manipulator %s name changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    if (_info._vChuckingDirection != info._vChuckingDirection) {
        _info._vChuckingDirection = info._vChuckingDirection;
        GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorTool);
        RAVELOG_VERBOSE_FORMAT("manipulator %s chucking direction changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    if (GetLocalToolTransform().CompareTransform(info._tLocalTool, g_fEpsilon)) {
        SetLocalToolTransform(info._tLocalTool);
        RAVELOG_VERBOSE_FORMAT("manipulator %s local tool transform changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    if (GetLocalToolDirection() != info._vdirection) {
        SetLocalToolDirection(info._vdirection);
        RAVELOG_VERBOSE_FORMAT("manipulator %s direction changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    return updateFromInfoResult;
}

int RobotBase::Manipulator::GetArmDOF() const
{
    return static_cast<int>(__varmdofindices.size());
}

int RobotBase::Manipulator::GetGripperDOF() const
{
    return static_cast<int>(__vgripperdofindices.size());
}

void RobotBase::Manipulator::SetLocalToolTransform(const Transform& t)
{
    // only reset the iksolver if transform is different. note that the correct way is to see if the hash changes, but that could take too much computation. also hashes truncate the numbers.
    if( !!__pIkSolver && TransformDistance2(_info._tLocalTool, t) > g_fEpsilonLinear ) {
        __pIkSolver.reset();
        _info._sIkSolverXMLId.resize(0);
    }
    _info._tLocalTool = t;
    __hashkinematicsstructure.resize(0);
    __hashstructure.resize(0);
    __maphashikstructure.clear();
    GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorTool);
}

void RobotBase::Manipulator::SetLocalToolDirection(const Vector& direction)
{
    // only reset the iksolver if direction is different. note that the correct way is to see if the hash changes, but that could take too much computation. also hashes truncate the numbers.
    if( (_info._vdirection-direction).lengthsqr3() > g_fEpsilonLinear ) {
        // only reset if direction is actually used
        if( !!__pIkSolver && (__pIkSolver->Supports(IKP_TranslationDirection5D) || __pIkSolver->Supports(IKP_Direction3D)) ) {
            __pIkSolver.reset();
            _info._sIkSolverXMLId.resize(0);
        }
    }
    _info._vdirection = direction*(1/RaveSqrt(direction.lengthsqr3())); // should normalize
    __hashkinematicsstructure.resize(0);
    __hashstructure.resize(0);
    __maphashikstructure.clear();
    GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorTool);
}

void RobotBase::Manipulator::SetName(const std::string& name)
{
    RobotBasePtr probot=GetRobot();
    FOREACHC(itmanip,probot->GetManipulators()) {
        if( *itmanip != shared_from_this() && name == (*itmanip)->GetName() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("manipulator name change '%s'->'%s' is colliding with other manipulator"), _info._name%name, ORE_InvalidArguments);
        }
    }
    _info._name=name;
    probot->_PostprocessChangedParameters(Prop_RobotManipulatorName);
}

Transform RobotBase::Manipulator::GetTransform() const
{
    if( !__pEffector ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, robot '%s' manipulator '%s' has no end effector with name '%s'"), __probot.lock()->GetEnv()->GetNameId()%__probot.lock()->GetName()%_info._name%_info._sEffectorLinkName, ORE_InvalidArguments);
    }
    return __pEffector->GetTransform() * _info._tLocalTool;
}

std::pair<Vector,Vector> RobotBase::Manipulator::GetVelocity() const
{
    Vector vdifference = __pEffector->GetTransform().rotate(_info._tLocalTool.trans);
    std::pair<Vector,Vector> velocity = __pEffector->GetVelocity();
    velocity.first += velocity.second.cross(vdifference);
    return velocity;
}

IkSolverBasePtr RobotBase::Manipulator::GetIkSolver() const
{
    if( !!__pIkSolver || _info._sIkSolverXMLId.size() == 0 ) {
        return __pIkSolver;
    }

    // initialize ik solver
    try {
        if( _info._sIkSolverXMLId.size() > 0 ) {
            RobotBasePtr probot(__probot);
            __pIkSolver = RaveCreateIkSolver(probot->GetEnv(), _info._sIkSolverXMLId);
            if( !!__pIkSolver ) {
                // note that ik solvers might look at the manipulator hashes for verification
                __pIkSolver->Init(shared_from_this());
            }
        }
    }
    catch(const std::exception& e) {
        RAVELOG_WARN(str(boost::format("failed to init ik solver: %s\n")%e.what()));
        __pIkSolver.reset();
    }
    return __pIkSolver;
}

bool RobotBase::Manipulator::SetIkSolver(IkSolverBasePtr iksolver)
{
    if( !iksolver ) {
        __pIkSolver.reset();
        return true;
    }

    if( iksolver->GetXMLId().size() == 0 ) {
        RAVELOG_WARN(str(boost::format("robot %s manip %s IkSolver XML is not initialized\n")%GetRobot()->GetName()%GetName()));
    }
    if( iksolver == __pIkSolver && _info._sIkSolverXMLId == iksolver->GetXMLId() ) {
        return true;
    }

    // only call the changed message if something changed
    if( iksolver->Init(shared_from_this()) ) {
        __pIkSolver = iksolver;
        _info._sIkSolverXMLId = iksolver->GetXMLId();
        GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorSolver);
        return true;
    }

    return false;
}

void RobotBase::Manipulator::GetArmDOFValues(std::vector<dReal>& v) const
{
    if( __varmdofindices.empty() ) {
        v.resize(0);
    }
    else {
        GetRobot()->GetDOFValues(v, __varmdofindices);
    }
}

void RobotBase::Manipulator::GetGripperDOFValues(std::vector<dReal>& v) const
{
    if( __vgripperdofindices.empty() ) {
        v.resize(0);
    }
    else {
        GetRobot()->GetDOFValues(v, __vgripperdofindices);
    }
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, vector<dReal>& solution, int filteroptions) const
{
    return FindIKSolution(goal, vector<dReal>(), solution, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, vector<dReal>& solution, int filteroptions) const
{
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(__probot)->GetName()%GetName(),ORE_Failed);
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    solution.resize(__varmdofindices.size());
    for(size_t i = 0; i < __varmdofindices.size(); ++i) {
        JointConstPtr pjoint = probot->GetJointFromDOFIndex(__varmdofindices[i]);
        solution[i] = pjoint->GetValue(__varmdofindices[i]-pjoint->GetDOFIndex());
    }
    IkParameterization localgoal;
    if( !!__pBase ) {
        localgoal = __pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    boost::shared_ptr< vector<dReal> > psolution(&solution, utils::null_deleter());
    return vFreeParameters.size() == 0 ? pIkSolver->Solve(localgoal, solution, filteroptions, psolution) : pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, psolution);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    return FindIKSolutions(goal, vector<dReal>(), solutions, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(__probot)->GetName()%GetName(),ORE_Failed);
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!__pBase ) {
        localgoal = __pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? pIkSolver->SolveAll(localgoal,filteroptions,solutions) : pIkSolver->SolveAll(localgoal,vFreeParameters,filteroptions,solutions);
}


bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, int filteroptions, IkReturnPtr ikreturn, IkFailureAccumulatorBasePtr paccumulator) const
{
    return FindIKSolution(goal, vector<dReal>(), filteroptions, ikreturn, paccumulator);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn, IkFailureAccumulatorBasePtr paccumulator) const
{
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(__probot)->GetName()%GetName(),ORE_Failed);
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    vector<dReal> solution(__varmdofindices.size());
    for(size_t i = 0; i < __varmdofindices.size(); ++i) {
        JointConstPtr pjoint = probot->GetJointFromDOFIndex(__varmdofindices[i]);
        solution[i] = pjoint->GetValue(__varmdofindices[i]-pjoint->GetDOFIndex());
    }
    IkParameterization localgoal;
    if( !!__pBase ) {
        localgoal = __pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? pIkSolver->Solve(localgoal, solution, filteroptions, paccumulator, ikreturn) : pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, paccumulator, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, int filteroptions, std::vector<IkReturnPtr>& vikreturns, IkFailureAccumulatorBasePtr paccumulator) const
{
    return FindIKSolutions(goal, vector<dReal>(), filteroptions, vikreturns, paccumulator);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns, IkFailureAccumulatorBasePtr paccumulator) const
{
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(__probot)->GetName()%GetName(),ORE_Failed);
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!__pBase ) {
        localgoal = __pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? pIkSolver->SolveAll(localgoal,filteroptions,paccumulator,vikreturns) : pIkSolver->SolveAll(localgoal,vFreeParameters,filteroptions,paccumulator,vikreturns);
}

IkParameterization RobotBase::Manipulator::GetIkParameterization(IkParameterizationType iktype, bool inworld) const
{
    IkParameterization ikp;
    Transform t = GetTransform();
    if( !inworld ) {
        t = GetBase()->GetTransform().inverse()*t;
    }
    switch(iktype) {
    case IKP_Transform6D: ikp.SetTransform6D(t); break;
    case IKP_Rotation3D: ikp.SetRotation3D(t.rot); break;
    case IKP_Translation3D: ikp.SetTranslation3D(t.trans); break;
    case IKP_Direction3D: ikp.SetDirection3D(t.rotate(_info._vdirection)); break;
    case IKP_Ray4D: {
        ikp.SetRay4D(RAY(t.trans,t.rotate(_info._vdirection)));
        break;
    }
    case IKP_Lookat3D: {
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: Lookat3D type setting goal a distance of 1 from the origin.\n");
        Vector vdir = t.rotate(_info._vdirection);
        ikp.SetLookat3D(RAY(t.trans + vdir,vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(_info._vdirection)));
        break;
    }
    case IKP_TranslationXY2D: {
        ikp.SetTranslationXY2D(t.trans);
        break;
    }
    case IKP_TranslationXYOrientation3D: {
        //dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,RaveAtan2(vglobaldirection.y,vglobaldirection.x)));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: TranslationLocalGlobal6D type setting local translation to (0,0,0).\n");
        ikp.SetTranslationLocalGlobal6D(Vector(0,0,0),t.trans);
        break;
    }
    case IKP_TranslationXAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        else {
            Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        else {
            Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        else {
            Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        break;
    }
    // velocities
    case IKP_Transform6DVelocity: {
        Vector vlinearvel, vangularvel;
        GetEndEffector()->GetVelocity(vlinearvel, vangularvel);

        Transform tee = GetTransform();
        //__pEffector->GetTransform() * _info._tLocalTool
        // TODO, have to convert relative to the end effector's velocity
        vlinearvel += vangularvel.cross(tee.trans - GetEndEffector()->GetTransform().trans); // t.trans = GetTransform().trans - tbase.trans

        if( !inworld ) {
            // remove the base link velocity frame
            // v_B = v_A + angularvel x (B-A)
            //Transform tbase = GetBase()->GetTransform();
            // TODO, have to convert relative to the base's velocity!
            vlinearvel -= vangularvel.cross(t.trans); // t.trans = GetTransform().trans - tbase.trans
            //Vector vbaselinear, vbaseangular;
            //GetBase()->GetVelocity(vlinearvel, vangularvel);
            //Vector voffset = t.trans - tbase.trans;
            //vLinkVelocities[2][i].first = vbaselinear + vbaseangular.cross(voffset);
            //vLinkVelocities[2][i].second = vbaseangular;
        }
        Transform tvel;
        tvel.trans = vlinearvel;
        tvel.rot = quatMultiply(t.rot,Vector(0,vangularvel.x,vangularvel.y,vangularvel.z)) * 0.5;
        ikp.SetTransform6DVelocity(tvel);
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("invalid ik type 0x%x"),iktype,ORE_InvalidArguments);
    }
    return ikp;
}

IkParameterization RobotBase::Manipulator::GetIkParameterization(const IkParameterization& ikparam, bool inworld) const
{
    IkParameterization ikp = ikparam; // copies the custom data
    Transform t = GetTransform();
    if( !inworld ) {
        t = GetBase()->GetTransform().inverse()*t;
    }
    switch(ikparam.GetType()) {
    case IKP_Transform6D: ikp.SetTransform6D(t); break;
    case IKP_Rotation3D: ikp.SetRotation3D(t.rot); break;
    case IKP_Translation3D: ikp.SetTranslation3D(t.trans); break;
    case IKP_Direction3D: ikp.SetDirection3D(t.rotate(_info._vdirection)); break;
    case IKP_Ray4D: {
        ikp.SetRay4D(RAY(t.trans,t.rotate(_info._vdirection)));
        break;
    }
    case IKP_Lookat3D: {
        // find the closest point to ikparam.GetLookat3D() to the current ray
        Vector vdir = t.rotate(_info._vdirection);
        ikp.SetLookat3D(RAY(t.trans + vdir*vdir.dot(ikparam.GetLookat3D()-t.trans),vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(_info._vdirection)));
        break;
    }
    case IKP_TranslationXY2D: {
        ikp.SetTranslationXY2D(t.trans);
        break;
    }
    case IKP_TranslationXYOrientation3D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,RaveAtan2(vglobaldirection.y,vglobaldirection.x)));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        Vector localtrans = ikparam.GetTranslationLocalGlobal6D().first;
        ikp.SetTranslationLocalGlobal6D(localtrans,t * localtrans);
        break;
    }
    case IKP_TranslationXAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        else {
            Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        else {
            Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        else {
            Vector vlocaldirection = t.rotate(_info._vdirection);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        break;
    }
    // velocities
    case IKP_Transform6DVelocity: {
        Vector vlinearvel, vangularvel;
        GetEndEffector()->GetVelocity(vlinearvel, vangularvel);

        Transform tee = GetTransform();
        //__pEffector->GetTransform() * _info._tLocalTool
        // TODO, have to convert relative to the end effector's velocity
        vlinearvel += vangularvel.cross(tee.trans - GetEndEffector()->GetTransform().trans); // t.trans = GetTransform().trans - tbase.trans

        if( !inworld ) {
            // remove the base link velocity frame
            // v_B = v_A + angularvel x (B-A)
            //Transform tbase = GetBase()->GetTransform();
            // TODO, have to convert relative to the base's velocity!
            vlinearvel -= vangularvel.cross(t.trans); // t.trans = GetTransform().trans - tbase.trans
            //Vector vbaselinear, vbaseangular;
            //GetBase()->GetVelocity(vlinearvel, vangularvel);
            //Vector voffset = t.trans - tbase.trans;
            //vLinkVelocities[2][i].first = vbaselinear + vbaseangular.cross(voffset);
            //vLinkVelocities[2][i].second = vbaseangular;
        }
        Transform tvel;
        tvel.trans = vlinearvel;
        tvel.rot = quatMultiply(t.rot,Vector(0,vangularvel.x,vangularvel.y,vangularvel.z)) * 0.5;
        ikp.SetTransform6DVelocity(tvel);
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("invalid ik type 0x%x"),ikparam.GetType(),ORE_InvalidArguments);
    }
    return ikp;
}

void RobotBase::Manipulator::GetChildJoints(std::vector<JointPtr>& vjoints) const
{
    RobotBasePtr probot(__probot);
    vjoints.resize(0);
    int iattlink = __pEffector->GetIndex();
    vector<uint8_t> vhasjoint(probot->GetJoints().size(),false);
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmjoint)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        FOREACHC(itjoint, probot->GetJoints()) {
            if( !(*itjoint)->IsStatic() && !vhasjoint[(*itjoint)->GetJointIndex()] && probot->DoesAffect((*itjoint)->GetJointIndex(),ilink) && !probot->DoesAffect((*itjoint)->GetJointIndex(),iattlink) ) {
                vjoints.push_back(*itjoint);
                vhasjoint[(*itjoint)->GetJointIndex()] = true;
            }
        }
    }
}

void RobotBase::Manipulator::GetChildDOFIndices(std::vector<int>& vdofindices) const
{
    vdofindices.resize(0);
    RobotBasePtr probot(__probot);
    int iattlink = __pEffector->GetIndex();
    vector<uint8_t> vhasjoint(probot->GetJoints().size(),false);
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmjoint)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        FOREACHC(itjoint, probot->GetJoints()) {
            if( !(*itjoint)->IsStatic() && !vhasjoint[(*itjoint)->GetJointIndex()] && probot->DoesAffect((*itjoint)->GetJointIndex(),ilink) && !probot->DoesAffect((*itjoint)->GetJointIndex(),iattlink) ) {
                vhasjoint[(*itjoint)->GetJointIndex()] = true;
                int idofbase = (*itjoint)->GetDOFIndex();
                for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                    vdofindices.push_back(idofbase+idof);
                }
            }
        }
    }
}

void RobotBase::Manipulator::GetChildLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(__probot);
    // get all child links of the manipualtor
    vlinks.clear();
    __pEffector->GetRigidlyAttachedLinks(vlinks);
    int iattlink = __pEffector->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }
        // there could be passive joints that are attached to the end effector that are non-static. if a link is affected by all the joints in the chain, then it is most likely a child just by the fact that all the arm joints affect it.
        if( find(vlinks.begin(), vlinks.end(), *itlink) == vlinks.end() ) { // if joint is static but still in the dof indices, it could already be rigidly attached to the end effector link and therefore inside vlinks
            vlinks.push_back(*itlink);
        }
    }
}

bool RobotBase::Manipulator::IsChildLink(const KinBody::Link &link) const
{
    if( __pEffector->IsRigidlyAttached(link) ) {
        return true;
    }

    RobotBasePtr probot(__probot);
    int iattlink = __pEffector->GetIndex();
    int ilink = link.GetIndex();
    if( ilink == iattlink ) {
        return true;
    }
    // gripper needs to be affected by all joints
    FOREACHC(itarmdof,__varmdofindices) {
        if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
            return false;
        }
    }
    return true;
}

void RobotBase::Manipulator::GetIndependentLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(__probot);
    _GetIndependentLinks(vlinks, probot, __varmdofindices, __vgripperdofindices);
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(CollisionReportPtr report) const
{
    RobotBasePtr probot(__probot);

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        // Do not skip the iteration even if the link is not enabled since the link can be grabbing stuff that is
        // enabled. CheckLinkCollision will take care of the fact that this link is not enabled.
        // if( !(*itlink)->IsEnabled() ) {
        //     continue;
        // }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        // check all gripper links including
        // 1. links that are rigidly attached to the end effector
        // 2. links that are controlled by joints other than arm joints
        // 3. links that are connected with passive but non-static joints
        if( probot->CheckLinkCollision(ilink, report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report) const
{
    KinBodyConstPtr pdummynull;
    return _CheckEndEffectorCollision(tEE, pdummynull, report);
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const Transform& tEE, KinBodyConstPtr pbody, CollisionReportPtr report) const
{
    OPENRAVE_ASSERT_FORMAT(!!pbody, "the body is not specified for collision checking with manipulator %s:%s",RobotBasePtr(__probot)->GetName()%GetName(),ORE_InvalidArguments);
    return _CheckEndEffectorCollision(tEE, pbody, report);
}

bool RobotBase::Manipulator::_CheckEndEffectorCollision(const Transform& tEE, KinBodyConstPtr pbody, CollisionReportPtr report) const
{
    RobotBasePtr probot(__probot);
    const Transform toldEE = GetTransform();
    const Transform tdelta = tEE*toldEE.inverse();
    // get all child links of the manipualtor

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        // Do not skip the iteration even if the link is not enabled since the link can be grabbing stuff that is
        // enabled. CheckLinkCollision will take care of the fact that this link is not enabled.
        // if( !(*itlink)->IsEnabled() ) {
        //     continue;
        // }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        // check all gripper links including
        // 1. links that are rigidly attached to the end effector
        // 2. links that are controlled by joints other than arm joints
        // 3. links that are connected with passive but non-static joints
        if( !!pbody ) {
            // check with the specified body
            bincollision |= probot->CheckLinkCollision(ilink,tdelta*(*itlink)->GetTransform(),pbody,report);
        }
        else {
            // check with the environment
            bincollision |= probot->CheckLinkCollision(ilink,tdelta*(*itlink)->GetTransform(),report);
        }
        if( bincollision && !bAllLinkCollisions ) { // if checking all collisions, have to continue
            return true;
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(CollisionReportPtr report, bool bIgnoreManipulatorLinks) const
{
    RobotBasePtr probot(__probot);

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    // parameters used only when bIgnoreManipulatorLinks is true
    CollisionCheckerBasePtr pselfchecker;
    std::vector<LinkConstPtr> vindependentinks;
    if( bIgnoreManipulatorLinks ) {
        _GetIndependentLinks<LinkConstPtr>(vindependentinks, probot, __varmdofindices, __vgripperdofindices);
        pselfchecker = !!probot->GetSelfCollisionChecker() ? probot->GetSelfCollisionChecker() : probot->GetEnv()->GetCollisionChecker();
    }

    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( !(*itlink)->IsEnabled() ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        // check all gripper links including
        // 1. links that are rigidly attached to the end effector
        // 2. links that are controlled by joints other than arm joints
        // 3. links that are connected with passive but non-static joints
        if( bIgnoreManipulatorLinks ) {
            FOREACHC(itindependentlink,vindependentinks) {
                if( *itlink != *itindependentlink && (*itindependentlink)->IsEnabled() ) {
                    if( pselfchecker->CheckCollision(*itlink, *itindependentlink,report) ) {
                        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                            RAVELOG_VERBOSE_FORMAT("gripper link self collision with link %s", (*itlink)->GetName());
                            return true;
                        }
                        bincollision = true;
                    }
                }
            }
            if( probot->_CheckGrabbedBodiesSelfCollision(pselfchecker, report, *itlink, bAllLinkCollisions, nullptr, vindependentinks) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    //RAVELOG_VERBOSE_FORMAT("gripper link self collision with link %s", (*itlink)->GetName());
                    return true;
                }
                bincollision = true;
            }
        }
        else {
            if( probot->CheckLinkSelfCollision(ilink,report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    return true;
                }
                bincollision = true;
            }
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(const Transform& tEE, CollisionReportPtr report, bool bIgnoreManipulatorLinks) const
{
    RobotBasePtr probot(__probot);
    const Transform toldEE = GetTransform();
    const Transform tdelta = tEE*toldEE.inverse();

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    // parameters used only when bIgnoreManipulatorLinks is true
    CollisionCheckerBasePtr pselfchecker;
    std::vector<LinkConstPtr> vindependentinks;
    if( bIgnoreManipulatorLinks ) {
        _GetIndependentLinks<LinkConstPtr>(vindependentinks, probot, __varmdofindices, __vgripperdofindices);
        pselfchecker = !!probot->GetSelfCollisionChecker() ? probot->GetSelfCollisionChecker() : probot->GetEnv()->GetCollisionChecker();
    }

    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( !(*itlink)->IsEnabled() ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,__varmdofindices) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        // check all gripper links including
        // 1. links that are rigidly attached to the end effector
        // 2. links that are controlled by joints other than arm joints
        // 3. links that are connected with passive but non-static joints
        if( bIgnoreManipulatorLinks ) {
            boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(*itlink)); // gcc optimization bug when linksaver is on stack?
            const Transform tlinktrans = tdelta*(*itlink)->GetTransform();
            (*itlink)->SetTransform(tlinktrans);

            FOREACHC(itindependentlink,vindependentinks) {
                if( *itlink != *itindependentlink && (*itindependentlink)->IsEnabled() ) {
                    if( pselfchecker->CheckCollision(*itlink, *itindependentlink,report) ) {
                        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                            RAVELOG_VERBOSE_FORMAT("gripper link self collision with link %s", (*itlink)->GetName());
                            return true;
                        }
                        bincollision = true;
                    }
                }
            }
            if( probot->_CheckGrabbedBodiesSelfCollision(pselfchecker, report, *itlink, bAllLinkCollisions, std::bind(_UpdateGrabbedBodyTransformWithSaver, std::placeholders::_1, std::placeholders::_2, std::cref(tlinktrans)), vindependentinks) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    //RAVELOG_VERBOSE_FORMAT("gripper link self collision with link %s", (*itlink)->GetName());
                    return true;
                }
                bincollision = true;
            }
        }
        else {
            if( probot->CheckLinkSelfCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    return true;
                }
                bincollision = true;
            }
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const IkParameterization& ikparam, CollisionReportPtr report, int numredundantsamples) const
{
    KinBodyConstPtr pdummynull; // pass null to check with the environment
    return _CheckEndEffectorCollision(ikparam, pdummynull, report, numredundantsamples);
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const IkParameterization& ikparam, KinBodyConstPtr pbody, CollisionReportPtr report, int numredundantsamples) const
{
    OPENRAVE_ASSERT_FORMAT(!!pbody, "the body is not specified for collision checking with manipulator %s:%s",RobotBasePtr(__probot)->GetName()%GetName(),ORE_InvalidArguments);
    return _CheckEndEffectorCollision(ikparam, pbody, report, numredundantsamples);
}

bool RobotBase::Manipulator::_CheckEndEffectorCollision(const IkParameterization& ikparam, KinBodyConstPtr pbody, CollisionReportPtr report, int numredundantsamples) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return _CheckEndEffectorCollision(ikparam.GetTransform6D(),pbody,report);
    }
    RobotBasePtr probot = GetRobot();
    if( numredundantsamples > 0 ) {
        if( ikparam.GetType() == IKP_TranslationDirection5D ) {
            Transform tStartEE;
            tStartEE.rot = quatRotateDirection(_info._vdirection, ikparam.GetTranslationDirection5D().dir);
            tStartEE.trans = ikparam.GetTranslationDirection5D().pos;
            Vector qdelta = quatFromAxisAngle(_info._vdirection, 2*M_PI/dReal(numredundantsamples));
            bool bNotInCollision = false;
            for(int i = 0; i < numredundantsamples; ++i) {
                if( !_CheckEndEffectorCollision(tStartEE,pbody,report) ) {
                    // doesn't collide, but will need to verify that there actually exists an IK solution there...
                    // if we accidentally return here even there's no IK solution, then later processes could waste a lot of time looking for it.
                    bNotInCollision = true;
                    break;
                }
                tStartEE.rot = quatMultiply(tStartEE.rot, qdelta);
            }
            if( !bNotInCollision ) {
                return true;
            }
        }
        else {
            RAVELOG_WARN_FORMAT("do not support redundant checking for iktype 0x%x", ikparam.GetType());
        }
    }

    IkSolverBasePtr pIkSolver = GetIkSolver();
    //OPENRAVE_ASSERT_OP_FORMAT(GetArmDOF(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",probot->GetName()%GetName(),ORE_Failed);
    OPENRAVE_ASSERT_FORMAT(pIkSolver->Supports(ikparam.GetType()),"manipulator %s:%s ik solver %s does not support ik type 0x%x",probot->GetName()%GetName()%pIkSolver->GetXMLId()%ikparam.GetType(),ORE_InvalidState);
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!__pBase ) {
        localgoal = __pBase->GetTransform().inverse()*ikparam;
    }
    else {
        localgoal=ikparam;
    }

    // if IK can be solved, then there exists a solution for the end effector that is not in collision
    IkReturn ikreturn(IKRA_Success);
    IkReturnPtr pikreturn(&ikreturn,utils::null_deleter());

    // need to use free params here since sometimes IK can have 3+ free DOF and it would freeze searching for all of them
    std::vector<dReal> vFreeParameters;
    pIkSolver->GetFreeParameters(vFreeParameters);
    if( pIkSolver->Solve(localgoal, std::vector<dReal>(), vFreeParameters, IKFO_CheckEnvCollisions|IKFO_IgnoreCustomFilters, pikreturn) ) { // TODO need to specify IKFO_IgnoreSelfCollisions?
        return false;
    }
    else {
        if( (ikreturn._action&IKRA_RejectSelfCollision) != IKRA_RejectSelfCollision && (ikreturn._action&IKRA_RejectEnvCollision) != IKRA_RejectEnvCollision ) {
            RAVELOG_VERBOSE_FORMAT("ik solution not found due to non-collision reasons (0x%x), returning true anway...", ikreturn._action);
            // is this a good idea?
        }
        if( !!report ) {
            // solver failed, should have some way of initializing the report...
            if( numredundantsamples > 0 ) {
                if( ikparam.GetType() == IKP_TranslationDirection5D || ikparam.GetType() == IKP_TranslationXAxisAngleZNorm4D || ikparam.GetType() == IKP_TranslationYAxisAngleXNorm4D || ikparam.GetType() == IKP_TranslationZAxisAngleYNorm4D ) {
                    // if here, then already determined that there is a roll that is collision free, so return False
                    return false;
                }
            }
        }
        return true;
    }
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(const IkParameterization& ikparam, CollisionReportPtr report, int numredundantsamples, bool bIgnoreManipulatorLinks) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return CheckEndEffectorSelfCollision(ikparam.GetTransform6D(),report,bIgnoreManipulatorLinks);
    }
    RobotBasePtr probot = GetRobot();
    if( numredundantsamples > 0 ) {
        if( ikparam.GetType() == IKP_TranslationDirection5D ) {
            Transform tStartEE;
            tStartEE.rot = quatRotateDirection(_info._vdirection, ikparam.GetTranslationDirection5D().dir);
            tStartEE.trans = ikparam.GetTranslationDirection5D().pos;
            Vector qdelta = quatFromAxisAngle(_info._vdirection, 2*M_PI/dReal(numredundantsamples));
            bool bNotInCollision = false;
            for(int i = 0; i < numredundantsamples; ++i) {
                if( !CheckEndEffectorSelfCollision(tStartEE,report, bIgnoreManipulatorLinks) ) {
                    // doesn't collide, but will need to verify that there actually exists an IK solution there...
                    // if we accidentally return here even there's no IK solution, then later processes could waste a lot of time looking for it.
                    bNotInCollision = true;
                    break;
                }
                tStartEE.rot = quatMultiply(tStartEE.rot, qdelta);
            }
            if( !bNotInCollision ) {
                return true;
            }
        }
        else {
            RAVELOG_WARN_FORMAT("do not support redundant checking for iktype 0x%x", ikparam.GetType());
        }
    }

    IkSolverBasePtr pIkSolver = GetIkSolver();
    //OPENRAVE_ASSERT_OP_FORMAT(GetArmDOF(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",probot->GetName()%GetName(),ORE_Failed);
    OPENRAVE_ASSERT_FORMAT(pIkSolver->Supports(ikparam.GetType()),"manipulator %s:%s ik solver %s does not support ik type 0x%x",probot->GetName()%GetName()%pIkSolver->GetXMLId()%ikparam.GetType(),ORE_InvalidState);
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!__pBase ) {
        localgoal = __pBase->GetTransform().inverse()*ikparam;
    }
    else {
        localgoal=ikparam;
    }

    // if IK can be solved, then there exists a solution for the end effector that is not in collision
    IkReturn ikreturn(IKRA_Success);
    IkReturnPtr pikreturn(&ikreturn,utils::null_deleter());

    // need to use free params here since sometimes IK can have 3+ free DOF and it would freeze searching for all of them
    std::vector<dReal> vFreeParameters;
    pIkSolver->GetFreeParameters(vFreeParameters);
    if( pIkSolver->Solve(localgoal, std::vector<dReal>(), vFreeParameters, IKFO_IgnoreCustomFilters, pikreturn) ) {
        return false;
    }
    else {
        if( (ikreturn._action&IKRA_RejectSelfCollision) != IKRA_RejectSelfCollision && (ikreturn._action&IKRA_RejectEnvCollision) != IKRA_RejectEnvCollision ) {
            RAVELOG_VERBOSE_FORMAT("ik solution not found due to non-collision reasons (0x%x), returning true anway...", ikreturn._action);
            // is this a good idea?
        }
        if( !!report ) {
            // solver failed, should have some way of initializing the report...
            if( numredundantsamples > 0 ) {
                if( ikparam.GetType() == IKP_TranslationDirection5D || ikparam.GetType() == IKP_TranslationXAxisAngleZNorm4D || ikparam.GetType() == IKP_TranslationYAxisAngleXNorm4D || ikparam.GetType() == IKP_TranslationZAxisAngleYNorm4D ) {
                    // if here, then already determined that there is a roll that is collision free, so return False
                    return false;
                }
            }
        }
        return true;
    }

//    // only care about the end effector position, so disable all time consuming options. still leave the custom options in case the user wants to call some custom stuff?
//    // is it necessary to call with IKFO_IgnoreJointLimits knowing that the robot will never reach those solutions?
//    std::vector< std::vector<dReal> > vsolutions;
//    if( !pIkSolver->SolveAll(localgoal, vector<dReal>(), IKFO_IgnoreSelfCollisions,vsolutions) ) {
//        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find ik solution for type 0x%x"),ikparam.GetType(),ORE_InvalidArguments);
//    }
//    RobotStateSaver saver(probot);
//    probot->SetActiveDOFs(GetArmIndices());
//    // have to check all solutions since the 6D transform can change even though the ik parameterization doesn't
//    std::list<Transform> listprevtransforms;
//    FOREACH(itsolution,vsolutions) {
//        probot->SetActiveDOFValues(*itsolution,false);
//        Transform t = GetTransform();
//        // check if previous transforms exist
//        bool bhassimilar = false;
//        FOREACH(ittrans,listprevtransforms) {
//            if( TransformDistanceFast(t,*ittrans) < g_fEpsilonLinear*10 ) {
//                bhassimilar = true;
//                break;
//            }
//        }
//        if( !bhassimilar ) {
//            if( CheckEndEffectorSelfCollision(GetTransform(),report) ) {
//                return true;
//            }
//            listprevtransforms.push_back(t);
//        }
//    }
//
//    return false;
}

bool RobotBase::Manipulator::CheckIndependentCollision(CollisionReportPtr report) const
{
    RobotBasePtr probot(__probot);
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    FOREACHC(itlink, probot->GetLinks()) {
        if( !(*itlink)->IsEnabled() ) {
            continue;
        }
        bool bAffected = false;
        FOREACHC(itindex,__varmdofindices) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        if( !bAffected ) {
            FOREACHC(itindex,__vgripperdofindices) {
                if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                    bAffected = true;
                    break;
                }
            }
        }

        if( !bAffected ) {
            if( probot->GetEnv()->GetCollisionChecker()->GetCollisionOptions() & CO_ActiveDOFs ) {
                // collision checker has the CO_ActiveDOFs option set, so check if link is active
                if( !probot->GetAffineDOF() ) {
                    bool bActive = false;
                    FOREACHC(itdofindex, probot->GetActiveDOFIndices()) {
                        if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itdofindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                            bActive = true;
                            break;
                        }
                    }
                    if( !bActive ) {
                        continue;
                    }
                }
            }
            if( probot->GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink),report) ) {
                return true;
            }

            // check if any grabbed bodies are attached to this link
            for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : probot->_grabbedBodiesByEnvironmentIndex) {
                const GrabbedPtr& pgrabbed = grabPair.second;
                if( pgrabbed->_pGrabbingLink == *itlink ) {
                    if( vbodyexcluded.empty() ) {
                        vbodyexcluded.push_back(KinBodyConstPtr(probot));
                    }
                    KinBodyPtr pbody = pgrabbed->_pGrabbedBody.lock();
                    if( !!pbody && probot->GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) ) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool RobotBase::Manipulator::IsGrabbing(const KinBody &body) const
{
    RobotBasePtr probot(__probot);
    KinBody::LinkPtr plink = probot->IsGrabbing(body);
    if( !!plink ) {
        if( plink == __pEffector || plink == __pBase ) {
            return true;
        }
        int iattlink = __pEffector->GetIndex();
        FOREACHC(itlink, probot->GetLinks()) {
            int ilink = (*itlink)->GetIndex();
            if( ilink == iattlink ) {
                continue;
            }
            // gripper needs to be affected by all joints
            bool bGripperLink = true;
            FOREACHC(itarmdof,__varmdofindices) {
                if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                    bGripperLink = false;
                    break;
                }
            }
            if( bGripperLink &&(plink == *itlink)) {
                return true;
            }
        }
    }
    return false;
}

void RobotBase::Manipulator::CalculateJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(__probot);
    probot->ComputeJacobianTranslation(__pEffector->GetIndex(), __pEffector->GetTransform() * _info._tLocalTool.trans, jacobian, __varmdofindices);
}

void RobotBase::Manipulator::CalculateJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][__varmdofindices.size()]);
    if( __varmdofindices.size() == 0 ) {
        return;
    }
    RobotBasePtr probot(__probot);
    std::vector<dReal> vjacobian;
    probot->ComputeJacobianTranslation(__pEffector->GetIndex(), __pEffector->GetTransform() * _info._tLocalTool.trans, vjacobian, __varmdofindices);
    OPENRAVE_ASSERT_OP(vjacobian.size(),==,3*__varmdofindices.size());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+__varmdofindices.size(),itdst->begin());
        itsrc += __varmdofindices.size();
    }
}

void RobotBase::Manipulator::CalculateRotationJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(__probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveRotationJacobian(__pEffector->GetIndex(),quatMultiply(__pEffector->GetTransform().rot, _info._tLocalTool.rot),jacobian);
}

void RobotBase::Manipulator::CalculateRotationJacobian(boost::multi_array<dReal,2>& jacobian) const
{
    RobotBasePtr probot(__probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveRotationJacobian(__pEffector->GetIndex(),quatMultiply(__pEffector->GetTransform().rot, _info._tLocalTool.rot),jacobian);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(__probot);
    probot->ComputeJacobianAxisAngle(__pEffector->GetIndex(), jacobian, __varmdofindices);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][__varmdofindices.size()]);
    if( __varmdofindices.size() == 0 ) {
        return;
    }
    RobotBasePtr probot(__probot);
    std::vector<dReal> vjacobian;
    probot->ComputeJacobianAxisAngle(__pEffector->GetIndex(), vjacobian, __varmdofindices);
    OPENRAVE_ASSERT_OP(vjacobian.size(),==,3*__varmdofindices.size());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+__varmdofindices.size(),itdst->begin());
        itsrc += __varmdofindices.size();
    }
}

void RobotBase::Manipulator::serialize(std::ostream& o, int options, IkParameterizationType iktype) const
{
    if( options & SO_RobotManipulators ) {
        o << (!__pBase ? -1 : __pBase->GetIndex()) << " " << (!__pEffector ? -1 : __pEffector->GetIndex()) << " ";
        // don't include __varmdofindices and __vgripperdofindices since they are generated from the data
        o << __vGripperJointNames.size() << " ";
        FOREACHC(it, __vGripperJointNames) {
            o << *it << " ";
        }
        FOREACHC(it, __vChuckingDirection) {
            o << *it << " ";
        }
        SerializeRound(o,_info._tLocalTool);
    }
    if( options & (SO_Kinematics|SO_InverseKinematics) ) {
        RobotBasePtr probot(__probot);
        Transform tcur;
        vector<JointPtr> vjoints;
        if(!!__pIkChainEndLink) {
            probot->GetChain(__pBase->GetIndex(),__pIkChainEndLink->GetIndex(), vjoints);
        }

        if( vjoints.size() > 0 || probot->GetChain(__pBase->GetIndex(),__pEffector->GetIndex(), vjoints) ) {
            // due to back compat issues, have to compute the end effector transform first
            FOREACH(itjoint, vjoints) {
                tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform() * (*itjoint)->GetInternalHierarchyRightTransform();
            }
            tcur = tcur;
            // if 6D transform IK, then don't inlucde the local tool transform!
            if( !!(options & SO_Kinematics) || iktype != IKP_Transform6D ) {
                tcur *= _info._tLocalTool;
            }
            SerializeRound(o,tcur);
            o << __varmdofindices.size() << " ";

            tcur = Transform();
            int index = 0;
            FOREACH(itjoint, vjoints) {
                if( !(*itjoint)->IsStatic() ) {
                    o << (*itjoint)->GetType() << " ";
                }

                if( (*itjoint)->GetDOFIndex() >= 0 && find(__varmdofindices.begin(),__varmdofindices.end(),(*itjoint)->GetDOFIndex()) != __varmdofindices.end() ) {
                    tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform();
                    for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                        SerializeRound3(o,tcur.trans);
                        SerializeRound3(o,tcur.rotate((*itjoint)->GetInternalHierarchyAxis(idof)));
                    }
                    tcur = tcur * (*itjoint)->GetInternalHierarchyRightTransform();
                }
                else {
                    // not sure if this is correct for a mimic joint...
                    tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform() * (*itjoint)->GetInternalHierarchyRightTransform();
                    if( (*itjoint)->IsMimic() ) {
                        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                            if( (*itjoint)->IsMimic(idof) ) {
                                o << "mimic " << index << " ";
                                for(int ieq = 0; ieq < 3; ++ieq) {
                                    o << (*itjoint)->GetMimicEquation(idof,ieq) << " ";
                                }
                            }
                        }
                    }
                }
                index += 1;
            }
        }
    }

    // output direction if SO_Kinematics|SO_RobotManipulators. if IK hash, then output only on certain ik types.
    if( !!(options & (SO_Kinematics|SO_RobotManipulators)) || (!!(options&SO_InverseKinematics) && (iktype == IKP_TranslationDirection5D || iktype == IKP_Direction3D || iktype == IKP_Ray4D)) ) {
        SerializeRound3(o,_info._vdirection);
    }
}

ConfigurationSpecification RobotBase::Manipulator::GetArmConfigurationSpecification(const std::string& interpolation) const
{
    if( interpolation.size() == 0 ) {
        return __armspec;
    }
    ConfigurationSpecification spec = __armspec;
    FOREACH(itgroup,spec._vgroups) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

ConfigurationSpecification RobotBase::Manipulator::GetIkConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation) const
{
    return IkParameterization::GetConfigurationSpecification(iktype, GetRobot()->GetName(), GetName());
}

const std::string& RobotBase::Manipulator::GetStructureHash() const
{
    if( __hashstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_RobotManipulators|SO_Kinematics);
        __hashstructure = utils::GetMD5HashString(ss.str());
    }
    return __hashstructure;
}

const std::string& RobotBase::Manipulator::GetKinematicsStructureHash() const
{
    if( __hashkinematicsstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics);
        __hashkinematicsstructure = utils::GetMD5HashString(ss.str());
    }
    return __hashkinematicsstructure;
}

const std::string& RobotBase::Manipulator::GetInverseKinematicsStructureHash(IkParameterizationType iktype) const
{
    std::map<IkParameterizationType, std::string>::const_iterator it = __maphashikstructure.find(iktype);
    if( it == __maphashikstructure.end() ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_InverseKinematics,iktype);
        __maphashikstructure[iktype] = utils::GetMD5HashString(ss.str());
        return __maphashikstructure[iktype];
    }
    else {
        return it->second;
    }
}

void RobotBase::Manipulator::_ComputeInternalInformation()
{
    if( !utils::IsValidName(_info._name) ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("manipulator name \"%s\" is not valid"), GetName(), ORE_Failed);
    }
    RobotBasePtr probot(__probot);
    __pBase = probot->GetLink(_info._sBaseLinkName);
    __pIkChainEndLink = probot->GetLink(_info._sIkChainEndLinkName);
    __pEffector = probot->GetLink(_info._sEffectorLinkName);
    __varmdofindices.resize(0);
    __vgripperdofindices.resize(0);
    __pIkSolver.reset();
    __hashstructure.resize(0);
    __hashkinematicsstructure.resize(0);
    __maphashikstructure.clear();
    if( !__pBase || !__pEffector ) {
        RAVELOG_WARN(str(boost::format("manipulator %s has undefined base and end effector links %s, %s\n")%GetName()%_info._sBaseLinkName%_info._sEffectorLinkName));
        __armspec = ConfigurationSpecification();
    }
    else {
        vector<JointPtr> vjoints;
        std::vector<int> vmimicdofs;
        if(!!__pIkChainEndLink) {
            probot->GetChain(__pBase->GetIndex(),__pIkChainEndLink->GetIndex(), vjoints);
        }

        if( vjoints.size() > 0 || probot->GetChain(__pBase->GetIndex(),__pEffector->GetIndex(), vjoints) ) {
            FOREACH(it,vjoints) {
                if( (*it)->IsStatic() ) {
                    // ignore
                }
                else if( (*it)->IsMimic() ) {
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        if( (*it)->IsMimic(i) ) {
                            (*it)->GetMimicDOFIndices(vmimicdofs,i);
                            FOREACHC(itmimicdof,vmimicdofs) {
                                if( find(__varmdofindices.begin(),__varmdofindices.end(),*itmimicdof) == __varmdofindices.end() ) {
                                    __varmdofindices.push_back(*itmimicdof);
                                }
                            }
                        }
                        else if( (*it)->GetDOFIndex() >= 0 ) {
                            __varmdofindices.push_back((*it)->GetDOFIndex()+i);
                        }
                    }
                }
                else if( (*it)->GetDOFIndex() < 0) {
                    RAVELOG_WARN(str(boost::format("manipulator arm contains joint %s without a dof index, ignoring...\n")%(*it)->GetName()));
                }
                else { // ignore static joints
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        __varmdofindices.push_back((*it)->GetDOFIndex()+i);
                    }
                }
            }
            // initialize the arm configuration spec
            __armspec = probot->GetConfigurationSpecificationIndices(__varmdofindices);

            // sanity check
            FOREACHC(itarmdof,__varmdofindices) {
                if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(), __pEffector->GetIndex()) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("manipulator \"%s\" is not valid. end effector link \"%s\" is not affected by joint \"%s\""), GetName()%__pEffector->GetName()%probot->GetJointFromDOFIndex(*itarmdof)->GetName(), ORE_Failed);
                }
            }
        }
        else {
            RAVELOG_WARN(str(boost::format("manipulator %s failed to find chain between %s and %s links\n")%GetName()%__pBase->GetName()%__pEffector->GetName()));
        }
    }


    // Initialize __vGripperJointNames and __vChuckingDirection, both based on ManipulatorInfo and GripperInfo.
    // In the latest way, recommended to use GripperInfo. But, for backward compatibility, prioritize those in ManipulatorInfo.
    // - The usual robot model files do not need the backward compatibility since they should be properly migrated, e.g. GripperInfo should only have settings. However, the saved scene from the old openrave is not migrated. When we load such scene from the new openrave, it requires this backward compatibility code like this.
    // - Some of the user code call "AddManipulator" to temporarily define the manipulator with newer chuckingDirections/gripperJointNames setting. To support this, still need to read those from ManipulatorInfo.
    GripperInfoPtr pGripperInfo;
    __vGripperJointNames = _info._vGripperJointNames;
    __vChuckingDirection = _info._vChuckingDirection;
    if( !_info._grippername.empty() ) {
        pGripperInfo = probot->GetGripperInfo(_info._grippername);
        if( !!pGripperInfo ) {
            if( _info._vGripperJointNames.empty() ) {
                __vGripperJointNames = pGripperInfo->gripperJointNames;
            }
            if( _info._vChuckingDirection.empty() ) {
                __vChuckingDirection = pGripperInfo->vChuckingDirections;
            }
        }
    }

    // init the gripper dof indices
    std::vector<int> vValidChuckingDirections;
    std::vector<std::string> vValidGripperJointNames;
    size_t ichuckingdirection = 0;
    FOREACHC(itjointname,__vGripperJointNames) {
        JointPtr pjoint = probot->GetJoint(*itjointname);
        if( !pjoint ) {
            RAVELOG_WARN(str(boost::format("could not find gripper joint %s for manipulator %s")%*itjointname%GetName()));
            ichuckingdirection++;
        }
        else {
            if( pjoint->GetDOFIndex() >= 0 ) {
                for(int i = 0; i < pjoint->GetDOF(); ++i) {
                    if( find(__varmdofindices.begin(), __varmdofindices.end(), pjoint->GetDOFIndex()+i) != __varmdofindices.end() ) {

                        std::stringstream ss;
                        FOREACH(itindex, __varmdofindices) {
                            ss << *itindex << ", ";
                        }
                        RAVELOG_ERROR_FORMAT("manipulator %s has gripper joint %s (dofIndex=%d) is also part of arm dofindices [%s]! Perhaps the manipulator's end effector '%s' is not set correctly. So excluding from gripper...", GetName()%pjoint->GetName()%(pjoint->GetDOFIndex()+i)%ss.str()%_info._sEffectorLinkName);
                    }
                    else {
                        __vgripperdofindices.push_back(pjoint->GetDOFIndex()+i);
                        vValidGripperJointNames.push_back(*itjointname);
                        if( ichuckingdirection < __vChuckingDirection.size() ) {
                            vValidChuckingDirections.push_back(__vChuckingDirection[ichuckingdirection++]);
                        }
                        else {
                            vValidChuckingDirections.push_back(0);
                            RAVELOG_WARN_FORMAT("manipulator %s chucking direction not correct length, might get bad chucking/release grasping", GetName());
                        }
                    }
                }
            }
            else {
                ++ichuckingdirection;
                RAVELOG_WARN(str(boost::format("manipulator %s gripper joint %s is not active, so has no dof index. ignoring.")%GetName()%*itjointname));
            }
        }
    }

    // If the above validated chuckingDirections/gripperJointNames are different from those in gripperInfo, show warin.
    if( !!pGripperInfo ) {
        if( (pGripperInfo->vChuckingDirections != vValidChuckingDirections) || (pGripperInfo->gripperJointNames != vValidGripperJointNames ) ) {
            std::stringstream ss;
            ss << "'chuckingDirection' in gripperInfo [";
            FOREACH(itdir, pGripperInfo->vChuckingDirections) {
                ss << *itdir << ", ";
            }
            ss << "], 'gripperJointNames' in gripperInfo [";
            FOREACH(itname, pGripperInfo->gripperJointNames) {
                ss << *itname << ", ";
            }
            ss << "], 'chuckingDirection' validated in tool [";
            FOREACH(itdir, vValidChuckingDirections) {
                ss << *itdir << ", ";
            }
            ss << "], 'gripperJointNames' validated in tool [";
            FOREACH(itname, vValidGripperJointNames) {
                ss << *itname << ", ";
            }
            ss << "]";
            RAVELOG_WARN_FORMAT("'chuckingDirection' and/or 'gripperJointNames' in gripperInfo '%s' is inconsistent with that in manipulator '%s'. %s", _info._grippername % GetName()%ss.str());
        }
    }

    __vChuckingDirection.swap(vValidChuckingDirections);
    __vGripperJointNames.swap(vValidGripperJointNames);
}

}
