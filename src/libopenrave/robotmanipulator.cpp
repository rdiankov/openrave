// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov (rosen.diankov@gmail.com)
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
    if( !!r->GetEndEffector() ) {
        __pEffector = probot->GetLinks().at(r->GetEndEffector()->GetIndex());
    }
    __pIkSolver.reset(); // will be initialized when needed
//    if( _info._sIkSolverXMLId.size() > 0 ) {
//        //__pIkSolver = RaveCreateIkSolver(probot->GetEnv(), _info._sIkSolverXMLId);
//        // cannot call __pIkSolver->Init since this is the constructor...
//    }
}

void RobotBase::Manipulator::SetLocalToolTransform(const Transform& t)
{
    _info._sIkSolverXMLId.resize(0);
    __pIkSolver.reset();
    _info._tLocalTool = t;
    GetRobot()->_ParametersChanged(Prop_RobotManipulatorTool);
    __hashkinematicsstructure.resize(0);
    __hashstructure.resize(0);
}

void RobotBase::Manipulator::SetLocalToolDirection(const Vector& direction)
{
    _info._sIkSolverXMLId.resize(0);
    __pIkSolver.reset();
    _info._vdirection = direction;
    GetRobot()->_ParametersChanged(Prop_RobotManipulatorTool);
    __hashkinematicsstructure.resize(0);
    __hashstructure.resize(0);
}

void RobotBase::Manipulator::SetName(const std::string& name)
{
    RobotBasePtr probot=GetRobot();
    FOREACHC(itmanip,probot->GetManipulators()) {
        if( *itmanip != shared_from_this() && name == (*itmanip)->GetName() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("manipulator name change '%s'->'%s' is colliding with other manipulator", _info._name%name, ORE_InvalidArguments);
        }
    }
    _info._name=name;
    probot->_ParametersChanged(Prop_RobotManipulatorName);
}

Transform RobotBase::Manipulator::GetTransform() const
{
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
        GetRobot()->_ParametersChanged(Prop_RobotManipulatorSolver);
        return true;
    }

    return false;
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


bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, int filteroptions, IkReturnPtr ikreturn) const
{
    return FindIKSolution(goal, vector<dReal>(), filteroptions, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn) const
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
    return vFreeParameters.size() == 0 ? pIkSolver->Solve(localgoal, solution, filteroptions, ikreturn) : pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const
{
    return FindIKSolutions(goal, vector<dReal>(), filteroptions, vikreturns);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const
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
    return vFreeParameters.size() == 0 ? pIkSolver->SolveAll(localgoal,filteroptions,vikreturns) : pIkSolver->SolveAll(localgoal,vFreeParameters,filteroptions,vikreturns);
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
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,zangle));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: TranslationLocalGlobal6D type setting local translation to (0,0,0).\n");
        ikp.SetTranslationLocalGlobal6D(Vector(0,0,0),t.trans);
        break;
    }
    case IKP_TranslationXAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(vglobaldirection.z));
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vglobaldirection.y,vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vglobaldirection.z,vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vglobaldirection.x,vglobaldirection.z));
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("invalid ik type 0x%x",iktype,ORE_InvalidArguments);
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
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,zangle));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        Vector localtrans = ikparam.GetTranslationLocalGlobal6D().first;
        ikp.SetTranslationLocalGlobal6D(localtrans,t * localtrans);
        break;
    }
    case IKP_TranslationXAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(vglobaldirection.z));
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vglobaldirection.y,vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vglobaldirection.z,vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        Vector vglobaldirection = t.rotate(_info._vdirection);
        ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vglobaldirection.x,vglobaldirection.z));
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("invalid ik type 0x%x",ikparam.GetType(),ORE_InvalidArguments);
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
    vlinks.resize(0);
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
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                vlinks.push_back(*itlink);
                break;
            }
        }
    }
}

bool RobotBase::Manipulator::IsChildLink(LinkConstPtr plink) const
{
    if( __pEffector->IsRigidlyAttached(plink) ) {
        return true;
    }

    RobotBasePtr probot(__probot);
    // get all child links of the manipualtor
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
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                return true;
            }
        }
    }
    return false;
}

void RobotBase::Manipulator::GetIndependentLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(__probot);
    FOREACHC(itlink, probot->GetLinks()) {
        bool bAffected = false;
        FOREACHC(itindex,__varmdofindices) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        FOREACHC(itindex,__vgripperdofindices) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }

        if( !bAffected )
            vlinks.push_back(*itlink);
    }
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report) const
{
    RobotBasePtr probot(__probot);
    Transform toldEE = GetTransform();
    Transform tdelta = tEE*toldEE.inverse();
    // get all child links of the manipualtor
    int iattlink = __pEffector->GetIndex();
    vector<LinkPtr> vattachedlinks;
    __pEffector->GetRigidlyAttachedLinks(vattachedlinks);
    FOREACHC(itlink,vattachedlinks) {
        if( probot->CheckLinkCollision((*itlink)->GetIndex(),tdelta*(*itlink)->GetTransform(),report) ) {
            return true;
        }
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if((ilink == iattlink)|| !(*itlink)->IsEnabled() ) {
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
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                if( probot->CheckLinkCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(const Transform& tEE, CollisionReportPtr report) const
{
    RobotBasePtr probot(__probot);
    Transform toldEE = GetTransform();
    Transform tdelta = tEE*toldEE.inverse();
    // get all child links of the manipualtor
    int iattlink = __pEffector->GetIndex();
    vector<LinkPtr> vattachedlinks;
    __pEffector->GetRigidlyAttachedLinks(vattachedlinks);
    FOREACHC(itlink,vattachedlinks) {
        if( probot->CheckLinkSelfCollision((*itlink)->GetIndex(),tdelta*(*itlink)->GetTransform(),report) ) {
            return true;
        }
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if((ilink == iattlink)|| !(*itlink)->IsEnabled() ) {
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
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                if( probot->CheckLinkSelfCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const IkParameterization& ikparam, CollisionReportPtr report) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return CheckEndEffectorCollision(ikparam.GetTransform6D(),report);
    }
    RobotBasePtr probot = GetRobot();
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_OP_FORMAT((int)GetArmIndices().size(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
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

    // only care about the end effector position, so disable all time consuming options. still leave the custom options in case the user wants to call some custom stuff?
    // is it necessary to call with IKFO_IgnoreJointLimits knowing that the robot will never reach those solutions?
    std::vector< std::vector<dReal> > vsolutions;
    if( !pIkSolver->SolveAll(localgoal, vector<dReal>(), IKFO_IgnoreSelfCollisions,vsolutions) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to find ik solution for type 0x%x",ikparam.GetType(),ORE_InvalidArguments);
    }
    RobotStateSaver saver(probot);
    probot->SetActiveDOFs(GetArmIndices());
    // have to check all solutions since the 6D transform can change even though the ik parameterization doesn't
    std::list<Transform> listprevtransforms;
    FOREACH(itsolution,vsolutions) {
        probot->SetActiveDOFValues(*itsolution,false);
        Transform t = GetTransform();
        // check if previous transforms exist
        bool bhassimilar = false;
        FOREACH(ittrans,listprevtransforms) {
            if( TransformDistanceFast(t,*ittrans) < g_fEpsilonLinear*10 ) {
                bhassimilar = true;
                break;
            }
        }
        if( !bhassimilar ) {
            if( CheckEndEffectorCollision(GetTransform(),report) ) {
                return true;
            }
            listprevtransforms.push_back(t);
        }
    }

    return false;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(const IkParameterization& ikparam, CollisionReportPtr report) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return CheckEndEffectorSelfCollision(ikparam.GetTransform6D(),report);
    }
    RobotBasePtr probot = GetRobot();
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_OP_FORMAT((int)GetArmIndices().size(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
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

    // only care about the end effector position, so disable all time consuming options. still leave the custom options in case the user wants to call some custom stuff?
    // is it necessary to call with IKFO_IgnoreJointLimits knowing that the robot will never reach those solutions?
    std::vector< std::vector<dReal> > vsolutions;
    if( !pIkSolver->SolveAll(localgoal, vector<dReal>(), IKFO_IgnoreSelfCollisions,vsolutions) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to find ik solution for type 0x%x",ikparam.GetType(),ORE_InvalidArguments);
    }
    RobotStateSaver saver(probot);
    probot->SetActiveDOFs(GetArmIndices());
    // have to check all solutions since the 6D transform can change even though the ik parameterization doesn't
    std::list<Transform> listprevtransforms;
    FOREACH(itsolution,vsolutions) {
        probot->SetActiveDOFValues(*itsolution,false);
        Transform t = GetTransform();
        // check if previous transforms exist
        bool bhassimilar = false;
        FOREACH(ittrans,listprevtransforms) {
            if( TransformDistanceFast(t,*ittrans) < g_fEpsilonLinear*10 ) {
                bhassimilar = true;
                break;
            }
        }
        if( !bhassimilar ) {
            if( CheckEndEffectorSelfCollision(GetTransform(),report) ) {
                return true;
            }
            listprevtransforms.push_back(t);
        }
    }

    return false;
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
            FOREACHC(itgrabbed,probot->_vGrabbedBodies) {
                GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
                if( pgrabbed->_plinkrobot == *itlink ) {
                    if( vbodyexcluded.empty() ) {
                        vbodyexcluded.push_back(KinBodyConstPtr(probot));
                    }
                    KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
                    if( !!pbody && probot->GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) ) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool RobotBase::Manipulator::IsGrabbing(KinBodyConstPtr pbody) const
{
    RobotBasePtr probot(__probot);
    KinBody::LinkPtr plink = probot->IsGrabbing(pbody);
    if( !!plink ) {
        if((plink == __pEffector)||(plink == __pBase)) {
            return true;
        }
        int iattlink = __pEffector->GetIndex();
        FOREACHC(itlink, probot->GetLinks()) {
            int ilink = (*itlink)->GetIndex();
            if( ilink == iattlink )
                continue;
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

void RobotBase::Manipulator::serialize(std::ostream& o, int options) const
{
    if( options & SO_RobotManipulators ) {
        o << (!__pBase ? -1 : __pBase->GetIndex()) << " " << (!__pEffector ? -1 : __pEffector->GetIndex()) << " ";
        // don't include __varmdofindices and __vgripperdofindices since they are generated from the data
        o << _info._vGripperJointNames.size() << " ";
        FOREACHC(it,_info._vGripperJointNames) {
            o << *it << " ";
        }
        FOREACHC(it,_info._vClosingDirection) {
            SerializeRound(o,*it);
        }
        SerializeRound(o,_info._tLocalTool);
    }
    if( options & SO_Kinematics ) {
        RobotBasePtr probot(__probot);
        Transform tcur;
        std::vector<JointPtr> vjoints;
        if( probot->GetChain(__pBase->GetIndex(),__pEffector->GetIndex(), vjoints) ) {
            // due to back compat issues, have to compute the end effector transform first
            FOREACH(itjoint, vjoints) {
                tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform() * (*itjoint)->GetInternalHierarchyRightTransform();
            }
            tcur = tcur*_info._tLocalTool;
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
    if( options & (SO_Kinematics|SO_RobotManipulators) ) {
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

const std::string& RobotBase::Manipulator::GetStructureHash() const
{
    if( __hashstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_RobotManipulators);
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

void RobotBase::Manipulator::_ComputeInternalInformation()
{
    if( !utils::IsValidName(_info._name) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("manipulator name \"%s\" is not valid", GetName(), ORE_Failed);
    }
    RobotBasePtr probot(__probot);
    __pBase = probot->GetLink(_info._sBaseLinkName);
    __pEffector = probot->GetLink(_info._sEffectorLinkName);
    __varmdofindices.resize(0);
    __vgripperdofindices.resize(0);
    __pIkSolver.reset();
    __hashstructure.resize(0);
    __hashkinematicsstructure.resize(0);
    if( !__pBase || !__pEffector ) {
        RAVELOG_WARN(str(boost::format("manipulator %s has undefined base and end effector links %s, %s\n")%GetName()%_info._sBaseLinkName%_info._sEffectorLinkName));
        __armspec = ConfigurationSpecification();
    }
    else {
        vector<JointPtr> vjoints;
        std::vector<int> vmimicdofs;
        if( probot->GetChain(__pBase->GetIndex(),__pEffector->GetIndex(), vjoints) ) {
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
        }
        else {
            RAVELOG_WARN(str(boost::format("manipulator %s failed to find chain between %s and %s links\n")%GetName()%__pBase->GetName()%__pEffector->GetName()));
        }
    }

    // init the gripper dof indices
    std::vector<dReal> vClosingDirection;
    size_t iclosingdirection = 0;
    FOREACHC(itjointname,_info._vGripperJointNames) {
        JointPtr pjoint = probot->GetJoint(*itjointname);
        if( !pjoint ) {
            RAVELOG_WARN(str(boost::format("could not find gripper joint %s for manipulator %s")%*itjointname%GetName()));
            iclosingdirection++;
        }
        else {
            if( pjoint->GetDOFIndex() >= 0 ) {
                for(int i = 0; i < pjoint->GetDOF(); ++i) {
                    if( find(__varmdofindices.begin(), __varmdofindices.end(), pjoint->GetDOFIndex()+i) != __varmdofindices.end() ) {
                        RAVELOG_ERROR(str(boost::format("manipulator %s gripper dof %d is also part of arm dof! excluding from gripper...")%GetName()%(pjoint->GetDOFIndex()+i)));
                    }
                    else {
                        __vgripperdofindices.push_back(pjoint->GetDOFIndex()+i);
                        if( iclosingdirection < _info._vClosingDirection.size() ) {
                            vClosingDirection.push_back(_info._vClosingDirection[iclosingdirection++]);
                        }
                        else {
                            vClosingDirection.push_back(0);
                            RAVELOG_WARN(str(boost::format("manipulator %s closing direction not correct length, might get bad closing/release grasping")%GetName()));
                        }
                    }
                }
            }
            else {
                ++iclosingdirection;
                RAVELOG_WARN(str(boost::format("manipulator %s gripper joint %s is not active, so has no dof index. ignoring.")%GetName()%*itjointname));
            }
        }
    }
    _info._vClosingDirection.swap(vClosingDirection);
}

}
