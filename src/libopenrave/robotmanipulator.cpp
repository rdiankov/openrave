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

RobotBase::Manipulator::Manipulator(RobotBasePtr probot) : _vdirection(0,0,1), _probot(probot) {
}
RobotBase::Manipulator::~Manipulator() {
}

RobotBase::Manipulator::Manipulator(const RobotBase::Manipulator& r)
{
    *this = r;
    _pIkSolver.reset();
    if( _strIkSolver.size() > 0 ) {
        _pIkSolver = RaveCreateIkSolver(GetRobot()->GetEnv(), _strIkSolver);
    }
}

RobotBase::Manipulator::Manipulator(RobotBasePtr probot, const RobotBase::Manipulator& r)
{
    *this = r;
    _probot = probot;
    if( !!r.GetBase() ) {
        _pBase = probot->GetLinks().at(r.GetBase()->GetIndex());
    }
    if( !!r.GetEndEffector() ) {
        _pEndEffector = probot->GetLinks().at(r.GetEndEffector()->GetIndex());
    }
    _pIkSolver.reset();
    if( _strIkSolver.size() > 0 ) {
        _pIkSolver = RaveCreateIkSolver(probot->GetEnv(), _strIkSolver);
        // cannot call _pIkSolver->Init since this is the constructor...
    }
}

void RobotBase::Manipulator::SetLocalToolTransform(const Transform& t)
{
    _strIkSolver.resize(0);
    _pIkSolver.reset();
    _tLocalTool = t;
    GetRobot()->_ParametersChanged(Prop_RobotManipulatorTool);
    __hashkinematicsstructure.resize(0);
    __hashstructure.resize(0);
}

void RobotBase::Manipulator::SetName(const std::string& name)
{
    RobotBasePtr probot=GetRobot();
    FOREACHC(itmanip,probot->GetManipulators()) {
        if( *itmanip != shared_from_this() && name == (*itmanip)->GetName() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("manipulator name change '%s'->'%s' is colliding with other manipulator", _name%name, ORE_InvalidArguments);
        }
    }
    _name=name;
    probot->_ParametersChanged(Prop_RobotManipulatorName);
}

Transform RobotBase::Manipulator::GetTransform() const
{
    return _pEndEffector->GetTransform() * _tLocalTool;
}

bool RobotBase::Manipulator::SetIkSolver(IkSolverBasePtr iksolver)
{
    if( !iksolver ) {
        _pIkSolver.reset();
        return true;
    }

    if( iksolver->GetXMLId().size() == 0 ) {
        RAVELOG_WARN(str(boost::format("robot %s manip %s IkSolver XML is not initialized\n")%GetRobot()->GetName()%GetName()));
    }
    if( iksolver->Init(shared_from_this()) ) {
        _pIkSolver = iksolver;
        _strIkSolver = iksolver->GetXMLId();
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
    OPENRAVE_ASSERT_FORMAT(!!_pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(_probot)->GetName()%GetName(),ORE_Failed);
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    solution.resize(__varmdofindices.size());
    for(size_t i = 0; i < __varmdofindices.size(); ++i) {
        JointConstPtr pjoint = probot->GetJointFromDOFIndex(__varmdofindices[i]);
        solution[i] = pjoint->GetValue(__varmdofindices[i]-pjoint->GetDOFIndex());
    }
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    boost::shared_ptr< vector<dReal> > psolution(&solution, utils::null_deleter());
    return vFreeParameters.size() == 0 ? _pIkSolver->Solve(localgoal, solution, filteroptions, psolution) : _pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, psolution);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    return FindIKSolutions(goal, vector<dReal>(), solutions, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    OPENRAVE_ASSERT_FORMAT(!!_pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(_probot)->GetName()%GetName(),ORE_Failed);
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? _pIkSolver->SolveAll(localgoal,filteroptions,solutions) : _pIkSolver->SolveAll(localgoal,vFreeParameters,filteroptions,solutions);
}


bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, int filteroptions, IkReturnPtr ikreturn) const
{
    return FindIKSolution(goal, vector<dReal>(), filteroptions, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn) const
{
    OPENRAVE_ASSERT_FORMAT(!!_pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(_probot)->GetName()%GetName(),ORE_Failed);
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    vector<dReal> solution(__varmdofindices.size());
    for(size_t i = 0; i < __varmdofindices.size(); ++i) {
        JointConstPtr pjoint = probot->GetJointFromDOFIndex(__varmdofindices[i]);
        solution[i] = pjoint->GetValue(__varmdofindices[i]-pjoint->GetDOFIndex());
    }
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? _pIkSolver->Solve(localgoal, solution, filteroptions, ikreturn) : _pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const
{
    return FindIKSolutions(goal, vector<dReal>(), filteroptions, vikreturns);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const
{
    OPENRAVE_ASSERT_FORMAT(!!_pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(_probot)->GetName()%GetName(),ORE_Failed);
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? _pIkSolver->SolveAll(localgoal,filteroptions,vikreturns) : _pIkSolver->SolveAll(localgoal,vFreeParameters,filteroptions,vikreturns);
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
    case IKP_Direction3D: ikp.SetDirection3D(t.rotate(_vdirection)); break;
    case IKP_Ray4D: {
        ikp.SetRay4D(RAY(t.trans,t.rotate(_vdirection)));
        break;
    }
    case IKP_Lookat3D: {
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: Lookat3D type setting goal a distance of 1 from the origin.\n");
        Vector vdir = t.rotate(_vdirection);
        ikp.SetLookat3D(RAY(t.trans + vdir,vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(_vdirection)));
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
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(vglobaldirection.z));
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vglobaldirection.y,vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vglobaldirection.z,vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
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
    case IKP_Direction3D: ikp.SetDirection3D(t.rotate(_vdirection)); break;
    case IKP_Ray4D: {
        ikp.SetRay4D(RAY(t.trans,t.rotate(_vdirection)));
        break;
    }
    case IKP_Lookat3D: {
        // find the closest point to ikparam.GetLookat3D() to the current ray
        Vector vdir = t.rotate(_vdirection);
        ikp.SetLookat3D(RAY(t.trans + vdir*vdir.dot(ikparam.GetLookat3D()-t.trans),vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(_vdirection)));
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
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(vglobaldirection.z));
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vglobaldirection.y,vglobaldirection.x));
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
        ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vglobaldirection.z,vglobaldirection.y));
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        Vector vglobaldirection = t.rotate(_vdirection);
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
    // get all child links of the manipualtor
    RobotBasePtr probot(_probot);
    vjoints.resize(0);
    vector<dReal> lower,upper;
    vector<uint8_t> vhasjoint(probot->GetJoints().size(),false);
    int iattlink = _pEndEffector->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink )
            continue;
        if((__varmdofindices.size() > 0)&& !probot->DoesAffect(__varmdofindices[0],ilink) )
            continue;
        for(int idof = 0; idof < probot->GetDOF(); ++idof) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(idof);
            if( probot->DoesAffect(pjoint->GetJointIndex(),ilink) && !probot->DoesAffect(pjoint->GetJointIndex(),iattlink) ) {
                // only insert if its limits are different (ie, not a dummy joint)
                pjoint->GetLimits(lower,upper);
                for(int i = 0; i < pjoint->GetDOF(); ++i) {
                    if( lower[i] != upper[i] ) {
                        if( !vhasjoint[pjoint->GetJointIndex()] ) {
                            vjoints.push_back(pjoint);
                            vhasjoint[pjoint->GetJointIndex()] = true;
                        }
                        break;
                    }
                }
            }
        }
    }
}

void RobotBase::Manipulator::GetChildDOFIndices(std::vector<int>& vdofindices) const
{
    // get all child links of the manipualtor
    RobotBasePtr probot(_probot);
    vdofindices.resize(0);
    vector<uint8_t> vhasjoint(probot->GetJoints().size(),false);
    vector<dReal> lower,upper;
    int iattlink = _pEndEffector->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink )
            continue;
        if((__varmdofindices.size() > 0)&& !probot->DoesAffect(__varmdofindices[0],ilink) )
            continue;
        for(int idof = 0; idof < probot->GetDOF(); ++idof) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(idof);
            if( probot->DoesAffect(pjoint->GetJointIndex(),ilink) && !probot->DoesAffect(pjoint->GetJointIndex(),iattlink) ) {
                // only insert if its limits are different (ie, not a dummy joint)
                pjoint->GetLimits(lower,upper);
                for(int i = 0; i < pjoint->GetDOF(); ++i) {
                    if( lower[i] != upper[i] ) {
                        if( !vhasjoint[pjoint->GetJointIndex()] ) {
                            vhasjoint[pjoint->GetJointIndex()] = true;
                            int idofbase = pjoint->GetDOFIndex();
                            for(int idof = 0; idof < pjoint->GetDOF(); ++idof)
                                vdofindices.push_back(idofbase+idof);
                        }
                        break;
                    }
                }
            }
        }
    }
}

void RobotBase::Manipulator::GetChildLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(_probot);
    // get all child links of the manipualtor
    vlinks.resize(0);
    _pEndEffector->GetRigidlyAttachedLinks(vlinks);
    int iattlink = _pEndEffector->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,__varmdofindices) {
            if( !probot->DoesAffect(*itarmjoint,ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }
        for(int ijoint = 0; ijoint < probot->GetDOF(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                vlinks.push_back(*itlink);
                break;
            }
        }
    }
}

bool RobotBase::Manipulator::IsChildLink(LinkConstPtr plink) const
{
    if( _pEndEffector->IsRigidlyAttached(plink) ) {
        return true;
    }

    RobotBasePtr probot(_probot);
    // get all child links of the manipualtor
    int iattlink = _pEndEffector->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,__varmdofindices) {
            if( !probot->DoesAffect(*itarmjoint,ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }
        for(int ijoint = 0; ijoint < probot->GetDOF(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                return true;
            }
        }
    }
    return false;
}

void RobotBase::Manipulator::GetIndependentLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(_probot);
    FOREACHC(itlink, probot->GetLinks()) {
        bool bAffected = false;
        FOREACHC(itindex,__varmdofindices) {
            if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        FOREACHC(itindex,__vgripperdofindices) {
            if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
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
    RobotBasePtr probot(_probot);
    Transform toldEE = GetTransform();
    Transform tdelta = tEE*toldEE.inverse();
    // get all child links of the manipualtor
    int iattlink = _pEndEffector->GetIndex();
    vector<LinkPtr> vattachedlinks;
    _pEndEffector->GetRigidlyAttachedLinks(vattachedlinks);
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
        FOREACHC(itarmjoint,__varmdofindices) {
            if( !probot->DoesAffect(*itarmjoint,ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }
        for(int ijoint = 0; ijoint < probot->GetDOF(); ++ijoint) {
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

bool RobotBase::Manipulator::CheckEndEffectorCollision(const IkParameterization& ikparam, CollisionReportPtr report) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return CheckEndEffectorCollision(ikparam.GetTransform6D());
    }
    RobotBasePtr probot = GetRobot();
    OPENRAVE_ASSERT_OP_FORMAT((int)GetArmIndices().size(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!_pIkSolver, "manipulator %s:%s does not have an IK solver set",probot->GetName()%GetName(),ORE_Failed);
    OPENRAVE_ASSERT_FORMAT(_pIkSolver->Supports(ikparam.GetType()),"manipulator %s:%s ik solver %s does not support ik type 0x%x",probot->GetName()%GetName()%_pIkSolver->GetXMLId()%ikparam.GetType(),ORE_InvalidState);
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*ikparam;
    }
    else {
        localgoal=ikparam;
    }
    vector<dReal> vsolution;
    boost::shared_ptr< vector<dReal> > psolution(&vsolution, utils::null_deleter());
    // only care about the end effector position, so disable all time consuming options. still leave the custom options in case the user wants to call some custom stuff?
    if( !_pIkSolver->Solve(localgoal, vector<dReal>(), IKFO_IgnoreSelfCollisions|IKFO_IgnoreJointLimits,psolution) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to find ik solution for type 0x%x",ikparam.GetType(),ORE_InvalidArguments);
    }
    RobotStateSaver saver(probot);
    probot->SetActiveDOFs(GetArmIndices());
    probot->SetActiveDOFValues(vsolution,false);
    return CheckEndEffectorCollision(GetTransform(),report);
}

bool RobotBase::Manipulator::CheckIndependentCollision(CollisionReportPtr report) const
{
    RobotBasePtr probot(_probot);
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    FOREACHC(itlink, probot->GetLinks()) {
        if( !(*itlink)->IsEnabled() ) {
            continue;
        }
        bool bAffected = false;
        FOREACHC(itindex,__varmdofindices) {
            if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        if( !bAffected ) {
            FOREACHC(itindex,__vgripperdofindices) {
                if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
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
    RobotBasePtr probot(_probot);
    KinBody::LinkPtr plink = probot->IsGrabbing(pbody);
    if( !!plink ) {
        if((plink == _pEndEffector)||(plink == _pBase)) {
            return true;
        }
        int iattlink = _pEndEffector->GetIndex();
        FOREACHC(itlink, probot->GetLinks()) {
            int ilink = (*itlink)->GetIndex();
            if( ilink == iattlink )
                continue;
            // gripper needs to be affected by all joints
            bool bGripperLink = true;
            FOREACHC(itarmjoint,__varmdofindices) {
                if( !probot->DoesAffect(*itarmjoint,ilink) ) {
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
    RobotBasePtr probot(_probot);
    probot->ComputeJacobianTranslation(_pEndEffector->GetIndex(), _pEndEffector->GetTransform() * _tLocalTool.trans, jacobian, __varmdofindices);
}

void RobotBase::Manipulator::CalculateJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][__varmdofindices.size()]);
    if( __varmdofindices.size() == 0 ) {
        return;
    }
    RobotBasePtr probot(_probot);
    std::vector<dReal> vjacobian;
    probot->ComputeJacobianTranslation(_pEndEffector->GetIndex(), _pEndEffector->GetTransform() * _tLocalTool.trans, vjacobian, __varmdofindices);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*__varmdofindices.size());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+__varmdofindices.size(),itdst->begin());
        itsrc += __varmdofindices.size();
    }
}

void RobotBase::Manipulator::CalculateRotationJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(_probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveRotationJacobian(_pEndEffector->GetIndex(),quatMultiply(_pEndEffector->GetTransform().rot, _tLocalTool.rot),jacobian);
}

void RobotBase::Manipulator::CalculateRotationJacobian(boost::multi_array<dReal,2>& jacobian) const
{
    RobotBasePtr probot(_probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveRotationJacobian(_pEndEffector->GetIndex(),quatMultiply(_pEndEffector->GetTransform().rot, _tLocalTool.rot),jacobian);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(_probot);
    probot->ComputeJacobianAxisAngle(_pEndEffector->GetIndex(), jacobian, __varmdofindices);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][__varmdofindices.size()]);
    if( __varmdofindices.size() == 0 ) {
        return;
    }
    RobotBasePtr probot(_probot);
    std::vector<dReal> vjacobian;
    probot->ComputeJacobianAxisAngle(_pEndEffector->GetIndex(), vjacobian, __varmdofindices);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*__varmdofindices.size());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+__varmdofindices.size(),itdst->begin());
        itsrc += __varmdofindices.size();
    }
}

void RobotBase::Manipulator::serialize(std::ostream& o, int options) const
{
    if( options & SO_RobotManipulators ) {
        o << (!_pBase ? -1 : _pBase->GetIndex()) << " " << (!_pEndEffector ? -1 : _pEndEffector->GetIndex()) << " ";
        // don't include __varmdofindices and __vgripperdofindices since they are generated from the data
        o << _vgripperjointnames.size() << " ";
        FOREACHC(it,_vgripperjointnames) {
            o << *it << " ";
        }
        FOREACHC(it,_vClosingDirection) {
            SerializeRound(o,*it);
        }
        SerializeRound(o,_tLocalTool);
    }
    if( options & SO_Kinematics ) {
        RobotBasePtr probot(_probot);
        KinBody::KinBodyStateSaver saver(probot,Save_LinkTransformation);
        vector<dReal> vzeros(probot->GetDOF(),0);
        probot->SetDOFValues(vzeros,Transform(),true);
        Transform tbaseinv;
        if( !!_pBase ) {
            tbaseinv = _pBase->GetTransform().inverse();
        }
        if( !_pEndEffector ) {
            SerializeRound(o,tbaseinv * _tLocalTool);
        }
        else {
            SerializeRound(o,tbaseinv * GetTransform());
        }
        o << __varmdofindices.size() << " ";
        FOREACHC(it,__varmdofindices) {
            JointPtr pjoint = probot->GetJointFromDOFIndex(*it);
            o << pjoint->GetType() << " ";
            SerializeRound3(o,tbaseinv*pjoint->GetAnchor());
            SerializeRound3(o,tbaseinv.rotate(pjoint->GetAxis(*it-pjoint->GetDOFIndex())));
        }
        // the arm might be dependent on mimic joints, so recompute the chain and output the equations
        std::vector<JointPtr> vjoints;
        if( probot->GetChain(GetBase()->GetIndex(),GetEndEffector()->GetIndex(), vjoints) ) {
            int index = 0;
            FOREACH(it,vjoints) {
                if( !(*it)->IsStatic() && (*it)->IsMimic() ) {
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        if( (*it)->IsMimic(i) ) {
                            o << "mimic " << index << " ";
                            for(int ieq = 0; ieq < 3; ++ieq) {
                                o << (*it)->GetMimicEquation(i,ieq) << " ";
                            }
                        }
                    }
                }
                ++index;
            }
        }
    }
    if( options & (SO_Kinematics|SO_RobotManipulators) ) {
        SerializeRound3(o,_vdirection);
    }
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

}
