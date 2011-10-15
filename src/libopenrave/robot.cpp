// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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

#define CHECK_INTERNAL_COMPUTATION { \
        if( _nHierarchyComputed != 2 ) { \
            throw OPENRAVE_EXCEPTION_FORMAT0("joint hierarchy needs to be computed (is body added to environment?)", ORE_Failed); \
        } \
} \

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
    }
}

void RobotBase::Manipulator::SetLocalToolTransform(const Transform& t)
{
    _strIkSolver.resize(0);
    _pIkSolver.reset();
    _tLocalTool = t;
    RobotBasePtr probot = GetRobot();
    probot->_ParametersChanged(Prop_RobotManipulatorTool);
    __hashkinematicsstructure.resize(0);
    __hashstructure.resize(0);
}

Transform RobotBase::Manipulator::GetTransform() const
{
    return _pEndEffector->GetTransform() * _tLocalTool;
}

bool RobotBase::Manipulator::SetIkSolver(IkSolverBasePtr iksolver)
{
    _pIkSolver = iksolver;
    if( !!_pIkSolver ) {
        _strIkSolver = _pIkSolver->GetXMLId();
        return _pIkSolver->Init(shared_from_this());
    }
    return true;
}

int RobotBase::Manipulator::GetNumFreeParameters() const
{
    return !_pIkSolver ? 0 : _pIkSolver->GetNumFreeParameters();
}

bool RobotBase::Manipulator::GetFreeParameters(std::vector<dReal>& vFreeParameters) const
{
    return !_pIkSolver ? false : _pIkSolver->GetFreeParameters(vFreeParameters);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, vector<dReal>& solution, int filteroptions) const
{
    return FindIKSolution(goal, vector<dReal>(), solution, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, vector<dReal>& solution, int filteroptions) const
{
    if( !_pIkSolver ) {
        throw OPENRAVE_EXCEPTION_FORMAT("manipulator %s:%s does not have an IK solver set",RobotBasePtr(_probot)->GetName()%GetName(),ORE_Failed);
    }
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    vector<dReal> temp;
    probot->GetDOFValues(temp);
    solution.resize(__varmdofindices.size());
    for(size_t i = 0; i < __varmdofindices.size(); ++i) {
        solution[i] = temp[__varmdofindices[i]];
    }
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    boost::shared_ptr< vector<dReal> > psolution(&solution, null_deleter());
    return vFreeParameters.size() == 0 ? _pIkSolver->Solve(localgoal, solution, filteroptions, psolution) : _pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, psolution);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    return FindIKSolutions(goal, vector<dReal>(), solutions, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    if( !_pIkSolver ) {
        throw OPENRAVE_EXCEPTION_FORMAT("manipulator %s:%s does not have an IK solver set", RobotBasePtr(_probot)->GetName()%GetName(),ORE_Failed);
    }
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!_pBase ) {
        localgoal = _pBase->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? _pIkSolver->Solve(localgoal,filteroptions,solutions) : _pIkSolver->Solve(localgoal,vFreeParameters,filteroptions,solutions);
}

IkParameterization RobotBase::Manipulator::GetIkParameterization(IkParameterizationType iktype) const
{
    IkParameterization ikp;
    switch(iktype) {
    case IKP_Transform6D: ikp.SetTransform6D(GetTransform()); break;
    case IKP_Rotation3D: ikp.SetRotation3D(GetTransform().rot); break;
    case IKP_Translation3D: ikp.SetTranslation3D(GetTransform().trans); break;
    case IKP_Direction3D: ikp.SetDirection3D(GetTransform().rotate(_vdirection)); break;
    case IKP_Ray4D: {
        Transform t = GetTransform();
        ikp.SetRay4D(RAY(t.trans,t.rotate(_vdirection)));
        break;
    }
    case IKP_Lookat3D: {
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: Lookat3D type setting goal a distance of 1 from the origin.\n");
        Transform t = GetTransform();
        Vector vdir = t.rotate(_vdirection);
        ikp.SetLookat3D(RAY(t.trans + vdir,vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        Transform t = GetTransform();
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(_vdirection)));
        break;
    }
    case IKP_TranslationXY2D: {
        ikp.SetTranslationXY2D(GetTransform().trans);
        break;
    }
    case IKP_TranslationXYOrientation3D: {
        Transform t = GetTransform();
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),GetTransform().rot).first;
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,zangle));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: TranslationLocalGlobal6D type setting local translation to (0,0,0).\n");
        ikp.SetTranslationLocalGlobal6D(Vector(0,0,0),GetTransform().trans); break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("invalid ik type 0x%x",iktype,ORE_InvalidArguments);
    }
    return ikp;
}

IkParameterization RobotBase::Manipulator::GetIkParameterization(const IkParameterization& ikparam) const
{
    IkParameterization ikp;
    switch(ikparam.GetType()) {
    case IKP_Transform6D: ikp.SetTransform6D(GetTransform()); break;
    case IKP_Rotation3D: ikp.SetRotation3D(GetTransform().rot); break;
    case IKP_Translation3D: ikp.SetTranslation3D(GetTransform().trans); break;
    case IKP_Direction3D: ikp.SetDirection3D(GetTransform().rotate(_vdirection)); break;
    case IKP_Ray4D: {
        Transform t = GetTransform();
        ikp.SetRay4D(RAY(t.trans,t.rotate(_vdirection)));
        break;
    }
    case IKP_Lookat3D: {
        // find the closest point to ikparam.GetLookat3D() to the current ray
        Transform t = GetTransform();
        Vector vdir = t.rotate(_vdirection);
        ikp.SetLookat3D(RAY(t.trans + vdir*vdir.dot(ikparam.GetLookat3D()-t.trans),vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        Transform t = GetTransform();
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(_vdirection)));
        break;
    }
    case IKP_TranslationXY2D: {
        ikp.SetTranslationXY2D(GetTransform().trans);
        break;
    }
    case IKP_TranslationXYOrientation3D: {
        Transform t = GetTransform();
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),GetTransform().rot).first;
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,zangle));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        Vector localtrans = ikparam.GetTranslationLocalGlobal6D().first;
        ikp.SetTranslationLocalGlobal6D(localtrans,GetTransform() * localtrans);
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
                if( itgrabbed->plinkrobot == *itlink ) {
                    if( vbodyexcluded.empty() ) {
                        vbodyexcluded.push_back(KinBodyConstPtr(probot));
                    }
                    KinBodyPtr pbody = itgrabbed->pbody.lock();
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

void RobotBase::Manipulator::CalculateJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    RobotBasePtr probot(_probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveJacobian(_pEndEffector->GetIndex(),_pEndEffector->GetTransform() * _tLocalTool.trans,mjacobian);
}

void RobotBase::Manipulator::CalculateRotationJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    RobotBasePtr probot(_probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveRotationJacobian(_pEndEffector->GetIndex(),quatMultiply(_pEndEffector->GetTransform().rot, _tLocalTool.rot),mjacobian);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    RobotBasePtr probot(_probot);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(__varmdofindices);
    probot->CalculateActiveAngularVelocityJacobian(_pEndEffector->GetIndex(),mjacobian);
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
        __hashstructure = GetMD5HashString(ss.str());
    }
    return __hashstructure;
}

const std::string& RobotBase::Manipulator::GetKinematicsStructureHash() const
{
    if( __hashkinematicsstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics);
        __hashkinematicsstructure = GetMD5HashString(ss.str());
    }
    return __hashkinematicsstructure;
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot) : _probot(probot)
{
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot, const AttachedSensor& sensor,int cloningoptions)
{
    *this = sensor;
    _probot = probot;
    psensor.reset();
    pdata.reset();
    pattachedlink.reset();
    if( (cloningoptions&Clone_Sensors) && !!sensor.psensor ) {
        psensor = RaveCreateSensor(probot->GetEnv(), sensor.psensor->GetXMLId());
        if( !!psensor ) {
            psensor->Clone(sensor.psensor,cloningoptions);
            if( !!psensor ) {
                pdata = psensor->CreateSensorData();
            }
        }
    }
    int index = LinkPtr(sensor.pattachedlink)->GetIndex();
    if((index >= 0)&&(index < (int)probot->GetLinks().size())) {
        pattachedlink = probot->GetLinks().at(index);
    }
}

RobotBase::AttachedSensor::~AttachedSensor()
{
}

SensorBase::SensorDataPtr RobotBase::AttachedSensor::GetData() const
{
    if( psensor->GetSensorData(pdata) ) {
        return pdata;
    }
    return SensorBase::SensorDataPtr();
}

void RobotBase::AttachedSensor::SetRelativeTransform(const Transform& t)
{
    trelative = t;
    GetRobot()->_ParametersChanged(Prop_SensorPlacement);
}

void RobotBase::AttachedSensor::serialize(std::ostream& o, int options) const
{
    o << (pattachedlink.expired() ? -1 : LinkPtr(pattachedlink)->GetIndex()) << " ";
    SerializeRound(o,trelative);
    o << (!pdata ? -1 : pdata->GetType()) << " ";
    // it is also important to serialize some of the geom parameters for the sensor (in case models are cached to it)
    if( !!psensor ) {
        SensorBase::SensorGeometryPtr prawgeom = psensor->GetSensorGeometry();
        if( !!prawgeom ) {
            switch(prawgeom->GetType()) {
            case SensorBase::ST_Laser: {
                boost::shared_ptr<SensorBase::LaserGeomData> pgeom = boost::static_pointer_cast<SensorBase::LaserGeomData>(prawgeom);
                o << pgeom->min_angle[0] << " " << pgeom->max_angle[0] << " " << pgeom->resolution[0] << " " << pgeom->max_range << " ";
                break;
            }
            case SensorBase::ST_Camera: {
                boost::shared_ptr<SensorBase::CameraGeomData> pgeom = boost::static_pointer_cast<SensorBase::CameraGeomData>(prawgeom);
                o << pgeom->KK.fx << " " << pgeom->KK.fy << " " << pgeom->KK.cx << " " << pgeom->KK.cy << " " << pgeom->width << " " << pgeom->height << " ";
                break;
            }
            default:
                // don't support yet
                break;
            }
        }
    }
}

const std::string& RobotBase::AttachedSensor::GetStructureHash() const
{
    if( __hashstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_RobotSensors);
        __hashstructure = GetMD5HashString(ss.str());
    }
    return __hashstructure;
}

RobotBase::RobotStateSaver::RobotStateSaver(RobotBasePtr probot, int options) : KinBodyStateSaver(probot, options), _probot(probot)
{
    if( _options & Save_ActiveDOF ) {
        vactivedofs = _probot->GetActiveDOFIndices();
        affinedofs = _probot->GetAffineDOF();
        rotationaxis = _probot->GetAffineRotationAxis();
    }
    if( _options & Save_ActiveManipulator ) {
        nActiveManip = _probot->_nActiveManip;
    }
    if( _options & Save_GrabbedBodies ) {
        _vGrabbedBodies = _probot->_vGrabbedBodies;
    }
}

RobotBase::RobotStateSaver::~RobotStateSaver()
{
    _RestoreRobot();
}

void RobotBase::RobotStateSaver::Restore()
{
    _RestoreRobot();
    KinBodyStateSaver::Restore();
}

void RobotBase::RobotStateSaver::_RestoreRobot()
{
    if( _options & Save_ActiveDOF ) {
        _probot->SetActiveDOFs(vactivedofs, affinedofs, rotationaxis);
    }
    if( _options & Save_ActiveManipulator ) {
        _probot->_nActiveManip = nActiveManip;
    }
    if( _options & Save_GrabbedBodies ) {
        // have to release all grabbed first
        _probot->ReleaseAllGrabbed();
        BOOST_ASSERT(_probot->_vGrabbedBodies.size()==0);
        FOREACH(itgrabbed, _vGrabbedBodies) {
            KinBodyPtr pbody = itgrabbed->pbody.lock();
            if( !!pbody ) {
                _probot->_AttachBody(pbody);
                _probot->_vGrabbedBodies.push_back(*itgrabbed);
            }
        }
    }
}

RobotBase::RobotBase(EnvironmentBasePtr penv) : KinBody(PT_Robot, penv)
{
    _nAffineDOFs = 0;
    _nActiveDOF = -1;
    vActvAffineRotationAxis = Vector(0,0,1);

    _nActiveManip = 0;
    _vecManipulators.reserve(16); // make sure to reseve enough, otherwise pIkSolver pointer might get messed up when resizing

    //set limits for the affine DOFs
    _vTranslationLowerLimits = Vector(-100,-100,-100);
    _vTranslationUpperLimits = Vector(100,100,100);
    _vTranslationMaxVels = Vector(1.0f,1.0f,1.0f);
    _vTranslationResolutions = Vector(0.001f,0.001f,0.001f);
    _vTranslationWeights = Vector(2.0f,2.0f,2.0f);

    _vRotationAxisLowerLimits = Vector(-PI,-PI,-PI,-PI);
    _vRotationAxisUpperLimits = Vector(PI,PI,PI,PI);
    _vRotationAxisMaxVels = Vector(0.07f,0.07f,0.07f,0.07f);
    _vRotationAxisResolutions = Vector(0.01f,0.01f,0.01f,0.01f);
    _vRotationAxisWeights = Vector(2.0f,2.0f,2.0f,2.0f);

    _vRotation3DLowerLimits = Vector(-10000,-10000,-10000);
    _vRotation3DUpperLimits = Vector(10000,10000,10000);
    _vRotation3DMaxVels = Vector(0.07f,0.07f,0.07f);
    _vRotation3DResolutions = Vector(0.01f,0.01f,0.01f);
    _vRotation3DWeights = Vector(1.0f,1.0f,1.0f);

    _vRotationQuatLimitStart = Vector(1,0,0,0);
    _fQuatLimitMaxAngle = PI;
    _fQuatMaxAngleVelocity = 1.0;
    _fQuatAngleResolution = 0.01f;
    _fQuatAngleWeight = 0.4f;
}

RobotBase::~RobotBase()
{
    Destroy();
}

void RobotBase::Destroy()
{
    ReleaseAllGrabbed();
    _vecManipulators.clear();
    _vecSensors.clear();
    SetController(ControllerBasePtr(),std::vector<int>(),0);

    KinBody::Destroy();
}

bool RobotBase::SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int nControlTransformation)
{
    RAVELOG_DEBUG("default robot doesn't not support setting controllers (try GenericRobot)\n");
    return false;
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, bool bCheckLimits)
{
    KinBody::SetDOFValues(vJointValues, bCheckLimits);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transbase, bool bCheckLimits)
{
    KinBody::SetDOFValues(vJointValues, transbase, bCheckLimits); // should call RobotBase::SetDOFValues, so no need to upgrade grabbed bodies, attached sensors
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& vbodies)
{
    KinBody::SetLinkTransformations(vbodies);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetTransform(const Transform& trans)
{
    KinBody::SetTransform(trans);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::_UpdateGrabbedBodies()
{
    //RAVELOG_VERBOSE("update grabbed objects\n");
    vector<Grabbed>::iterator itbody;
    FORIT(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody = itbody->pbody.lock();
        if( !!pbody ) {
            pbody->SetTransform(itbody->plinkrobot->GetTransform() * itbody->troot);
        }
        else {
            RAVELOG_DEBUG(str(boost::format("erasing invaliding grabbed body from %s")%GetName()));
            itbody = _vGrabbedBodies.erase(itbody);
        }
    }
}

void RobotBase::_UpdateAttachedSensors()
{
    FOREACH(itsensor, _vecSensors) {
        if( !!(*itsensor)->psensor && !(*itsensor)->pattachedlink.expired() )
            (*itsensor)->psensor->SetTransform(LinkPtr((*itsensor)->pattachedlink)->GetTransform()*(*itsensor)->trelative);
    }
}

void RobotBase::SetAffineTranslationLimits(const Vector& lower, const Vector& upper)
{
    _vTranslationLowerLimits = lower;
    _vTranslationUpperLimits = upper;
}

void RobotBase::SetAffineRotationAxisLimits(const Vector& lower, const Vector& upper)
{
    _vRotationAxisLowerLimits = lower;
    _vRotationAxisUpperLimits = upper;
}

void RobotBase::SetAffineRotation3DLimits(const Vector& lower, const Vector& upper)
{
    _vRotation3DLowerLimits = lower;
    _vRotation3DUpperLimits = upper;
}

void RobotBase::SetAffineRotationQuatLimits(const Vector& quatangle)
{
    _fQuatLimitMaxAngle = RaveSqrt(quatangle.lengthsqr4());
    if( _fQuatLimitMaxAngle > 0 ) {
        _vRotationQuatLimitStart = quatangle * (1/_fQuatLimitMaxAngle);
    }
    else {
        _vRotationQuatLimitStart = GetTransform().rot;
    }
}

void RobotBase::SetAffineTranslationMaxVels(const Vector& vels)
{
    _vTranslationMaxVels = vels;
}

void RobotBase::SetAffineRotationAxisMaxVels(const Vector& vels)
{
    _vRotationAxisMaxVels = vels;
}

void RobotBase::SetAffineRotation3DMaxVels(const Vector& vels)
{
    _vRotation3DMaxVels = vels;
}

void RobotBase::SetAffineRotationQuatMaxVels(dReal anglevelocity)
{
    _fQuatMaxAngleVelocity = anglevelocity;
}

void RobotBase::SetAffineTranslationResolution(const Vector& resolution)
{
    _vTranslationResolutions = resolution;
}

void RobotBase::SetAffineRotationAxisResolution(const Vector& resolution)
{
    _vRotationAxisResolutions = resolution;
}

void RobotBase::SetAffineRotation3DResolution(const Vector& resolution)
{
    _vRotation3DResolutions = resolution;
}

void RobotBase::SetAffineRotationQuatResolution(dReal angleresolution)
{
    _fQuatAngleResolution = angleresolution;
}

void RobotBase::SetAffineTranslationWeights(const Vector& weights)
{
    _vTranslationWeights = weights;
}

void RobotBase::SetAffineRotationAxisWeights(const Vector& weights)
{
    _vRotationAxisWeights = weights;
}

void RobotBase::SetAffineRotation3DWeights(const Vector& weights)
{
    _vRotation3DWeights = weights;
}

void RobotBase::SetAffineRotationQuatWeights(dReal angleweight)
{
    _fQuatAngleWeight = angleweight;
}

void RobotBase::GetAffineTranslationLimits(Vector& lower, Vector& upper) const
{
    lower = _vTranslationLowerLimits;
    upper = _vTranslationUpperLimits;
}

void RobotBase::GetAffineRotationAxisLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotationAxisLowerLimits;
    upper = _vRotationAxisUpperLimits;
}

void RobotBase::GetAffineRotation3DLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotation3DLowerLimits;
    upper = _vRotation3DUpperLimits;
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask, const Vector& vRotationAxis)
{
    vActvAffineRotationAxis = vRotationAxis;
    SetActiveDOFs(vJointIndices,nAffineDOFBitmask);
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask)
{
    FOREACHC(itj, vJointIndices) {
        if((*itj < 0)||(*itj >= (int)GetDOF())) {
            throw OPENRAVE_EXCEPTION_FORMAT("bad indices %d",*itj,ORE_InvalidArguments);
        }
    }
    // only reset the cache if the dof values are different
    if( _vActiveDOFIndices.size() != vJointIndices.size() ) {
        _nNonAdjacentLinkCache &= ~AO_ActiveDOFs;
    }
    else {
        for(size_t i = 0; i < vJointIndices.size(); ++i) {
            if( _vActiveDOFIndices[i] != vJointIndices[i] ) {
                _nNonAdjacentLinkCache &= ~AO_ActiveDOFs;
                break;
            }
        }
    }
    _vActiveDOFIndices = vJointIndices;
    _nAffineDOFs = nAffineDOFBitmask;
    _nActiveDOF = vJointIndices.size() + RaveGetAffineDOF(_nAffineDOFs);

    int offset = 0;
    _activespec._vgroups.resize(0);
    if( GetActiveDOFIndices().size() > 0 ) {
        ConfigurationSpecification::Group group;
        stringstream ss;
        ss << "joint_values " << GetName();
        FOREACHC(it,GetActiveDOFIndices()) {
            ss << " " << *it;
        }
        group.name = ss.str();
        group.dof = (int)GetActiveDOFIndices().size();
        group.offset = offset;
        group.interpolation = "linear";
        offset += group.dof;
        _activespec._vgroups.push_back(group);
    }
    if( GetAffineDOF() > 0 ) {
        ConfigurationSpecification::Group group;
        group.name = str(boost::format("affine_transform %s %d")%GetName()%GetAffineDOF());
        group.offset = offset;
        group.dof = RaveGetAffineDOF(GetAffineDOF());
        group.interpolation = "linear";
        _activespec._vgroups.push_back(group);
    }

    _ParametersChanged(Prop_RobotActiveDOFs);
}

void RobotBase::SetActiveDOFValues(const std::vector<dReal>& values, bool bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFValues(values,bCheckLimits);
        return;
    }
    if( (int)values.size() < GetActiveDOF() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("not enough values %d<%d",values.size()%GetActiveDOF(),ORE_InvalidArguments);
    }

    Transform t;
    if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
        t = GetTransform();
        RaveGetTransformFromAffineDOFValues(t, values.begin()+_vActiveDOFIndices.size(),_nAffineDOFs,vActvAffineRotationAxis);
        if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
            t.rot = quatMultiply(_vRotationQuatLimitStart, t.rot);
        }
        if( _vActiveDOFIndices.size() == 0 ) {
            SetTransform(t);
        }
    }

    if( _vActiveDOFIndices.size() > 0 ) {
        GetDOFValues(_vTempRobotJoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            _vTempRobotJoints[_vActiveDOFIndices[i]] = values[i];
        }
        if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
            SetDOFValues(_vTempRobotJoints, t, bCheckLimits);
        }
        else {
            SetDOFValues(_vTempRobotJoints, bCheckLimits);
        }
    }
}

void RobotBase::GetActiveDOFValues(std::vector<dReal>& values) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFValues(values);
        return;
    }

    values.resize(GetActiveDOF());
    if( values.size() == 0 ) {
        return;
    }
    vector<dReal>::iterator itvalues = values.begin();
    if( _vActiveDOFIndices.size() != 0 ) {
        GetDOFValues(_vTempRobotJoints);
        FOREACHC(it, _vActiveDOFIndices) {
            *itvalues++ = _vTempRobotJoints[*it];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    Transform t = GetTransform();
    if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        t.rot = quatMultiply(quatInverse(_vRotationQuatLimitStart), t.rot);
    }
    RaveGetAffineDOFValuesFromTransform(itvalues,GetTransform(),_nAffineDOFs,vActvAffineRotationAxis);
}

void RobotBase::SetActiveDOFVelocities(const std::vector<dReal>& velocities, bool bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFVelocities(velocities);
        return;
    }

    Vector linearvel, angularvel;
    if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = &velocities[_vActiveDOFIndices.size()];

        _veclinks.at(0)->GetVelocity(linearvel, angularvel);

        if( _nAffineDOFs & OpenRAVE::DOF_X ) linearvel.x = *pAffineValues++;
        if( _nAffineDOFs & OpenRAVE::DOF_Y ) linearvel.y = *pAffineValues++;
        if( _nAffineDOFs & OpenRAVE::DOF_Z ) linearvel.z = *pAffineValues++;
        if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
            angularvel = vActvAffineRotationAxis * *pAffineValues++;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
            angularvel.x = *pAffineValues++;
            angularvel.y = *pAffineValues++;
            angularvel.z = *pAffineValues++;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("quaternions not supported",ORE_InvalidArguments);
        }

        if( _vActiveDOFIndices.size() == 0 ) {
            SetVelocity(linearvel, angularvel);
        }
    }

    if( _vActiveDOFIndices.size() > 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        std::vector<dReal>::const_iterator itvel = velocities.begin();
        FOREACHC(it, _vActiveDOFIndices) {
            _vTempRobotJoints[*it] = *itvel++;
        }
        if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
            SetDOFVelocities(_vTempRobotJoints,linearvel,angularvel,bCheckLimits);
        }
        else {
            SetDOFVelocities(_vTempRobotJoints,bCheckLimits);
        }
    }
}

void RobotBase::GetActiveDOFVelocities(std::vector<dReal>& velocities) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFVelocities(velocities);
        return;
    }

    velocities.resize(GetActiveDOF());
    if( velocities.size() == 0 )
        return;
    dReal* pVelocities = &velocities[0];
    if( _vActiveDOFIndices.size() != 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        FOREACHC(it, _vActiveDOFIndices) {
            *pVelocities++ = _vTempRobotJoints[*it];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    Vector linearvel, angularvel;
    _veclinks.at(0)->GetVelocity(linearvel, angularvel);

    if( _nAffineDOFs & OpenRAVE::DOF_X ) *pVelocities++ = linearvel.x;
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) *pVelocities++ = linearvel.y;
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) *pVelocities++ = linearvel.z;
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {

        *pVelocities++ = vActvAffineRotationAxis.dot3(angularvel);
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pVelocities++ = angularvel.x;
        *pVelocities++ = angularvel.y;
        *pVelocities++ = angularvel.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("quaternions not supported",ORE_InvalidArguments);
    }
}

void RobotBase::GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const
{
    lower.resize(GetActiveDOF());
    upper.resize(GetActiveDOF());
    if( GetActiveDOF() == 0 ) {
        return;
    }
    dReal* pLowerLimit = &lower[0];
    dReal* pUpperLimit = &upper[0];
    vector<dReal> alllower,allupper;

    if( _nAffineDOFs == 0 ) {
        if( _nActiveDOF < 0 ) {
            GetDOFLimits(lower,upper);
            return;
        }
        else {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, _vActiveDOFIndices) {
                *pLowerLimit++ = alllower[*it];
                *pUpperLimit++ = allupper[*it];
            }
        }
    }
    else {
        if( _vActiveDOFIndices.size() > 0 ) {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, _vActiveDOFIndices) {
                *pLowerLimit++ = alllower[*it];
                *pUpperLimit++ = allupper[*it];
            }
        }

        if( _nAffineDOFs & OpenRAVE::DOF_X ) {
            *pLowerLimit++ = _vTranslationLowerLimits.x;
            *pUpperLimit++ = _vTranslationUpperLimits.x;
        }
        if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
            *pLowerLimit++ = _vTranslationLowerLimits.y;
            *pUpperLimit++ = _vTranslationUpperLimits.y;
        }
        if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
            *pLowerLimit++ = _vTranslationLowerLimits.z;
            *pUpperLimit++ = _vTranslationUpperLimits.z;
        }

        if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
            *pLowerLimit++ = _vRotationAxisLowerLimits.x;
            *pUpperLimit++ = _vRotationAxisUpperLimits.x;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
            *pLowerLimit++ = _vRotation3DLowerLimits.x;
            *pLowerLimit++ = _vRotation3DLowerLimits.y;
            *pLowerLimit++ = _vRotation3DLowerLimits.z;
            *pUpperLimit++ = _vRotation3DUpperLimits.x;
            *pUpperLimit++ = _vRotation3DUpperLimits.y;
            *pUpperLimit++ = _vRotation3DUpperLimits.z;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
            // this is actually difficult to do correctly...
            dReal fsin = RaveSin(_fQuatLimitMaxAngle);
            *pLowerLimit++ = RaveCos(_fQuatLimitMaxAngle);
            *pLowerLimit++ = -fsin;
            *pLowerLimit++ = -fsin;
            *pLowerLimit++ = -fsin;
            *pUpperLimit++ = 1;
            *pUpperLimit++ = fsin;
            *pUpperLimit++ = fsin;
            *pUpperLimit++ = fsin;
        }
    }
}

void RobotBase::GetActiveDOFResolutions(std::vector<dReal>& resolution) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFResolutions(resolution);
        return;
    }

    resolution.resize(GetActiveDOF());
    if( resolution.size() == 0 ) {
        return;
    }
    dReal* pResolution = &resolution[0];

    GetDOFResolutions(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pResolution++ = _vTempRobotJoints[*it];
    }
    // set some default limits
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        *pResolution++ = _vTranslationResolutions.x;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        *pResolution++ = _vTranslationResolutions.y;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pResolution++ = _vTranslationResolutions.z; }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        *pResolution++ = _vRotationAxisResolutions.x;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pResolution++ = _vRotation3DResolutions.x;
        *pResolution++ = _vRotation3DResolutions.y;
        *pResolution++ = _vRotation3DResolutions.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
    }
}

void RobotBase::GetActiveDOFWeights(std::vector<dReal>& weights) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFWeights(weights);
        return;
    }

    weights.resize(GetActiveDOF());
    if( weights.size() == 0 ) {
        return;
    }
    dReal* pweight = &weights[0];

    GetDOFWeights(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pweight++ = _vTempRobotJoints[*it];
    }
    // set some default limits
    if( _nAffineDOFs & OpenRAVE::DOF_X ) { *pweight++ = _vTranslationWeights.x; }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) { *pweight++ = _vTranslationWeights.y; }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pweight++ = _vTranslationWeights.z; }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) { *pweight++ = _vRotationAxisWeights.x; }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pweight++ = _vRotation3DWeights.x;
        *pweight++ = _vRotation3DWeights.y;
        *pweight++ = _vRotation3DWeights.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
    }
}

void RobotBase::GetActiveDOFVelocityLimits(std::vector<dReal>& maxvel) const
{
    std::vector<dReal> dummy;
    if( _nActiveDOF < 0 ) {
        GetDOFVelocityLimits(dummy,maxvel);
        return;
    }
    maxvel.resize(GetActiveDOF());
    if( maxvel.size() == 0 ) {
        return;
    }
    dReal* pMaxVel = &maxvel[0];

    GetDOFVelocityLimits(dummy,_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxVel++ = _vTempRobotJoints[*it];
    }
    if( _nAffineDOFs & OpenRAVE::DOF_X ) { *pMaxVel++ = _vTranslationMaxVels.x; }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) { *pMaxVel++ = _vTranslationMaxVels.y; }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pMaxVel++ = _vTranslationMaxVels.z; }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) { *pMaxVel++ = _vRotationAxisMaxVels.x; }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pMaxVel++ = _vRotation3DMaxVels.x;
        *pMaxVel++ = _vRotation3DMaxVels.y;
        *pMaxVel++ = _vRotation3DMaxVels.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
    }
}

void RobotBase::GetActiveDOFAccelerationLimits(std::vector<dReal>& maxaccel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFAccelerationLimits(maxaccel);
        return;
    }
    maxaccel.resize(GetActiveDOF());
    if( maxaccel.size() == 0 ) {
        return;
    }
    dReal* pMaxAccel = &maxaccel[0];

    GetDOFAccelerationLimits(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxAccel++ = _vTempRobotJoints[*it];
    }
    if( _nAffineDOFs & OpenRAVE::DOF_X ) { *pMaxAccel++ = _vTranslationMaxVels.x; } // wrong
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) { *pMaxAccel++ = _vTranslationMaxVels.y; } // wrong
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pMaxAccel++ = _vTranslationMaxVels.z; } // wrong

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) { *pMaxAccel++ = _vRotationAxisMaxVels.x; } // wrong
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pMaxAccel++ = _vRotation3DMaxVels.x; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.y; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.z; // wrong
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
    }
}

void RobotBase::SubtractActiveDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    if( _nActiveDOF < 0 ) {
        SubtractDOFValues(q1,q2);
        return;
    }

    // go through all active joints
    int index = 0;
    FOREACHC(it,_vActiveDOFIndices) {
        JointConstPtr pjoint = _vecjoints.at(*it);
        for(int i = 0; i < pjoint->GetDOF(); ++i, index++) {
            if( pjoint->IsCircular(i) ) {
                q1.at(index) = ANGLE_DIFF(q1.at(index), q2.at(index));
            }
            else {
                q1.at(index) -= q2.at(index);
            }
        }
    }

    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        q1.at(index) -= q2.at(index);
        index++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        q1.at(index) -= q2.at(index);
        index++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        q1.at(index) -= q2.at(index);
        index++;
    }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        q1.at(index) = ANGLE_DIFF(q1.at(index),q2.at(index));
        index++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        // would like to do q2^-1 q1, but that might break rest of planners...?
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
}

const std::vector<int>& RobotBase::GetActiveDOFIndices() const
{
    return _nActiveDOF < 0 ? _vAllDOFIndices : _vActiveDOFIndices;
}

const ConfigurationSpecification& RobotBase::GetActiveConfigurationSpecification() const
{
    return _activespec;
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(index, offset, mjacobian);
        return;
    }

    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    if( _vActiveDOFIndices.size() != 0 ) {
        boost::multi_array<dReal,2> mjacobianjoints;
        CalculateJacobian(index, offset, mjacobianjoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            mjacobian[0][i] = mjacobianjoints[0][_vActiveDOFIndices[i]];
            mjacobian[1][i] = mjacobianjoints[1][_vActiveDOFIndices[i]];
            mjacobian[2][i] = mjacobianjoints[2][_vActiveDOFIndices[i]];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        mjacobian[0][ind] = 1;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 1;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 1;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        Vector vj = vActvAffineRotationAxis.cross(GetTransform().trans-offset);
        mjacobian[0][ind] = vj.x;
        mjacobian[1][ind] = vj.y;
        mjacobian[2][ind] = vj.z;
        ind++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        // have to take the partial derivative dT/dA of the axis*angle representation with respect to the transformation it induces
        // can introduce converting to quaternions in the middle, then by chain rule,  dT/dA = dT/tQ * dQ/dA
        // for questions on derivation email rdiankov@cs.cmu.edu
        Transform t = GetTransform();
        dReal Qx = t.rot.x, Qy = t.rot.y, Qz = t.rot.z, Qw = t.rot.w;
        dReal Tx = offset.x-t.trans.x, Ty = offset.y-t.trans.y, Tz = offset.z-t.trans.z;

        // after some math, the dT/dQ looks like:
        dReal dRQ[12] = { 2*Qy*Ty+2*Qz*Tz,         -4*Qy*Tx+2*Qx*Ty+2*Qw*Tz,   -4*Qz*Tx-2*Qw*Ty+2*Qx*Tz,   -2*Qz*Ty+2*Qy*Tz,
                          2*Qy*Tx-4*Qx*Ty-2*Qw*Tz, 2*Qx*Tx+2*Qz*Tz,            2*Qw*Tx-4*Qz*Ty+2*Qy*Tz,    2*Qz*Tx-2*Qx*Tz,
                          2*Qz*Tx+2*Qw*Ty-4*Qx*Tz, -2*Qw*Tx+2*Qz*Ty-4*Qy*Tz,   2*Qx*Tx+2*Qy*Ty,            -2*Qy*Tx+2*Qx*Ty };

        // calc dQ/dA
        dReal fsin = sqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
        dReal fcos = t.rot.x;
        dReal fangle = 2 * atan2(fsin, fcos);
        dReal normalizer = fangle / fsin;
        dReal Ax = normalizer * t.rot.y;
        dReal Ay = normalizer * t.rot.z;
        dReal Az = normalizer * t.rot.w;

        if( RaveFabs(fangle) < 1e-8f )
            fangle = 1e-8f;

        dReal fangle2 = fangle*fangle;
        dReal fiangle2 = 1/fangle2;
        dReal inormalizer = normalizer > 0 ? 1/normalizer : 0;
        dReal fconst = inormalizer*fiangle2;
        dReal fconst2 = fcos*fiangle2;
        dReal dQA[12] = { -0.5f*Ax*inormalizer,                     -0.5f*Ay*inormalizer,                       -0.5f*Az*inormalizer,
                          inormalizer+0.5f*Ax*Ax*(fconst2-fconst),  0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,            0.5f*Ax*fconst2*Az-Ax*fconst*Az,
                          0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,          inormalizer+0.5f*Ay*Ay*(fconst2-fconst),    0.5f*Ay*fconst2*Az-Ay*fconst*Az,
                          0.5f*Ax*fconst2*Az-Ax*fconst*Az,          0.5f*Ay*fconst2*Az-Ay*fconst*Az,            inormalizer+0.5f*Az*Az*(fconst2-fconst)};

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                mjacobian[i][ind+j] = dRQ[4*i+0]*dQA[3*0+j] + dRQ[4*i+1]*dQA[3*1+j] + dRQ[4*i+2]*dQA[3*2+j] + dRQ[4*i+3]*dQA[3*3+j];
            }
        }
        ind += 3;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        Transform t; t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        dReal Qx = t.rot.x, Qy = t.rot.y, Qz = t.rot.z, Qw = t.rot.w;
        dReal Tx = offset.x-t.trans.x, Ty = offset.y-t.trans.y, Tz = offset.z-t.trans.z;

        // after some hairy math, the dT/dQ looks like:
        dReal dRQ[12] = { 2*Qy*Ty+2*Qz*Tz,         -4*Qy*Tx+2*Qx*Ty+2*Qw*Tz,   -4*Qz*Tx-2*Qw*Ty+2*Qx*Tz,   -2*Qz*Ty+2*Qy*Tz,
                          2*Qy*Tx-4*Qx*Ty-2*Qw*Tz, 2*Qx*Tx+2*Qz*Tz,            2*Qw*Tx-4*Qz*Ty+2*Qy*Tz,    2*Qz*Tx-2*Qx*Tz,
                          2*Qz*Tx+2*Qw*Ty-4*Qx*Tz, -2*Qw*Tx+2*Qz*Ty-4*Qy*Tz,   2*Qx*Tx+2*Qy*Ty,            -2*Qy*Tx+2*Qx*Ty };

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 4; ++j) {
                mjacobian[i][j] = dRQ[4*i+j];
            }
        }
        ind += 3;
    }
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(index, offset, vjacobian);
        return;
    }

    boost::multi_array<dReal,2> mjacobian;
    RobotBase::CalculateActiveJacobian(index,offset,mjacobian);
    vjacobian.resize(3*GetActiveDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(index, q, mjacobian);
        return;
    }

    mjacobian.resize(boost::extents[4][GetActiveDOF()]);
    if( _vActiveDOFIndices.size() != 0 ) {
        boost::multi_array<dReal,2> mjacobianjoints;
        CalculateRotationJacobian(index, q, mjacobianjoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            mjacobian[0][i] = mjacobianjoints[0][_vActiveDOFIndices[i]];
            mjacobian[1][i] = mjacobianjoints[1][_vActiveDOFIndices[i]];
            mjacobian[2][i] = mjacobianjoints[2][_vActiveDOFIndices[i]];
            mjacobian[3][i] = mjacobianjoints[3][_vActiveDOFIndices[i]];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform )
        return;

    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        mjacobian[3][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        mjacobian[3][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        mjacobian[3][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        mjacobian[0][ind] = dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
        mjacobian[1][ind] = dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
        mjacobian[2][ind] = dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
        mjacobian[3][ind] = dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
        ind++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        BOOST_ASSERT(!"rotation 3d not supported");
        ind += 3;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        BOOST_ASSERT(!"quaternion not supported");
        ind += 4;
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(index, q, vjacobian);
        return;
    }

    boost::multi_array<dReal,2> mjacobian;
    RobotBase::CalculateActiveRotationJacobian(index,q,mjacobian);
    vjacobian.resize(4*GetActiveDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int index, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateAngularVelocityJacobian(index, mjacobian);
        return;
    }

    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    if( _vActiveDOFIndices.size() != 0 ) {
        boost::multi_array<dReal,2> mjacobianjoints;
        CalculateAngularVelocityJacobian(index, mjacobianjoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            mjacobian[0][i] = mjacobianjoints[0][_vActiveDOFIndices[i]];
            mjacobian[1][i] = mjacobianjoints[1][_vActiveDOFIndices[i]];
            mjacobian[2][i] = mjacobianjoints[2][_vActiveDOFIndices[i]];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        mjacobian[0][ind] = v.x;
        mjacobian[1][ind] = v.y;
        mjacobian[2][ind] = v.z;

    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        BOOST_ASSERT(!"rotation 3d not supported");
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        BOOST_ASSERT(!"quaternions not supported");

        // most likely wrong
        Transform t; t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        dReal fnorm = t.rot.y*t.rot.y+t.rot.z*t.rot.z+t.rot.w*t.rot.w;
        if( fnorm > 0 ) {
            fnorm = dReal(1)/RaveSqrt(fnorm);
            mjacobian[0][ind] = t.rot.y*fnorm;
            mjacobian[1][ind] = t.rot.z*fnorm;
            mjacobian[2][ind] = t.rot.w*fnorm;
        }
        else {
            mjacobian[0][ind] = 0;
            mjacobian[1][ind] = 0;
            mjacobian[2][ind] = 0;
        }

        ++ind;
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateAngularVelocityJacobian(index, vjacobian);
        return;
    }

    boost::multi_array<dReal,2> mjacobian;
    RobotBase::CalculateActiveAngularVelocityJacobian(index,mjacobian);
    vjacobian.resize(3*GetActiveDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetActiveDOF();
    }
}

bool RobotBase::InitFromFile(const std::string& filename, const AttributesList& atts)
{
    bool bSuccess = GetEnv()->ReadRobotURI(shared_robot(), filename, atts)==shared_robot();
    if( !bSuccess ) {
        Destroy();
        return false;
    }
    return true;
}

bool RobotBase::InitFromData(const std::string& data, const AttributesList& atts)
{
    bool bSuccess = GetEnv()->ReadRobotData(shared_robot(), data, atts)==shared_robot();
    if( !bSuccess ) {
        Destroy();
        return false;
    }
    return true;
}

const std::set<int>& RobotBase::GetNonAdjacentLinks(int adjacentoptions) const
{
    KinBody::GetNonAdjacentLinks(0); // need to call to set the cache
    if( (_nNonAdjacentLinkCache&adjacentoptions) != adjacentoptions ) {
        int requestedoptions = (~_nNonAdjacentLinkCache)&adjacentoptions;
        // find out what needs to computed
        boost::array<uint8_t,4> compute={ { 0,0,0,0}};
        if( requestedoptions & AO_Enabled ) {
            for(size_t i = 0; i < compute.size(); ++i) {
                if( i & AO_Enabled ) {
                    compute[i] = 1;
                }
            }
        }
        if( requestedoptions & AO_ActiveDOFs ) {
            for(size_t i = 0; i < compute.size(); ++i) {
                if( i & AO_ActiveDOFs ) {
                    compute[i] = 1;
                }
            }
        }
        if( requestedoptions & ~(AO_Enabled|AO_ActiveDOFs) ) {
            throw OPENRAVE_EXCEPTION_FORMAT("does not support adjacentoptions %d",adjacentoptions,ORE_InvalidArguments);
        }

        // compute it
        if( compute.at(AO_Enabled) ) {
            _setNonAdjacentLinks.at(AO_Enabled).clear();
            FOREACHC(itset, _setNonAdjacentLinks[0]) {
                KinBody::LinkConstPtr plink1(_veclinks.at(*itset&0xffff)), plink2(_veclinks.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    _setNonAdjacentLinks[AO_Enabled].insert(*itset);
                }
            }
        }
        if( compute.at(AO_ActiveDOFs) ) {
            _setNonAdjacentLinks.at(AO_ActiveDOFs).clear();
            FOREACHC(itset, _setNonAdjacentLinks[0]) {
                FOREACHC(it, GetActiveDOFIndices()) {
                    if( IsDOFInChain(*itset&0xffff,*itset>>16,*it) ) {
                        _setNonAdjacentLinks[AO_ActiveDOFs].insert(*itset);
                        break;
                    }
                }
            }
        }
        if( compute.at(AO_Enabled|AO_ActiveDOFs) ) {
            _setNonAdjacentLinks.at(AO_Enabled|AO_ActiveDOFs).clear();
            FOREACHC(itset, _setNonAdjacentLinks[AO_ActiveDOFs]) {
                KinBody::LinkConstPtr plink1(_veclinks.at(*itset&0xffff)), plink2(_veclinks.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    _setNonAdjacentLinks[AO_Enabled|AO_ActiveDOFs].insert(*itset);
                }
            }
        }
        _nNonAdjacentLinkCache |= requestedoptions;
    }
    return _setNonAdjacentLinks.at(adjacentoptions);
}

bool RobotBase::Grab(KinBodyPtr pbody)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip ) {
        return false;
    }
    return Grab(pbody, pmanip->GetEndEffector());
}

bool RobotBase::Grab(KinBodyPtr pbody, const std::set<int>& setRobotLinksToIgnore)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip ) {
        return false;
    }
    return Grab(pbody, pmanip->GetEndEffector(), setRobotLinksToIgnore);
}

bool RobotBase::Grab(KinBodyPtr pbody, LinkPtr plink)
{
    if( !pbody || !plink ||(plink->GetParent() != shared_kinbody())) {
        throw OPENRAVE_EXCEPTION_FORMAT0("invalid grab arguments",ORE_InvalidArguments);
    }
    if( pbody == shared_kinbody() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("robot %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
    }
    if( IsGrabbing(pbody) ) {
        RAVELOG_VERBOSE(str(boost::format("Robot %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
        return true;
    }

    _vGrabbedBodies.push_back(Grabbed());
    Grabbed& g = _vGrabbedBodies.back();
    g.pbody = pbody;
    g.plinkrobot = plink;
    g.troot = plink->GetTransform().inverse() * pbody->GetTransform();

    {
        CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
        // check collision with all links to see which are valid
        FOREACH(itlink, _veclinks) {
            if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), KinBodyConstPtr(pbody)) ) {
                g.vCollidingLinks.push_back(*itlink);
            }
            else {
                g.vNonCollidingLinks.push_back(*itlink);
            }
        }
        FOREACH(itgrabbed, _vGrabbedBodies) {
            KinBodyConstPtr pgrabbedbody(itgrabbed->pbody);
            if( pgrabbedbody != pbody ) {
                FOREACHC(itlink, pgrabbedbody->GetLinks()) {
                    if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), pbody) ) {
                        g.vCollidingLinks.push_back(*itlink);
                    }
                }
            }
        }
    }

    _AttachBody(pbody);
    return true;
}

bool RobotBase::Grab(KinBodyPtr pbody, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore)
{
    if( !pbody || !pRobotLinkToGrabWith ||(pRobotLinkToGrabWith->GetParent() != shared_kinbody())) {
        throw OPENRAVE_EXCEPTION_FORMAT0("invalid grab arguments",ORE_InvalidArguments);
    }
    if( pbody == shared_kinbody() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("robot %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
    }
    if( IsGrabbing(pbody) ) {
        RAVELOG_VERBOSE(str(boost::format("Robot %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
        return true;
    }

    _vGrabbedBodies.push_back(Grabbed());
    Grabbed& g = _vGrabbedBodies.back();
    g.pbody = pbody;
    g.plinkrobot = pRobotLinkToGrabWith;
    g.troot = pRobotLinkToGrabWith->GetTransform().inverse() * pbody->GetTransform();

    {
        CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options

        // check collision with all links to see which are valid
        FOREACH(itlink, _veclinks) {
            if( setRobotLinksToIgnore.find((*itlink)->GetIndex()) != setRobotLinksToIgnore.end() ) {
                continue;
            }
            if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), KinBodyConstPtr(pbody)) ) {
                g.vCollidingLinks.push_back(*itlink);
            }
            else {
                g.vNonCollidingLinks.push_back(*itlink);
            }
        }

        FOREACH(itgrabbed, _vGrabbedBodies) {
            KinBodyConstPtr pgrabbedbody(itgrabbed->pbody);
            if( pgrabbedbody != pbody ) {
                FOREACHC(itlink, pgrabbedbody->GetLinks()) {
                    if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), pbody) ) {
                        g.vCollidingLinks.push_back(*itlink);
                    }
                }
            }
        }
    }

    _AttachBody(pbody);
    return true;
}

void RobotBase::Release(KinBodyPtr pbody)
{
    vector<Grabbed>::iterator itbody;
    FORIT(itbody, _vGrabbedBodies) {
        if( KinBodyPtr(itbody->pbody) == pbody )
            break;
    }

    if( itbody == _vGrabbedBodies.end() ) {
        RAVELOG_DEBUG(str(boost::format("Robot %s: body %s not grabbed\n")%GetName()%pbody->GetName()));
        return;
    }

    _vGrabbedBodies.erase(itbody);
    _RemoveAttachedBody(pbody);
}

void RobotBase::ReleaseAllGrabbed()
{
    FOREACH(itgrabbed, _vGrabbedBodies) {
        KinBodyPtr pbody = itgrabbed->pbody.lock();
        if( !!pbody ) {
            _RemoveAttachedBody(pbody);
        }
    }
    _vGrabbedBodies.clear();
}

void RobotBase::RegrabAll()
{
    CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
    FOREACH(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody(itbody->pbody);
        if( !!pbody ) {
            // check collision with all links to see which are valid
            itbody->vCollidingLinks.resize(0);
            itbody->vNonCollidingLinks.resize(0);
            _RemoveAttachedBody(pbody);
            FOREACH(itlink, _veclinks) {
                if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), KinBodyConstPtr(pbody)) ) {
                    itbody->vCollidingLinks.push_back(*itlink);
                }
                else {
                    itbody->vNonCollidingLinks.push_back(*itlink);
                }
            }
            FOREACH(itgrabbed, _vGrabbedBodies) {
                KinBodyConstPtr pgrabbedbody(itgrabbed->pbody);
                if( pgrabbedbody != pbody ) {
                    FOREACHC(itlink, pgrabbedbody->GetLinks()) {
                        if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), pbody) ) {
                            itbody->vCollidingLinks.push_back(*itlink);
                        }
                    }
                }
            }
            _AttachBody(pbody);
        }
    }
}

RobotBase::LinkPtr RobotBase::IsGrabbing(KinBodyConstPtr pbody) const
{
    FOREACHC(itbody, _vGrabbedBodies) {
        if( KinBodyConstPtr(itbody->pbody) == pbody ) {
            return itbody->plinkrobot;
        }
    }
    return LinkPtr();
}

void RobotBase::GetGrabbed(std::vector<KinBodyPtr>& vbodies) const
{
    vbodies.resize(0);
    FOREACHC(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody = itbody->pbody.lock();
        if( !!pbody && pbody->GetEnvironmentId() ) {
            vbodies.push_back(pbody);
        }
    }
}

void RobotBase::SetActiveManipulator(int index)
{
    _nActiveManip = index;
}

void RobotBase::SetActiveManipulator(const std::string& manipname)
{
    if( manipname.size() > 0 ) {
        for(size_t i = 0; i < _vecManipulators.size(); ++i ) {
            if( manipname == _vecManipulators[i]->GetName() ) {
                _nActiveManip = i;
                return;
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT("failed to find manipulator with name: %s", manipname, ORE_InvalidArguments);
    }

    _nActiveManip = -1;
}

RobotBase::ManipulatorPtr RobotBase::GetActiveManipulator()
{
    if((_nActiveManip < 0)&&(_nActiveManip >= (int)_vecManipulators.size())) {
        throw RobotBase::ManipulatorPtr();
    }
    return _vecManipulators.at(_nActiveManip);
}

RobotBase::ManipulatorConstPtr RobotBase::GetActiveManipulator() const
{
    if((_nActiveManip < 0)&&(_nActiveManip >= (int)_vecManipulators.size())) {
        return RobotBase::ManipulatorPtr();
    }
    return _vecManipulators.at(_nActiveManip);
}

/// Check if body is self colliding. Links that are joined together are ignored.
bool RobotBase::CheckSelfCollision(CollisionReportPtr report) const
{
    if( KinBody::CheckSelfCollision(report) ) {
        return true;
    }
    // check all grabbed bodies with (TODO: support CO_ActiveDOFs option)
    bool bCollision = false;
    FOREACHC(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody(itbody->pbody);
        if( !pbody ) {
            continue;
        }
        FOREACHC(itrobotlink,itbody->vNonCollidingLinks) {
            // have to use link/link collision since link/body checks attached bodies
            FOREACHC(itbodylink,pbody->GetLinks()) {
                if( GetEnv()->CheckCollision(*itrobotlink,KinBody::LinkConstPtr(*itbodylink),report) ) {
                    bCollision = true;
                    break;
                }
            }
            if( bCollision ) {
                break;
            }
        }
        if( bCollision ) {
            break;
        }

        if( pbody->CheckSelfCollision(report) ) {
            bCollision = true;
            break;
        }

        // check attached bodies with each other, this is actually tricky since they are attached "with each other", so regular CheckCollision will not work.
        // Instead, we will compare each of the body's links with every other
        if( _vGrabbedBodies.size() > 1 ) {
            FOREACHC(itbody2, _vGrabbedBodies) {
                KinBodyPtr pbody2(itbody2->pbody);
                if( pbody == pbody2 ) {
                    continue;
                }
                FOREACHC(itlink2, pbody2->GetLinks()) {
                    // make sure the two bodies were not initially colliding
                    if( find(itbody->vCollidingLinks.begin(),itbody->vCollidingLinks.end(),*itlink2) == itbody->vCollidingLinks.end() ) {
                        FOREACHC(itlink, pbody->GetLinks()) {
                            if( find(itbody2->vCollidingLinks.begin(),itbody2->vCollidingLinks.end(),*itlink) == itbody2->vCollidingLinks.end() ) {
                                if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink),KinBody::LinkConstPtr(*itlink2),report) ) {
                                    bCollision = true;
                                    break;
                                }
                            }
                            if( bCollision ) {
                                break;
                            }
                        }
                        if( bCollision ) {
                            break;
                        }
                    }
                }
                if( bCollision ) {
                    break;
                }
            }
            if( bCollision ) {
                break;
            }
        }
    }

    if( bCollision && !!report ) {
        RAVELOG_VERBOSE(str(boost::format("Self collision: %s\n")%report->__str__()));
    }
    return bCollision;
}

bool RobotBase::CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    if( plink->IsEnabled() ) {
        boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if( GetEnv()->CheckCollision(LinkConstPtr(plink),report) ) {
            return true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        if( itgrabbed->plinkrobot == plink ) {
            KinBodyPtr pbody = itgrabbed->pbody.lock();
            if( !!pbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_vGrabbedBodies) {
                    if( itgrabbed2 != itgrabbed ) {
                        KinBodyPtr pbody2 = itgrabbed2->pbody.lock();
                        if( !!pbody2 ) {
                            vbodyexcluded.push_back(pbody2);
                        }
                    }
                }
                KinBodyStateSaver bodysaver(pbody,Save_LinkTransformation);
                pbody->SetTransform(tlinktrans * itgrabbed->troot);
                if( GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) ) {
                    return true;
                }
            }
        }
    }
    return false;
}

void RobotBase::SimulationStep(dReal fElapsedTime)
{
    KinBody::SimulationStep(fElapsedTime);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::_ComputeInternalInformation()
{
    KinBody::_ComputeInternalInformation();
    _activespec._vgroups.reserve(2);
    _vAllDOFIndices.resize(GetDOF());
    for(int i = 0; i < GetDOF(); ++i) {
        _vAllDOFIndices[i] = i;
    }
    int manipindex=0;
    FOREACH(itmanip,_vecManipulators) {
        if( (*itmanip)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "manip" << manipindex;
            RAVELOG_WARN(str(boost::format("robot %s has a manipulator with no name, setting to %s\n")%GetName()%ss.str()));
            (*itmanip)->_name = ss.str();
        }
        else if( !IsValidName((*itmanip)->GetName()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT("manipulator name \"%s\" is not valid", (*itmanip)->GetName(), ORE_Failed);
        }
        if( !!(*itmanip)->GetBase() && !!(*itmanip)->GetEndEffector() ) {
            vector<JointPtr> vjoints;
            std::vector<int> vmimicdofs;
            (*itmanip)->__varmdofindices.resize(0);
            if( GetChain((*itmanip)->GetBase()->GetIndex(),(*itmanip)->GetEndEffector()->GetIndex(), vjoints) ) {
                FOREACH(it,vjoints) {
                    if( (*it)->IsStatic() ) {
                        // ignore
                    }
                    else if( (*it)->IsMimic() ) {
                        for(int i = 0; i < (*it)->GetDOF(); ++i) {
                            if( (*it)->IsMimic(i) ) {
                                (*it)->GetMimicDOFIndices(vmimicdofs,i);
                                FOREACHC(itmimicdof,vmimicdofs) {
                                    if( find((*itmanip)->__varmdofindices.begin(),(*itmanip)->__varmdofindices.end(),*itmimicdof) == (*itmanip)->__varmdofindices.end() ) {
                                        (*itmanip)->__varmdofindices.push_back(*itmimicdof);
                                    }
                                }
                            }
                            else if( (*it)->GetDOFIndex() >= 0 ) {
                                (*itmanip)->__varmdofindices.push_back((*it)->GetDOFIndex()+i);
                            }
                        }
                    }
                    else if( (*it)->GetDOFIndex() < 0) {
                        RAVELOG_WARN(str(boost::format("manipulator arm contains joint %s without a dof index, ignoring...\n")%(*it)->GetName()));
                    }
                    else { // ignore static joints
                        for(int i = 0; i < (*it)->GetDOF(); ++i) {
                            (*itmanip)->__varmdofindices.push_back((*it)->GetDOFIndex()+i);
                        }
                    }
                }
            }
            else {
                RAVELOG_WARN(str(boost::format("manipulator %s failed to find chain between %s and %s links\n")%(*itmanip)->GetName()%(*itmanip)->GetBase()->GetName()%(*itmanip)->GetEndEffector()->GetName()));
            }
        }
        else {
            RAVELOG_WARN(str(boost::format("manipulator %s has undefined base and end effector links\n")%(*itmanip)->GetName()));
        }
        // init the gripper dof indices
        (*itmanip)->__vgripperdofindices.resize(0);
        std::vector<dReal> vClosingDirection;
        size_t iclosingdirection = 0;
        FOREACHC(itjointname,(*itmanip)->_vgripperjointnames) {
            JointPtr pjoint = GetJoint(*itjointname);
            if( !pjoint ) {
                RAVELOG_WARN(str(boost::format("could not find gripper joint %s for manipulator %s")%*itjointname%(*itmanip)->GetName()));
                iclosingdirection++;
            }
            else {
                if( pjoint->GetDOFIndex() >= 0 ) {
                    for(int i = 0; i < pjoint->GetDOF(); ++i) {
                        if( find((*itmanip)->__varmdofindices.begin(), (*itmanip)->__varmdofindices.end(), pjoint->GetDOFIndex()+i) != (*itmanip)->__varmdofindices.end() ) {
                            RAVELOG_ERROR(str(boost::format("manipulator %s gripper dof %d is also part of arm dof! excluding from gripper...")%(*itmanip)->GetName()%(pjoint->GetDOFIndex()+i)));
                        }
                        else {
                            (*itmanip)->__vgripperdofindices.push_back(pjoint->GetDOFIndex()+i);
                            if( iclosingdirection < (*itmanip)->_vClosingDirection.size() ) {
                                vClosingDirection.push_back((*itmanip)->_vClosingDirection[iclosingdirection++]);
                            }
                            else {
                                vClosingDirection.push_back(0);
                                RAVELOG_WARN(str(boost::format("manipulator %s closing direction not correct length, might get bad closing/release grasping")%(*itmanip)->GetName()));
                            }
                        }
                    }
                }
                else {
                    ++iclosingdirection;
                    RAVELOG_WARN(str(boost::format("manipulator %s gripper joint %s is not active, so has no dof index. ignoring.")%(*itmanip)->GetName()%*itjointname));
                }
            }
        }
        (*itmanip)->_vClosingDirection.swap(vClosingDirection);
        vector<ManipulatorPtr>::iterator itmanip2 = itmanip; ++itmanip2;
        for(; itmanip2 != _vecManipulators.end(); ++itmanip2) {
            if( (*itmanip)->GetName() == (*itmanip2)->GetName() ) {
                RAVELOG_WARN(str(boost::format("robot %s has two manipulators with the same name: %s!\n")%GetName()%(*itmanip)->GetName()));
            }
        }
        manipindex++;
    }

    int sensorindex=0;
    FOREACH(itsensor,_vecSensors) {
        if( (*itsensor)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "sensor" << sensorindex;
            RAVELOG_WARN(str(boost::format("robot %s has a sensor with no name, setting to %s\n")%GetName()%ss.str()));
            (*itsensor)->_name = ss.str();
        }
        else if( !IsValidName((*itsensor)->GetName()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT("sensor name \"%s\" is not valid", (*itsensor)->GetName(), ORE_Failed);
        }
        if( !!(*itsensor)->GetSensor() ) {
            stringstream ss; ss << GetName() << "_" << (*itsensor)->GetName(); // global unique name?
            (*itsensor)->GetSensor()->SetName(ss.str());
        }
        sensorindex++;
    }

    {
        __hashrobotstructure.resize(0);
        FOREACH(itmanip,_vecManipulators) {
            (*itmanip)->__hashstructure.resize(0);
            (*itmanip)->__hashkinematicsstructure.resize(0);
        }
        FOREACH(itsensor,_vecSensors) {
            (*itsensor)->__hashstructure.resize(0);
        }
    }
    // finally initialize the ik solvers (might depend on the hashes)
    FOREACH(itmanip, _vecManipulators) {
        if( !!(*itmanip)->_pIkSolver ) {
            try {
                (*itmanip)->_pIkSolver->Init(*itmanip);
            }
            catch(const openrave_exception& e) {
                RAVELOG_WARN(str(boost::format("failed to init ik solver: %s\n")%e.what()));
                (*itmanip)->SetIkSolver(IkSolverBasePtr());
            }
        }
    }
    if( ComputeAABB().extents.lengthsqr3() > 900.0f ) {
        RAVELOG_WARN(str(boost::format("Robot %s span is greater than 30 meaning that it is most likely defined in a unit other than meters. It is highly encouraged to define all OpenRAVE robots in meters since many metrics, database models, and solvers have been specifically optimized for this unit\n")%GetName()));
    }

    if( !GetController() ) {
        RAVELOG_VERBOSE(str(boost::format("no default controller set on robot %s\n")%GetName()));
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        SetController(RaveCreateController(GetEnv(), "IdealController"),dofindices,1);
    }

    // reset the power on the sensors
    FOREACH(itsensor,_vecSensors) {
        SensorBasePtr psensor = (*itsensor)->GetSensor();
        if( !!psensor ) {
            int ispower = psensor->Configure(SensorBase::CC_PowerCheck);
            psensor->Configure(ispower ? SensorBase::CC_PowerOn : SensorBase::CC_PowerOff);
        }
    }
}

void RobotBase::_ParametersChanged(int parameters)
{
    KinBody::_ParametersChanged(parameters);
    if( parameters & (Prop_Sensors|Prop_SensorPlacement) ) {
        FOREACH(itsensor,_vecSensors) {
            (*itsensor)->__hashstructure.resize(0);
        }
    }
    if( parameters & Prop_Manipulators ) {
        FOREACH(itmanip,_vecManipulators) {
            (*itmanip)->__hashstructure.resize(0);
            (*itmanip)->__hashkinematicsstructure.resize(0);
        }
    }
}

void RobotBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    // note that grabbed bodies are not cloned (check out Environment::Clone)
    KinBody::Clone(preference,cloningoptions);
    RobotBaseConstPtr r = RaveInterfaceConstCast<RobotBase>(preference);
    __hashrobotstructure = r->__hashrobotstructure;
    _vecManipulators.clear();
    FOREACHC(itmanip, r->_vecManipulators) {
        _vecManipulators.push_back(ManipulatorPtr(new Manipulator(shared_robot(),**itmanip)));
    }

    _vecSensors.clear();
    FOREACHC(itsensor, r->_vecSensors) {
        _vecSensors.push_back(AttachedSensorPtr(new AttachedSensor(shared_robot(),**itsensor,cloningoptions)));
    }
    _UpdateAttachedSensors();

    _vActiveDOFIndices = r->_vActiveDOFIndices;
    _vAllDOFIndices = r->_vAllDOFIndices;
    vActvAffineRotationAxis = r->vActvAffineRotationAxis;
    _nActiveManip = r->_nActiveManip;
    _nActiveDOF = r->_nActiveDOF;
    _nAffineDOFs = r->_nAffineDOFs;

    _vTranslationLowerLimits = r->_vTranslationLowerLimits;
    _vTranslationUpperLimits = r->_vTranslationUpperLimits;
    _vTranslationMaxVels = r->_vTranslationMaxVels;
    _vTranslationResolutions = r->_vTranslationResolutions;
    _vRotationAxisLowerLimits = r->_vRotationAxisLowerLimits;
    _vRotationAxisUpperLimits = r->_vRotationAxisUpperLimits;
    _vRotationAxisMaxVels = r->_vRotationAxisMaxVels;
    _vRotationAxisResolutions = r->_vRotationAxisResolutions;
    _vRotation3DLowerLimits = r->_vRotation3DLowerLimits;
    _vRotation3DUpperLimits = r->_vRotation3DUpperLimits;
    _vRotation3DMaxVels = r->_vRotation3DMaxVels;
    _vRotation3DResolutions = r->_vRotation3DResolutions;
    _vRotationQuatLimitStart = r->_vRotationQuatLimitStart;
    _fQuatLimitMaxAngle = r->_fQuatLimitMaxAngle;
    _fQuatMaxAngleVelocity = r->_fQuatMaxAngleVelocity;
    _fQuatAngleResolution = r->_fQuatAngleResolution;
    _fQuatAngleWeight = r->_fQuatAngleWeight;

    // clone the controller
    if( (cloningoptions&Clone_RealControllers) && !!r->GetController() ) {
        if( !SetController(RaveCreateController(GetEnv(), r->GetController()->GetXMLId()),r->GetController()->GetControlDOFIndices(),r->GetController()->IsControlTransformation()) ) {
            RAVELOG_WARN(str(boost::format("failed to set %s controller for robot %s\n")%r->GetController()->GetXMLId()%GetName()));
        }
    }

    if( !GetController() ) {
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        if( !SetController(RaveCreateController(GetEnv(), "IdealController"),dofindices, 1) ) {
            RAVELOG_WARN("failed to set IdealController\n");
        }
    }
}

void RobotBase::serialize(std::ostream& o, int options) const
{
    KinBody::serialize(o,options);
    if( options & SO_RobotManipulators ) {
        FOREACHC(itmanip,_vecManipulators) {
            (*itmanip)->serialize(o,options);
        }
    }
    if( options & SO_RobotSensors ) {
        FOREACHC(itsensor,_vecSensors) {
            (*itsensor)->serialize(o,options);
        }
    }
}

const std::string& RobotBase::GetRobotStructureHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    if( __hashrobotstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics|SO_Geometry|SO_RobotManipulators|SO_RobotSensors);
        __hashrobotstructure = GetMD5HashString(ss.str());
    }
    return __hashrobotstructure;
}

bool RobotBase::SetMotion(TrajectoryBaseConstPtr ptraj)
{
    if( !!GetController() ) {
        return GetController()->SetPath(ptraj);
    }
    return false;
}

bool RobotBase::SetActiveMotion(TrajectoryBaseConstPtr ptraj)
{
    if( !!GetController() ) {
        return GetController()->SetPath(ptraj);
    }
    return false;
}

bool RobotBase::SetActiveMotion(TrajectoryBaseConstPtr ptraj, dReal)
{
    if( !!GetController() ) {
        return GetController()->SetPath(ptraj);
    }
    return false;
}

void RobotBase::GetFullTrajectoryFromActive(TrajectoryBasePtr pfulltraj, TrajectoryBaseConstPtr pActiveTraj, bool bOverwriteTransforms)
{
    ConfigurationSpecification spec;
    spec._vgroups.resize(2);
    spec._vgroups[0].offset = 0;
    spec._vgroups[0].dof = GetDOF();
    stringstream ss;
    ss << "joint_values " << GetName();
    for(int i = 0; i < GetDOF(); ++i) {
        ss << " " << i;
    }
    spec._vgroups[0].name = ss.str();
    spec._vgroups[0].interpolation = "linear";
    spec._vgroups[1].offset = GetDOF();
    spec._vgroups[1].dof = 1;
    spec._vgroups[1].name = "deltatime";
    if( !bOverwriteTransforms ) {
        spec._vgroups.resize(3);
        spec._vgroups[2].offset = GetDOF()+1;
        spec._vgroups[2].dof = RaveGetAffineDOF(DOF_Transform);
        spec._vgroups[2].name = str(boost::format("affine_transform %s %d")%GetName()%DOF_Transform);
        spec._vgroups[2].interpolation = "linear";
    }
    pfulltraj->Init(spec);
    std::vector<dReal> vdata;
    pActiveTraj->GetWaypoints(0,pActiveTraj->GetNumWaypoints(),vdata);
    pfulltraj->Insert(0,vdata,pActiveTraj->GetConfigurationSpecification());
}

} // end namespace OpenRAVE
