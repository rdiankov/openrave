// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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
/*! --------------------------------------------------------------------
  \file   Robot.cpp
  \brief  Encapsulate a virtual robot description
 -------------------------------------------------------------------- */

#include "libopenrave.h"

#define CHECK_INTERNAL_COMPUTATION { \
    if( !_bHierarchyComputed ) { \
        throw openrave_exception(str(boost::format("%s: joint hierarchy needs to be computed (is body added to environment?)\n")%__PRETTY_FUNCTION__)); \
    } \
} \

namespace OpenRAVE {

RobotBase::Manipulator::Manipulator(RobotBasePtr probot) : _probot(probot), _vdirection(0,0,1) {}
RobotBase::Manipulator::~Manipulator() {}

RobotBase::Manipulator::Manipulator(const RobotBase::Manipulator& r)
{
    *this = r;
    _pIkSolver.reset();
    if( _strIkSolver.size() > 0 )
        _pIkSolver = GetRobot()->GetEnv()->CreateIkSolver(_strIkSolver);
}

RobotBase::Manipulator::Manipulator(RobotBasePtr probot, const RobotBase::Manipulator& r)
{
    *this = r;
    _probot = probot;
    if( !!r.GetBase() )
        _pBase = probot->GetLinks().at(r.GetBase()->GetIndex());
    if( !!r.GetEndEffector() )
        _pEndEffector = probot->GetLinks().at(r.GetEndEffector()->GetIndex());
    
    _pIkSolver.reset();
    if( _strIkSolver.size() > 0 )
        _pIkSolver = probot->GetEnv()->CreateIkSolver(_strIkSolver);
}

Transform RobotBase::Manipulator::GetEndEffectorTransform() const
{
    return _pEndEffector->GetTransform() * _tGrasp;
}

void RobotBase::Manipulator::SetIKSolver(IkSolverBasePtr iksolver)
{
    _pIkSolver = iksolver;
    if( !!_pIkSolver )
        _strIkSolver = _pIkSolver->GetXMLId();
}

bool RobotBase::Manipulator::InitIKSolver()
{
    return !_pIkSolver ? false : _pIkSolver->Init(shared_from_this());
}

const std::string& RobotBase::Manipulator::GetIKSolverName() const
{
    return _strIkSolver;
}

bool RobotBase::Manipulator::HasIKSolver() const
{
    return !!_pIkSolver;
}

int RobotBase::Manipulator::GetNumFreeParameters() const
{
    return !_pIkSolver ? 0 : _pIkSolver->GetNumFreeParameters();
}

bool RobotBase::Manipulator::GetFreeParameters(std::vector<dReal>& vFreeParameters) const
{
    return !_pIkSolver ? false : _pIkSolver->GetFreeParameters(vFreeParameters);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, vector<dReal>& solution, bool bColCheck) const
{
    return FindIKSolution(goal, vector<dReal>(), solution, bColCheck);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, vector<dReal>& solution, bool bColCheck) const
{
    if( !_pIkSolver )
        throw openrave_exception(str(boost::format("manipulator %s:%s does not have an IK solver set")%RobotBasePtr(_probot)->GetName()%GetName()));
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );

    vector<dReal> temp;
    GetRobot()->GetDOFValues(temp);

    solution.resize(_varmjoints.size());
    for(size_t i = 0; i < _varmjoints.size(); ++i)
        solution[i] = temp[_varmjoints[i]];

    IkParameterization localgoal;
    if( goal.GetType() == IkParameterization::Type_Transform6D ) {
        Transform tgoal = goal.GetTransform()*_tGrasp.inverse();
        if( !!_pBase )
            tgoal = _pBase->GetTransform().inverse() * tgoal;
        localgoal.SetTransform(tgoal);
    }
    else if( goal.GetType() == IkParameterization::Type_Rotation3D ) {
        Vector q = quatMultiply(goal.GetRotation(),quatInverse(_tGrasp.rot));
        if( !!_pBase )
            q = quatMultiply(quatInverse(_pBase->GetTransform().rot),q);
        localgoal.SetRotation(q);
    }
    else if( goal.GetType() == IkParameterization::Type_Translation3D ) {
        if( TransformDistanceFast(_tGrasp,Transform()) > 1e-4f )
            throw openrave_exception("Grasp transform has to be the identity for translation-only IK");
        if( !!_pBase )
            localgoal.SetTranslation(_pBase->GetTransform().inverse()*goal.GetTranslation());
        else
            localgoal.SetTranslation(goal.GetTranslation());
    }
    else if( goal.GetType() == IkParameterization::Type_Direction3D ) {
//        if( min((_tGrasp.rot-Vector(1,0,0,0)).lengthsqr4(),(_tGrasp.rot+Vector(1,0,0,0)).lengthsqr4()) > 1e-8f )
//            throw openrave_exception("Grasp transform has to be the identity for direction-only IK");
        if( !!_pBase )
            localgoal.SetDirection(_pBase->GetTransform().inverse().rotate(goal.GetDirection()));
        else
            localgoal.SetDirection(goal.GetDirection());
    }
    else if( goal.GetType() == IkParameterization::Type_Ray4D ) {
//        if( min((_tGrasp.rot-Vector(1,0,0,0)).lengthsqr4(),(_tGrasp.rot+Vector(1,0,0,0)).lengthsqr4()) > 1e-8f )
//            throw openrave_exception("Grasp transform has to be the identity for ray-only IK");
        if( !!_pBase ) {
            Transform tbaseinv = _pBase->GetTransform().inverse();
            localgoal.SetRay(RAY(tbaseinv*goal.GetRay().pos,tbaseinv.rotate(goal.GetRay().dir)));
        }
        else {
            localgoal.SetRay(goal.GetRay());
        }
    }
    else
        throw openrave_exception(str(boost::format("does not support parameterization %d")%goal.GetType()));

    boost::shared_ptr< vector<dReal> > psolution(&solution, null_deleter());
    return vFreeParameters.size() == 0 ? _pIkSolver->Solve(localgoal, solution, bColCheck, psolution) : _pIkSolver->Solve(localgoal, solution, vFreeParameters, bColCheck, psolution);
}
   
bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const
{
    return FindIKSolutions(goal, vector<dReal>(), solutions, bColCheck);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const
{
    if( !_pIkSolver )
        throw openrave_exception(str(boost::format("manipulator %s:%s does not have an IK solver set")%RobotBasePtr(_probot)->GetName()%GetName()));
    BOOST_ASSERT(_pIkSolver->GetManipulator() == shared_from_this() );

    IkParameterization localgoal;
    if( goal.GetType() == IkParameterization::Type_Transform6D ) {
        Transform tgoal = goal.GetTransform()*_tGrasp.inverse();
        if( !!_pBase )
            tgoal = _pBase->GetTransform().inverse() * tgoal;
        localgoal.SetTransform(tgoal);
    }
    else if( goal.GetType() == IkParameterization::Type_Rotation3D ) {
        Vector q = quatMultiply(goal.GetRotation(),quatInverse(_tGrasp.rot));
        if( !!_pBase )
            q = quatMultiply(quatInverse(_pBase->GetTransform().rot),q);
        localgoal.SetRotation(q);
    }
    else if( goal.GetType() == IkParameterization::Type_Translation3D ) {
        if( !!_pBase )
            localgoal.SetTranslation(_pBase->GetTransform().inverse()*goal.GetTranslation());
        else
            localgoal.SetTranslation(goal.GetTranslation());
    }
    else if( goal.GetType() == IkParameterization::Type_Direction3D ) {
        if( min((_tGrasp.rot-Vector(1,0,0,0)).lengthsqr4(),(_tGrasp.rot+Vector(1,0,0,0)).lengthsqr4()) > 1e-8f )
            throw openrave_exception("Grasp transform has to be the identity for direction-only IK");
        if( !!_pBase )
            localgoal.SetDirection(_pBase->GetTransform().inverse().rotate(goal.GetDirection()));
        else
            localgoal.SetDirection(goal.GetDirection());
    }
    else if( goal.GetType() == IkParameterization::Type_Ray4D ) {
        if( min((_tGrasp.rot-Vector(1,0,0,0)).lengthsqr4(),(_tGrasp.rot+Vector(1,0,0,0)).lengthsqr4()) > 1e-8f )
            throw openrave_exception("Grasp transform has to be the identity for ray-only IK");
        if( !!_pBase ) {
            Transform tbaseinv = _pBase->GetTransform().inverse();
            localgoal.SetRay(RAY(tbaseinv*goal.GetRay().pos,tbaseinv.rotate(goal.GetRay().dir)));
        }
        else {
            localgoal.SetRay(goal.GetRay());
        }
    }
    else
        throw openrave_exception(str(boost::format("does not support parameterization %d")%goal.GetType()));

    return vFreeParameters.size() == 0 ? _pIkSolver->Solve(localgoal,bColCheck,solutions) : _pIkSolver->Solve(localgoal,vFreeParameters,bColCheck,solutions);
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
        if( _varmjoints.size() > 0 && !probot->DoesAffect(_varmjoints[0],ilink) )
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
        if( _varmjoints.size() > 0 && !probot->DoesAffect(_varmjoints[0],ilink) )
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
    probot->GetRigidlyAttachedLinks(_pEndEffector->GetIndex(),vlinks);
    int iattlink = _pEndEffector->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink )
            continue;
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,_varmjoints) {
            if( !probot->DoesAffect(*itarmjoint,ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink )
            continue;
        for(int ijoint = 0; ijoint < probot->GetDOF(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                vlinks.push_back(*itlink);
                break;
            }
        }
    }
}

void RobotBase::Manipulator::GetIndependentLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(_probot);
    FOREACHC(itlink, probot->GetLinks()) {
        bool bAffected = false;
        FOREACHC(itindex,_varmjoints) {
            if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        FOREACHC(itindex,_vgripperjoints) {
            if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }

        if( !bAffected )
            vlinks.push_back(*itlink);
    }
}

template <typename T>
class TransformSaver
{
public:
    TransformSaver(T plink) : _plink(plink) { _t = _plink->GetTransform(); }
    ~TransformSaver() { _plink->SetTransform(_t); }
    const Transform& GetTransform() { return _t; }
private:
    T _plink;
    Transform _t;
};

bool RobotBase::Manipulator::CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report) const
{
    RobotBasePtr probot(_probot);
    Transform toldEE = GetEndEffectorTransform();
    Transform tdelta = tEE*toldEE.inverse();

    // get all child links of the manipualtor
    int iattlink = _pEndEffector->GetIndex();
    vector<LinkPtr> vattachedlinks;
    probot->GetRigidlyAttachedLinks(_pEndEffector->GetIndex(),vattachedlinks);
    FOREACHC(itlink,vattachedlinks) {
        if( probot->CheckLinkCollision((*itlink)->GetIndex(),tdelta*(*itlink)->GetTransform(),report) )
            return true;
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink )
            continue;
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,_varmjoints) {
            if( !probot->DoesAffect(*itarmjoint,ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink )
            continue;
        for(int ijoint = 0; ijoint < probot->GetDOF(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                if( probot->CheckLinkCollision(ilink,tdelta*(*itlink)->GetTransform(),report) )
                    return true;
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
        bool bAffected = false;
        FOREACHC(itindex,_varmjoints) {
            if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        if( !bAffected ) {
            FOREACHC(itindex,_vgripperjoints) {
                if( probot->DoesAffect(*itindex,(*itlink)->GetIndex()) ) {
                    bAffected = true;
                    break;
                }
            }
        }

        if( !bAffected ) {
            if( probot->GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink),report) )
                return true;

            // check if any grabbed bodies are attached to this link
            FOREACHC(itgrabbed,probot->_vGrabbedBodies) {
                if( itgrabbed->plinkrobot == *itlink ) {
                    if( vbodyexcluded.size() == 0 )
                        vbodyexcluded.push_back(KinBodyConstPtr(probot));
                    KinBodyPtr pbody = itgrabbed->pbody.lock();
                    if( !!pbody && probot->GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) )
                        return true;
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
        if( plink == _pEndEffector || plink == _pBase )
            return true;
        int iattlink = _pEndEffector->GetIndex();
        FOREACHC(itlink, probot->GetLinks()) {
            int ilink = (*itlink)->GetIndex();
            if( ilink == iattlink )
                continue;
            // gripper needs to be affected by all joints
            bool bGripperLink = true;
            FOREACHC(itarmjoint,_varmjoints) {
                if( !probot->DoesAffect(*itarmjoint,ilink) ) {
                    bGripperLink = false;
                    break;
                }
            }
            if( bGripperLink && plink == *itlink )
                return true;
        }
    }
    return false;
}

void RobotBase::Manipulator::serialize(std::ostream& o, int options) const
{
    o << (!_pBase ? -1 : _pBase->GetIndex()) << " " << (!_pEndEffector ? -1 : _pEndEffector->GetIndex()) << " ";
    SerializeRound(o,_tGrasp);
    SerializeRound3(o,_vdirection);
    o << _vgripperjoints.size() << " " << _varmjoints.size() << " " << _vClosingDirection.size() << " ";
    FOREACHC(it,_vgripperjoints)
        o << *it << " ";
    FOREACHC(it,_varmjoints)
        o << *it << " ";
    FOREACHC(it,_vClosingDirection)
        SerializeRound(o,*it);
}

const std::string& RobotBase::Manipulator::GetStructureHash() const
{
    return __hashstructure;
}

const std::string& RobotBase::Manipulator::GetKinematicsStructureHash() const
{
    BOOST_ASSERT(0);
    return __hashstructure;
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
    if( !!sensor.psensor ) {
        psensor = probot->GetEnv()->CreateSensor(sensor.psensor->GetXMLId());
        if( !!psensor ) {
            psensor->Clone(sensor.psensor,cloningoptions);
            if( !!psensor )
                pdata = psensor->CreateSensorData();
        }
    }
    
    int index = LinkPtr(sensor.pattachedlink)->GetIndex();
    if( index >= 0 && index < (int)probot->GetLinks().size())
        pattachedlink = probot->GetLinks().at(index);
}

RobotBase::AttachedSensor::~AttachedSensor()
{
}

SensorBase::SensorDataPtr RobotBase::AttachedSensor::GetData() const
{
    if( psensor->GetSensorData(pdata) )
        return pdata;
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
    return __hashstructure;
}

    RobotBase::RobotStateSaver::RobotStateSaver(RobotBasePtr probot, int options) : KinBodyStateSaver(probot, options), _probot(probot)
{
    if( _options & Save_ActiveDOF ) {
        vactivedofs = _probot->GetActiveJointIndices();
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
    _nActiveDOF = 0;
    vActvAffineRotationAxis = Vector(0,0,1);

    _nActiveManip = 0;
    _vecManipulators.reserve(16); // make sure to reseve enough, otherwise pIkSolver pointer might get messed up when resizing

    //set limits for the affine DOFs
    _vTranslationLowerLimits = Vector(-100,-100,-100);
    _vTranslationUpperLimits = Vector(100,100,100);
    _vTranslationMaxVels = Vector(0.07f,0.07f,0.07f);
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

    _vRotationQuatLowerLimits = Vector(-1,-1,-1,-1);
    _vRotationQuatUpperLimits = Vector(1,1,1,1);
    _vRotationQuatMaxVels = Vector(0.07f,0.07f,0.07f,0.07f);
    _vRotationQuatResolutions = Vector(0.01f,0.01f,0.01f,0.01f);
    _vRotationQuatWeights = Vector(0.4f,0.4f,0.4f,0.4f);
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
    SetController(ControllerBasePtr(),"");
    
    KinBody::Destroy();
}

void RobotBase::SetJointValues(const std::vector<dReal>& vJointValues, bool bCheckLimits)
{
    KinBody::SetJointValues(vJointValues, bCheckLimits);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetJointValues(const std::vector<dReal>& vJointValues, const Transform& transbase, bool bCheckLimits)
{
    KinBody::SetJointValues(vJointValues, transbase, bCheckLimits); // should call RobotBase::SetJointValues
}

void RobotBase::SetBodyTransformations(const std::vector<Transform>& vbodies)
{
    KinBody::SetBodyTransformations(vbodies);
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
    // update grabbed objects
    vector<Grabbed>::iterator itbody;
    FORIT(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody = itbody->pbody.lock();
        if( !!pbody )
            pbody->SetTransform(itbody->plinkrobot->GetTransform() * itbody->troot);
        else {
            RAVELOG_DEBUGA(str(boost::format("erasing invaliding grabbed body from %s")%GetName()));
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

int RobotBase::GetAffineDOFIndex(DOFAffine dof) const
{
    if( !(_nAffineDOFs&dof) )
        return -1;

    int index = (int)_vActiveJointIndices.size();
    if( dof&DOF_X ) return index;
    else if( _nAffineDOFs & DOF_X ) ++index;

    if( dof&DOF_Y ) return index;
    else if( _nAffineDOFs & DOF_Y ) ++index;

    if( dof&DOF_Z ) return index;
    else if( _nAffineDOFs & DOF_Z ) ++index;

    if( dof&DOF_RotationAxis ) return index;
    if( dof&DOF_Rotation3D ) return index;
    if( dof&DOF_RotationQuat ) return index;

    throw openrave_exception("unspecified dow",ORE_InvalidArguments);
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

void RobotBase::SetAffineRotationQuatLimits(const Vector& lower, const Vector& upper)
{
    _vRotationQuatLowerLimits = lower;
    _vRotationQuatUpperLimits = upper;
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

void RobotBase::SetAffineRotationQuatMaxVels(const Vector& vels)
{
    _vRotationQuatMaxVels = vels;
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

void RobotBase::SetAffineRotationQuatResolution(const Vector& resolution)
{
    _vRotationQuatResolutions = resolution;
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

void RobotBase::SetAffineRotationQuatWeights(const Vector& weights)
{
    _vRotationQuatWeights = weights;
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

void RobotBase::GetAffineRotationQuatLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotationQuatLowerLimits;
    upper = _vRotationQuatUpperLimits;
}


void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask, const Vector& vRotationAxis)
{
    vActvAffineRotationAxis = vRotationAxis;
    SetActiveDOFs(vJointIndices,nAffineDOFBitmask);
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask)
{
    FOREACHC(itj, vJointIndices)
        if( *itj < 0 || *itj >= (int)GetDOF() )
            throw openrave_exception("bad indices",ORE_InvalidArguments);

    _vActiveJointIndices = vJointIndices;
    _nAffineDOFs = nAffineDOFBitmask;

    // mutually exclusive
    if( _nAffineDOFs & DOF_RotationAxis )
        _nAffineDOFs &= ~(DOF_Rotation3D|DOF_RotationQuat);
    else if( _nAffineDOFs & DOF_Rotation3D )
        _nAffineDOFs &= ~(DOF_RotationAxis|DOF_RotationQuat);
    else if( _nAffineDOFs & DOF_RotationQuat )
        _nAffineDOFs &= ~(DOF_RotationAxis|DOF_Rotation3D);

    _nActiveDOF = 0;
    if( _nAffineDOFs & DOF_X ) _nActiveDOF++;
    if( _nAffineDOFs & DOF_Y ) _nActiveDOF++;
    if( _nAffineDOFs & DOF_Z ) _nActiveDOF++;
    if( _nAffineDOFs & DOF_RotationAxis ) {
        _nActiveDOF++; 
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) _nActiveDOF += 3;
    else if( _nAffineDOFs & DOF_RotationQuat ) _nActiveDOF += 4;

    _nActiveDOF += vJointIndices.size();
}

void RobotBase::SetActiveDOFValues(const std::vector<dReal>& values, bool bCheckLimits)
{
    if(_nActiveDOF == 0) {
        SetJointValues(values,bCheckLimits);
        return;
    }

    if( (int)values.size() != GetActiveDOF() )
        throw openrave_exception(str(boost::format("dof not equal %d!=%d")%values.size()%GetActiveDOF()),ORE_InvalidArguments);

    if( (int)values.size() != GetActiveDOF() )
        throw openrave_exception(str(boost::format("dof not equal %d!=%d")%values.size()%GetActiveDOF()));

    Transform t;
    if( (int)_vActiveJointIndices.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = &values[_vActiveJointIndices.size()];

        t = GetTransform();
        
        if( _nAffineDOFs & DOF_X ) t.trans.x = *pAffineValues++;
        if( _nAffineDOFs & DOF_Y ) t.trans.y = *pAffineValues++;
        if( _nAffineDOFs & DOF_Z ) t.trans.z = *pAffineValues++;
        if( _nAffineDOFs & DOF_RotationAxis ) {
            dReal fsin = sin(pAffineValues[0]*(dReal)0.5);
            t.rot.x = cos(pAffineValues[0]*(dReal)0.5);
            t.rot.y = vActvAffineRotationAxis.x * fsin;
            t.rot.z = vActvAffineRotationAxis.y * fsin;
            t.rot.w = vActvAffineRotationAxis.z * fsin;
        }
        else if( _nAffineDOFs & DOF_Rotation3D ) {
            dReal fang = sqrt(pAffineValues[0] * pAffineValues[0] + pAffineValues[1] * pAffineValues[1] + pAffineValues[2] * pAffineValues[2]);
            if( fang > 0 ) {
                dReal fnormalizer = sin((dReal)0.5 * fang) / fang;
                t.rot.x = cos((dReal)0.5 * fang);
                t.rot.y = fnormalizer * pAffineValues[0];
                t.rot.z = fnormalizer * pAffineValues[1];
                t.rot.w = fnormalizer * pAffineValues[2];
            }
            else
                t.rot = Vector(1,0,0,0); // identity
        }
        else if( _nAffineDOFs & DOF_RotationQuat ) {
            // have to normalize since user might not be aware of this particular parameterization of rotations
            t.rot = *(Vector*)pAffineValues;
            dReal flength = t.rot.lengthsqr4();
            if( flength > 0 ) t.rot /= RaveSqrt(flength);
            else t.rot = Vector(1,0,0,0);
        }

        if( _vActiveJointIndices.size() == 0 )
            SetTransform(t);
    }

    if( _vActiveJointIndices.size() > 0 ) {
        GetDOFValues(_vTempRobotJoints);

        for(size_t i = 0; i < _vActiveJointIndices.size(); ++i)
            _vTempRobotJoints[_vActiveJointIndices[i]] = values[i];

        if( (int)_vActiveJointIndices.size() < _nActiveDOF )
            SetJointValues(_vTempRobotJoints, t, bCheckLimits);
        else
            SetJointValues(_vTempRobotJoints, bCheckLimits);
    }
}

void RobotBase::GetActiveDOFValues(std::vector<dReal>& values) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFValues(values);
        return;
    }

    values.resize(GetActiveDOF());
    if( values.size() == 0 )
        return;
    dReal* pValues = &values[0];
    if( _vActiveJointIndices.size() != 0 ) {
        GetDOFValues(_vTempRobotJoints);

        FOREACHC(it, _vActiveJointIndices)
            *pValues++ = _vTempRobotJoints[*it];
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    Transform t = GetTransform();
    if( _nAffineDOFs & DOF_X ) *pValues++ = t.trans.x;
    if( _nAffineDOFs & DOF_Y ) *pValues++ = t.trans.y;
    if( _nAffineDOFs & DOF_Z ) *pValues++ = t.trans.z;
    if( _nAffineDOFs & DOF_RotationAxis ) {
        // assume that rot.yzw ~= vActvAffineRotationAxis
        dReal fsin = RaveSqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);

        // figure out correct sign
        if( (t.rot.y > 0) != (vActvAffineRotationAxis.x>0) || (t.rot.z > 0) != (vActvAffineRotationAxis.y>0) || (t.rot.w > 0) != (vActvAffineRotationAxis.z>0) )
            fsin = -fsin;

        *pValues++ = 2 * RaveAtan2(fsin, t.rot.x);
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        dReal fsin = RaveSqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
        dReal fangle = 2 * atan2(fsin, t.rot.x);

        if( fsin > 0 ) {
            dReal normalizer = fangle / fsin;
            *pValues++ = normalizer * t.rot.y;
            *pValues++ = normalizer * t.rot.z;
            *pValues++ = normalizer * t.rot.w;
        }
        else {
            *pValues++ = 0;
            *pValues++ = 0;
            *pValues++ = 0;
        }
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *(Vector*)pValues = t.rot;
    }
}

void RobotBase::SetActiveDOFVelocities(const std::vector<dReal>& velocities)
{
    if(_nActiveDOF == 0) {
        SetJointVelocities(velocities);
        return;
    }

    Vector linearvel, angularvel;

    if( (int)_vActiveJointIndices.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = &velocities[_vActiveJointIndices.size()];

        GetVelocity(linearvel, angularvel);
        
        if( _nAffineDOFs & DOF_X ) linearvel.x = *pAffineValues++;
        if( _nAffineDOFs & DOF_Y ) linearvel.y = *pAffineValues++;
        if( _nAffineDOFs & DOF_Z ) linearvel.z = *pAffineValues++;
        if( _nAffineDOFs & DOF_RotationAxis ) {
            angularvel = vActvAffineRotationAxis * *pAffineValues++;
        }
        else if( _nAffineDOFs & DOF_Rotation3D ) {
            angularvel.x = *pAffineValues++;
            angularvel.y = *pAffineValues++;
            angularvel.z = *pAffineValues++;
        }
        else if( _nAffineDOFs & DOF_RotationQuat ) {
            throw openrave_exception("quaternions not supported",ORE_InvalidArguments);
        }

        if( _vActiveJointIndices.size() == 0 )
            SetVelocity(linearvel, angularvel);
    }

    if( _vActiveJointIndices.size() > 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        std::vector<dReal>::const_iterator itvel = velocities.begin();
        FOREACHC(it, _vActiveJointIndices)
            _vTempRobotJoints[*it] = *itvel++;
        SetJointVelocities(_vTempRobotJoints);
    }
}

void RobotBase::GetActiveDOFVelocities(std::vector<dReal>& velocities) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFVelocities(velocities);
        return;
    }

    velocities.resize(GetActiveDOF());
    if( velocities.size() == 0 )
        return;
    dReal* pVelocities = &velocities[0];
    if( _vActiveJointIndices.size() != 0 ) {
        GetDOFVelocities(_vTempRobotJoints);

        FOREACHC(it, _vActiveJointIndices)
            *pVelocities++ = _vTempRobotJoints[*it];
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    Vector linearvel, angularvel;
    GetVelocity(linearvel, angularvel);

    if( _nAffineDOFs & DOF_X ) *pVelocities++ = linearvel.x;
    if( _nAffineDOFs & DOF_Y ) *pVelocities++ = linearvel.y;
    if( _nAffineDOFs & DOF_Z ) *pVelocities++ = linearvel.z;
    if( _nAffineDOFs & DOF_RotationAxis ) {

        *pVelocities++ = dot3(vActvAffineRotationAxis, angularvel);
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pVelocities++ = angularvel.x;
        *pVelocities++ = angularvel.y;
        *pVelocities++ = angularvel.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        throw openrave_exception("quaternions not supported",ORE_InvalidArguments);
    }
}

void RobotBase::GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const
{
    lower.resize(GetActiveDOF());
    upper.resize(GetActiveDOF());
    if( GetActiveDOF() == 0 )
        return;

    dReal* pLowerLimit = &lower[0];
    dReal* pUpperLimit = &upper[0];
    vector<dReal> alllower,allupper;

    if( _nAffineDOFs == 0 ) {
        if( _nActiveDOF == 0 ) {
            GetDOFLimits(lower,upper);
            return;
        }
        else {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, _vActiveJointIndices) {
                *pLowerLimit++ = alllower[*it];
                *pUpperLimit++ = allupper[*it];
            }
        }
    }
    else {
        if( _vActiveJointIndices.size() > 0 ) {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, _vActiveJointIndices) {
                *pLowerLimit++ = alllower[*it];
                *pUpperLimit++ = allupper[*it];
            }
        }

        if( _nAffineDOFs & DOF_X ) { *pLowerLimit++ = _vTranslationLowerLimits.x; *pUpperLimit++ = _vTranslationUpperLimits.x; }
        if( _nAffineDOFs & DOF_Y ) { *pLowerLimit++ = _vTranslationLowerLimits.y; *pUpperLimit++ = _vTranslationUpperLimits.y; }
        if( _nAffineDOFs & DOF_Z ) { *pLowerLimit++ = _vTranslationLowerLimits.z; *pUpperLimit++ = _vTranslationUpperLimits.z; }

        if( _nAffineDOFs & DOF_RotationAxis ) { *pLowerLimit++ = _vRotationAxisLowerLimits.x; *pUpperLimit++ = _vRotationAxisUpperLimits.x; }
        else if( _nAffineDOFs & DOF_Rotation3D ) {
            *pLowerLimit++ = _vRotation3DLowerLimits.x;
            *pLowerLimit++ = _vRotation3DLowerLimits.y;
            *pLowerLimit++ = _vRotation3DLowerLimits.z;
            *pUpperLimit++ = _vRotation3DUpperLimits.x;
            *pUpperLimit++ = _vRotation3DUpperLimits.y;
            *pUpperLimit++ = _vRotation3DUpperLimits.z;
        }
        else if( _nAffineDOFs & DOF_RotationQuat ) {
            *pLowerLimit++ = _vRotationQuatLowerLimits.x;
            *pLowerLimit++ = _vRotationQuatLowerLimits.y;
            *pLowerLimit++ = _vRotationQuatLowerLimits.z;
            *pLowerLimit++ = _vRotationQuatLowerLimits.w;
            *pUpperLimit++ = _vRotationQuatUpperLimits.x;
            *pUpperLimit++ = _vRotationQuatUpperLimits.y;
            *pUpperLimit++ = _vRotationQuatUpperLimits.z;
            *pUpperLimit++ = _vRotationQuatUpperLimits.w;
        }
    }
}    

void RobotBase::GetActiveDOFResolutions(std::vector<dReal>& resolution) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFResolutions(resolution);
        return;
    }
    
    resolution.resize(GetActiveDOF());
    if( resolution.size() == 0 )
        return;
    dReal* pResolution = &resolution[0];

    GetDOFResolutions(_vTempRobotJoints);
    FOREACHC(it, _vActiveJointIndices)
        *pResolution++ = _vTempRobotJoints[*it];

    // set some default limits 
    if( _nAffineDOFs & DOF_X ) { *pResolution++ = _vTranslationResolutions.x;}
    if( _nAffineDOFs & DOF_Y ) { *pResolution++ = _vTranslationResolutions.y;}
    if( _nAffineDOFs & DOF_Z ) { *pResolution++ = _vTranslationResolutions.z;}

    if( _nAffineDOFs & DOF_RotationAxis ) { *pResolution++ = _vRotationAxisResolutions.x; }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pResolution++ = _vRotation3DResolutions.x;
        *pResolution++ = _vRotation3DResolutions.y;
        *pResolution++ = _vRotation3DResolutions.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pResolution++ = _vRotationQuatResolutions.x;
        *pResolution++ = _vRotationQuatResolutions.y;
        *pResolution++ = _vRotationQuatResolutions.z;
        *pResolution++ = _vRotationQuatResolutions.w;
    }
}

void RobotBase::GetActiveDOFWeights(std::vector<dReal>& weights) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFWeights(weights);
        return;
    }
    
    weights.resize(GetActiveDOF());
    if( weights.size() == 0 )
        return;
    dReal* pweight = &weights[0];

    GetDOFWeights(_vTempRobotJoints);
    FOREACHC(it, _vActiveJointIndices)
        *pweight++ = _vTempRobotJoints[*it];

    // set some default limits 
    if( _nAffineDOFs & DOF_X ) { *pweight++ = _vTranslationWeights.x;}
    if( _nAffineDOFs & DOF_Y ) { *pweight++ = _vTranslationWeights.y;}
    if( _nAffineDOFs & DOF_Z ) { *pweight++ = _vTranslationWeights.z;}

    if( _nAffineDOFs & DOF_RotationAxis ) { *pweight++ = _vRotationAxisWeights.x; }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pweight++ = _vRotation3DWeights.x;
        *pweight++ = _vRotation3DWeights.y;
        *pweight++ = _vRotation3DWeights.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pweight++ = _vRotationQuatWeights.x;
        *pweight++ = _vRotationQuatWeights.y;
        *pweight++ = _vRotationQuatWeights.z;
        *pweight++ = _vRotationQuatWeights.w;
    }
}

void RobotBase::GetActiveDOFMaxVel(std::vector<dReal>& maxvel) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFMaxVel(maxvel);
        return;
    }

    maxvel.resize(GetActiveDOF());
    if( maxvel.size() == 0 )
        return;
    dReal* pMaxVel = &maxvel[0];

    GetDOFMaxVel(_vTempRobotJoints);
    FOREACHC(it, _vActiveJointIndices)
        *pMaxVel++ = _vTempRobotJoints[*it];

    if( _nAffineDOFs & DOF_X ) { *pMaxVel++ = _vTranslationMaxVels.x;}
    if( _nAffineDOFs & DOF_Y ) { *pMaxVel++ = _vTranslationMaxVels.y;}
    if( _nAffineDOFs & DOF_Z ) { *pMaxVel++ = _vTranslationMaxVels.z;}

    if( _nAffineDOFs & DOF_RotationAxis ) { *pMaxVel++ = _vRotationAxisMaxVels.x; }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pMaxVel++ = _vRotation3DMaxVels.x;
        *pMaxVel++ = _vRotation3DMaxVels.y;
        *pMaxVel++ = _vRotation3DMaxVels.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pMaxVel++ = _vRotationQuatMaxVels.x;
        *pMaxVel++ = _vRotationQuatMaxVels.y;
        *pMaxVel++ = _vRotationQuatMaxVels.z;
        *pMaxVel++ = _vRotationQuatMaxVels.w;
    }
}

void RobotBase::GetActiveDOFMaxAccel(std::vector<dReal>& maxaccel) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFMaxAccel(maxaccel);
        return;
    }

    maxaccel.resize(GetActiveDOF());
    if( maxaccel.size() == 0 )
        return;
    dReal* pMaxAccel = &maxaccel[0];

    GetDOFMaxAccel(_vTempRobotJoints);
    FOREACHC(it, _vActiveJointIndices)
        *pMaxAccel++ = _vTempRobotJoints[*it];

    if( _nAffineDOFs & DOF_X ) { *pMaxAccel++ = _vTranslationMaxVels.x;} // wrong
    if( _nAffineDOFs & DOF_Y ) { *pMaxAccel++ = _vTranslationMaxVels.y;} // wrong
    if( _nAffineDOFs & DOF_Z ) { *pMaxAccel++ = _vTranslationMaxVels.z;} // wrong

    if( _nAffineDOFs & DOF_RotationAxis ) { *pMaxAccel++ = _vRotationAxisMaxVels.x; } // wrong
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pMaxAccel++ = _vRotation3DMaxVels.x; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.y; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.z; // wrong
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pMaxAccel++ = _vRotationQuatMaxVels.x; // wrong
        *pMaxAccel++ = _vRotationQuatMaxVels.y; // wrong
        *pMaxAccel++ = _vRotationQuatMaxVels.z; // wrong
        *pMaxAccel++ = _vRotationQuatMaxVels.w; // wrong
    }
}

void RobotBase::SubtractActiveDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    if( _nActiveDOF == 0 ) {
        SubtractJointValues(q1,q2);
        return;
    }

    // go through all active joints
    int index = 0;
    FOREACHC(it,_vActiveJointIndices) {
        JointConstPtr pjoint = _vecjoints.at(*it);
        if( pjoint->IsCircular() ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i, index++)
                q1.at(index) = ANGLE_DIFF(q1.at(index), q2.at(index));
        }
        else {
            for(int i = 0; i < pjoint->GetDOF(); ++i, index++)
                q1.at(index) -= q2.at(index);
        }
    }

    if( _nAffineDOFs & DOF_X ) { q1.at(index) -= q2.at(index); index++; }
    if( _nAffineDOFs & DOF_Y ) { q1.at(index) -= q2.at(index); index++; }
    if( _nAffineDOFs & DOF_Z ) { q1.at(index) -= q2.at(index); index++; }

    if( _nAffineDOFs & DOF_RotationAxis ) { q1.at(index) = ANGLE_DIFF(q1.at(index),q2.at(index)); index++;  }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
}

void RobotBase::GetControlMaxTorques(std::vector<dReal>& maxtorques) const
{
    if( _nActiveDOF == 0 ) {
        GetDOFMaxTorque(maxtorques);
        return;
    }

    maxtorques.resize(GetActiveDOF());
    if( maxtorques.size() == 0 )
        return;
    dReal* pMaxTorques = &maxtorques[0];

    if( _vActiveJointIndices.size() != 0 ) {
        GetDOFMaxTorque(_vTempRobotJoints);

        FOREACHC(it, _vActiveJointIndices)
            *pMaxTorques++ = _vTempRobotJoints[*it];
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    if( _nAffineDOFs & DOF_X ) *pMaxTorques++ = 0;
    if( _nAffineDOFs & DOF_Y ) *pMaxTorques++ = 0;
    if( _nAffineDOFs & DOF_Z ) *pMaxTorques++ = 0;
    if( _nAffineDOFs & DOF_RotationAxis ) *pMaxTorques++ = 0;
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pMaxTorques++ = 0;
        *pMaxTorques++ = 0;
        *pMaxTorques++ = 0;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pMaxTorques++ = 0;
        *pMaxTorques++ = 0;
        *pMaxTorques++ = 0;
        *pMaxTorques++ = 0;
    }
}

void RobotBase::SetControlTorques(const std::vector<dReal>& vtorques)
{
    if(_nActiveDOF == 0) {
        SetJointTorques(vtorques, false);
        return;
    }

    if( _vActiveJointIndices.size() > 0 ) {
        _vTempRobotJoints.resize(GetDOF());
        std::vector<dReal>::const_iterator ittorque = vtorques.begin();
        FOREACHC(it, _vActiveJointIndices)
            _vTempRobotJoints[*it] = *ittorque++;
        SetJointTorques(_vTempRobotJoints,false);
    }
}

void RobotBase::GetFullTrajectoryFromActive(TrajectoryBasePtr pFullTraj, TrajectoryBaseConstPtr pActiveTraj, bool bOverwriteTransforms)
{
    BOOST_ASSERT( !!pFullTraj && !!pActiveTraj );
    BOOST_ASSERT( pFullTraj != pActiveTraj );

    pFullTraj->Reset(GetDOF());

    if( _nActiveDOF == 0 ) {
        // make sure the affine transformation stays the same!
        Transform tbase = GetTransform();
        FOREACHC(it, pActiveTraj->GetPoints()) {
            Trajectory::TPOINT p = *it;
            if( bOverwriteTransforms )
                p.trans = tbase;
            else
                p.trans = it->trans;
            pFullTraj->AddPoint(p);
        }
    }
    else {
        Trajectory::TPOINT p;
        p.trans = GetTransform();
        GetDOFValues(p.q);
        p.qdot.resize(GetDOF());
        memset(&p.qdot[0], 0, sizeof(p.qdot[0]) * GetDOF());

        vector<Trajectory::TPOINT>::const_iterator itp;
        FORIT(itp, pActiveTraj->GetPoints()) {

            int i=0;
            for(i = 0; i < (int)_vActiveJointIndices.size(); ++i) {
                p.q[_vActiveJointIndices[i]] = itp->q[i];
            }

            if( !bOverwriteTransforms )
                p.trans = itp->trans;
            
            if( _nAffineDOFs != DOF_NoTransform ) {
                if( _nAffineDOFs & DOF_X ) p.trans.trans.x = itp->q[i++];
                if( _nAffineDOFs & DOF_Y ) p.trans.trans.y = itp->q[i++];
                if( _nAffineDOFs & DOF_Z ) p.trans.trans.z = itp->q[i++];
                if( _nAffineDOFs & DOF_RotationAxis ) {
                    dReal fsin = sin(itp->q[i]*(dReal)0.5);
                    p.trans.rot.x = cos(itp->q[i]*(dReal)0.5);
                    p.trans.rot.y = vActvAffineRotationAxis.x * fsin;
                    p.trans.rot.z = vActvAffineRotationAxis.y * fsin;
                    p.trans.rot.w = vActvAffineRotationAxis.z * fsin;
                }
                else if( _nAffineDOFs & DOF_Rotation3D ) {
                    dReal fang = sqrt(itp->q[i] * itp->q[i] + itp->q[i+1] * itp->q[i+1] + itp->q[i+2] * itp->q[i+2]);
                    dReal fnormalizer = sin((dReal)0.5 * fang) / fang;
                    p.trans.rot.x = cos((dReal)0.5 * fang);
                    p.trans.rot.y = fnormalizer * itp->q[i];
                    p.trans.rot.z = fnormalizer * itp->q[i+1];
                    p.trans.rot.w = fnormalizer * itp->q[i+2];
                }
                else if( _nAffineDOFs & DOF_RotationQuat ) {
                    p.trans.rot = *(Vector*)&itp->q[i];
                }
            }

            p.time = itp->time;
            p.qtorque = itp->qtorque;
            pFullTraj->AddPoint(p);
        }
    }

    pFullTraj->CalcTrajTiming(shared_robot(), pActiveTraj->GetInterpMethod(), false, false);
}

const std::vector<int>& RobotBase::GetActiveJointIndices() const
{
    return _nActiveDOF == 0 ? _vAllDOFIndices : _vActiveJointIndices;
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF == 0 ) {
        CalculateJacobian(index, offset, mjacobian);
        return;
    }

    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    if( _vActiveJointIndices.size() != 0 ) {
        boost::multi_array<dReal,2> mjacobianjoints;
        CalculateJacobian(index, offset, mjacobianjoints);
        for(size_t i = 0; i < _vActiveJointIndices.size(); ++i) {
            mjacobian[0][i] = mjacobianjoints[0][_vActiveJointIndices[i]];
            mjacobian[1][i] = mjacobianjoints[1][_vActiveJointIndices[i]];
            mjacobian[2][i] = mjacobianjoints[2][_vActiveJointIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    size_t ind = _vActiveJointIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        mjacobian[0][ind] = 1;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 1;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 1;
        ind++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        Vector vj;
        cross3(vj, vActvAffineRotationAxis, GetTransform().trans-offset);
        mjacobian[0][ind] = vj.x;
        mjacobian[1][ind] = vj.y;
        mjacobian[2][ind] = vj.z;
        ind++;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
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
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        Transform t = GetTransform();
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
    if( _nActiveDOF == 0 ) {
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
    if( _nActiveDOF == 0 ) {
        CalculateJacobian(index, q, mjacobian);
        return;
    }

    mjacobian.resize(boost::extents[4][GetActiveDOF()]);
    if( _vActiveJointIndices.size() != 0 ) {
        boost::multi_array<dReal,2> mjacobianjoints;
        CalculateRotationJacobian(index, q, mjacobianjoints);
        for(size_t i = 0; i < _vActiveJointIndices.size(); ++i) {
            mjacobian[0][i] = mjacobianjoints[0][_vActiveJointIndices[i]];
            mjacobian[1][i] = mjacobianjoints[1][_vActiveJointIndices[i]];
            mjacobian[2][i] = mjacobianjoints[2][_vActiveJointIndices[i]];
            mjacobian[3][i] = mjacobianjoints[3][_vActiveJointIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    size_t ind = _vActiveJointIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        mjacobian[3][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        mjacobian[3][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        mjacobian[3][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        mjacobian[0][ind] = -q.y*v.x - q.z*v.y - q.w*v.z;
        mjacobian[1][ind] = q.x*v.x - q.z*v.z + q.w*v.y;
        mjacobian[2][ind] = q.x*v.y + q.y*v.z - q.w*v.x;
        mjacobian[3][ind] = q.x*v.z - q.y*v.y + q.z*v.x;
        ind++;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        BOOST_ASSERT(!"rotation 3d not supported");
        ind += 3;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        BOOST_ASSERT(!"quaternion not supported");
        ind += 4;
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF == 0 ) {
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
    if( _nActiveDOF == 0 ) {
        CalculateAngularVelocityJacobian(index, mjacobian);
        return;
    }

    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    if( _vActiveJointIndices.size() != 0 ) {
        boost::multi_array<dReal,2> mjacobianjoints;
        CalculateAngularVelocityJacobian(index, mjacobianjoints);
        for(size_t i = 0; i < _vActiveJointIndices.size(); ++i) {
            mjacobian[0][i] = mjacobianjoints[0][_vActiveJointIndices[i]];
            mjacobian[1][i] = mjacobianjoints[1][_vActiveJointIndices[i]];
            mjacobian[2][i] = mjacobianjoints[2][_vActiveJointIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    size_t ind = _vActiveJointIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        mjacobian[0][ind] = 0;
        mjacobian[1][ind] = 0;
        mjacobian[2][ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        mjacobian[0][ind] = v.x;
        mjacobian[1][ind] = v.y;
        mjacobian[2][ind] = v.z;

    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        BOOST_ASSERT(!"rotation 3d not supported");
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        BOOST_ASSERT(!"quaternions not supported");

        // most likely wrong
        Transform t = GetTransform();
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
    if( _nActiveDOF == 0 ) {
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

bool RobotBase::InitFromFile(const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts)
{
    bool bSuccess = GetEnv()->ReadRobotXMLFile(shared_robot(), filename, atts)==shared_robot();
    if( !bSuccess ) {
        Destroy();
        return false;
    }

    return true;
}

bool RobotBase::InitFromData(const std::string& data, const std::list<std::pair<std::string,std::string> >& atts)
{
    bool bSuccess = GetEnv()->ReadRobotXMLData(shared_robot(), data, atts)==shared_robot();
    if( !bSuccess ) {
        Destroy();
        return false;
    }

    return true;
}

bool RobotBase::Grab(KinBodyPtr pbody)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip )
        return false;
    return Grab(pbody, pmanip->GetEndEffector());
}

bool RobotBase::Grab(KinBodyPtr pbody, const std::set<int>& setRobotLinksToIgnore)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip )
        return false;
    return Grab(pbody, pmanip->GetEndEffector(), setRobotLinksToIgnore);
}

bool RobotBase::Grab(KinBodyPtr pbody, LinkPtr plink)
{
    if( !pbody || !plink || plink->GetParent() != shared_kinbody() )
        throw openrave_exception("invalid grab arguments",ORE_InvalidArguments);
    if( pbody == shared_kinbody() )
        throw openrave_exception("robot cannot grab itself",ORE_InvalidArguments);

    if( IsGrabbing(pbody) ) {
        RAVELOG_VERBOSEA("Robot %s: body %s already grabbed\n", GetName().c_str(), pbody->GetName().c_str());
        return true;
    }

    _vGrabbedBodies.push_back(Grabbed());
    Grabbed& g = _vGrabbedBodies.back();
    g.pbody = pbody;
    g.plinkrobot = plink;
    g.troot = plink->GetTransform().inverse() * pbody->GetTransform();

    // check collision with all links to see which are valid
    FOREACH(itlink, _veclinks) {
        if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), KinBodyConstPtr(pbody)) )
            g.vCollidingLinks.push_back(*itlink);
        else
            g.vNonCollidingLinks.push_back(*itlink);
    }

    _AttachBody(pbody);
    return true;
}

bool RobotBase::Grab(KinBodyPtr pbody, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore)
{
    if( !pbody || !pRobotLinkToGrabWith || pRobotLinkToGrabWith->GetParent() != shared_kinbody() )
        throw openrave_exception("invalid grab arguments",ORE_InvalidArguments);
    if( pbody == shared_kinbody() )
        throw openrave_exception("robot cannot grab itself",ORE_InvalidArguments);

    if( IsGrabbing(pbody) ) {
        RAVELOG_VERBOSEA("Robot %s: body %s already grabbed\n", GetName().c_str(), pbody->GetName().c_str());
        return true;
    }

    _vGrabbedBodies.push_back(Grabbed());
    Grabbed& g = _vGrabbedBodies.back();
    g.pbody = pbody;
    g.plinkrobot = pRobotLinkToGrabWith;
    g.troot = pRobotLinkToGrabWith->GetTransform().inverse() * pbody->GetTransform();

    // check collision with all links to see which are valid
    FOREACH(itlink, _veclinks) {
        if( setRobotLinksToIgnore.find((*itlink)->GetIndex()) != setRobotLinksToIgnore.end() )
            continue;
        if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), KinBodyConstPtr(pbody)) )
            g.vCollidingLinks.push_back(*itlink);
        else
            g.vNonCollidingLinks.push_back(*itlink);
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
        RAVELOG_DEBUGA(str(boost::format("Robot %s: body %s not grabbed\n")%GetName()%pbody->GetName()));
        return;
    }

    _vGrabbedBodies.erase(itbody);
    _RemoveAttachedBody(pbody);
}

void RobotBase::ReleaseAllGrabbed()
{
    FOREACH(itgrabbed, _vGrabbedBodies) {
        KinBodyPtr pbody = itgrabbed->pbody.lock();
        if( !!pbody )
            _RemoveAttachedBody(pbody);
    }
    _vGrabbedBodies.clear();
}

void RobotBase::RegrabAll()
{
    FOREACH(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody(itbody->pbody);
        if( !!pbody ) {
            // check collision with all links to see which are valid
            itbody->vCollidingLinks.resize(0);
            itbody->vNonCollidingLinks.resize(0);
            _RemoveAttachedBody(pbody);
            FOREACH(itlink, _veclinks) {
                if( GetEnv()->CheckCollision(LinkConstPtr(*itlink), KinBodyConstPtr(pbody)) )
                    itbody->vCollidingLinks.push_back(*itlink);
                else
                    itbody->vNonCollidingLinks.push_back(*itlink);
            }
            _AttachBody(pbody);
        }
    }
}

RobotBase::LinkPtr RobotBase::IsGrabbing(KinBodyConstPtr pbody) const
{
    FOREACHC(itbody, _vGrabbedBodies) {
        if( KinBodyConstPtr(itbody->pbody) == pbody )
            return itbody->plinkrobot;
    }

    return LinkPtr();
}

void RobotBase::GetGrabbed(std::vector<KinBodyPtr>& vbodies) const
{
    vbodies.resize(0);
    FOREACHC(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody = itbody->pbody.lock();
        if( !!pbody && pbody->GetEnvironmentId() )
            vbodies.push_back(pbody);
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
        throw openrave_exception(str(boost::format("failed to find manipulator with name: %s")%manipname));
    }

    _nActiveManip = -1;
}

RobotBase::ManipulatorPtr RobotBase::GetActiveManipulator()
{
    if(_nActiveManip < 0 && _nActiveManip >= (int)_vecManipulators.size() )
        throw RobotBase::ManipulatorPtr();
    return _vecManipulators.at(_nActiveManip);
}

RobotBase::ManipulatorConstPtr RobotBase::GetActiveManipulator() const
{
    if(_nActiveManip < 0 && _nActiveManip >= (int)_vecManipulators.size() )
        return RobotBase::ManipulatorPtr();
    return _vecManipulators.at(_nActiveManip);
}

/// Check if body is self colliding. Links that are joined together are ignored.
bool RobotBase::CheckSelfCollision(CollisionReportPtr report) const
{
    if( KinBody::CheckSelfCollision(report) )
        return true;

    // check all grabbed bodies with 
    bool bCollision = false;
    FOREACHC(itbody, _vGrabbedBodies) {
        KinBodyPtr pbody(itbody->pbody);
        if( !pbody )
            continue;
        FOREACHC(itrobotlink,itbody->vNonCollidingLinks) {
            // have to use link/link collision since link/body checks attached bodies
            FOREACHC(itbodylink,pbody->GetLinks()) {
                if( GetEnv()->CheckCollision(*itrobotlink,KinBody::LinkConstPtr(*itbodylink),report) ) {
                    bCollision = true;
                    break;
                }
            }
            if( bCollision )
                break;
        }
        if( bCollision )
            break;

        if( pbody->CheckSelfCollision(report) ) {
            bCollision = true;
            break;
        }

        // check attached bodies with each other, this is actually tricky since they are attached "with each other", so regular CheckCollision will not work.
        // Instead, we will compare each of the body's links with every other
        if( _vGrabbedBodies.size() > 1 ) {
            FOREACHC(itbody2, _vGrabbedBodies) {
                KinBodyPtr pbody2(itbody2->pbody);
                if( pbody != pbody2 )
                    continue;
                FOREACHC(itlink, pbody->GetLinks()) {
                    FOREACHC(itlink2, pbody2->GetLinks()) {
                        if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink),KinBody::LinkConstPtr(*itlink2),report) ) {
                            bCollision = true;
                            break;
                        }
                    }
                    if( bCollision )
                        break;
                }
                if( bCollision )
                    break;
            }
            if( bCollision )
                break;
        }
    }
    
    if( bCollision && !!report )
        RAVELOG_VERBOSEA(str(boost::format("Self collision: %s\n")%report->__str__()));
    return bCollision;
}

bool RobotBase::CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    boost::shared_ptr<TransformSaver<RobotBase::LinkPtr> > linksaver(new TransformSaver<RobotBase::LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack
    plink->SetTransform(tlinktrans);
    if( GetEnv()->CheckCollision(LinkConstPtr(plink),report) )
        return true;
    
    // check if any grabbed bodies are attached to this link
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        if( itgrabbed->plinkrobot == plink ) {
            KinBodyPtr pbody = itgrabbed->pbody.lock();
            if( !!pbody ) {
                if( vbodyexcluded.size() == 0 )
                    vbodyexcluded.push_back(shared_kinbody_const());
                KinBodyStateSaver bodysaver(pbody);
                pbody->SetTransform(plink->GetTransform() * itgrabbed->troot);
                if( GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) )
                    return true;
            }
        }
    }
    return false;
}

void RobotBase::SimulationStep(dReal fElapsedTime)
{
    KinBody::SimulationStep(fElapsedTime);

    FOREACH(itsensor, _vecSensors) {
        if( !!(*itsensor)->psensor )
            (*itsensor)->psensor->SimulationStep(fElapsedTime);
    }

    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::_ComputeInternalInformation()
{
    KinBody::_ComputeInternalInformation();
    int manipindex=0;
    FOREACH(itmanip,_vecManipulators) {
        if( (*itmanip)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "manip" << manipindex;
            RAVELOG_WARN(str(boost::format("robot %s has a manipulator with no name, setting to %s\n")%GetName()%ss.str()));
            (*itmanip)->_name = ss.str();
        }
        else if( !IsValidName((*itmanip)->GetName()) )
            throw openrave_exception(str(boost::format("manipulator name \"%s\" is not valid")%(*itmanip)->GetName()));
        if( !!(*itmanip)->GetBase() && !!(*itmanip)->GetEndEffector() ) {
            vector<JointPtr> vjoints;
            if( GetChain((*itmanip)->GetBase()->GetIndex(),(*itmanip)->GetEndEffector()->GetIndex(), vjoints) ) {
                (*itmanip)->_varmjoints.resize(0);
                FOREACH(it,vjoints) {
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        (*itmanip)->_varmjoints.push_back((*it)->GetDOFIndex()+i);
                    }
                }
            }
            else {
                RAVELOG_WARN(str(boost::format("manipulator %s failed to find chain between %s and %s links\n")%(*itmanip)->GetName()%(*itmanip)->GetBase()->GetName()%(*itmanip)->GetEndEffector()->GetName()));
            }
        }
        else
            RAVELOG_WARN(str(boost::format("manipulator %s has undefined base and end effector links\n")%(*itmanip)->GetName()));
        (*itmanip)->InitIKSolver();
        vector<ManipulatorPtr>::iterator itmanip2 = itmanip; ++itmanip2;
        for(;itmanip2 != _vecManipulators.end(); ++itmanip2) {
            if( (*itmanip)->GetName() == (*itmanip2)->GetName() )
                RAVELOG_WARN(str(boost::format("robot %s has two manipulators with the same name: %s!\n")%GetName()%(*itmanip)->GetName()));
        }
    }

    FOREACH(itsensor,_vecSensors) {
        if( (*itsensor)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "sensor" << manipindex;
            RAVELOG_WARN(str(boost::format("robot %s has a sensor with no name, setting to %s\n")%GetName()%ss.str()));
            (*itsensor)->_name = ss.str();
        }
        else if( !IsValidName((*itsensor)->GetName()) )
            throw openrave_exception(str(boost::format("sensor name \"%s\" is not valid")%(*itsensor)->GetName()));
    }

    {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics|SO_Geometry|SO_RobotManipulators|SO_RobotSensors);
        __hashrobotstructure = GetMD5HashString(ss.str());

        FOREACH(itmanip,_vecManipulators) {
            ss.str("");
            (*itmanip)->serialize(ss,0);
            (*itmanip)->__hashstructure = GetMD5HashString(ss.str());
        }
        FOREACH(itsensor,_vecSensors) {
            ss.str("");
            (*itsensor)->serialize(ss,0);
            (*itsensor)->__hashstructure = GetMD5HashString(ss.str());
        }
    }
    _vAllDOFIndices.resize(GetDOF());
    for(int i = 0; i < GetDOF(); ++i) {
        _vAllDOFIndices[i] = i;
    }
    if( ComputeAABB().extents.lengthsqr3() > 900.0f )
        RAVELOG_WARN(str(boost::format("Robot %s span is greater than 30 meaning that it is most likely defined in a unit other than meters. It is highly encouraged to define all OpenRAVE robots in meters since many metrics, database models, and solvers have been specifically optimized for this unit\n")%GetName()));
}

bool RobotBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    // note that grabbed bodies are not cloned (check out Environment::Clone)
    if( !KinBody::Clone(preference,cloningoptions) ) {
        return false;
    }
    RobotBaseConstPtr r = RaveInterfaceConstCast<RobotBase>(preference);
    __hashrobotstructure = r->__hashrobotstructure;
    
    _vecManipulators.clear();
    FOREACHC(itmanip, r->_vecManipulators)
        _vecManipulators.push_back(ManipulatorPtr(new Manipulator(shared_robot(),**itmanip)));

    _vecSensors.clear();
    FOREACHC(itsensor, r->_vecSensors)
        _vecSensors.push_back(AttachedSensorPtr(new AttachedSensor(shared_robot(),**itsensor,cloningoptions)));
    _UpdateAttachedSensors();

    _vActiveJointIndices = r->_vActiveJointIndices;
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
    _vRotationQuatLowerLimits = r->_vRotationQuatLowerLimits;
    _vRotationQuatUpperLimits = r->_vRotationQuatUpperLimits;
    _vRotationQuatMaxVels = r->_vRotationQuatMaxVels;
    _vRotationQuatResolutions = r->_vRotationQuatResolutions;

    // clone the controller
    if( (cloningoptions&Clone_RealControllers) && !!r->GetController() ) {
        if( !SetController(GetEnv()->CreateController(r->GetController()->GetXMLId()),"") ) {
            RAVELOG_WARNA("failed to set %s controller for robot %s\n", r->GetController()->GetXMLId().c_str(), GetName().c_str());
        }
    }

    if( !GetController() ) {
        if( !SetController(GetEnv()->CreateController("IdealController"),"") ) {
            RAVELOG_WARNA("failed to set IdealController\n");
            return false;
        }
    }
    
    return true;
}

void RobotBase::serialize(std::ostream& o, int options) const
{
    KinBody::serialize(o,options);
    if( options & SO_RobotManipulators ) {
        FOREACHC(itmanip,_vecManipulators)
            (*itmanip)->serialize(o,options);
    }
    if( options & SO_RobotSensors ) {
        FOREACHC(itsensor,_vecSensors)
            (*itsensor)->serialize(o,options);
    }
}

const std::string& RobotBase::GetRobotStructureHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    return __hashrobotstructure;
}

} // end namespace OpenRAVE
