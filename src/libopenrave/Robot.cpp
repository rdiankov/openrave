// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

namespace OpenRAVE {

RobotBase::Manipulator::Manipulator(RobotBase* probot)
  : pBase(NULL), pEndEffector(NULL), _probot(probot), _pIkSolver(NULL), _ikoptions(0)
{
}

RobotBase::Manipulator::~Manipulator()
{
    delete _pIkSolver; _pIkSolver = NULL;
}

RobotBase::Manipulator::Manipulator(const RobotBase::Manipulator& r)
{
    *this = r;
    assert( _probot != NULL );
    if( _probot != NULL && _strIkSolver.size() > 0 ) {
        _pIkSolver = _probot->GetEnv()->CreateIkSolver(_strIkSolver.c_str());
        if( _pIkSolver != NULL ) {
            if( !_pIkSolver->Init(_probot, this, _ikoptions) )
                RAVELOG_WARNA("ik solver %S failed to init\n", _strIkSolver.c_str());
        }
    }
}

RobotBase::Manipulator::Manipulator(RobotBase* probot, const RobotBase::Manipulator& r)
{
    assert( probot != NULL );
    assert( r._probot != NULL );

    *this = r;
    _probot = probot;
    if( r.pBase != NULL )
        pBase = probot->GetLinks()[r.pBase->GetIndex()];
    if( r.pEndEffector != NULL )
        pEndEffector = probot->GetLinks()[r.pEndEffector->GetIndex()];
    
    _pIkSolver = NULL;
    if( _strIkSolver.size() > 0 ) {
        _pIkSolver = probot->GetEnv()->CreateIkSolver(_strIkSolver.c_str());
        if( !_pIkSolver->Init(_probot, this, _ikoptions) )
            RAVELOG_WARNA("ik solver %S failed to init\n", _strIkSolver.c_str());
    }
}

Transform RobotBase::Manipulator::GetEndEffectorTransform() const
{
    assert( pEndEffector != NULL );
    return pEndEffector->GetTransform() * tGrasp;
}

void RobotBase::Manipulator::SetIKSolver(IkSolverBase* iksolver)
{
    if( iksolver != _pIkSolver) {
        delete _pIkSolver; _pIkSolver = iksolver;
        if( _pIkSolver != NULL )
            _strIkSolver = _pIkSolver->GetXMLId();
        else
            _strIkSolver.clear();
    }
}

bool RobotBase::Manipulator::InitIKSolver(int options)
{
    return _pIkSolver != NULL && _pIkSolver->Init(_probot, this, options);
}

const std::string& RobotBase::Manipulator::GetIKSolverName() const
{
    return _strIkSolver;
}

bool RobotBase::Manipulator::HasIKSolver() const
{
    return _pIkSolver != NULL;
}

int RobotBase::Manipulator::GetNumFreeParameters() const
{
    return _pIkSolver != NULL ? _pIkSolver->GetNumFreeParameters() : 0;
}

bool RobotBase::Manipulator::GetFreeParameters(dReal* pFreeParameters) const
{
    assert( pFreeParameters != NULL );
    if( _pIkSolver == NULL )
        return false;
    return _pIkSolver->GetFreeParameters(pFreeParameters);
}

bool RobotBase::Manipulator::FindIKSolution(const Transform& goal, vector<dReal>& solution, bool bColCheck) const
{
    return FindIKSolution(goal, NULL, solution, bColCheck);
}

bool RobotBase::Manipulator::FindIKSolution(const Transform& goal, const dReal* pFreeParameters, vector<dReal>& solution, bool bColCheck) const
{
    if( _pIkSolver == NULL ) {
        RAVELOG(L"Need to set IK solver for robot\n");
        return false;
    }
    assert( _pIkSolver->GetRobot() != NULL );

    vector<dReal> temp;
    _pIkSolver->GetRobot()->GetJointValues(temp);

    solution.resize(_vecarmjoints.size());
    for(size_t i = 0; i < _vecarmjoints.size(); ++i)
        solution[i] = temp[_vecarmjoints[i]];

    Transform tgoal;

    if( pBase != NULL ) {
        Transform tEE = pBase->GetTransform();
        tgoal = tEE.inverse() * goal*tGrasp.inverse();
    }
    else tgoal = goal;

    return pFreeParameters == NULL ? _pIkSolver->Solve(tgoal, &solution[0], bColCheck, &solution[0]) : _pIkSolver->Solve(tgoal, &solution[0], pFreeParameters, bColCheck, &solution[0]);
}

bool RobotBase::Manipulator::FindIKSolutions(const Transform& goal, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const
{
    return FindIKSolutions(goal, NULL, solutions, bColCheck);
}

bool RobotBase::Manipulator::FindIKSolutions(const Transform& goal, const dReal* pFreeParameters, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const
{
    if( _pIkSolver == NULL ) {
        RAVELOG(L"Need to set IK solver for robot\n");
        return false;
    }
    assert( _pIkSolver->GetRobot() != NULL );

    Transform tgoal;
    Transform tEE;

    if( pBase != NULL ) {
        
        tEE = pBase->GetTransform();
        tgoal = tEE.inverse() * goal*tGrasp.inverse();
        
    }
    else tgoal = goal;

    return pFreeParameters == NULL ? _pIkSolver->Solve(tgoal,bColCheck,solutions) : _pIkSolver->Solve(tgoal,pFreeParameters,bColCheck,solutions);
}

RobotBase::AttachedSensor::AttachedSensor(RobotBase* probot) : _probot(probot), psensor(NULL), pattachedlink(NULL), pdata(NULL)
{
}

RobotBase::AttachedSensor::AttachedSensor(RobotBase* probot, const AttachedSensor& sensor,int cloningoptions)
{
    assert(probot != NULL);
    *this = sensor;
    _probot = probot;
    psensor = NULL;
    if( sensor.psensor != NULL ) {
        psensor = _probot->GetEnv()->CreateSensor(sensor.psensor->GetXMLId());
        if( psensor != NULL ) {
            psensor->Clone(sensor.psensor,cloningoptions);
            pdata = psensor != NULL ? psensor->CreateSensorData() : NULL;
        }
    }
    
    int index = sensor.pattachedlink->GetIndex();
    if( index >= 0 && index < (int)_probot->GetLinks().size())
        pattachedlink = _probot->GetLinks()[index];
}

RobotBase::AttachedSensor::~AttachedSensor()
{
}

SensorBase::SensorData* RobotBase::AttachedSensor::GetData() const
{
    if( psensor != NULL ) {
        if( psensor->GetSensorData(pdata) )
            return pdata;
    }

    return NULL;
}

RobotBase::RobotStateSaver::RobotStateSaver(RobotBase* probot) : KinBodyStateSaver(probot), _probot(probot)
{
    assert( _probot != NULL );
    vactivedofs = _probot->GetActiveJointIndices();
    affinedofs = _probot->GetAffineDOF();
    rotationaxis = _probot->GetAffineRotationAxis();
}

RobotBase::RobotStateSaver::~RobotStateSaver()
{
    _probot->SetActiveDOFs(vactivedofs, affinedofs, &rotationaxis);
}

RobotBase::RobotBase(EnvironmentBase* penv) : KinBody(PT_Robot, penv)
{
    _nAffineDOFs = 0;
    _nActiveDOF = 0;

    _nActiveManip = 0;
    _vecManipulators.reserve(16); // make sure to reseve enough, otherwise pIkSolver pointer might get messed up when resizing

    //set limits for the affine DOFs
    _vTranslationLowerLimits = Vector(-100,-100,-100);
    _vTranslationUpperLimits = Vector(100,100,100);
    _vTranslationMaxVels = Vector(0.07f,0.07f,0.07f);
    _vTranslationResolutions = Vector(0.001f,0.001f,0.001f);

    _vRotationAxisLowerLimits = Vector(-4*PI,-4*PI,-4*PI);
    _vRotationAxisUpperLimits = Vector(4*PI,4*PI,4*PI);
    _vRotationAxisMaxVels = Vector(0.07f,0.07f,0.07f);
    _vRotationAxisResolutions = Vector(0.01f,0.01f,0.01f);
    
    _vRotation3DLowerLimits = Vector(-10000,-10000,-10000);
    _vRotation3DUpperLimits = Vector(10000,10000,10000);
    _vRotation3DMaxVels = Vector(0.07f,0.07f,0.07f);
    _vRotation3DResolutions = Vector(0.01f,0.01f,0.01f);

    _vRotationQuatLowerLimits = Vector(-1,-1,-1,-1);
    _vRotationQuatUpperLimits = Vector(1,1,1,1);
    _vRotationQuatMaxVels = Vector(0.07f,0.07f,0.07f);
    _vRotationQuatResolutions = Vector(0.01f,0.01f,0.01f);
}

RobotBase::~RobotBase()
{
    Destroy();
}

void RobotBase::Destroy()
{
    _vGrabbedBodies.clear();
    _vecManipulators.clear();

    FOREACH(itsensor, _vecSensors) {
        delete itsensor->pdata;
        delete itsensor->psensor;
    }
    _vecSensors.clear();
    
    KinBody::Destroy();
}

void RobotBase::SetJointValues(std::vector<Transform>* pvbodies, const Transform* ptrans, const dReal* pJointValues, bool bCheckLimits)
{
    KinBody::SetJointValues(pvbodies, ptrans, pJointValues, bCheckLimits);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
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

void RobotBase::ApplyTransform(const Transform& trans)
{
    KinBody::ApplyTransform(trans);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::_UpdateGrabbedBodies()
{
    // update grabbed objects
    FOREACH(itbody, _vGrabbedBodies) {
        assert( itbody->pbody != NULL && itbody->plinkrobot != NULL );
        itbody->pbody->SetTransform(itbody->plinkrobot->GetTransform() * itbody->troot);
    }
}

void RobotBase::_UpdateAttachedSensors()
{
    FOREACH(itsensor, _vecSensors) {
        if( itsensor->psensor != NULL && itsensor->pattachedlink != NULL )
            itsensor->psensor->SetTransform(itsensor->pattachedlink->GetTransform()*itsensor->trelative);
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

    assert(0); // unspecified dof
    return index;
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

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask, const Vector* pRotationAxis)
{
    FOREACHC(itj, vJointIndices)
        assert( *itj >= 0 && *itj < (int)GetDOF());
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
        if( pRotationAxis != NULL )
            vActvAffineRotationAxis = *pRotationAxis;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) _nActiveDOF += 3;
    else if( _nAffineDOFs & DOF_RotationQuat ) _nActiveDOF += 4;

    _nActiveDOF += vJointIndices.size();
}

void RobotBase::SetActiveDOFValues(std::vector<Transform>* pvbodies, const dReal* pValues, bool bCheckLimits)
{
    assert(pValues != NULL);

    if(_nActiveDOF == 0) {
        SetJointValues(pvbodies, NULL, pValues, bCheckLimits);
        return;
    }

    Transform t;
    Transform* pglobtrans = NULL;

    if( (int)_vActiveJointIndices.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = pValues + _vActiveJointIndices.size();

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
            if( flength > 0 ) t.rot /= sqrtf(flength);
            else t.rot = Vector(1,0,0,0);
        }

        pglobtrans = &t;
        if( _vActiveJointIndices.size() == 0 )
            SetTransform(t);
    }

    if( _vActiveJointIndices.size() > 0 ) {
        GetJointValues(_vtempjoints);

        for(size_t i = 0; i < _vActiveJointIndices.size(); ++i)
            _vtempjoints[_vActiveJointIndices[i]] = *pValues++;

        SetJointValues(pvbodies, pglobtrans, &_vtempjoints[0], bCheckLimits);
    }
    
}

void RobotBase::GetActiveDOFValues(dReal* pValues) const
{
    assert(pValues != NULL);
    if( _nActiveDOF == 0 ) {
        GetJointValues(pValues);
        return;
    }

    if( _vActiveJointIndices.size() != 0 ) {
        GetJointValues(_vtempjoints);

        FOREACHC(it, _vActiveJointIndices)
            *pValues++ = _vtempjoints[*it];
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

        *pValues++ = 2 * atan2(fsin, t.rot.x);
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

void RobotBase::GetActiveDOFValues(std::vector<dReal>& v) const
{
    v.resize(GetActiveDOF());
    GetActiveDOFValues(&v[0]);
}

void RobotBase::SetActiveDOFVelocities(dReal* pVelocities)
{
    assert(pVelocities != NULL);

    if(_nActiveDOF == 0) {
        SetJointVelocities(pVelocities);
        return;
    }

    Vector linearvel, angularvel;

    if( (int)_vActiveJointIndices.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = pVelocities + _vActiveJointIndices.size();

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
            assert(0);
        }

        if( _vActiveJointIndices.size() == 0 )
            SetVelocity(linearvel, angularvel);
    }

    if( _vActiveJointIndices.size() > 0 ) {
        GetJointVelocities(_vtempjoints);

        FOREACHC(it, _vActiveJointIndices)
            _vtempjoints[*it] = *pVelocities++;

        SetJointVelocities(&_vtempjoints[0]);
    }
}

void RobotBase::GetActiveDOFVelocities(dReal* pVelocities) const
{
    assert(pVelocities != NULL);
    if( _nActiveDOF == 0 ) {
        GetJointVelocities(pVelocities);
        return;
    }

    if( _vActiveJointIndices.size() != 0 ) {
        GetJointVelocities(_vtempjoints);

        FOREACHC(it, _vActiveJointIndices)
            *pVelocities++ = _vtempjoints[*it];
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
        assert(0);
    }
}

void RobotBase::GetActiveDOFVelocities(std::vector<dReal>& velocities) const
{
    velocities.resize(GetDOF());
    GetActiveDOFVelocities(&velocities[0]);
}

void RobotBase::GetActiveDOFLimits(dReal* pLowerLimit, dReal* pUpperLimit) const
{
    // lower
    if( pLowerLimit != NULL ) {

        if( _nAffineDOFs == 0 ) {

            if( _nActiveDOF == 0 )
                GetJointLimits(pLowerLimit, NULL);
            else {
                _vtempjoints.resize(GetDOF());
                GetJointLimits(&_vtempjoints[0], NULL);
                FOREACHC(it, _vActiveJointIndices)
                    *pLowerLimit++ = _vtempjoints[*it];
            }
        }
        else {

            if( _vActiveJointIndices.size() > 0 ) {
                _vtempjoints.resize(GetDOF());
                GetJointLimits(&_vtempjoints[0], NULL);
                FOREACHC(it, _vActiveJointIndices)
                    *pLowerLimit++ = _vtempjoints[*it];
            }

            if( _nAffineDOFs & DOF_X ) { *pLowerLimit++ = _vTranslationLowerLimits.x; }
            if( _nAffineDOFs & DOF_Y ) { *pLowerLimit++ = _vTranslationLowerLimits.y; }
            if( _nAffineDOFs & DOF_Z ) { *pLowerLimit++ = _vTranslationLowerLimits.z; }

            if( _nAffineDOFs & DOF_RotationAxis ) { *pLowerLimit++ = _vRotationAxisLowerLimits.x; }
            else if( _nAffineDOFs & DOF_Rotation3D ) {
                //should this be *pLowerLimit++ instead of pLowerLimit[0]?
                *pLowerLimit++ = _vRotation3DLowerLimits.x;
                *pLowerLimit++ = _vRotation3DLowerLimits.y;
                *pLowerLimit++ = _vRotation3DLowerLimits.z;
            }
            else if( _nAffineDOFs & DOF_RotationQuat ) {
                //should this be *pLowerLimit++ instead of pLowerLimit[0]?
                *pLowerLimit++ = _vRotationQuatLowerLimits.x;
                *pLowerLimit++ = _vRotationQuatLowerLimits.y;
                *pLowerLimit++ = _vRotationQuatLowerLimits.z;
                *pLowerLimit++ = _vRotationQuatLowerLimits.w;
            }
        }
    }

    // upper limit
    if( pUpperLimit != NULL ) {

        if( _nAffineDOFs == 0 ) {
            if( _nActiveDOF == 0 )
                GetJointLimits(NULL, pUpperLimit);
            else {
                _vtempjoints.resize(GetDOF());
                GetJointLimits(NULL, &_vtempjoints[0]);
                FOREACHC(it, _vActiveJointIndices)
                    *pUpperLimit++ = _vtempjoints[*it];
            }
        }
        else {

            if( _vActiveJointIndices.size() > 0 ) {
                _vtempjoints.resize(GetDOF());
                GetJointLimits(NULL, &_vtempjoints[0]);
                FOREACHC(it, _vActiveJointIndices)
                    *pUpperLimit++ = _vtempjoints[*it];
            }

            if( _nAffineDOFs & DOF_X ) { *pUpperLimit++ = _vTranslationUpperLimits.x; }
            if( _nAffineDOFs & DOF_Y ) { *pUpperLimit++ = _vTranslationUpperLimits.y; }
            if( _nAffineDOFs & DOF_Z ) { *pUpperLimit++ = _vTranslationUpperLimits.z; }

            if( _nAffineDOFs & DOF_RotationAxis ) { *pUpperLimit++ = _vRotationAxisUpperLimits.x; }
            else if( _nAffineDOFs & DOF_Rotation3D ) {
                //should this be *pUpperLimit++ instead of pUpperLimit[0]?
                *pUpperLimit++ = _vRotation3DUpperLimits.x;
                *pUpperLimit++ = _vRotation3DUpperLimits.y;
                *pUpperLimit++ = _vRotation3DUpperLimits.z;
            }
            else if( _nAffineDOFs & DOF_RotationQuat ) {
                //should this be *pUpperLimit++ instead of pUpperLimit[0]?
                *pUpperLimit++ = _vRotationQuatUpperLimits.x;
                *pUpperLimit++ = _vRotationQuatUpperLimits.y;
                *pUpperLimit++ = _vRotationQuatUpperLimits.z;
                *pUpperLimit++ = _vRotationQuatUpperLimits.w;
            }
        }
    }
}    

void RobotBase::GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const
{
    lower.resize(GetActiveDOF());
    upper.resize(GetActiveDOF());
    GetActiveDOFLimits(&lower[0], &upper[0]);
}

void RobotBase::GetActiveDOFResolutions(dReal* pResolution) const
{
    if( _nActiveDOF == 0 ) {
        GetJointResolutions(pResolution);
        return;
    }

    GetJointResolutions(_vtempjoints);
    FOREACHC(it, _vActiveJointIndices)
        *pResolution++ = _vtempjoints[*it];

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

void RobotBase::GetActiveDOFResolutions(std::vector<dReal>& v) const
{
    v.resize(GetActiveDOF());
    GetActiveDOFResolutions(&v[0]);
}

void RobotBase::GetActiveDOFMaxVel(dReal* pMaxVel) const
{
    if( _nActiveDOF == 0 ) {
        GetJointMaxVel(pMaxVel);
        return;
    }

    GetJointMaxVel(_vtempjoints);
    FOREACHC(it, _vActiveJointIndices)
        *pMaxVel++ = _vtempjoints[*it];

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

void RobotBase::GetActiveDOFMaxVel(std::vector<dReal>& v) const
{
    v.resize(GetActiveDOF());
    GetActiveDOFMaxVel(&v[0]);
}

void RobotBase::GetActiveDOFMaxAccel(dReal* pMaxAccel) const
{
    if( _nActiveDOF == 0 ) {
        GetJointMaxAccel(pMaxAccel);
        return;
    }

    GetJointMaxAccel(_vtempjoints);
    FOREACHC(it, _vActiveJointIndices)
        *pMaxAccel++ = _vtempjoints[*it];

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

void RobotBase::GetActiveDOFMaxAccel(std::vector<dReal>& v) const
{
    v.resize(GetActiveDOF());
    GetActiveDOFMaxAccel(&v[0]);
}

void RobotBase::GetControlMaxTorques(dReal* pMaxTorques) const
{
    assert(pMaxTorques != NULL);
    if( _nActiveDOF == 0 ) {
        GetJointMaxTorque(pMaxTorques);
        return;
    }

    if( _vActiveJointIndices.size() != 0 ) {
        GetJointMaxTorque(_vtempjoints);

        FOREACHC(it, _vActiveJointIndices)
            *pMaxTorques++ = _vtempjoints[*it];
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

void RobotBase::GetControlMaxTorques(std::vector<dReal>& vmaxtorque) const
{
    vmaxtorque.resize(GetControlDOF());
    GetControlMaxTorques(&vmaxtorque[0]);
}

void RobotBase::SetControlTorques(dReal* pTorques)
{
    assert(pTorques != NULL);

    if(_nActiveDOF == 0) {
        SetJointTorques(pTorques, false);
        return;
    }

    if( _vActiveJointIndices.size() > 0 ) {
        _vtempjoints.resize(GetDOF());

        FOREACHC(it, _vActiveJointIndices)
            _vtempjoints[*it] = *pTorques++;

        SetJointTorques(&_vtempjoints[0], false);
    }
}

void RobotBase::GetFullTrajectoryFromActive(Trajectory* pFullTraj, const Trajectory* pActiveTraj, bool bOverwriteTransforms)
{
    assert( pFullTraj != NULL && pActiveTraj != NULL );
    assert( pFullTraj != pActiveTraj );

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
        GetJointValues(p.q);
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
            pFullTraj->AddPoint(p);
        }
    }

    pFullTraj->CalcTrajTiming(this, pActiveTraj->GetInterpMethod(), false, false);
}

const std::vector<int>& RobotBase::GetActiveJointIndices()
{
    if( (int)_vAllJointIndices.size() != GetDOF() ) {
        _vAllJointIndices.resize(GetDOF());
        for(int i = 0; i < GetDOF(); ++i) _vAllJointIndices[i] = i;
    }

    return _nActiveDOF == 0 ? _vAllJointIndices : _vActiveJointIndices;
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, dReal* pfJacobian) const
{
    if( _nActiveDOF == 0 ) {
        CalculateJacobian(index, offset, pfJacobian);
        return;
    }

    if( _vActiveJointIndices.size() != 0 ) {
        vector<dReal> jac(3*GetDOF());
        CalculateJacobian(index, offset, &jac[0]);

        for(int i = 0; i < (int)_vActiveJointIndices.size(); ++i) {
            pfJacobian[i] = jac[_vActiveJointIndices[i]];
            pfJacobian[GetActiveDOF()+i] = jac[GetDOF()+_vActiveJointIndices[i]];
            pfJacobian[2*GetActiveDOF()+i] = jac[2*GetDOF()+_vActiveJointIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    pfJacobian += _vActiveJointIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        pfJacobian[0] = 1;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 1;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 1;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        Vector vj;
        cross3(vj, vActvAffineRotationAxis, GetTransform().trans-offset);
        
        pfJacobian[0] = vj.x;
        pfJacobian[GetActiveDOF()] = vj.y;
        pfJacobian[2*GetActiveDOF()] = vj.z;
        pfJacobian++;
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

        if( fabsf(fangle) < 1e-8f )
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
                pfJacobian[i*GetActiveDOF()+j] = dRQ[4*i+0]*dQA[3*0+j] + dRQ[4*i+1]*dQA[3*1+j] + dRQ[4*i+2]*dQA[3*2+j] + dRQ[4*i+3]*dQA[3*3+j];
            }
        }
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
                pfJacobian[i*GetActiveDOF()+j] = dRQ[4*i+j];
            }
        }
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, dReal* pfJacobian) const
{
    if( _nActiveDOF == 0 ) {
        CalculateJacobian(index, q, pfJacobian);
        return;
    }

    if( _vActiveJointIndices.size() != 0 ) {
        vector<dReal> jac(4*GetDOF());
        CalculateRotationJacobian(index, q, &jac[0]);

        for(int i = 0; i < (int)_vActiveJointIndices.size(); ++i) {
            pfJacobian[i] = jac[_vActiveJointIndices[i]];
            pfJacobian[GetActiveDOF()+i] = jac[GetDOF()+_vActiveJointIndices[i]];
            pfJacobian[2*GetActiveDOF()+i] = jac[2*GetDOF()+_vActiveJointIndices[i]];
            pfJacobian[3*GetActiveDOF()+i] = jac[3*GetDOF()+_vActiveJointIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    pfJacobian += _vActiveJointIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian[3*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian[3*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian[3*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        pfJacobian[0] =            -q.y*v.x - q.z*v.y - q.w*v.z;
        pfJacobian[GetActiveDOF()] =      q.x*v.x - q.z*v.z + q.w*v.y;
        pfJacobian[GetActiveDOF()*2] =    q.x*v.y + q.y*v.z - q.w*v.x;
        pfJacobian[GetActiveDOF()*3] =    q.x*v.z - q.y*v.y + q.z*v.x;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        assert(0);
        // todo
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        pfJacobian[0] = 1;
        pfJacobian[GetActiveDOF()] = 1;
        pfJacobian[GetActiveDOF()*2] = 1;
        pfJacobian[GetActiveDOF()*3] = 1;
    }
}


void RobotBase::CalculateActiveAngularVelocityJacobian(int index, dReal* pfJacobian) const
{
    if( _nActiveDOF == 0 ) {
        CalculateAngularVelocityJacobian(index, pfJacobian);
        return;
    }

    if( _vActiveJointIndices.size() != 0 ) {
        vector<dReal> jac(3*GetDOF());
        CalculateAngularVelocityJacobian(index, &jac[0]);

        for(int i = 0; i < (int)_vActiveJointIndices.size(); ++i) {
            pfJacobian[i] = jac[_vActiveJointIndices[i]];
            pfJacobian[GetActiveDOF()+i] = jac[GetDOF()+_vActiveJointIndices[i]];
            pfJacobian[2*GetActiveDOF()+i] = jac[2*GetDOF()+_vActiveJointIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform )
        return;

    pfJacobian += _vActiveJointIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        pfJacobian[0] = 0;
        pfJacobian[GetActiveDOF()] = 0;
        pfJacobian[2*GetActiveDOF()] = 0;
        pfJacobian++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        pfJacobian[0] =            v.x;
        pfJacobian[GetActiveDOF()] =  v.y;
        pfJacobian[GetActiveDOF()*2] =   v.z;

    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        assert(0);
        // todo
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        assert(0);
        // most likely wrong
        Transform t = GetTransform();
        dReal fnorm = RaveSqrt(t.rot.y*t.rot.y+t.rot.z*t.rot.z+t.rot.w*t.rot.w);

        if( fnorm > 0 ) {
            pfJacobian[0] = t.rot.y/fnorm;
            pfJacobian[1] = t.rot.z/fnorm;
            pfJacobian[2] = t.rot.w/fnorm;
        }
        else {
            pfJacobian[0] = 0;
            pfJacobian[1] = 0;
            pfJacobian[2] = 0;
        }
    }
}


bool RobotBase::Init(const char* strData, const char**atts)
{
    assert( GetEnv() != NULL );

    // check if added
    FOREACHC(itbody, GetEnv()->GetBodies()) {
        if( *itbody == this ) {
            RAVEPRINT(L"Robot::Init for %S, cannot Init a body while it is added to the environment\n", GetName());
            return false;
        }
    }

    SetGuiData(NULL);

    bool bSuccess = GetEnv()->ReadRobotXML(this, strData, atts)==this;

    if( !bSuccess ) {
        Destroy();
        return false;
    }

    strXMLFilename = strData;

    return true;
}

bool RobotBase::Grab(KinBody* pbody, const std::set<int>* psetRobotLinksToIgnore)
{
    Manipulator* pmanip = GetActiveManipulator();
    if( pmanip == NULL || pmanip->pEndEffector == NULL )
        return false;
    return Grab(pbody, pmanip->pEndEffector->GetIndex(), psetRobotLinksToIgnore);
}

bool RobotBase::Grab(KinBody* pbody, int linkindex, const std::set<int>* psetRobotLinksToIgnore)
{
    if( pbody == NULL || linkindex < 0 || linkindex >= (int)GetLinks().size() )
        return false;

    if( IsGrabbing(pbody) ) {
        RAVELOG(L"Robot %S: body %S already grabbed\n", GetName(), pbody->GetName());
        return true;
    }

    _vGrabbedBodies.push_back(GRABBED());
    GRABBED& g = _vGrabbedBodies.back();
    g.pbody = pbody;
    g.plinkrobot = _veclinks[linkindex];
    g.troot = g.plinkrobot->GetTransform().inverse() * pbody->GetTransform();

    // check collision with all links to see which are valid
    FOREACH(itlink, _veclinks) {
        if( psetRobotLinksToIgnore != NULL && psetRobotLinksToIgnore->find((*itlink)->GetIndex()) != psetRobotLinksToIgnore->end() )
            continue;
        if( !GetEnv()->CheckCollision(*itlink, pbody) )
            g.sValidColLinks.insert(*itlink);
    }

    this->AttachBody(pbody);
    return true;
}

void RobotBase::Release(KinBody* pbody)
{
    vector<GRABBED>::iterator itbody;
    FORIT(itbody, _vGrabbedBodies) {
        if( itbody->pbody == pbody )
            break;
    }

    if( itbody == _vGrabbedBodies.end() ) {
        RAVELOG(L"Robot %S: body %S not grabbed\n", GetName(), pbody->GetName());
        return;
    }

    _vGrabbedBodies.erase(itbody);
    this->RemoveBody(pbody);
}

void RobotBase::ReleaseAllGrabbed()
{
    FOREACH(itbody, _vGrabbedBodies) {
        this->RemoveBody(itbody->pbody);
    }
    _vGrabbedBodies.clear();
}

void RobotBase::RegrabAll()
{
    FOREACH(itbody, _vGrabbedBodies) {
        // check collision with all links to see which are valid
        itbody->sValidColLinks.clear();
        FOREACH(itlink, _veclinks) {
            if( !GetEnv()->CheckCollision(*itlink, itbody->pbody) )
                itbody->sValidColLinks.insert(*itlink);
        }
    }
}

bool RobotBase::IsGrabbing(KinBody* pbody) const
{
    vector<GRABBED>::const_iterator itbody;
    FORIT(itbody, _vGrabbedBodies) {
        if( itbody->pbody == pbody )
            return true;
    }

    return false;
}

void RobotBase::SetActiveManipulator(int index)
{
    assert( index >= 0 && index < (int)_vecManipulators.size());
    _nActiveManip = index;
}

RobotBase::Manipulator* RobotBase::GetActiveManipulator()
{
    if(_nActiveManip >= 0 && _nActiveManip < (int)_vecManipulators.size() ) {
        return &_vecManipulators[_nActiveManip];
    }
    return NULL;
}

/// Check if body is self colliding. Links that are joined together are ignored.
bool RobotBase::CheckSelfCollision(COLLISIONREPORT* pReport) const
{
    if( KinBody::CheckSelfCollision(pReport) )
        return true;

    // check all grabbed bodies with 
    vector<GRABBED>::const_iterator itbody;
    set<Link*>::const_iterator itlink;

    COLLISIONREPORT report;
    if( pReport == NULL )
        pReport = &report;

    FORIT(itbody, _vGrabbedBodies) {
        FORIT(itlink, itbody->sValidColLinks) {
            if( GetEnv()->CheckCollision(*itlink, itbody->pbody, pReport) && pReport != NULL ) {
                RAVELOG(L"Self collision: (%S:%S)x(%S:%S).\n",
                          pReport->plink1!=NULL?pReport->plink1->GetParent()->GetName():L"",
                          pReport->plink1!=NULL?pReport->plink1->GetName():L"",
                          pReport->plink2!=NULL?pReport->plink2->GetParent()->GetName():L"",
                          pReport->plink2!=NULL?pReport->plink2->GetName():L"");
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
        if( itsensor->psensor != NULL )
            itsensor->psensor->SimulationStep(fElapsedTime);
    }

    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

bool RobotBase::Clone(const InterfaceBase* preference, int cloningoptions)
{
    // note that grabbed bodies are not cloned (check out Environment::Clone)
    if( !KinBody::Clone(preference,cloningoptions) )
        return false;
    const RobotBase& r = *(const RobotBase*)preference;

    _vecManipulators.clear();
    FOREACHC(itmanip, r._vecManipulators)
        _vecManipulators.push_back(Manipulator(this,*itmanip));

    _vecSensors.clear();
    FOREACHC(itsensor, r._vecSensors)
        _vecSensors.push_back(AttachedSensor(this,*itsensor,cloningoptions));
    _UpdateAttachedSensors();

    _vActiveJointIndices = r._vActiveJointIndices;
    _vAllJointIndices = r._vAllJointIndices;
    vActvAffineRotationAxis = r.vActvAffineRotationAxis;
    _nActiveManip = r._nActiveManip;
    _nActiveDOF = r._nActiveDOF;
    _nAffineDOFs = r._nAffineDOFs;
    _vtempjoints = r._vtempjoints;
    
    _vTranslationLowerLimits = r._vTranslationLowerLimits;
    _vTranslationUpperLimits = r._vTranslationUpperLimits;
    _vTranslationMaxVels = r._vTranslationMaxVels;
    _vTranslationResolutions = r._vTranslationResolutions;
    _vRotationAxisLowerLimits = r._vRotationAxisLowerLimits;
    _vRotationAxisUpperLimits = r._vRotationAxisUpperLimits;
    _vRotationAxisMaxVels = r._vRotationAxisMaxVels;
    _vRotationAxisResolutions = r._vRotationAxisResolutions;
    _vRotation3DLowerLimits = r._vRotation3DLowerLimits;
    _vRotation3DUpperLimits = r._vRotation3DUpperLimits;
    _vRotation3DMaxVels = r._vRotation3DMaxVels;
    _vRotation3DResolutions = r._vRotation3DResolutions;
    _vRotationQuatLowerLimits = r._vRotationQuatLowerLimits;
    _vRotationQuatUpperLimits = r._vRotationQuatUpperLimits;
    _vRotationQuatMaxVels = r._vRotationQuatMaxVels;
    _vRotationQuatResolutions = r._vRotationQuatResolutions;

    // clone the controller
    if( (cloningoptions&Clone_RealControllers) && r.GetController() != NULL ) {
        if( !SetController(_ravembstowcs(r.GetController()->GetXMLId()).c_str()) ) {
            RAVEPRINT(L"failed to set %S controller for robot %S\n", r.GetController()->GetXMLId(), GetName());
        }
    }

    if( GetController() == NULL ) {
        if( !SetController(L"IdealController") ) {
            RAVEPRINT(L"failed to set IdealController\n");
            return false;
        }
    }
    
    return true;
}

} // end namespace OpenRAVE
