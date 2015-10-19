// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
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

#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 2, "robot %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%_nHierarchyComputed, ORE_NotInitialized);

namespace OpenRAVE {

class CallOnDestruction
{
public:
    CallOnDestruction(const boost::function<void()>& fn) : _fn(fn) {
    }
    ~CallOnDestruction() {
        _fn();
    }
private:
    boost::function<void()> _fn;
};

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
            psensor->SetName(str(boost::format("%s:%s")%probot->GetName()%_info._name)); // need a unique targettable name
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

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot, const RobotBase::AttachedSensorInfo& info)
{
    _info = info;
    _probot = probot;
    pattachedlink = probot->GetLink(_info._linkname);
    psensor = RaveCreateSensor(probot->GetEnv(), _info._sensorname);
    if( !!psensor ) {
        psensor->SetName(str(boost::format("%s:%s")%probot->GetName()%_info._name)); // need a unique targettable name
        if(!!_info._sensorgeometry) {
            psensor->SetSensorGeometry(_info._sensorgeometry);
        }
        pdata = psensor->CreateSensorData();
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
    _info._trelative = t;
    GetRobot()->_PostprocessChangedParameters(Prop_SensorPlacement);
}

void RobotBase::AttachedSensor::UpdateInfo()
{
    if( !!psensor ) {
        _info._sensorname = psensor->GetXMLId();
        // TODO try to get the sensor geometry...?
        //GetSensorGeometry(ST_
        //_info._sensorgeometry
    }
    LinkPtr prealattachedlink = pattachedlink.lock();
    if( !!prealattachedlink ) {
        _info._linkname = prealattachedlink->GetName();
    }
}

void RobotBase::AttachedSensor::serialize(std::ostream& o, int options) const
{
    o << (pattachedlink.expired() ? -1 : LinkPtr(pattachedlink)->GetIndex()) << " ";
    SerializeRound(o,_info._trelative);
    o << (!pdata ? -1 : pdata->GetType()) << " ";
    // it is also important to serialize some of the geom parameters for the sensor (in case models are cached to it)
    if( !!psensor ) {
        SensorBase::SensorGeometryConstPtr prawgeom = psensor->GetSensorGeometry();
        if( !!prawgeom ) {
            switch(prawgeom->GetType()) {
            case SensorBase::ST_Laser: {
                SensorBase::LaserGeomDataConstPtr pgeom = boost::static_pointer_cast<SensorBase::LaserGeomData const>(prawgeom);
                o << pgeom->min_angle[0] << " " << pgeom->max_angle[0] << " " << pgeom->resolution[0] << " " << pgeom->max_range << " ";
                break;
            }
            case SensorBase::ST_Camera: {
                SensorBase::CameraGeomDataConstPtr pgeom = boost::static_pointer_cast<SensorBase::CameraGeomData const>(prawgeom);
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
        __hashstructure = utils::GetMD5HashString(ss.str());
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
        _pManipActive = _probot->GetActiveManipulator();
    }
    if( _options & Save_GrabbedBodies ) {
        _vGrabbedBodies = _probot->_vGrabbedBodies;
    }
    if( _options & Save_ActiveManipulatorToolTransform ) {
        _pManipActive = _probot->GetActiveManipulator();
        if( !!_pManipActive ) {
            _tActiveManipLocalTool = _pManipActive->GetLocalToolTransform();
            _vActiveManipLocalDirection = _pManipActive->GetLocalToolDirection();
            _pActiveManipIkSolver = _pManipActive->GetIkSolver();
        }
    }
}

RobotBase::RobotStateSaver::~RobotStateSaver()
{
    if( _bRestoreOnDestructor && !!_probot && _probot->GetEnvironmentId() != 0 ) {
        _RestoreRobot(_probot);
    }
}

void RobotBase::RobotStateSaver::Restore(boost::shared_ptr<RobotBase> robot)
{
    _RestoreRobot(!robot ? _probot : robot);
    KinBodyStateSaver::Restore(!robot ? KinBodyPtr(_probot) : KinBodyPtr(robot));
}

void RobotBase::RobotStateSaver::Release()
{
    _probot.reset();
    KinBodyStateSaver::Release();
}
void RobotBase::RobotStateSaver::_RestoreRobot(boost::shared_ptr<RobotBase> probot)
{
    if( !probot ) {
        return;
    }
    if( probot->GetEnvironmentId() == 0 ) {
        RAVELOG_WARN(str(boost::format("robot %s not added to environment, skipping restore")%_pbody->GetName()));
        return;
    }
    if( _options & Save_ActiveDOF ) {
        probot->SetActiveDOFs(vactivedofs, affinedofs, rotationaxis);
    }
    if( _options & Save_ActiveManipulator ) {
        if( probot == _probot ) {
            probot->SetActiveManipulator(_pManipActive);
        }
        else {
            if( !_pManipActive ) {
                probot->SetActiveManipulator(ManipulatorPtr());
            }
            else {
                probot->SetActiveManipulator(_pManipActive->GetName());
            }
        }
    }
    if( _options & Save_GrabbedBodies ) {
        // have to release all grabbed first
        probot->ReleaseAllGrabbed();
        OPENRAVE_ASSERT_OP(probot->_vGrabbedBodies.size(),==,0);
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
            if( !!pbody ) {
                if( probot->GetEnv() == _probot->GetEnv() ) {
                    probot->_AttachBody(pbody);
                    probot->_vGrabbedBodies.push_back(*itgrabbed);
                }
                else {
                    // pgrabbed points to a different environment, so have to re-initialize
                    KinBodyPtr pnewbody = probot->GetEnv()->GetBodyFromEnvironmentId(pbody->GetEnvironmentId());
                    if( pbody->GetKinematicsGeometryHash() != pnewbody->GetKinematicsGeometryHash() ) {
                        RAVELOG_WARN(str(boost::format("body %s is not similar across environments")%pbody->GetName()));
                    }
                    else {
                        GrabbedPtr pnewgrabbed(new Grabbed(pnewbody,probot->GetLinks().at(KinBody::LinkPtr(pgrabbed->_plinkrobot)->GetIndex())));
                        pnewgrabbed->_troot = pgrabbed->_troot;
                        pnewgrabbed->_listNonCollidingLinks.clear();
                        FOREACHC(itlinkref, pgrabbed->_listNonCollidingLinks) {
                            pnewgrabbed->_listNonCollidingLinks.push_back(probot->GetLinks().at((*itlinkref)->GetIndex()));
                        }
                        probot->_AttachBody(pnewbody);
                        probot->_vGrabbedBodies.push_back(pnewgrabbed);
                    }
                }
            }
        }
    }
    if( _options & Save_ActiveManipulatorToolTransform ) {
        if( !!_pManipActive ) {
            if( probot == _probot ) {
                _pManipActive->SetLocalToolTransform(_tActiveManipLocalTool);
                _pManipActive->SetLocalToolDirection(_vActiveManipLocalDirection);
                _pManipActive->SetIkSolver(_pActiveManipIkSolver);
            }
            else {
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(_pManipActive->GetName());
                if( !!pmanip ) {
                    pmanip->SetLocalToolTransform(_tActiveManipLocalTool);
                    pmanip->SetLocalToolDirection(_vActiveManipLocalDirection);
                    if( !!_pActiveManipIkSolver ) {
                        IkSolverBasePtr pnewsolver = RaveCreateIkSolver(probot->GetEnv(), _pActiveManipIkSolver->GetXMLId());
                        pnewsolver->Clone(_pActiveManipIkSolver, 0);
                        pmanip->SetIkSolver(pnewsolver);
                    }
                    else {
                        pmanip->SetIkSolver(IkSolverBasePtr());
                    }
                }
            }
        }
    }
}

RobotBase::RobotBase(EnvironmentBasePtr penv) : KinBody(PT_Robot, penv)
{
    _nAffineDOFs = 0;
    _nActiveDOF = -1;
    vActvAffineRotationAxis = Vector(0,0,1);

    //set limits for the affine DOFs
    _vTranslationLowerLimits = Vector(-100,-100,-100);
    _vTranslationUpperLimits = Vector(100,100,100);
    _vTranslationMaxVels = Vector(1.0f,1.0f,1.0f);
    _vTranslationResolutions = Vector(0.001f,0.001f,0.001f);
    _vTranslationWeights = Vector(2.0f,2.0f,2.0f);

    // rotation axis has infinite movement, so make sure the limits are big
    _vRotationAxisLowerLimits = Vector(-10000,-10000,-10000,10000);
    _vRotationAxisUpperLimits = Vector(10000,10000,10000,10000);

    _vRotationAxisMaxVels = Vector(0.4f,0.4f,0.4f,0.4f);
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

bool RobotBase::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos, const std::vector<RobotBase::ManipulatorInfoConstPtr>& manipinfos, const std::vector<RobotBase::AttachedSensorInfoConstPtr>& attachedsensorinfos, const std::string& uri)
{
    if( !KinBody::Init(linkinfos, jointinfos, uri) ) {
        return false;
    }
    _vecManipulators.resize(0);
    FOREACHC(itmanipinfo, manipinfos) {
        ManipulatorPtr newmanip(new Manipulator(shared_robot(),**itmanipinfo));
        _vecManipulators.push_back(newmanip);
        __hashrobotstructure.resize(0);
    }
    // TODO: sensors
    _vecSensors.resize(0);
    if( attachedsensorinfos.size() > 0 ) {
        RAVELOG_WARN("currently do not support initializing from AttachedSensorInfo\n");
    }
    return true;
}

bool RobotBase::SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int nControlTransformation)
{
    RAVELOG_DEBUG("default robot doesn't not support setting controllers (try GenericRobot)\n");
    return false;
}

void RobotBase::SetName(const std::string& newname)
{
    if( _name != newname ) {
        // have to replace the 2nd word of all the groups with the robot name
        FOREACH(itgroup, _activespec._vgroups) {
            stringstream ss(itgroup->name);
            string grouptype, oldname;
            ss >> grouptype >> oldname;
            stringbuf buf;
            ss.get(buf,0);
            itgroup->name = str(boost::format("%s %s %s")%grouptype%newname%buf.str());
        }
        KinBody::SetName(newname);
    }
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t bCheckLimits, const std::vector<int>& dofindices)
{
    KinBody::SetDOFValues(vJointValues, bCheckLimits,dofindices);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transbase, uint32_t bCheckLimits)
{
    KinBody::SetDOFValues(vJointValues, transbase, bCheckLimits); // should call RobotBase::SetDOFValues, so no need to upgrade grabbed bodies, attached sensors
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms)
{
    KinBody::SetLinkTransformations(transforms);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues)
{
    KinBody::SetLinkTransformations(transforms,doflastsetvalues);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<int>& dofbranches)
{
    KinBody::SetLinkTransformations(transforms,dofbranches);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

void RobotBase::SetTransform(const Transform& trans)
{
    KinBody::SetTransform(trans);
    _UpdateGrabbedBodies();
    _UpdateAttachedSensors();
}

bool RobotBase::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    if( !KinBody::SetVelocity(linearvel,angularvel) ) {
        return false;
    }
    _UpdateGrabbedBodies();
    return true;
}

void RobotBase::SetDOFVelocities(const std::vector<dReal>& dofvelocities, const Vector& linearvel, const Vector& angularvel,uint32_t checklimits)
{
    KinBody::SetDOFVelocities(dofvelocities,linearvel,angularvel,checklimits);
    _UpdateGrabbedBodies();
    // do sensors need to have their velocities updated?
}

void RobotBase::SetDOFVelocities(const std::vector<dReal>& dofvelocities, uint32_t checklimits, const std::vector<int>& dofindices)
{
    KinBody::SetDOFVelocities(dofvelocities,checklimits, dofindices); // RobotBase::SetDOFVelocities should be called internally
}

void RobotBase::_UpdateGrabbedBodies()
{
    vector<UserDataPtr>::iterator itgrabbed = _vGrabbedBodies.begin();
    while(itgrabbed != _vGrabbedBodies.end() ) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
        if( !!pbody ) {
            Transform t = pgrabbed->_plinkrobot->GetTransform();
            pbody->SetTransform(t * pgrabbed->_troot);
            // set the correct velocity
            std::pair<Vector, Vector> velocity = pgrabbed->_plinkrobot->GetVelocity();
            velocity.first += velocity.second.cross(t.rotate(pgrabbed->_troot.trans));
            pbody->SetVelocity(velocity.first, velocity.second);
            ++itgrabbed;
        }
        else {
            RAVELOG_DEBUG(str(boost::format("erasing invaliding grabbed body from %s")%GetName()));
            itgrabbed = _vGrabbedBodies.erase(itgrabbed);
        }
    }
}

void RobotBase::_UpdateAttachedSensors()
{
    FOREACH(itsensor, _vecSensors) {
        if( !!(*itsensor)->psensor && !(*itsensor)->pattachedlink.expired() )
            (*itsensor)->psensor->SetTransform(LinkPtr((*itsensor)->pattachedlink)->GetTransform()*(*itsensor)->GetRelativeTransform());
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
        OPENRAVE_ASSERT_FORMAT(*itj>=0 && *itj<GetDOF(), "bad indices %d (dof=%d)",*itj%GetDOF(),ORE_InvalidArguments);
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

    // do not initialize interpolation, since it implies a motion sampling strategy
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
        offset += group.dof;
        _activespec._vgroups.push_back(group);
    }
    if( GetAffineDOF() > 0 ) {
        ConfigurationSpecification::Group group;
        group.name = str(boost::format("affine_transform %s %d")%GetName()%GetAffineDOF());
        group.offset = offset;
        group.dof = RaveGetAffineDOF(GetAffineDOF());
        _activespec._vgroups.push_back(group);
    }

    _PostprocessChangedParameters(Prop_RobotActiveDOFs);
}

void RobotBase::SetActiveDOFValues(const std::vector<dReal>& values, uint32_t bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFValues(values,bCheckLimits);
        return;
    }
    OPENRAVE_ASSERT_OP_FORMAT((int)values.size(),>=,GetActiveDOF(), "not enough values %d<%d",values.size()%GetActiveDOF(),ORE_InvalidArguments);

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
    RaveGetAffineDOFValuesFromTransform(itvalues,t,_nAffineDOFs,vActvAffineRotationAxis);
}

void RobotBase::SetActiveDOFVelocities(const std::vector<dReal>& velocities, uint32_t bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFVelocities(velocities,true);
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
            throw OPENRAVE_EXCEPTION_FORMAT0(_("quaternions not supported"),ORE_InvalidArguments);
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
        throw OPENRAVE_EXCEPTION_FORMAT0(_("quaternions not supported"),ORE_InvalidArguments);
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
    size_t index = 0;
    for(; index < _vActiveDOFIndices.size(); ++index) {
        JointConstPtr pjoint = GetJointFromDOFIndex(_vActiveDOFIndices[index]);
        q1.at(index) = pjoint->SubtractValue(q1.at(index),q2.at(index),_vActiveDOFIndices[index]-pjoint->GetDOFIndex());
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
        q1.at(index) = utils::SubtractCircularAngle(q1.at(index),q2.at(index));
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

ConfigurationSpecification RobotBase::GetActiveConfigurationSpecification(const std::string& interpolation) const
{
    if( interpolation.size() == 0 ) {
        return _activespec;
    }
    ConfigurationSpecification spec = _activespec;
    FOREACH(itgroup,spec._vgroups) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(index, offset, vjacobian);
        return;
    }

    int dofstride = GetActiveDOF();
    vjacobian.resize(3*dofstride);
    if( _vActiveDOFIndices.size() != 0 ) {
        if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
            ComputeJacobianTranslation(index, offset, vjacobian, _vActiveDOFIndices);
            return;
        }
        // have to copy
        std::vector<dReal> vjacobianjoints;
        ComputeJacobianTranslation(index, offset, vjacobianjoints, _vActiveDOFIndices);
        for(size_t i = 0; i < 3; ++i) {
            std::copy(vjacobianjoints.begin()+i*_vActiveDOFIndices.size(),vjacobianjoints.begin()+(i+1)*_vActiveDOFIndices.size(),vjacobian.begin()+i*dofstride);
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        vjacobian[ind] = 1;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 1;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 1;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        Vector vj = vActvAffineRotationAxis.cross(offset-GetTransform().trans);
        vjacobian[ind] = vj.x;
        vjacobian[dofstride+ind] = vj.y;
        vjacobian[2*dofstride+ind] = vj.z;
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
                vjacobian[i*dofstride+ind+j] = dRQ[4*i+0]*dQA[3*0+j] + dRQ[4*i+1]*dQA[3*1+j] + dRQ[4*i+2]*dQA[3*2+j] + dRQ[4*i+3]*dQA[3*3+j];
            }
        }
        ind += 3;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        Transform t; t.identity(); t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        // note: qw, qx, qy, qz here follow the standard quaternion convention, not the openrave one
        dReal qw = t.rot[0], qx = t.rot[1], qy = t.rot[2], qz = t.rot[3];
        Vector offset_local = t.inverse() * offset;
        dReal x = offset_local.x, y = offset_local.y, z = offset_local.z;

        dReal dRQ[12] = {-2*qz*y + 2*qw*x + 2*qy*z,2*qx*x + 2*qy*y + 2*qz*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qw*y - 2*qz*x + 2*qx*z,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*z - 2*qx*y + 2*qy*x,2*qx*x + 2*qy*y + 2*qz*z,-2*qz*y + 2*qw*x + 2*qy*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*x - 2*qy*z + 2*qz*y,2*qx*x + 2*qy*y + 2*qz*z};
        for (int i=0; i < 3; ++i) {
            double qdotrow = dRQ[4*i]*qw + dRQ[4*i+1]*qx + dRQ[4*i+2]*qy + dRQ[4*i+3]*qz;
            dRQ[4*i] -= qdotrow*qw;
            dRQ[4*i+1] -= qdotrow*qx;
            dRQ[4*i+2] -= qdotrow*qy;
            dRQ[4*i+3] -= qdotrow*qz;
        }

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 4; ++j) {
                vjacobian[i*dofstride+ind + j] = dRQ[4*i+j];
            }
        }
        ind += 4;
    }
}

void RobotBase::CalculateActiveJacobian(int linkindex, const Vector& offset, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(linkindex, offset, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    RobotBase::CalculateActiveJacobian(linkindex,offset,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetActiveDOF());
    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(index, q, vjacobian);
        return;
    }
    int dofstride = GetActiveDOF();
    vjacobian.resize(4*dofstride);
    if( _vActiveDOFIndices.size() != 0 ) {
        std::vector<dReal> vjacobianjoints;
        CalculateRotationJacobian(index, q, vjacobianjoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            vjacobian[i] = vjacobianjoints[_vActiveDOFIndices[i]];
            vjacobian[dofstride+i] = vjacobianjoints[GetDOF()+_vActiveDOFIndices[i]];
            vjacobian[2*dofstride+i] = vjacobianjoints[2*GetDOF()+_vActiveDOFIndices[i]];
            vjacobian[3*dofstride+i] = vjacobianjoints[3*GetDOF()+_vActiveDOFIndices[i]];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }

    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        vjacobian[ind] = dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
        vjacobian[dofstride+ind] = dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
        vjacobian[2*dofstride+ind] = dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
        vjacobian[3*dofstride+ind] = dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
        ind++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s rotation 3d not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
        ind += 3;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s quaternion not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
        ind += 4;
    }
}

void RobotBase::CalculateActiveRotationJacobian(int linkindex, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(linkindex, q, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    RobotBase::CalculateActiveRotationJacobian(linkindex,q,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,4*GetActiveDOF());
    mjacobian.resize(boost::extents[4][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        ComputeJacobianAxisAngle(index, vjacobian);
        return;
    }

    int dofstride = GetActiveDOF();
    vjacobian.resize(3*dofstride);
    if( _vActiveDOFIndices.size() != 0 ) {
        if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
            ComputeJacobianAxisAngle(index, vjacobian, _vActiveDOFIndices);
            return;
        }
        // have to copy
        std::vector<dReal> vjacobianjoints;
        ComputeJacobianAxisAngle(index, vjacobianjoints, _vActiveDOFIndices);
        for(size_t i = 0; i < 3; ++i) {
            std::copy(vjacobianjoints.begin()+i*_vActiveDOFIndices.size(),vjacobianjoints.begin()+(i+1)*_vActiveDOFIndices.size(),vjacobianjoints.begin()+i*dofstride);
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        vjacobian[ind] = v.x;
        vjacobian[dofstride+ind] = v.y;
        vjacobian[2*dofstride+ind] = v.z;

    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s rotation 3d not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s quaternion not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);

        // most likely wrong
        Transform t; t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        dReal fnorm = t.rot.y*t.rot.y+t.rot.z*t.rot.z+t.rot.w*t.rot.w;
        if( fnorm > 0 ) {
            fnorm = dReal(1)/RaveSqrt(fnorm);
            vjacobian[ind] = t.rot.y*fnorm;
            vjacobian[dofstride+ind] = t.rot.z*fnorm;
            vjacobian[2*dofstride+ind] = t.rot.w*fnorm;
        }
        else {
            vjacobian[ind] = 0;
            vjacobian[dofstride+ind] = 0;
            vjacobian[2*dofstride+ind] = 0;
        }

        ++ind;
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateAngularVelocityJacobian(linkindex, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    CalculateActiveAngularVelocityJacobian(linkindex,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetActiveDOF());
    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
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
            throw OPENRAVE_EXCEPTION_FORMAT(_("does not support adjacentoptions %d"),adjacentoptions,ORE_InvalidArguments);
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

void RobotBase::SetNonCollidingConfiguration()
{
    KinBody::SetNonCollidingConfiguration();
    RegrabAll();
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
    OPENRAVE_ASSERT_FORMAT(!!pbody, "robot %s invalid bod to grab",GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!plink && plink->GetParent() == shared_kinbody(), "robot %s grabbing link needs to be part of robot",GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pbody != shared_kinbody(),"robot %s cannot grab itself",GetName(), ORE_InvalidArguments);

    //uint64_t starttime0 = utils::GetMicroTime();

    // if grabbing, check if the transforms are different. If they are, then update the transform
    GrabbedPtr pPreviousGrabbed;
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        if( KinBodyConstPtr(pgrabbed->_pgrabbedbody) == pbody ) {
            pPreviousGrabbed = pgrabbed;
            break;
        }
    }

    Transform t = plink->GetTransform();
    Transform tbody = pbody->GetTransform();
    // new body velocity is measured from robot link
    std::pair<Vector, Vector> velocity = plink->GetVelocity();
    velocity.first += velocity.second.cross(tbody.trans - t.trans);
    if( !!pPreviousGrabbed ) {
        dReal disterror = TransformDistance2(t*pPreviousGrabbed->_troot, tbody);
        if( pPreviousGrabbed->_plinkrobot == plink && disterror <= g_fEpsilonLinear ) {
            // links and transforms are the same, so no worries
            return true;
        }
        RAVELOG_VERBOSE_FORMAT("Robot %s: body %s already grabbed, but transforms differ by %f \n", GetName()%pbody->GetName()%disterror);
        _RemoveAttachedBody(pbody);
        CallOnDestruction destructigonhook(boost::bind(&RobotBase::_AttachBody,this,pbody));
        pPreviousGrabbed->_plinkrobot = plink;
        pPreviousGrabbed->_troot = t.inverse() * tbody;
        pPreviousGrabbed->_ProcessCollidingLinks(pPreviousGrabbed->_setRobotLinksToIgnore);
        pbody->SetVelocity(velocity.first, velocity.second);
        return true;
    }

    GrabbedPtr pgrabbed(new Grabbed(pbody,plink));
    pgrabbed->_troot = t.inverse() * tbody;
    //uint64_t starttime1 = utils::GetMicroTime();
    // always ignore links that are statically attached to plink (ie assume they are always colliding with the body)
    
    std::vector<boost::shared_ptr<Link> > vattachedlinks;
    plink->GetRigidlyAttachedLinks(vattachedlinks);
    std::set<int> setRobotLinksToIgnore;
    FOREACHC(itlink, vattachedlinks) {
        setRobotLinksToIgnore.insert((*itlink)->GetIndex());
    }
    pgrabbed->_ProcessCollidingLinks(setRobotLinksToIgnore);
    pbody->SetVelocity(velocity.first, velocity.second);
    _vGrabbedBodies.push_back(pgrabbed);
    //uint64_t starttime2 = utils::GetMicroTime();
    try {
        // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
        _AttachBody(pbody);
    }
    catch(...) {
        BOOST_ASSERT(_vGrabbedBodies.back()==pgrabbed);
        _vGrabbedBodies.pop_back();
        throw;
    }
    //uint64_t starttime3 = utils::GetMicroTime();
    _PostprocessChangedParameters(Prop_RobotGrabbed);
    //RAVELOG_DEBUG_FORMAT("env=%d, post process elapsed (%d) %fs, %fs, %fs, %fs", GetEnv()->GetId()%vattachedlinks.size()%(1e-6*(starttime1-starttime0))%(1e-6*(starttime2-starttime0))%(1e-6*(starttime3-starttime0))%(1e-6*(utils::GetMicroTime()-starttime0)));
    return true;
}

bool RobotBase::Grab(KinBodyPtr pbody, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore)
{
    OPENRAVE_ASSERT_FORMAT(!!pbody && !!pRobotLinkToGrabWith && pRobotLinkToGrabWith->GetParent() == shared_kinbody(), "robot %s invalid grab arguments",GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pbody != shared_kinbody(), "robot %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
    if( IsGrabbing(pbody) ) {
        if( setRobotLinksToIgnore.size() > 0 ) {
            // update the current grabbed info with setRobotLinksToIgnore
            FOREACHC(itgrabbed, _vGrabbedBodies) {
                GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                if( KinBodyConstPtr(pgrabbed->_pgrabbedbody) == pbody ) {
                    pgrabbed->AddMoreIgnoreLinks(setRobotLinksToIgnore);
                    break;
                }
            }
        }
        RAVELOG_VERBOSE(str(boost::format("Robot %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
        return true;
    }

    GrabbedPtr pgrabbed(new Grabbed(pbody,pRobotLinkToGrabWith));
    Transform t = pRobotLinkToGrabWith->GetTransform();
    Transform tbody = pbody->GetTransform();
    pgrabbed->_troot = t.inverse() * tbody;
    pgrabbed->_ProcessCollidingLinks(setRobotLinksToIgnore);
    // set velocity
    std::pair<Vector, Vector> velocity = pRobotLinkToGrabWith->GetVelocity();
    velocity.first += velocity.second.cross(tbody.trans - t.trans);
    pbody->SetVelocity(velocity.first, velocity.second);
    _vGrabbedBodies.push_back(pgrabbed);
    _AttachBody(pbody);
    _AttachBody(pbody);
    try {
        // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
        _AttachBody(pbody);
    }
    catch(...) {
        BOOST_ASSERT(_vGrabbedBodies.back()==pgrabbed);
        _vGrabbedBodies.pop_back();
        throw;
    }
    _PostprocessChangedParameters(Prop_RobotGrabbed);
    return true;
}

void RobotBase::Release(KinBodyPtr pbody)
{
    FOREACH(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        if( KinBodyPtr(pgrabbed->_pgrabbedbody) == pbody ) {
            _vGrabbedBodies.erase(itgrabbed);
            _RemoveAttachedBody(pbody);
            _PostprocessChangedParameters(Prop_RobotGrabbed);
            return;
        }
    }

    RAVELOG_DEBUG(str(boost::format("Robot %s: body %s not grabbed\n")%GetName()%pbody->GetName()));
}

void RobotBase::ReleaseAllGrabbed()
{
    if( _vGrabbedBodies.size() > 0 ) {
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
            if( !!pbody ) {
                _RemoveAttachedBody(pbody);
            }
        }
        _vGrabbedBodies.clear();
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
}

void RobotBase::RegrabAll()
{
    CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
    CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
    std::vector<LinkPtr > vattachedlinks;
    FOREACH(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyPtr pbody(pgrabbed->_pgrabbedbody);
        if( !!pbody ) {
            _RemoveAttachedBody(pbody);
            CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pbody));
            pgrabbed->_ProcessCollidingLinks(pgrabbed->_setRobotLinksToIgnore);
        }
    }
}

void RobotBase::_Regrab(UserDataPtr _pgrabbed)
{
    GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(_pgrabbed);
    KinBodyPtr pgrabbedbody = pgrabbed->_pgrabbedbody.lock();
    if( !!pgrabbedbody ) {
        // have to re-grab the body, which means temporarily resetting the collision checker and attachment
        CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        _RemoveAttachedBody(pgrabbedbody);
        CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pgrabbedbody));
        pgrabbed->_ProcessCollidingLinks(pgrabbed->_setRobotLinksToIgnore);
    }
}

RobotBase::LinkPtr RobotBase::IsGrabbing(KinBodyConstPtr pbody) const
{
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        if( KinBodyConstPtr(pgrabbed->_pgrabbedbody) == pbody ) {
            return pgrabbed->_plinkrobot;
        }
    }
    return LinkPtr();
}

void RobotBase::GetGrabbed(std::vector<KinBodyPtr>& vbodies) const
{
    vbodies.resize(0);
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
        if( !!pbody && pbody->GetEnvironmentId() ) {
            vbodies.push_back(pbody);
        }
    }
}

void RobotBase::GetGrabbedInfo(std::vector<RobotBase::GrabbedInfoPtr>& vgrabbedinfo) const
{
    vgrabbedinfo.resize(_vGrabbedBodies.size());
    for(size_t i = 0; i < vgrabbedinfo.size(); ++i) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(_vGrabbedBodies[i]);
        vgrabbedinfo[i].reset(new GrabbedInfo());
        vgrabbedinfo[i]->_grabbedname = pgrabbed->_pgrabbedbody.lock()->GetName();
        vgrabbedinfo[i]->_robotlinkname = pgrabbed->_plinkrobot->GetName();
        vgrabbedinfo[i]->_trelative = pgrabbed->_troot;
        vgrabbedinfo[i]->_setRobotLinksToIgnore = pgrabbed->_setRobotLinksToIgnore;
        FOREACHC(itlink, _veclinks) {
            if( find(pgrabbed->_listNonCollidingLinks.begin(), pgrabbed->_listNonCollidingLinks.end(), *itlink) == pgrabbed->_listNonCollidingLinks.end() ) {
                vgrabbedinfo[i]->_setRobotLinksToIgnore.insert((*itlink)->GetIndex());
            }
        }
    }
}

void RobotBase::ResetGrabbed(const std::vector<RobotBase::GrabbedInfoConstPtr>& vgrabbedinfo)
{
    ReleaseAllGrabbed();
    if( vgrabbedinfo.size() > 0 ) {
        CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        FOREACHC(itgrabbedinfo, vgrabbedinfo) {
            GrabbedInfoConstPtr pgrabbedinfo = *itgrabbedinfo;
            KinBodyPtr pbody = GetEnv()->GetKinBody(pgrabbedinfo->_grabbedname);
            KinBody::LinkPtr pRobotLinkToGrabWith = GetLink(pgrabbedinfo->_robotlinkname);
            OPENRAVE_ASSERT_FORMAT(!!pbody && !!pRobotLinkToGrabWith, "robot %s invalid grab arguments",GetName(), ORE_InvalidArguments);
            OPENRAVE_ASSERT_FORMAT(pbody != shared_kinbody(), "robot %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
            if( IsGrabbing(pbody) ) {
                RAVELOG_VERBOSE(str(boost::format("Robot %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
                continue;
            }

            GrabbedPtr pgrabbed(new Grabbed(pbody,pRobotLinkToGrabWith));
            pgrabbed->_troot = pgrabbedinfo->_trelative;
            pgrabbed->_ProcessCollidingLinks(pgrabbedinfo->_setRobotLinksToIgnore);
            Transform tlink = pRobotLinkToGrabWith->GetTransform();
            Transform tbody = tlink * pgrabbed->_troot;
            pbody->SetTransform(tbody);
            // set velocity
            std::pair<Vector, Vector> velocity = pRobotLinkToGrabWith->GetVelocity();
            velocity.first += velocity.second.cross(tbody.trans - tlink.trans);
            pbody->SetVelocity(velocity.first, velocity.second);
            _vGrabbedBodies.push_back(pgrabbed);
            _AttachBody(pbody);
        }
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
}

void RobotBase::GetIgnoredLinksOfGrabbed(KinBodyConstPtr body, std::list<KinBody::LinkConstPtr>& ignorelinks) const
{
    ignorelinks.clear();
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr grabbedbody = pgrabbed->_pgrabbedbody.lock();
        if( grabbedbody == body ) {
            FOREACHC(itrobotlink, _veclinks) {
                if( find(pgrabbed->_listNonCollidingLinks.begin(), pgrabbed->_listNonCollidingLinks.end(), *itrobotlink) == pgrabbed->_listNonCollidingLinks.end() ) {
                    ignorelinks.push_back(*itrobotlink);
                }
            }
            return;
        }
    }
    RAVELOG_WARN(str(boost::format("body %s is not currently grabbed")%body->GetName()));
}

void RobotBase::SetActiveManipulator(int index)
{
    _pManipActive = _vecManipulators.at(index);
}

void RobotBase::SetActiveManipulator(ManipulatorConstPtr pmanip)
{
    if( !pmanip ) {
        _pManipActive.reset();
    }
    else {
        FOREACH(itmanip,_vecManipulators) {
            if( *itmanip == pmanip ) {
                _pManipActive = *itmanip;
                return;
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find manipulator with name: %s"), pmanip->GetName(), ORE_InvalidArguments);
    }
}

RobotBase::ManipulatorPtr RobotBase::SetActiveManipulator(const std::string& manipname)
{
    if( manipname.size() > 0 ) {
        FOREACH(itmanip,_vecManipulators) {
            if( (*itmanip)->GetName() == manipname ) {
                _pManipActive = *itmanip;
                return _pManipActive;
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find manipulator with name: %s"), manipname, ORE_InvalidArguments);
    }
    _pManipActive.reset();
    return _pManipActive;
}

RobotBase::ManipulatorPtr RobotBase::GetActiveManipulator()
{
    return _pManipActive;
}

RobotBase::ManipulatorConstPtr RobotBase::GetActiveManipulator() const
{
    return _pManipActive;
}

int RobotBase::GetActiveManipulatorIndex() const
{
    for(size_t i = 0; i < _vecManipulators.size(); ++i) {
        if( _pManipActive == _vecManipulators[i] ) {
            return (int)i;
        }
    }
    return -1;
}

RobotBase::ManipulatorPtr RobotBase::AddManipulator(const RobotBase::ManipulatorInfo& manipinfo, bool removeduplicate)
{
    OPENRAVE_ASSERT_OP(manipinfo._name.size(),>,0);
    int iremoveindex = -1;
    for(int imanip = 0; imanip < (int)_vecManipulators.size(); ++imanip) {
        if( _vecManipulators[imanip]->GetName() == manipinfo._name ) {
            if( removeduplicate ) {
                iremoveindex = imanip;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("manipulator with name %s already exists"),manipinfo._name,ORE_InvalidArguments);
            }
        }
    }
    ManipulatorPtr newmanip(new Manipulator(shared_robot(),manipinfo));
    newmanip->_ComputeInternalInformation();
    if( iremoveindex >= 0 ) {
        // replace the old one
        _vecManipulators[iremoveindex] = newmanip;
    }
    else {
        _vecManipulators.push_back(newmanip);
    }
    __hashrobotstructure.resize(0);
    return newmanip;
}

bool RobotBase::RemoveManipulator(ManipulatorPtr manip)
{
    if( _pManipActive == manip ) {
        _pManipActive.reset();
    }
    FOREACH(itmanip,_vecManipulators) {
        if( *itmanip == manip ) {
            _vecManipulators.erase(itmanip);
            __hashrobotstructure.resize(0);
            return true;
        }
    }
    return false;
}

RobotBase::AttachedSensorPtr RobotBase::AddAttachedSensor(const RobotBase::AttachedSensorInfo& attachedsensorinfo, bool removeduplicate)
{
    OPENRAVE_ASSERT_OP(attachedsensorinfo._name.size(),>,0);
    int iremoveindex = -1;
    for(int iasensor = 0; iasensor < (int)_vecSensors.size(); ++iasensor) {
        if( _vecSensors[iasensor]->GetName() == attachedsensorinfo._name ) {
            if( removeduplicate ) {
                iremoveindex = iasensor;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("attached sensor with name %s already exists"),attachedsensorinfo._name,ORE_InvalidArguments);
            }
        }
    }
    AttachedSensorPtr newattachedsensor(new AttachedSensor(shared_robot(),attachedsensorinfo));
    //newattachedsensor->_ComputeInternalInformation();
    if( iremoveindex >= 0 ) {
        // replace the old one
        _vecSensors[iremoveindex] = newattachedsensor;
    }
    else {
        _vecSensors.push_back(newattachedsensor);
    }
    newattachedsensor->UpdateInfo(); // just in case
    __hashrobotstructure.resize(0);
    return newattachedsensor;
}

RobotBase::AttachedSensorPtr RobotBase::GetAttachedSensor(const std::string& name) const
{
    FOREACHC(itsensor, _vecSensors) {
        if( (*itsensor)->GetName() == name ) {
            return *itsensor;
        }
    }
    return RobotBase::AttachedSensorPtr();
}

bool RobotBase::RemoveAttachedSensor(AttachedSensorPtr attsensor)
{
    FOREACH(itattsensor,_vecSensors) {
        if( *itattsensor == attsensor ) {
            _vecSensors.erase(itattsensor);
            __hashrobotstructure.resize(0);
            return true;
        }
    }
    return false;
}

/// Check if body is self colliding. Links that are joined together are ignored.
bool RobotBase::CheckSelfCollision(CollisionReportPtr report, CollisionCheckerBasePtr collisionchecker) const
{
    if( !collisionchecker ) {
        collisionchecker = _selfcollisionchecker;
        if( !collisionchecker ) {
            collisionchecker = GetEnv()->GetCollisionChecker();
            if( !collisionchecker ) {
                // no checker set
                return false;
            }
        }
    }

    bool bAllLinkCollisions = !!(collisionchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bCollision = false;
    if( KinBody::CheckSelfCollision(report, collisionchecker) ) {
        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
            return true;
        }
        bCollision = true;
    }

    // if collision checker is set to distance checking, have to compare reports for the minimum distance
    int coloptions = collisionchecker->GetCollisionOptions();
    CollisionReport tempreport;
    CollisionReportPtr pusereport = report;
    if( !!report && (coloptions & CO_Distance) ) {
        pusereport = boost::shared_ptr<CollisionReport>(&tempreport,utils::null_deleter());
    }

    // check all grabbed bodies with (TODO: support CO_ActiveDOFs option)
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr pbody(pgrabbed->_pgrabbedbody);
        if( !pbody ) {
            continue;
        }
        FOREACHC(itrobotlink,pgrabbed->_listNonCollidingLinks) {
            // have to use link/link collision since link/body checks attached bodies
            FOREACHC(itbodylink,pbody->GetLinks()) {
                if( collisionchecker->CheckCollision(*itrobotlink,KinBody::LinkConstPtr(*itbodylink),pusereport) ) {
                    bCollision = true;
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        break;
                    }
                }
                if( !!pusereport && pusereport->minDistance < report->minDistance ) {
                    *report = *pusereport;
                }
            }
            if( bCollision ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    break;
                }
            }
        }
        if( bCollision ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                break;
            }
        }

        if( pbody->CheckSelfCollision(pusereport, collisionchecker) ) {
            bCollision = true;
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                break;
            }
        }
        if( !!pusereport && pusereport->minDistance < report->minDistance ) {
            *report = *pusereport;
        }

        // check attached bodies with each other, this is actually tricky since they are attached "with each other", so regular CheckCollision will not work.
        // Instead, we will compare each of the body's links with every other
        if( _vGrabbedBodies.size() > 1 ) {
            FOREACHC(itgrabbed2, _vGrabbedBodies) {
                GrabbedConstPtr pgrabbed2 = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed2);
                KinBodyPtr pbody2(pgrabbed2->_pgrabbedbody);
                if( pbody == pbody2 ) {
                    continue;
                }
                FOREACHC(itlink2, pbody2->GetLinks()) {
                    // make sure the two bodies were not initially colliding
                    if( find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink2) != pgrabbed->_listNonCollidingLinks.end() ) {
                        FOREACHC(itlink, pbody->GetLinks()) {
                            if( find(pgrabbed2->_listNonCollidingLinks.begin(),pgrabbed2->_listNonCollidingLinks.end(),*itlink) != pgrabbed2->_listNonCollidingLinks.end() ) {
                                if( collisionchecker->CheckCollision(KinBody::LinkConstPtr(*itlink),KinBody::LinkConstPtr(*itlink2),pusereport) ) {
                                    bCollision = true;
                                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                        break;
                                    }
                                }
                                if( !!pusereport && pusereport->minDistance < report->minDistance ) {
                                    *report = *pusereport;
                                }
                            }
                            if( bCollision ) {
                                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                    break;
                                }
                            }
                        }
                        if( bCollision ) {
                            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                break;
                            }
                        }
                    }
                }
                if( bCollision ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        break;
                    }
                }
            }
            if( bCollision ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    break;
                }
            }
        }
    }

    if( bCollision && !!report ) {
        if( report != pusereport ) {
            *report = *pusereport;
        }
        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            RAVELOG_VERBOSE(str(boost::format("Self collision: %s\n")%report->__str__()));
            std::vector<OpenRAVE::dReal> v;
            GetDOFValues(v);
            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
            for(size_t i = 0; i < v.size(); ++i ) {
                if( i > 0 ) {
                    ss << "," << v[i];
                }
                else {
                    ss << "colvalues=[" << v[i];
                }
            }
            ss << "]";
            RAVELOG_VERBOSE(ss.str());
        }
    }
    return bCollision;
}

bool RobotBase::CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;
    if( plink->IsEnabled() ) {
        boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if( pchecker->CheckCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if( pgrabbed->_plinkrobot == plink ) {
            KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
            if( !!pbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_vGrabbedBodies) {
                    if( itgrabbed2 != itgrabbed ) {
                        GrabbedConstPtr pgrabbed2 = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed2);
                        KinBodyPtr pbody2 = pgrabbed2->_pgrabbedbody.lock();
                        if( !!pbody2 ) {
                            vbodyexcluded.push_back(pbody2);
                        }
                    }
                }
                KinBodyStateSaver bodysaver(pbody,Save_LinkTransformation);
                pbody->SetTransform(tlinktrans * pgrabbed->_troot);
                if( pchecker->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool RobotBase::CheckLinkCollision(int ilinkindex, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    if( plink->IsEnabled() ) {
        if( pchecker->CheckCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if( pgrabbed->_plinkrobot == plink ) {
            KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
            if( !!pbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_vGrabbedBodies) {
                    if( itgrabbed2 != itgrabbed ) {
                        GrabbedConstPtr pgrabbed2 = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed2);
                        KinBodyPtr pbody2 = pgrabbed2->_pgrabbedbody.lock();
                        if( !!pbody2 ) {
                            vbodyexcluded.push_back(pbody2);
                        }
                    }
                }
                if( pchecker->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool RobotBase::CheckLinkSelfCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    // TODO: have to consider rigidly attached links??
    CollisionCheckerBasePtr pchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    LinkPtr plink = _veclinks.at(ilinkindex);
    if( plink->IsEnabled() ) {
        boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if( pchecker->CheckStandaloneSelfCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    KinBodyStateSaverPtr linksaver;
    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if( pgrabbed->_plinkrobot == plink ) {
            KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
            if( !!pbody ) {
                if( !linksaver ) {
                    linksaver.reset(new KinBodyStateSaver(shared_kinbody()));
                    plink->Enable(false);
                    // also disable rigidly attached links?
                }
                KinBodyStateSaver bodysaver(pbody,Save_LinkTransformation);
                pbody->SetTransform(tlinktrans * pgrabbed->_troot);
                if( pchecker->CheckCollision(shared_kinbody_const(), KinBodyConstPtr(pbody),report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
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
    _vAllDOFIndices.resize(GetDOF());
    for(int i = 0; i < GetDOF(); ++i) {
        _vAllDOFIndices[i] = i;
    }

    _activespec._vgroups.reserve(2);
    _activespec._vgroups.resize(0);
    if( _vAllDOFIndices.size() > 0 ) {
        ConfigurationSpecification::Group group;
        stringstream ss;
        ss << "joint_values " << GetName();
        FOREACHC(it,_vAllDOFIndices) {
            ss << " " << *it;
        }
        group.name = ss.str();
        group.dof = (int)_vAllDOFIndices.size();
        group.offset = 0;
        // do not initialize interpolation, since it implies a motion sampling strategy
        _activespec._vgroups.push_back(group);
    }

    int manipindex=0;
    FOREACH(itmanip,_vecManipulators) {
        if( (*itmanip)->_info._name.size() == 0 ) {
            stringstream ss;
            ss << "manip" << manipindex;
            RAVELOG_WARN(str(boost::format("robot %s has a manipulator with no name, setting to %s\n")%GetName()%ss.str()));
            (*itmanip)->_info._name = ss.str();
        }
        (*itmanip)->_ComputeInternalInformation();
        vector<ManipulatorPtr>::iterator itmanip2 = itmanip; ++itmanip2;
        for(; itmanip2 != _vecManipulators.end(); ++itmanip2) {
            if( (*itmanip)->GetName() == (*itmanip2)->GetName() ) {
                RAVELOG_WARN(str(boost::format("robot %s has two manipulators with the same name: %s!\n")%GetName()%(*itmanip)->GetName()));
            }
        }
        manipindex++;
    }
    // set active manipulator to first manipulator
    if( _vecManipulators.size() > 0 ) {
        _pManipActive = _vecManipulators.at(0);
    }
    else {
        _pManipActive.reset();
    }

    int sensorindex=0;
    FOREACH(itsensor,_vecSensors) {
        if( (*itsensor)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "sensor" << sensorindex;
            RAVELOG_WARN(str(boost::format("robot %s has a sensor with no name, setting to %s\n")%GetName()%ss.str()));
            (*itsensor)->_info._name = ss.str();
        }
        else if( !utils::IsValidName((*itsensor)->GetName()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("sensor name \"%s\" is not valid"), (*itsensor)->GetName(), ORE_Failed);
        }
        if( !!(*itsensor)->GetSensor() ) {
            stringstream ss; ss << GetName() << ":" << (*itsensor)->GetName(); // global unique name?
            (*itsensor)->GetSensor()->SetName(ss.str());
        }
        sensorindex++;
    }

    {
        __hashrobotstructure.resize(0);
        FOREACH(itsensor,_vecSensors) {
            (*itsensor)->__hashstructure.resize(0);
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

void RobotBase::_PostprocessChangedParameters(uint32_t parameters)
{
    if( parameters & (Prop_Sensors|Prop_SensorPlacement) ) {
        FOREACH(itsensor,_vecSensors) {
            (*itsensor)->__hashstructure.resize(0);
        }
    }
    if( parameters & Prop_RobotManipulatorTool ) {
        FOREACH(itmanip,_vecManipulators) {
            (*itmanip)->__hashstructure.resize(0);
            (*itmanip)->__hashkinematicsstructure.resize(0);
        }
    }
    KinBody::_PostprocessChangedParameters(parameters);

    if( (parameters&Prop_LinkEnable) == Prop_LinkEnable ) {
        // check if any regrabbed bodies have the link in _listNonCollidingLinks and the link is enabled, or are missing the link in _listNonCollidingLinks and the link is disabled
        std::map<GrabbedPtr, list<KinBody::LinkConstPtr> > mapcheckcollisions;
        FOREACH(itlink,_veclinks) {
            if( (*itlink)->IsEnabled() ) {
                FOREACH(itgrabbed,_vGrabbedBodies) {
                    GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                    if( find(pgrabbed->GetRigidlyAttachedLinks().begin(),pgrabbed->GetRigidlyAttachedLinks().end(), *itlink) == pgrabbed->GetRigidlyAttachedLinks().end() ) {
                        std::list<KinBody::LinkConstPtr>::iterator itnoncolliding = find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink);
                        if( itnoncolliding != pgrabbed->_listNonCollidingLinks.end() ) {
                            if( pgrabbed->WasLinkNonColliding(*itlink) == 0 ) {
                                pgrabbed->_listNonCollidingLinks.erase(itnoncolliding);
                            }
                            mapcheckcollisions[pgrabbed].push_back(*itlink);
                        }
                        else {
                            // try to restore
                            if( pgrabbed->WasLinkNonColliding(*itlink) == 1 ) {
                                pgrabbed->_listNonCollidingLinks.push_back(*itlink);
                            }
                        }
                    }
                }
            }
            else {
                // add since it is disabled?
                FOREACH(itgrabbed,_vGrabbedBodies) {
                    GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                    if( find(pgrabbed->GetRigidlyAttachedLinks().begin(),pgrabbed->GetRigidlyAttachedLinks().end(), *itlink) == pgrabbed->GetRigidlyAttachedLinks().end() ) {
                        if( find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink) == pgrabbed->_listNonCollidingLinks.end() ) {
                            if( pgrabbed->WasLinkNonColliding(*itlink) != 0 ) {
                                pgrabbed->_listNonCollidingLinks.push_back(*itlink);
                            }
                        }
                    }
                }
            }
        }

//        if( mapcheckcollisions.size() > 0 ) {
//            CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
//            FOREACH(itgrabbed, mapcheckcollisions) {
//                KinBodyPtr pgrabbedbody(itgrabbed->first->_pgrabbedbody);
//                _RemoveAttachedBody(pgrabbedbody);
//                CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pgrabbedbody));
//                FOREACH(itlink, itgrabbed->second) {
//                    if( pchecker->CheckCollision(*itlink, KinBodyConstPtr(pgrabbedbody)) ) {
//                        itgrabbed->first->_listNonCollidingLinks.remove(*itlink);
//                    }
//                }
//            }
//        }
    }
}

std::vector<RobotBase::ManipulatorPtr>& RobotBase::GetManipulators()
{
    return _vecManipulators;
}

RobotBase::ManipulatorPtr RobotBase::GetManipulator(const std::string& name) const
{
    FOREACHC(itmanip, _vecManipulators) {
        if( (*itmanip)->GetName() == name ) {
            return *itmanip;
        }
    }
    return RobotBase::ManipulatorPtr();
}

void RobotBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    KinBody::Clone(preference,cloningoptions);
    RobotBaseConstPtr r = RaveInterfaceConstCast<RobotBase>(preference);
    _selfcollisionchecker.reset();
    if( !!r->_selfcollisionchecker ) {
        // TODO clone the self collision checker?
    }
    __hashrobotstructure = r->__hashrobotstructure;
    _vecManipulators.clear();
    _pManipActive.reset();
    FOREACHC(itmanip, r->_vecManipulators) {
        ManipulatorPtr pmanip(new Manipulator(shared_robot(),*itmanip));
        _vecManipulators.push_back(pmanip);
        if( !!r->GetActiveManipulator() && r->GetActiveManipulator()->GetName() == (*itmanip)->GetName() ) {
            _pManipActive = pmanip;
        }
    }

    _vecSensors.clear();
    FOREACHC(itsensor, r->_vecSensors) {
        _vecSensors.push_back(AttachedSensorPtr(new AttachedSensor(shared_robot(),**itsensor,cloningoptions)));
    }
    _UpdateAttachedSensors();

    _vActiveDOFIndices = r->_vActiveDOFIndices;
    _vAllDOFIndices = r->_vAllDOFIndices;
    vActvAffineRotationAxis = r->vActvAffineRotationAxis;
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

    // clone the grabbed bodies, note that this can fail if the new cloned environment hasn't added the bodies yet (check out Environment::Clone)
    _vGrabbedBodies.resize(0);
    FOREACHC(itgrabbedref, r->_vGrabbedBodies) {
        GrabbedConstPtr pgrabbedref = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbedref);

        KinBodyPtr pbodyref(pgrabbedref->_pgrabbedbody);
        KinBodyPtr pgrabbedbody;
        if( !!pbodyref ) {
            pgrabbedbody = GetEnv()->GetBodyFromEnvironmentId(pbodyref->GetEnvironmentId());
            BOOST_ASSERT(pgrabbedbody->GetName() == pbodyref->GetName());

            GrabbedPtr pgrabbed(new Grabbed(pgrabbedbody,_veclinks.at(KinBody::LinkPtr(pgrabbedref->_plinkrobot)->GetIndex())));
            pgrabbed->_troot = pgrabbedref->_troot;
            pgrabbed->_listNonCollidingLinks.clear();
            FOREACHC(itlinkref, pgrabbedref->_listNonCollidingLinks) {
                pgrabbed->_listNonCollidingLinks.push_back(_veclinks.at((*itlinkref)->GetIndex()));
            }
            _vGrabbedBodies.push_back(pgrabbed);
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
        __hashrobotstructure = utils::GetMD5HashString(ss.str());
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
