// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
#include "rplanners.h"

class WorkspaceTrajectoryTracker : public PlannerBase
{
public:
    WorkspaceTrajectoryTracker(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = "\
:Interface Author:  Rosen Diankov\n\n\
Given a workspace trajectory of the end effector of a manipulator (active manipulator of the robot), finds a configuration space trajectory that tracks it using analytical inverse kinematics.\n\
Options can be specified to prioritize trajectory time, trajectory smoothness, and planning time\n\
In the simplest case, the workspace trajectory can be a straight line from one point to another.\n\
\n\
Planner Parameters\n\
==================\n\
\n\
- **dReal maxdeviationangle** - the maximum angle the next iksolution can deviate from the expected direction computed by the jacobian.\n\
\n\
- **bool maintaintiming** - maintain timing with input trajectory\n\
\n\
- **dReal ignorefirstcollision** - if > 0, will allow the robot to be in environment collision for the initial 'ignorefirstcollision' seconds of the trajectory. Once the robot gets out of collision, it will execute its normal following phase until it gets into collision again. This option is used when lifting objects from a surface, where the object is already in collision with the surface.\n\
\n\
- **dReal minimumcompletetime** - specifies the minimum trajectory that must be followed for planner to declare success. If 0, then the entire trajectory has to be followed.\n\
\n\
- **TrajectoryBasePtr workspacetraj** - workspace trajectory of the end effector, needs to hold 'ikparam_values' groups\n\
\n\
";
        _report.reset(new CollisionReport());
        _filteroptions = 0;
    }
    virtual ~WorkspaceTrajectoryTracker() {
    }

    virtual bool InitPlan(RobotBasePtr probot, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        boost::shared_ptr<WorkspaceTrajectoryParameters> parameters(new WorkspaceTrajectoryParameters(GetEnv()));
        parameters->copy(params);
        _robot = probot;
        _manip = _robot->GetActiveManipulator();

        if( (int)_manip->GetArmIndices().size() != parameters->GetDOF() ) {
            RAVELOG_WARN("parameter configuraiton space must be the robot's active manipulator\n");
            return false;
        }

        if( !parameters->workspacetraj ||( parameters->workspacetraj->GetDuration() == 0) ||( parameters->workspacetraj->GetNumWaypoints() == 0) ) {
            RAVELOG_ERROR("input trajectory needs to be initialized with interpolation information\n");
        }

        // check if the parameters configuration space actually reflects the active manipulator, move to the upper and lower limits
        {
            RobotBase::RobotStateSaver saver(_robot);
            boost::array<std::vector<dReal>*,2> testvalues = { { &parameters->_vConfigLowerLimit,&parameters->_vConfigUpperLimit}};
            vector<dReal> dummyvalues;
            for(size_t i = 0; i < testvalues.size(); ++i) {
                parameters->_setstatefn(*testvalues[i]);
                Transform tstate = _manip->GetTransform();
                _robot->SetActiveDOFs(_manip->GetArmIndices());
                _robot->GetActiveDOFValues(dummyvalues);
                for(size_t j = 0; j < dummyvalues.size(); ++j) {
                    // this is necessary in case robot's have limits like [-100,100] for revolute joints (pa10 arm)
                    int dofindex = _manip->GetArmIndices().at(j);
                    KinBody::JointPtr pjoint = _robot->GetJointFromDOFIndex(dofindex);
                    dReal diff = RaveFabs(pjoint->SubtractValue(dummyvalues.at(j), testvalues[i]->at(j), dofindex-pjoint->GetDOFIndex()));
                    if( diff > g_fEpsilonLinear ) {
                        RAVELOG_ERROR(str(boost::format("parameter configuration space does not match active manipulator, dof %d=%f!\n")%j%RaveFabs(dummyvalues.at(j) - testvalues[i]->at(j))));
                        return false;
                    }
                }
            }
        }

        _fMaxCosDeviationAngle = RaveCos(parameters->maxdeviationangle);

        if( parameters->maintaintiming ) {
            RAVELOG_WARN("currently do not support maintaining timing\n");
        }

        RobotBase::RobotStateSaver savestate(_robot);
        // should check collisio only for independent links that do not move during the planning process. This might require a CO_IndependentFromActiveDOFs option.
        //if(CollisionFunctions::CheckCollision(parameters,_robot,parameters->vinitialconfig, _report)) {

        // validate the initial state if one exists
        if( parameters->vinitialconfig.size() > 0 ) {
            if( (int)parameters->vinitialconfig.size() != parameters->GetDOF() ) {
                RAVELOG_ERROR(str(boost::format("initial config wrong dim: %d\n")%parameters->vinitialconfig.size()));
                return false;
            }
            parameters->_setstatefn(parameters->vinitialconfig);
            //            if( !parameters->_checkpathconstraintsfn(parameters->vinitialconfig, parameters->vinitialconfig,IT_OpenStart,ConfigurationListPtr()) ) {
            //                RAVELOG_WARN("initial state rejected by constraint fn\n");
            //                return false;
            //            }
        }

        if( !_manip->GetIkSolver() ) {
            RAVELOG_ERROR(str(boost::format("manipulator %s does not have ik solver set\n")%_manip->GetName()));
            return false;
        }

        if( !_manip->GetIkSolver()->Supports(IKP_Transform6D) ) {
            RAVELOG_ERROR(str(boost::format("WorkspaceTrajectoryTracker: unsupported iktype for manipulator %s")%_manip->GetName()));
            return false;
        }

        _retimerplanner = RaveCreatePlanner(GetEnv(),"lineartrajectoryretimer");
        _parameters = parameters;
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr poutputtraj)
    {
        if(!_parameters) {
            RAVELOG_ERROR("WorkspaceTrajectoryTracker::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = utils::GetMilliTime();
        RobotBase::RobotStateSaver savestate(_robot);
        _robot->SetActiveDOFs(_manip->GetArmIndices());     // should be set by user anyway, but this is an extra precaution
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        if( poutputtraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            poutputtraj->Init(_parameters->_configurationspecification);
        }

        // first check if the end effectors are in collision
        TrajectoryBaseConstPtr workspacetraj = _parameters->workspacetraj;
        IkParameterization ikparam;
        vector<dReal> vtrajpoint;
        workspacetraj->Sample(vtrajpoint,workspacetraj->GetDuration());
        workspacetraj->GetConfigurationSpecification().ExtractIkParameterization(ikparam,vtrajpoint.begin());
        Transform tlasttrans = ikparam.GetTransform6D();
        dReal minimumcompletetime = _parameters->minimumcompletetime;
        if( minimumcompletetime <= 0 ) {
            minimumcompletetime += workspacetraj->GetDuration();
        }
        if( _manip->CheckEndEffectorCollision(tlasttrans,_report) ) {
            if( minimumcompletetime >= workspacetraj->GetDuration() ) {
                RAVELOG_DEBUG(str(boost::format("final configuration colliding: %s\n")%_report->__str__()));
                return PS_Failed;
            }
        }

        dReal fstarttime = 0, fendtime = workspacetraj->GetDuration();
        bool bPrevInCollision = true;
        list<Transform> listtransforms;
        dReal ftime = 0;
        for(; ftime < workspacetraj->GetDuration()-_parameters->_fStepLength*0.5; ftime += _parameters->_fStepLength) {
            workspacetraj->Sample(vtrajpoint,ftime);
            workspacetraj->GetConfigurationSpecification().ExtractIkParameterization(ikparam,vtrajpoint.begin());
            Transform t = ikparam.GetTransform6D();
            listtransforms.push_back(t);
            // end effector is only fully known given the entire 6D transform!
            if( _manip->CheckEndEffectorCollision(t,_report) ) {
                if(( ftime < _parameters->ignorefirstcollision) && bPrevInCollision ) {
                    continue;
                }
                if( !bPrevInCollision ) {
                    if( ftime >= minimumcompletetime ) {
                        fendtime = ftime;
                        break;
                    }
                }
                RAVELOG_WARN(str(boost::format("end effector collision at time %f/%f")%ftime%workspacetraj->GetDuration()));
                return PS_Failed;
            }
            else {
                if( bPrevInCollision ) {
                    fstarttime = ftime;
                }
                bPrevInCollision = false;
            }
        }

        if( bPrevInCollision ) {
            // only the last point is valid
            fstarttime = workspacetraj->GetDuration();
        }
        if( fstarttime > _parameters->ignorefirstcollision ) {
            RAVELOG_DEBUG(str(boost::format("initial end effector in collision, start time %f > %f\n")%fstarttime%_parameters->ignorefirstcollision));
            return PS_Failed;
        }

        listtransforms.push_back(tlasttrans);

        _vchildlinks.resize(0);
        // disable all child links since we've already checked their collision
        _manip->GetChildLinks(_vchildlinks);
        FOREACH(it,_vchildlinks) {
            (*it)->Enable(false);
        }

        if( !poutputtraj ) {
            poutputtraj = RaveCreateTrajectory(GetEnv(),"");
        }

        _mjacobian.resize(boost::extents[0][0]);
        _vprevsolution.resize(0);
        _tbaseinv = _manip->GetBase()->GetTransform().inverse();
        if( (int)_parameters->vinitialconfig.size() == _parameters->GetDOF() ) {
            _parameters->_setstatefn(_parameters->vinitialconfig);
            _SetPreviousSolution(_parameters->vinitialconfig,false);
            poutputtraj->Insert(poutputtraj->GetNumWaypoints(),_parameters->vinitialconfig,_parameters->_configurationspecification);
        }

        UserDataPtr filterhandle = _manip->GetIkSolver()->RegisterCustomFilter(0,boost::bind(&WorkspaceTrajectoryTracker::_ValidateSolution,this,_1,_2,_3));
        vector<dReal> vsolution;
        if( !_parameters->greedysearch ) {
            RAVELOG_ERROR("WorkspaceTrajectoryTracker::PlanPath - do not support non-greedy search\n");
        }

        list<Transform>::iterator ittrans = listtransforms.begin();
        bPrevInCollision = true;
        ftime = 0;
        for(; ittrans != listtransforms.end(); ftime += _parameters->_fStepLength, ++ittrans) {
            _filteroptions = (ftime >= fstarttime) ? IKFO_CheckEnvCollisions : 0;
            IkParameterization ikparam(*ittrans,IKP_Transform6D);
            if( !_manip->FindIKSolution(ikparam,vsolution,_filteroptions) ) {
                if( _filteroptions == 0 ) {
                    // haven't even checked with environment collisions, so a solution really doesn't exist
                    return PS_Failed;
                }
                if(( ftime < _parameters->ignorefirstcollision) && bPrevInCollision ) {
                    _filteroptions = 0;
                    if( !_manip->FindIKSolution(ikparam,vsolution,_filteroptions) ) {
                        return PS_Failed;
                    }
                }
                else {
                    if( !bPrevInCollision ) {
                        if( ftime >= minimumcompletetime ) {
                            fendtime = ftime;
                            break;
                        }
                    }
                    return PS_Failed;
                }
            }
            else {
                bPrevInCollision = false;
            }

            poutputtraj->Insert(poutputtraj->GetNumWaypoints(),vsolution,_parameters->_configurationspecification);
            _parameters->_setstatefn(vsolution);
            _SetPreviousSolution(vsolution);
        }

        if( bPrevInCollision ) {
            return PS_Failed;
        }

        if( !_retimerplanner->InitPlan(RobotBasePtr(),_parameters) || !_retimerplanner->PlanPath(poutputtraj) ) {
            return PS_Failed;
        }

        RAVELOG_DEBUG(str(boost::format("workspace trajectory tracker plan success, path=%d points, traj time=%e computed in %fs\n")%poutputtraj->GetNumWaypoints()%poutputtraj->GetDuration()%((0.001f*(float)(utils::GetMilliTime()-basetime)))));
        return PS_HasSolution;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

protected:
    void _SetPreviousSolution(const std::vector<dReal>& vsolution, bool bsetjacobian=true)
    {
        if( bsetjacobian ) {
            _manip->CalculateJacobian(_mjacobian);
            _manip->CalculateRotationJacobian(_mquatjacobian);
            Vector q0 = _tbaseinv.rot;
            // since will be using inside the ik custom filter _ValidateSolution, have to multiply be the inverse of the base
            for(size_t i = 0; i < _manip->GetArmIndices().size(); ++i) {
                Vector v = _tbaseinv.rotate(Vector(_mjacobian[0][i],_mjacobian[1][i],_mjacobian[2][i]));
                _mjacobian[0][i] = v.x; _mjacobian[1][i] = v.y; _mjacobian[2][i] = v.z;
                Vector q1(_mquatjacobian[0][i],_mquatjacobian[1][i],_mquatjacobian[2][i],_mquatjacobian[3][i]);
                Vector q0xq1(q0.x*q1.x - q0.y*q1.y - q0.z*q1.z - q0.w*q1.w,
                             q0.x*q1.y + q0.y*q1.x + q0.z*q1.w - q0.w*q1.z,
                             q0.x*q1.z + q0.z*q1.x + q0.w*q1.y - q0.y*q1.w,
                             q0.x*q1.w + q0.w*q1.x + q0.y*q1.z - q0.z*q1.y);
                _mquatjacobian[0][i] = q0xq1.x; _mquatjacobian[1][i] = q0xq1.y; _mquatjacobian[2][i] = q0xq1.z; _mquatjacobian[3][i] = q0xq1.w;
            }
        }
        else {
            _mjacobian.resize(boost::extents[0][0]);
            _mquatjacobian.resize(boost::extents[0][0]);
        }
        _ikprev = _manip->GetIkParameterization(IKP_Transform6D,false);
        _vprevsolution = vsolution;
    }

    IkReturnAction _ValidateSolution(std::vector<dReal>& vsolution, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikp)
    {
        RobotBase::RobotStateSaver saver(_robot);

        // check if continuous with previous solution using the jacobian
        if( _mjacobian.num_elements() > 0 ) {
            BOOST_ASSERT(ikp.GetType()==IKP_Transform6D);
            Vector expecteddeltatrans = ikp.GetTransform6D().trans - _ikprev.GetTransform6D().trans;
            Vector jdeltatrans;
            dReal solutiondiff = 0;
            for(size_t j = 0; j < vsolution.size(); ++j) {
                dReal d = vsolution[j]-_vprevsolution.at(j);
                jdeltatrans.x += _mjacobian[0][j]*d;
                jdeltatrans.y += _mjacobian[1][j]*d;
                jdeltatrans.z += _mjacobian[2][j]*d;
                solutiondiff += d*d;
            }
            dReal transangle = expecteddeltatrans.dot3(jdeltatrans);
            dReal expecteddeltatrans_len = expecteddeltatrans.lengthsqr3();
            dReal jdeltatrans_len = jdeltatrans.lengthsqr3();
            if( jdeltatrans_len > 1e-7 * solutiondiff ) {     // first see if there is a direction
                if(( transangle < 0) ||( transangle*transangle < _fMaxCosDeviationAngle*_fMaxCosDeviationAngle*expecteddeltatrans_len*jdeltatrans_len) ) {
                    //RAVELOG_INFO("rejected translation: %e < %e\n",transangle,RaveSqrt(_fMaxCosDeviationAngle*_fMaxCosDeviationAngle*expecteddeltatrans_len*jdeltatrans_len));
                    return IKRA_Reject;
                }
            }

            // constrain rotations
            Vector expecteddeltaquat = ikp.GetTransform6D().rot - _ikprev.GetTransform6D().rot;
            Vector jdeltaquat;
            solutiondiff = 0;
            for(size_t j = 0; j < vsolution.size(); ++j) {
                dReal d = vsolution[j]-_vprevsolution.at(j);
                jdeltaquat.x += _mquatjacobian[0][j]*d;
                jdeltaquat.y += _mquatjacobian[1][j]*d;
                jdeltaquat.z += _mquatjacobian[2][j]*d;
                jdeltaquat.w += _mquatjacobian[3][j]*d;
                solutiondiff += d*d;
            }
            dReal quatangle = expecteddeltaquat.dot(jdeltaquat);
            dReal expecteddeltaquat_len = expecteddeltaquat.lengthsqr4();
            dReal jdeltaquat_len = jdeltaquat.lengthsqr4();
            if( jdeltaquat_len > 1e-4 * solutiondiff ) {     // first see if there is a direction
                if(( quatangle < 0) ||( quatangle*quatangle < 0.95f*0.95f*expecteddeltaquat_len*jdeltaquat_len) ) {
                    //RAVELOG_INFO("rejected rotation: %e < %e\n",quatangle,RaveSqrt(_fMaxCosDeviationAngle*_fMaxCosDeviationAngle*expecteddeltaquat.lengthsqr3()*jdeltaquat.lengthsqr3()));
                    return IKRA_Reject;
                }
            }
        }
        else {
            // should be very close to _vprevsolution
            for(size_t i = 0; i < _vprevsolution.size(); ++i) {
                if( RaveFabs(_vprevsolution[i]-vsolution.at(i)) > 0.1f ) {
                    return IKRA_Reject;
                }
            }
        }

        if( _vprevsolution.size() > 0 ) {
            // take the midpoint of the solutions and ikparameterization and see if they are close
            std::vector<dReal> vmidsolution(vsolution.size());
            for(size_t i = 0; i < vsolution.size(); ++i) {
                vmidsolution[i] = 0.5*(vsolution[i]+_vprevsolution[i]);
            }
            //RobotBase::RobotStateSaver savestate(_robot);
            _robot->SetActiveDOFs(pmanip->GetArmIndices());
            _robot->SetActiveDOFValues(vmidsolution);
            IkParameterization ikmidreal = pmanip->GetIkParameterization(ikp.GetType(),false);

            IkParameterization ikmidest;
            ikmidest.SetTransform6D(Transform(quatSlerp(_ikprev.GetTransform6D().rot, ikp.GetTransform6D().rot,dReal(0.5)), 0.5*(_ikprev.GetTransform6D().trans+ikp.GetTransform6D().trans)));
            const dReal ikmidpointmaxdist2mult = 0.25;
            dReal middist2 = ikmidreal.ComputeDistanceSqr(ikmidest);
            dReal realdist2 = ikp.ComputeDistanceSqr(_ikprev);
            // note that ikp might be a little off from vsolution due to the ik solver!
            // realdist2 should also be great or otherwise we could be picking up noise in the subtraction
            if( realdist2 > g_fEpsilon && middist2 > g_fEpsilonWorkSpaceLimitSqr && middist2 > ikmidpointmaxdist2mult*realdist2 ) {
                RAVELOG_VERBOSE(str(boost::format("rejected due to discontinuity at mid-point %e > %e")%middist2%(ikmidpointmaxdist2mult*realdist2)));
                return IKRA_Reject;
            }
        }

        if( _filteroptions & IKFO_CheckEnvCollisions ) {
            // check rest of environment collisions
            FOREACH(it,_vchildlinks) {
                (*it)->Enable(true);
            }
            if( !_parameters->_checkpathconstraintsfn((_vprevsolution.size() > 0) ? _vprevsolution : vsolution, vsolution,IT_Open,ConfigurationListPtr()) ) {
                return IKRA_Reject;
            }
            FOREACH(it,_vchildlinks) {
                (*it)->Enable(false);
            }
        }
        return IKRA_Success;
    }

    RobotBasePtr _robot;
    RobotBase::ManipulatorPtr _manip;
    CollisionReportPtr _report;
    boost::shared_ptr<WorkspaceTrajectoryParameters> _parameters;
    dReal _fMaxCosDeviationAngle;
    int _filteroptions;
    vector<KinBody::LinkPtr> _vchildlinks;

    // planning state
    Transform _tbaseinv;
    boost::multi_array<dReal,2> _mjacobian, _mquatjacobian;
    IkParameterization _ikprev;
    vector<dReal> _vprevsolution;
    PlannerBasePtr _retimerplanner;
};

PlannerBasePtr CreateWorkspaceTrajectoryTracker(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new WorkspaceTrajectoryTracker(penv, sinput));
}
