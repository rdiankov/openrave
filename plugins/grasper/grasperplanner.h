// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef  RAVE_GRASPER_PLANNER_H
#define  RAVE_GRASPER_PLANNER_H

#define COARSE_STEP 0.1f  ///step for coarse planning
#define FINE_STEP (COARSE_STEP/(100.0f)) ///step for fine planning, THIS STEP MUST BE VERY SMALL [COARSE_STEP/(100.0f)] OR THE COLLISION CHECKER GIVES WILDLY BOGUS RESULTS
#define TRANSLATION_LIMIT ((int)(10.0f/step_size)) ///how far to translate before giving up

// plans a grasp
class GrasperPlanner:  public PlannerBase
{
public:
    GrasperPlanner(EnvironmentBasePtr penv) : PlannerBase(penv) {}
    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        _robot = pbase;
        _parameters.copy(pparams);
        _bInit = true;
        return true;
    }

    bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream = boost::shared_ptr<std::ostream>())
    {
        if(!_bInit)
            return false;

        // do not clear the trajectory because the user might want to append the grasp point to it
        if( ptraj->GetDOF() != _robot->GetActiveDOF() )
            ptraj->Reset(_robot->GetActiveDOF());

        CollisionCheckerMngr checkermngr(GetEnv(),"");
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
        Vector Vdirection = _parameters.direction;
        Vector Vnormalplane = _parameters.palmnormal;

        std::vector<Transform> vtrans;
        _robot->GetBodyTransformations(vtrans);

        //roll hand
        Transform rollhand;
        rollhand.rot.x = cos(_parameters.roll_hand/2);
        rollhand.rot.y = Vnormalplane.x*sin(_parameters.roll_hand/2);
        rollhand.rot.z = Vnormalplane.y*sin(_parameters.roll_hand/2);
        rollhand.rot.w = Vnormalplane.z*sin(_parameters.roll_hand/2);

        //rotate hand to face object
        Transform rothand;
        if(_parameters.face_target) {
            //get body transfroms (we only need the base one, which is index 0)

            //_robot->GetBodyTransformations(vtrans);
            RAVELOG_VERBOSEA("Direction: %f %f %f %f\n", Vdirection.w, Vdirection.x, Vdirection.y, Vdirection.z);
            RAVELOG_VERBOSEA("Before Transform: %f %f %f %f\n", Vnormalplane.w, Vnormalplane.x, Vnormalplane.y, Vnormalplane.z);        
            //don't need to divide by magnitude b/c they are unit vecs
            dReal rottheta = RaveAcos(dot3(Vdirection,Vnormalplane));
            Vnormalplane.Cross(Vdirection);
            normalize4(Vnormalplane, Vnormalplane);
            Vector rotaxis = Vnormalplane;
        
            RAVELOG_DEBUGA("RotAxis: %f %f %f \n",rotaxis.x,rotaxis.y,rotaxis.z);

            rothand.rot.x = cos(rottheta/2);
            rothand.rot.y = rotaxis.x*sin(rottheta/2);
            rothand.rot.z = rotaxis.y*sin(rottheta/2);
            rothand.rot.w = rotaxis.z*sin(rottheta/2);
        }

        _robot->SetTransform(vtrans[0]*rothand*rollhand);
    
        int ncollided = 0; ///number of links that have collided with an obstacle

        std::vector<dReal> dofvals = _parameters.vinitialconfig;

        const std::vector<KinBody::LinkPtr>& vlinks = _robot->GetLinks();

        // only reset when traj dof != active dof
        if( ptraj->GetDOF() != _robot->GetActiveDOF() )
            ptraj->Reset(_robot->GetActiveDOF());
        Trajectory::TPOINT ptemp;    
        ptemp.q.resize(_robot->GetActiveDOF());
        ptemp.qdot.resize(_robot->GetActiveDOF());

        for(int i = 0; i < _robot->GetActiveDOF(); i++)
            UpdateDependents(i,dofvals);

        //move hand toward object until something collides
        dReal step_size = 0.25f*COARSE_STEP;
        bool collision = false;
    
        bool coarse_pass = true; ///this parameter controls the coarseness of the step
    
        int num_iters = TRANSLATION_LIMIT;
        bool bMoved = false;

        dReal* pX = NULL, *pY = NULL, *pZ = NULL;

        if( _robot->GetAffineDOF() ) {
            // if using affine dofs, only get the translations
            dReal* p = &dofvals[0] + _robot->GetActiveDOF();
            if( _robot->GetAffineDOF() & RobotBase::DOF_Rotation3D ) p -= 3;
            else if( _robot->GetAffineDOF() & RobotBase::DOF_RotationAxis ) p -= 1;

            if( _robot->GetAffineDOF() & RobotBase::DOF_Z ) pZ = --p;
            if( _robot->GetAffineDOF() & RobotBase::DOF_Y ) pY = --p;
            if( _robot->GetAffineDOF() & RobotBase::DOF_X ) pX = --p;
    
            while(num_iters-- > 0) {
                for(int q = 0; q < (int)vlinks.size(); q++) {
                    if(GetEnv()->CheckCollision(KinBody::LinkConstPtr(vlinks[q]))) {   
                        if(coarse_pass) {
                            //coarse step collided, back up and shrink step
                            if(bMoved) {
                                coarse_pass = false;

                                if( pX != NULL )  *pX -= step_size*Vdirection.x;
                                if( pY != NULL )  *pY -= step_size*Vdirection.y;
                                if( pZ != NULL )  *pZ -= step_size*Vdirection.z;
                                step_size = 0.25f*FINE_STEP;
                                num_iters = (int)(COARSE_STEP/FINE_STEP)+1;
                                _robot->SetActiveDOFValues(dofvals);
                                break;
                            }
                            else {
                                collision = true;
                                break;
                            }
                        }
                        else {
                            if(_parameters.stand_off == 0.0f)
                                ncollided++;
                       
                            collision = true;
                            //move hand back by standoff
                        
                            if( pX != NULL )  *pX -= _parameters.stand_off*Vdirection.x;
                            if( pY != NULL )  *pY -= _parameters.stand_off*Vdirection.y;
                            if( pZ != NULL )  *pZ -= _parameters.stand_off*Vdirection.z;

                            break;
                        }
                    }
                }

                for(int i = 0; i < _robot->GetActiveDOF(); i++)
                    ptemp.q[i] = dofvals[i];

                if(_parameters.bReturnTrajectory) {
                    ptraj->AddPoint(ptemp); 
                }

                if(collision)
                    break;

                if( pX != NULL )  *pX += step_size*Vdirection.x;
                if( pY != NULL )  *pY += step_size*Vdirection.y;
                if( pZ != NULL )  *pZ += step_size*Vdirection.z;

                _robot->SetActiveDOFValues(dofvals);
                bMoved = true;
            }
        }
    
        std::vector<dReal> vlowerlim, vupperlim;
        _robot->GetActiveDOFLimits(vlowerlim,vupperlim);
        vector<dReal> vclosingsign(_robot->GetActiveDOF(),1);

        if( (int)_parameters.vgoalconfig.size() == _robot->GetActiveDOF() ) {
            vclosingsign = _parameters.vgoalconfig;
        }
        else {
            // get closing direction from manipulators
            for(size_t i = 0; i < _robot->GetActiveJointIndices().size(); ++i) {
                FOREACHC(itmanip, _robot->GetManipulators()) {
                    vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
                    FOREACHC(itgripper,(*itmanip)->GetGripperJoints()) {
                        if( *itgripper == _robot->GetActiveJointIndices().at(i) ) {
                            vclosingsign[i] = *itclosing;
                            break;
                        }
                        itclosing++;
                    }
                }
            }
        }

        //close the fingers one by one
        dReal fmult;
        for(int ifing = 0; ifing < _robot->GetActiveDOF(); ifing++) {
            int nJointIndex = _robot->GetActiveJointIndex(ifing);
            if( nJointIndex < 0 )
                // not a real joint, so skip
                continue;

            switch(_robot->GetJoints()[nJointIndex]->GetType()) {
            case KinBody::Joint::JointSlider:
                fmult = 0.05f;
                break;
            default:
                fmult = 1.0f;
            }
        
            step_size = COARSE_STEP*fmult;

            coarse_pass = true;
            bool collision = false;

            num_iters = (int)((vupperlim[ifing] - vlowerlim[ifing])/step_size);
            bMoved = false;
            while(num_iters-- > 0) {
                // set manip joints that haven't been covered so far
                UpdateDependents(ifing,dofvals);

                if( (vclosingsign[ifing] > 0 && dofvals[ifing] >  vupperlim[ifing]) || (vclosingsign[ifing] < 0 && dofvals[ifing] < vlowerlim[ifing]) ) {
                    break;

                }
                _robot->SetActiveDOFValues(dofvals);
            
                for(int q = 0; q < (int)vlinks.size(); q++)
                    {
                        bool bSelfCollision=false;
                        if(_robot->DoesAffect(_robot->GetActiveJointIndex(ifing),q)  && (GetEnv()->CheckCollision(KinBody::LinkConstPtr(vlinks[q]))||(bSelfCollision=_robot->CheckSelfCollision())) ) {
                            if(coarse_pass) {
                                //coarse step collided, back up and shrink step
                                coarse_pass = false;
                                //if it didn't start in collision, move back one step before switching to smaller step size
                                if(bMoved) {
                                    dofvals[ifing] -= vclosingsign[ifing] * step_size;
                                    UpdateDependents(ifing,dofvals);
                                    _robot->SetActiveDOFValues(dofvals);
                                    step_size = FINE_STEP*fmult;
                                    num_iters = (int)(COARSE_STEP/FINE_STEP)+1;
                                }
                                else {
                                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                        RAVELOG_VERBOSEA(str(boost::format("Collision of link %s using joint %d\n")%_robot->GetLinks()[q]->GetName()%_robot->GetActiveJointIndex(ifing)));
                                        stringstream ss; ss << "Joint Vals: ";
                                        for(int vi = 0; vi < _robot->GetActiveDOF();vi++)
                                            ss << dofvals[vi] << " ";
                                        ss << endl;
                                        RAVELOG_VERBOSEA(ss.str());
                                    }
                                    //vbcollidedlinks[q] = true;
                                    //vijointresponsible[q] = _robot->GetActiveJointIndex(ifing);
                                    if( bSelfCollision ) {
                                        // don't want the robot to end up in self collision, so back up
                                        dofvals[ifing] -= vclosingsign[ifing] * step_size;
                                    }

                                    ncollided++;
                                    collision = true;
                                    break;
                                }
                            }
                            else {
                                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                    RAVELOG_VERBOSEA(str(boost::format("Collision of link %s using joint %d\n")%_robot->GetLinks()[q]->GetName()%_robot->GetActiveJointIndex(ifing)));
                                    stringstream ss; ss << "Joint Vals: ";
                                    for(int vi = 0; vi < _robot->GetActiveDOF();vi++)
                                        ss << dofvals[vi];
                                    ss << endl;
                                    RAVELOG_VERBOSEA(ss.str());
                                }
                     
                                if( bSelfCollision ) {
                                    // don't want the robot to end up in self collision, so back up
                                    dofvals[ifing] -= vclosingsign[ifing] * step_size;
                                }
                                ncollided++;
                                collision = true;
                                break;
                            }
                        }
                    }

                for(int j = 0; j < _robot->GetActiveDOF(); j++)
                    ptemp.q[j] = dofvals[j];

                if(_parameters.bReturnTrajectory)
                    ptraj->AddPoint(ptemp); 

                if(collision)
                    break;

                dofvals[ifing] += vclosingsign[ifing] * step_size;
                bMoved = true;
            }

            if(ncollided == (int)vlinks.size())
                break;
        }
    
        if(!_parameters.bReturnTrajectory)
            ptraj->AddPoint(ptemp);

        return true;
    }

    virtual RobotBasePtr GetRobot() {return _robot; }

    void UpdateDependents(int ifing, vector<dReal>& dofvals)
    {
        for(int c = ifing; c < _robot->GetActiveDOF(); ++c ) {
            int index = _robot->GetActiveJointIndex(c);
            if( index < 0 )
                // not a real joint, so skip
                continue;
            KinBody::JointPtr pmimicjoint = _robot->GetJoints().at(index);
            //check that it's the right doff val
            if( pmimicjoint->GetMimicJointIndex() == _robot->GetActiveJointIndex(ifing)) {
                // set accordingly
                dofvals[c] = pmimicjoint->GetMimicCoeffs()[0]*dofvals[ifing] + pmimicjoint->GetMimicCoeffs()[1];
            }
        }

    }

protected:
    GraspParameters _parameters;
    RobotBasePtr _robot;
    bool _bInit;

};

#endif
