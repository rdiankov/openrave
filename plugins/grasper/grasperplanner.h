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
    enum CollisionType
    {
        CT_None = 0,
        CT_AvoidLinkHit = 1,
        CT_RegularCollision = 2,
        CT_SelfCollision = 4,
    };

public:
 GrasperPlanner(EnvironmentBasePtr penv) : PlannerBase(penv), _report(new COLLISIONREPORT()), _parameters(penv) {}
    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = pbase;
        _parameters.copy(pparams);

        if( _parameters.btightgrasp )
            RAVELOG_WARNA("tight grasping not supported yet\n");

        _vAvoidLinkGeometry.resize(0);
        FOREACH(itavoid,_parameters.vavoidlinkgeometry) {
            KinBody::LinkPtr plink = _robot->GetLink(*itavoid);
            if( !plink ) {
                RAVELOG_WARNA(str(boost::format("failed to find avoiding link\n")%*itavoid));
                continue;
            }
            _vAvoidLinkGeometry.push_back(plink);
        }

        _bInit = true;
        return true;
    }

    bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream = boost::shared_ptr<std::ostream>())
    {
        if(!_bInit)
            return false;

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        RobotBase::RobotStateSaver saver(_robot);

        RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();

        // do not clear the trajectory because the user might want to append the grasp point to it
        if( ptraj->GetDOF() != _robot->GetActiveDOF() )
            ptraj->Reset(_robot->GetActiveDOF());

        CollisionCheckerMngr checkermngr(GetEnv(),"");
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);

        std::vector<KinBody::LinkPtr> vlinks;
        KinBody::LinkPtr pbase;
        if( !!pmanip ) {
            // disable all links not children to the manipulator
            pbase = pmanip->GetBase();
            pmanip->GetChildLinks(vlinks);
            FOREACHC(itlink,_robot->GetLinks()) {
                if( std::find(vlinks.begin(),vlinks.end(),*itlink) == vlinks.end() )
                    (*itlink)->Enable(false);
            }
        }
        else {
            vlinks = _robot->GetLinks();
            pbase = vlinks.front();
        }

        if( (int)_parameters.vinitialconfig.size() == _robot->GetActiveDOF() )
            _robot->SetActiveDOFValues(_parameters.vinitialconfig);

        Transform tbase = pbase->GetTransform(), trobot = _robot->GetTransform();
        Transform ttorobot = tbase.inverse() * trobot;
        Vector vapproachdir;

        if( !_parameters.targetbody )
            vapproachdir = _parameters.vtargetdirection;
        else
            vapproachdir = _parameters.targetbody->GetTransform().rotate(_parameters.vtargetdirection);

        if( _parameters.btransformrobot ) {
            if( !!pmanip ) {
                tbase.rotfromaxisangle(pmanip->GetPalmDirection(),_parameters.ftargetroll);

                // set the robot so that its palm is facing the approach direction find the closest rotation
                Vector rottodirection;
                cross3(rottodirection, pmanip->GetPalmDirection(), _parameters.vtargetdirection);
                dReal fsin = RaveSqrt(rottodirection.lengthsqr3());
                dReal fcos = dot3(pmanip->GetPalmDirection(), _parameters.vtargetdirection);

                Transform torient;
                if( fsin > 1e-6f ) {
                    torient.rotfromaxisangle(rottodirection*(1/fsin), RaveAtan2(fsin, fcos));
                }
                else if( fcos < 0 ) {
                    // hand is flipped 180, rotate around x axis
                    rottodirection = Vector(1,0,0);
                    rottodirection -= pmanip->GetPalmDirection() * dot3(pmanip->GetPalmDirection(), rottodirection);
                    rottodirection.normalize3();
                    torient.rotfromaxisangle(rottodirection, RaveAtan2(fsin, fcos));
                }

                tbase = torient * tbase;

                // make sure origin of pbase is on target position
                tbase.trans = _parameters.vtargetposition;

                // transform into base
                tbase = tbase * pmanip->GetEndEffectorTransform().inverse() * pbase->GetTransform();
            }
            else                
                tbase.trans = _parameters.vtargetposition;
            if( !!_parameters.targetbody )
                tbase = _parameters.targetbody->GetTransform() * tbase;

            trobot = tbase*ttorobot;
            _robot->SetTransform(trobot);

            dReal step_size = 0.05f;
            //backup the robot until it is no longer colliding with the object
            if( !_parameters.targetbody ) {
                while(1) {
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot)))
                        break;
                    trobot.trans -= vapproachdir * step_size;
                    _robot->SetTransform(trobot);
                }
            }
            else {
                while(1) {
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(_parameters.targetbody)))
                        break;
                    trobot.trans -= vapproachdir * step_size;
                    _robot->SetTransform(trobot);
                }
            }
        }
    
        int ncollided = 0; ///number of links that have collided with an obstacle

        std::vector<dReal> dofvals;
        _robot->GetActiveDOFValues(dofvals);

        // only reset when traj dof != active dof
        if( ptraj->GetDOF() != _robot->GetActiveDOF() )
            ptraj->Reset(_robot->GetActiveDOF());
        Trajectory::TPOINT ptemp;    
        ptemp.trans = _robot->GetTransform();
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
                    int ct = CheckCollision(vlinks[q]);
                    if( ct & CT_AvoidLinkHit )
                        return false;
                    if( ct != CT_None ){ 
                        if(coarse_pass) {
                            //coarse step collided, back up and shrink step
                            if(bMoved) {
                                coarse_pass = false;

                                if( pX != NULL )  *pX -= step_size*vapproachdir.x;
                                if( pY != NULL )  *pY -= step_size*vapproachdir.y;
                                if( pZ != NULL )  *pZ -= step_size*vapproachdir.z;
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
                            if(_parameters.fstandoff == 0)
                                ncollided++;
                       
                            collision = true;
                            //move hand back by standoff
                        
                            if( pX != NULL )  *pX -= _parameters.fstandoff*vapproachdir.x;
                            if( pY != NULL )  *pY -= _parameters.fstandoff*vapproachdir.y;
                            if( pZ != NULL )  *pZ -= _parameters.fstandoff*vapproachdir.z;

                            break;
                        }
                    }
                }

                for(int i = 0; i < _robot->GetActiveDOF(); i++)
                    ptemp.q[i] = dofvals[i];

                if(_parameters.breturntrajectory) {
                    ptemp.trans = _robot->GetTransform();
                    ptraj->AddPoint(ptemp); 
                }

                if(collision)
                    break;

                if( pX != NULL )  *pX += step_size*vapproachdir.x;
                if( pY != NULL )  *pY += step_size*vapproachdir.y;
                if( pZ != NULL )  *pZ += step_size*vapproachdir.z;

                _robot->SetActiveDOFValues(dofvals);
                bMoved = true;
            }
        }
    
        std::vector<dReal> vlowerlim, vupperlim;
        _robot->GetActiveDOFLimits(vlowerlim,vupperlim);
        vector<dReal> vclosingdir(_robot->GetActiveDOF(),1);

        if( (int)_parameters.vgoalconfig.size() == _robot->GetActiveDOF() ) {
            vclosingdir = _parameters.vgoalconfig;
        }
        else {
            // get closing direction from manipulators
            for(size_t i = 0; i < _robot->GetActiveJointIndices().size(); ++i) {
                FOREACHC(itmanip, _robot->GetManipulators()) {
                    vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
                    FOREACHC(itgripper,(*itmanip)->GetGripperJoints()) {
                        if( *itgripper == _robot->GetActiveJointIndices().at(i) ) {
                            vclosingdir[i] = *itclosing;
                            break;
                        }
                        itclosing++;
                    }
                }
            }
        }

        ptemp.trans = _robot->GetTransform();

        //close the fingers one by one
        dReal fmult;
        for(int ifing = 0; ifing < _robot->GetActiveDOF(); ifing++) {
            int nJointIndex = _robot->GetActiveJointIndex(ifing);
            if( vclosingdir[ifing] == 0 || nJointIndex < 0 )
                // not a real joint, so skip
                continue;

            switch(_robot->GetJoints().at(nJointIndex)->GetType()) {
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

                if( (vclosingdir[ifing] > 0 && dofvals[ifing] >  vupperlim[ifing]) || (vclosingdir[ifing] < 0 && dofvals[ifing] < vlowerlim[ifing]) ) {
                    break;

                }
                _robot->SetActiveDOFValues(dofvals);
            
                for(int q = 0; q < (int)vlinks.size(); q++) {
                    int ct;
                    if(_robot->DoesAffect(_robot->GetActiveJointIndex(ifing),vlinks[q]->GetIndex())  && (ct = CheckCollision(vlinks[q])) != CT_None ) {
                        if( ct & CT_AvoidLinkHit )
                            return false;
                        if(coarse_pass) {
                            //coarse step collided, back up and shrink step
                            coarse_pass = false;
                            //if it didn't start in collision, move back one step before switching to smaller step size
                            if(bMoved) {
                                dofvals[ifing] -= vclosingdir[ifing] * step_size;
                                UpdateDependents(ifing,dofvals);
                                _robot->SetActiveDOFValues(dofvals);
                                step_size = FINE_STEP*fmult;
                                num_iters = (int)(COARSE_STEP/FINE_STEP)+1;
                            }
                            else {
                                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                    RAVELOG_VERBOSEA(str(boost::format("Collision of link %s using joint %d\n")%_robot->GetLinks().at(q)->GetName()%_robot->GetActiveJointIndex(ifing)));
                                    stringstream ss; ss << "Joint Vals: ";
                                    for(int vi = 0; vi < _robot->GetActiveDOF();vi++)
                                        ss << dofvals[vi] << " ";
                                    ss << endl;
                                    RAVELOG_VERBOSEA(ss.str());
                                }
                                //vbcollidedlinks[q] = true;
                                //vijointresponsible[q] = _robot->GetActiveJointIndex(ifing);
                                if( ct & CT_SelfCollision ) {
                                    // don't want the robot to end up in self collision, so back up
                                    dofvals[ifing] -= vclosingdir[ifing] * step_size;
                                }

                                ncollided++;
                                collision = true;
                                break;
                            }
                        }
                        else {
                            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                RAVELOG_VERBOSEA(str(boost::format("Collision of link %s using joint %d\n")%_robot->GetLinks().at(q)->GetName()%_robot->GetActiveJointIndex(ifing)));
                                stringstream ss; ss << "Joint Vals: ";
                                for(int vi = 0; vi < _robot->GetActiveDOF();vi++)
                                    ss << dofvals[vi] << " ";
                                ss << endl;
                                RAVELOG_VERBOSEA(ss.str());
                            }
                     
                            if( ct & CT_SelfCollision ) {
                                // don't want the robot to end up in self collision, so back up
                                dofvals[ifing] -= vclosingdir[ifing] * step_size;
                            }
                            ncollided++;
                            collision = true;
                            break;
                        }
                    }
                }

                for(int j = 0; j < _robot->GetActiveDOF(); j++)
                    ptemp.q[j] = dofvals[j];

                if(_parameters.breturntrajectory)
                    ptraj->AddPoint(ptemp); 

                if(collision)
                    break;

                dofvals[ifing] += vclosingdir[ifing] * step_size;
                bMoved = true;
            }

            if(ncollided == (int)vlinks.size())
                break;
        }
    
        if(!_parameters.breturntrajectory)
            ptraj->AddPoint(ptemp);

        return true;
    }

    virtual int CheckCollision(KinBody::LinkConstPtr plink)
    {
        int ct = 0;
        if( GetEnv()->CheckCollision(plink,_report) ) {
            ct |= CT_RegularCollision;
            FOREACH(itavoid,_vAvoidLinkGeometry) {
                if( *itavoid == _report->plink1 || *itavoid == _report->plink2 ) {
                    ct |= CT_AvoidLinkHit;
                    break;
                }
            }

            if( _parameters.bonlycontacttarget ) {
                // check if hit anything besides the target
                if( (!!_report->plink1 && _report->plink1->GetParent() != plink->GetParent() && _report->plink1->GetParent() != _parameters.targetbody) ||
                    (!!_report->plink2 && _report->plink2->GetParent() != plink->GetParent() && _report->plink2->GetParent() != _parameters.targetbody) ) {
                    ct |= CT_AvoidLinkHit;
                }
            }

            return ct;
        }

        if(_robot->CheckSelfCollision())
            ct |= CT_SelfCollision;
        return ct;
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
    CollisionReportPtr _report;
    GraspParameters _parameters;
    RobotBasePtr _robot;
    vector<KinBody::LinkPtr> _vAvoidLinkGeometry;
    bool _bInit;
};

#endif
