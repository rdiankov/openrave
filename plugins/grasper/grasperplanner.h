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
 GrasperPlanner(EnvironmentBasePtr penv) : PlannerBase(penv), _report(new COLLISIONREPORT()) {}
    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = pbase;
        _parameters.reset(new GraspParameters(GetEnv()));
        _parameters->copy(pparams);

        if( _parameters->btightgrasp )
            RAVELOG_WARNA("tight grasping not supported yet\n");

        _vAvoidLinkGeometry.resize(0);
        FOREACH(itavoid,_parameters->vavoidlinkgeometry) {
            KinBody::LinkPtr plink = _robot->GetLink(*itavoid);
            if( !plink ) {
                RAVELOG_WARNA(str(boost::format("failed to find avoiding link\n")%*itavoid));
                continue;
            }
            _vAvoidLinkGeometry.push_back(plink);
        }

        return true;
    }

    bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream = boost::shared_ptr<std::ostream>())
    {
        if(!_parameters)
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
            pbase = vlinks.at(0);
        }

        if( (int)_parameters->vinitialconfig.size() == _robot->GetActiveDOF() )
            _robot->SetActiveDOFValues(_parameters->vinitialconfig,true);

        Transform tbase = pbase->GetTransform(), trobot = _robot->GetTransform();
        Transform ttorobot = tbase.inverse() * trobot;
        Vector vapproachdir;
        Transform tTarget, tTargetOffset;
        if( !!_parameters->targetbody ) {
            tTarget = _parameters->targetbody->GetTransform();
            if( _parameters->fgraspingnoise > 0 ) {
                dReal frotratio = RaveRandomFloat(); // ratio due to rotation
                Vector vrandtrans = _parameters->fgraspingnoise*(1-frotratio)*Vector(2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f);
                Vector vrandaxis;
                while(1) {
                    vrandaxis = Vector(2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f);
                    if( vrandaxis.lengthsqr3() > 0 && vrandaxis.lengthsqr3() <= 1 )
                        break;
                }
                
                // find furthest point from origin of body and rotate around center
                AABB ab = _parameters->targetbody->ComputeAABB();
                dReal fmaxradius = RaveSqrt(ab.extents.lengthsqr3());
                tTargetOffset.rotfromaxisangle(vrandaxis.normalize3(),RaveRandomFloat()*_parameters->fgraspingnoise*frotratio*fmaxradius);
                tTargetOffset.trans = tTargetOffset.rotate(-ab.pos)+ab.pos+vrandtrans;
            }
        }

        vapproachdir = (tTargetOffset*tTarget).rotate(_parameters->vtargetdirection);

        if( _parameters->btransformrobot ) {
            _robot->SetTransform(Transform()); // this is necessary to reset any 'randomness' introduced from the current state

            if( !!pmanip ) {
                tbase.rotfromaxisangle(pmanip->GetPalmDirection(),_parameters->ftargetroll);

                // set the robot so that its palm is facing the approach direction find the closest rotation
                Transform torient; torient.rot = quatRotateDirection(pmanip->GetPalmDirection(), _parameters->vtargetdirection);
                tbase = torient * tbase;
                // make sure origin of pbase is on target position
                tbase.trans = _parameters->vtargetposition;

                // transform into base
                tbase = tbase * pmanip->GetEndEffectorTransform().inverse() * pbase->GetTransform();
            }
            else {
                RAVELOG_DEBUGA("no active manipulator for robot, cannot get palm direction\n");
                tbase.trans = _parameters->vtargetposition;
            }

            tbase = tTargetOffset * tTarget * tbase;
            trobot = tbase*ttorobot;
            _robot->SetTransform(trobot);

            dReal step_size = 0.05f;
            //backup the robot until it is no longer colliding with the object
            if( !_parameters->targetbody ) {
                while(1) {
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot)))
                        break;
                    trobot.trans -= vapproachdir * step_size;
                    _robot->SetTransform(trobot);
                }
            }
            else {
                while(1) {
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(_parameters->targetbody)))
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

        Vector vTargetCenter;
        dReal fTargetRadius;
        if( !_parameters->targetbody ) {
            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            Vector vmin, vmax;
            bool bInitialized = false;
            FOREACH(itbody,vbodies) {
                if( *itbody == _robot )
                    continue;
                AABB ab = (*itbody)->ComputeAABB();
                if( !bInitialized ) {
                    vmin = ab.pos-ab.extents;
                    vmax = ab.pos+ab.extents;
                    bInitialized = true;
                    continue;
                }

                Vector vmin2 = ab.pos-ab.extents, vmax2 = ab.pos+ab.extents;
                if( vmin.x > vmin2.x ) vmin.x = vmin2.x;
                if( vmin.y > vmin2.y ) vmin.y = vmin2.y;
                if( vmin.z > vmin2.z ) vmin.z = vmin2.z;
                if( vmax.x < vmax2.x ) vmax.x = vmax2.x;
                if( vmax.y < vmax2.y ) vmax.y = vmax2.y;
                if( vmax.z < vmax2.z ) vmax.z = vmax2.z;
            }

            if( !bInitialized ) {
                RAVELOG_WARNA("no objects in environment\n");
                vTargetCenter = pmanip->GetEndEffectorTransform().trans;
                fTargetRadius = 0;
                //return false;
            }
            else {
                vTargetCenter = 0.5f * (vmin+vmax);
                fTargetRadius = 0.5f * RaveSqrt((vmax-vmin).lengthsqr3());
            }
        }
        else {
            AABB ab = _parameters->targetbody->ComputeAABB();
            vTargetCenter = ab.pos;
            fTargetRadius = RaveSqrt(ab.extents.lengthsqr3());
        }

        if( _robot->GetAffineDOF() ) {
            dReal* pX = NULL, *pY = NULL, *pZ = NULL;
            if( _robot->GetAffineDOF() & RobotBase::DOF_X )
                pX = &dofvals.at(_robot->GetAffineDOFIndex(RobotBase::DOF_X));
            if( _robot->GetAffineDOF() & RobotBase::DOF_Y )
                pY = &dofvals.at(_robot->GetAffineDOFIndex(RobotBase::DOF_Y));
            if( _robot->GetAffineDOF() & RobotBase::DOF_Z )
                pZ = &dofvals.at(_robot->GetAffineDOFIndex(RobotBase::DOF_Z));
    
            Vector v = vapproachdir * (_parameters->fcoarsestep*_parameters->ftranslationstepmult);
            while(1) {
                int ct = 0;
                for(int q = 0; q < (int)vlinks.size(); q++) {
                    ct = CheckCollision(vlinks[q]);
                    if( ct&CT_RegularCollision )
                        break;
                }
                if( ct&CT_RegularCollision )
                    break;

                if(_parameters->breturntrajectory) {
                    ptemp.trans = _robot->GetTransform();
                    ptraj->AddPoint(ptemp); 
                }

                if( pX != NULL )  *pX += v.x;
                if( pY != NULL )  *pY += v.y;
                if( pZ != NULL )  *pZ += v.z;
                _robot->SetActiveDOFValues(dofvals);

                // check if robot is already past all objects
                AABB abRobot = _robot->ComputeAABB();
                if( dot3(vapproachdir, abRobot.pos-vTargetCenter) > fTargetRadius + RaveSqrt(abRobot.extents.lengthsqr3()) ) {
                    RAVELOG_DEBUG("robot did not hit anything, planner failing...\n");
                    return false;
                }
            }

            // move back and try again with a finer step
            if( pX != NULL )  *pX -= v.x;
            if( pY != NULL )  *pY -= v.y;
            if( pZ != NULL )  *pZ -= v.z;
            _robot->SetActiveDOFValues(dofvals); 

            v = vapproachdir * (_parameters->ffinestep*_parameters->ftranslationstepmult);
            while(1) {
                int ct = 0;
                for(int q = 0; q < (int)vlinks.size(); q++) {
                    ct = CheckCollision(vlinks[q]);
                    if( ct&CT_RegularCollision )
                        break;
                }
                if( ct&CT_RegularCollision )
                    break;

                if( pX != NULL )  *pX += v.x;
                if( pY != NULL )  *pY += v.y;
                if( pZ != NULL )  *pZ += v.z;
                _robot->SetActiveDOFValues(dofvals);

                if(_parameters->breturntrajectory) {
                    ptemp.trans = _robot->GetTransform();
                    ptraj->AddPoint(ptemp); 
                }
            }

            if( _parameters->fstandoff > 0 ) {
                v = -vapproachdir*_parameters->fstandoff;
                if( pX != NULL )  *pX += v.x;
                if( pY != NULL )  *pY += v.y;
                if( pZ != NULL )  *pZ += v.z;
                _robot->SetActiveDOFValues(dofvals);
                if( (!_parameters->targetbody && GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) || 
                    (!!_parameters->targetbody && GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(_parameters->targetbody))) ) {
                    // in collision, so move back
                    if( pX != NULL )  *pX -= v.x;
                    if( pY != NULL )  *pY -= v.y;
                    if( pZ != NULL )  *pZ -= v.z;
                    _robot->SetActiveDOFValues(dofvals);
                }
            }

            // check that anything that should be avoided is not hit
            int ct = 0;
            for(int q = 0; q < (int)vlinks.size(); q++) {
                ct = CheckCollision(vlinks[q]);
                if( ct&CT_AvoidLinkHit ) {
                    RAVELOG_VERBOSEA(str(boost::format("hit link that needed to be avoided %s\n")%vlinks.at(q)->GetName()));
                    return false;
                }
            }
        }
    
        std::vector<dReal> vlowerlim, vupperlim;
        _robot->GetActiveDOFLimits(vlowerlim,vupperlim);
        vector<dReal> vclosingdir(_robot->GetActiveDOF(),0);
        if( (int)_parameters->vgoalconfig.size() == _robot->GetActiveDOF() ) {
            vclosingdir = _parameters->vgoalconfig;
        }
        else {
            // get closing direction from manipulators
            for(size_t i = 0; i < _robot->GetActiveJointIndices().size(); ++i) {
                FOREACHC(itmanip, _robot->GetManipulators()) {
                    vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
                    FOREACHC(itgripper,(*itmanip)->GetGripperJoints()) {
                        if( *itclosing != 0 && *itgripper == _robot->GetActiveJointIndices().at(i) ) {
                            vclosingdir.at(i) = *itclosing;
                            break;
                        }
                        itclosing++;
                    }
                }
            }
        }

        ptemp.trans = _robot->GetTransform();

        //close the fingers one by one
        for(int ifing = 0; ifing < _robot->GetActiveDOF(); ifing++) {
            int nJointIndex = _robot->GetActiveJointIndex(ifing);
            if( vclosingdir[ifing] == 0 || nJointIndex < 0 )
                // not a real joint, so skip
                continue;

            dReal fmult = 1;
            if(_robot->GetJoints().at(nJointIndex)->GetType() == KinBody::Joint::JointSlider )
                fmult = _parameters->ftranslationstepmult;
            dReal step_size = _parameters->fcoarsestep*fmult;

            bool collision = false;
            bool coarse_pass = true; ///this parameter controls the coarseness of the step    
            int num_iters = (int)((vupperlim[ifing] - vlowerlim[ifing])/step_size)+1;
            bool bMoved = false;
            while(num_iters-- > 0) {
                // set manip joints that haven't been covered so far
                UpdateDependents(ifing,dofvals);

                if( (vclosingdir[ifing] > 0 && dofvals[ifing] >  vupperlim[ifing]) || (vclosingdir[ifing] < 0 && dofvals[ifing] < vlowerlim[ifing]) ) {
                    break;
                }
                _robot->SetActiveDOFValues(dofvals,true);
                _robot->GetActiveDOFValues(dofvals);
            
                for(int q = 0; q < (int)vlinks.size(); q++) {
                    int ct;
                    if(_robot->DoesAffect(_robot->GetActiveJointIndex(ifing),vlinks[q]->GetIndex())  && (ct = CheckCollision(vlinks[q])) != CT_None ) {
                        if( !coarse_pass && (ct & CT_AvoidLinkHit) ) {
                            RAVELOG_VERBOSEA(str(boost::format("hit link that needed to be avoided: %s\n")%_report->__str__()));
                            return false;
                        }
                        if(coarse_pass) {
                            //coarse step collided, back up and shrink step
                            coarse_pass = false;
                            //if it didn't start in collision, move back one step before switching to smaller step size
                            if(bMoved) {
                                dofvals[ifing] -= vclosingdir[ifing] * step_size;
                                UpdateDependents(ifing,dofvals);
                                _robot->SetActiveDOFValues(dofvals,true);
                                num_iters = (int)(step_size/(_parameters->ffinestep*fmult))+1;
                                step_size = _parameters->ffinestep*fmult;
                            }
                            else {
                                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                    RAVELOG_VERBOSEA(str(boost::format("Collision (%d) of link %s using joint %d [%s]\n")%ct%vlinks.at(q)->GetName()%_robot->GetActiveJointIndex(ifing)%_report->__str__()));
                                    stringstream ss; ss << "Transform: " << vlinks.at(q)->GetTransform() << ", Joint Vals: ";
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
                        else {
                            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                RAVELOG_VERBOSEA(str(boost::format("Collision (%d) of link %s using joint %d [%s]\n")%ct%vlinks.at(q)->GetName()%_robot->GetActiveJointIndex(ifing)%_report->__str__()));
                                stringstream ss; ss << "Transform: " << vlinks.at(q)->GetTransform() << "Joint Vals: ";
                                for(int vi = 0; vi < _robot->GetActiveDOF();vi++)
                                    ss << dofvals[vi] << " ";
                                ss << endl;
                                RAVELOG_VERBOSEA(ss.str());
                            }
                     
                            if( (ct & CT_SelfCollision) || _parameters->bavoidcontact ) {
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
                if(_parameters->breturntrajectory)
                    ptraj->AddPoint(ptemp); 

                if(collision)
                    break;

                dofvals[ifing] += vclosingdir[ifing] * step_size;
                bMoved = true;
            }
        }

        bool bAddLastPoint = true;
        for(int q = 0; q < (int)vlinks.size(); q++) {
            int ct = CheckCollision(vlinks[q]);
            if( ct & CT_AvoidLinkHit )
                return false;
            if( ct & CT_SelfCollision ) {
                bAddLastPoint = false;
            }
        }

        if( bAddLastPoint ) {
            // don't forget the final point!
            _robot->SetActiveDOFValues(dofvals,true);
            _robot->GetActiveDOFValues(dofvals);
            for(int j = 0; j < _robot->GetActiveDOF(); j++)
                ptemp.q[j] = dofvals[j];
            if(!_parameters->breturntrajectory)
                ptraj->AddPoint(ptemp);
        }

        return ptraj->GetPoints().size()>0; // only return true if there is at least one valid pose!
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

            if( _parameters->bonlycontacttarget ) {
                // check if hit anything besides the target
                if( (!!_report->plink1 && _report->plink1->GetParent() != plink->GetParent() && _report->plink1->GetParent() != _parameters->targetbody) ||
                    (!!_report->plink2 && _report->plink2->GetParent() != plink->GetParent() && _report->plink2->GetParent() != _parameters->targetbody) ) {
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

    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }
    
protected:
    CollisionReportPtr _report;
    boost::shared_ptr<GraspParameters> _parameters;
    RobotBasePtr _robot;
    vector<KinBody::LinkPtr> _vAvoidLinkGeometry;
};

#endif
