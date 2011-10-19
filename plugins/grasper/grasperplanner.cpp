// -*- coding: utf-8 -*-
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
#include "plugindefs.h"

#include <openrave/planningutils.h>

class GrasperPlanner :  public PlannerBase
{
    enum CollisionType
    {
        CT_None = 0,
        CT_AvoidLinkHit = 1,
        CT_SelfCollision = 2,
        CT_TargetCollision = 4,
        CT_EnvironmentCollision = 8,
        CT_RegularCollision = (CT_TargetCollision|CT_EnvironmentCollision),
    };

public:
    GrasperPlanner(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv), _report(new CollisionReport()) {
        __description = ":Interface Authors: Rosen Diankov, Dmitry Berenson\n\nSimple planner that performs a follow and squeeze operation of a robotic hand.";
    }
    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = pbase;
        _parameters.reset(new GraspParameters(GetEnv()));
        _parameters->copy(pparams);

        if( _parameters->btightgrasp ) {
            RAVELOG_WARN("tight grasping not supported yet\n");
        }
        _vAvoidLinkGeometry.resize(0);
        FOREACH(itavoid,_parameters->vavoidlinkgeometry) {
            KinBody::LinkPtr plink = _robot->GetLink(*itavoid);
            if( !plink ) {
                RAVELOG_WARN(str(boost::format("failed to find avoiding link\n")%*itavoid));
                continue;
            }
            _vAvoidLinkGeometry.push_back(plink);
        }

        return true;
    }

    PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        if(!_parameters) {
            return PS_Failed;
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        RobotBase::RobotStateSaver saver(_robot);
        RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();

        _trajspec._vgroups.resize(2);
        stringstream ss;
        _trajspec._vgroups[0].name = str(boost::format("affine_transform %s %d")%_robot->GetName()%DOF_Transform);
        _trajspec._vgroups[0].dof = RaveGetAffineDOF(DOF_Transform);
        _trajspec._vgroups[0].offset = 0;
        ss << "joint_values " << _robot->GetName();
        FOREACHC(it,_robot->GetActiveDOFIndices()) {
            ss << " " << *it;
        }
        _trajspec._vgroups[1].name = ss.str();
        _trajspec._vgroups[1].dof = _robot->GetActiveDOFIndices().size();
        _trajspec._vgroups[1].offset = _trajspec._vgroups[0].dof;

        {
            // need full 6D transform or otherwise data will be lost
            // do not clear the trajectory because the user might want to append the grasp point to it
            ConfigurationSpecification spec;
            if( ptraj->GetConfigurationSpecification().GetDOF() > 0 && ptraj->GetNumWaypoints() > 0 ) {
                spec = ptraj->GetConfigurationSpecification();
            }
            else {
                spec = _robot->GetActiveConfigurationSpecification();
            }
            FOREACH(itgroup,spec._vgroups) {
                if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
                    stringstream ss(itgroup->name.substr(16));
                    string robotname;
                    int affinedofs=0;
                    ss >> robotname >> affinedofs;
                    if( !!ss && robotname == _robot->GetName() && affinedofs != DOF_Transform ) {
                        stringstream snew;
                        snew << "affine_transform " << _robot->GetName() << " " << DOF_Transform;
                        itgroup->name = snew.str();
                        itgroup->dof = RaveGetAffineDOF(DOF_Transform);
                    }
                }
            }
            spec.ResetGroupOffsets();
            planningutils::ConvertTrajectorySpecification(ptraj,spec);
        }

        CollisionCheckerMngr checkermngr(GetEnv(),"");
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);

        KinBody::LinkPtr pbase;
        if( !!pmanip ) {
            // disable all links not children to the manipulator
            pbase = pmanip->GetBase();
            pmanip->GetChildLinks(_vlinks);
            FOREACHC(itlink,_robot->GetLinks()) {
                if( std::find(_vlinks.begin(),_vlinks.end(),*itlink) == _vlinks.end() ) {
                    (*itlink)->Enable(false);
                }
            }
        }
        else {
            _vlinks = _robot->GetLinks();
            pbase = _vlinks.at(0);
        }

        if( (int)_parameters->vinitialconfig.size() == _robot->GetActiveDOF() ) {
            _robot->SetActiveDOFValues(_parameters->vinitialconfig,true);
        }

        Transform tbase = pbase->GetTransform(), trobot = _robot->GetTransform();
        Transform ttorobot = tbase.inverse() * trobot;
        Vector vapproachdir;
        Transform tTarget, tTargetOffset;
        if( !!_parameters->targetbody ) {
            tTarget = _parameters->targetbody->GetTransform();
            if( _parameters->fgraspingnoise > 0 ) {
                dReal frotratio = RaveRandomFloat();     // ratio due to rotation
                Vector vrandtrans = _parameters->fgraspingnoise*(1-frotratio)*Vector(2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f);
                Vector vrandaxis;
                while(1) {
                    vrandaxis = Vector(2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f, 2.0f*RaveRandomFloat()-1.0f);
                    if(( vrandaxis.lengthsqr3() > 0) &&( vrandaxis.lengthsqr3() <= 1) )
                        break;
                }

                // find furthest point from origin of body and rotate around center
                AABB ab = _parameters->targetbody->ComputeAABB();
                dReal fmaxradius = RaveSqrt(ab.extents.lengthsqr3());
                tTargetOffset.rot = quatFromAxisAngle(vrandaxis,RaveRandomFloat()*_parameters->fgraspingnoise*frotratio*fmaxradius);
                tTargetOffset.trans = tTargetOffset.rotate(-ab.pos)+ab.pos+vrandtrans;
            }
        }

        vapproachdir = (tTargetOffset*tTarget).rotate(_parameters->vtargetdirection);

        if( _parameters->btransformrobot ) {
            _robot->SetTransform(Transform());     // this is necessary to reset any 'randomness' introduced from the current state

            if( !!pmanip ) {
                tbase.rot = quatFromAxisAngle(_parameters->vmanipulatordirection,_parameters->ftargetroll);
                tbase.trans = Vector(0,0,0);

                // set the robot so that its palm is facing the approach direction find the closest rotation
                Transform torient; torient.rot = quatRotateDirection(_parameters->vmanipulatordirection, _parameters->vtargetdirection);
                tbase = torient * tbase;
                // make sure origin of pbase is on target position
                tbase.trans = _parameters->vtargetposition;

                // transform into base
                tbase = tbase * pmanip->GetTransform().inverse() * pbase->GetTransform();
            }
            else {
                RAVELOG_DEBUG("no active manipulator for robot, cannot get palm direction\n");
                tbase.trans = _parameters->vtargetposition;
            }

            tbase = tTargetOffset * tTarget * tbase;
            trobot = tbase*ttorobot;
            _robot->SetTransform(trobot);

            //Transform tstartmanip = pmanip->GetTransform();

            dReal step_size = 0.05f;
            //backup the robot until it is no longer colliding with the object
            if( !_parameters->targetbody ) {
                while(1) {
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) {
                        break;
                    }
                    trobot.trans -= vapproachdir * step_size;
                    _robot->SetTransform(trobot);
                }
            }
            else {
                while(1) {
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(_parameters->targetbody))) {
                        break;
                    }
                    trobot.trans -= vapproachdir * step_size;
                    _robot->SetTransform(trobot);
                }
            }
        }

        int ncollided = 0;     ///number of links that have collided with an obstacle

        std::vector<dReal> dofvals;
        _robot->GetActiveDOFValues(dofvals);

        vector<dReal> trajpoint(_trajspec.GetDOF());
        RaveGetAffineDOFValuesFromTransform(trajpoint.begin(),_robot->GetTransform(),DOF_Transform);
        std::copy(dofvals.begin(),dofvals.begin()+_robot->GetActiveDOFIndices().size(),trajpoint.begin()+_trajspec._vgroups[1].offset);

        if( !_parameters->targetbody ) {
            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            Vector vmin, vmax;
            bool bInitialized = false;
            FOREACH(itbody,vbodies) {
                if( *itbody == _robot ) {
                    continue;
                }
                AABB ab = (*itbody)->ComputeAABB();
                if( !bInitialized ) {
                    vmin = ab.pos-ab.extents;
                    vmax = ab.pos+ab.extents;
                    bInitialized = true;
                    continue;
                }

                Vector vmin2 = ab.pos-ab.extents, vmax2 = ab.pos+ab.extents;
                if( vmin.x > vmin2.x ) {
                    vmin.x = vmin2.x;
                }
                if( vmin.y > vmin2.y ) {
                    vmin.y = vmin2.y;
                }
                if( vmin.z > vmin2.z ) {
                    vmin.z = vmin2.z;
                }
                if( vmax.x < vmax2.x ) {
                    vmax.x = vmax2.x;
                }
                if( vmax.y < vmax2.y ) {
                    vmax.y = vmax2.y;
                }
                if( vmax.z < vmax2.z ) {
                    vmax.z = vmax2.z;
                }
            }

            if( !bInitialized ) {
                RAVELOG_WARN("no objects in environment\n");
                _vTargetCenter = pmanip->GetTransform().trans;
                _fTargetRadius = 0;
                //return false;
            }
            else {
                _vTargetCenter = 0.5f * (vmin+vmax);
                _fTargetRadius = 0.5f * RaveSqrt((vmax-vmin).lengthsqr3());
            }
        }
        else {
            AABB ab = _parameters->targetbody->ComputeAABB();
            _vTargetCenter = ab.pos;
            _fTargetRadius = RaveSqrt(ab.extents.lengthsqr3());
        }

        if( _robot->GetAffineDOF() ) {
            int ct = _MoveStraight(ptraj, vapproachdir, dofvals, CT_RegularCollision);

            if( _parameters->fstandoff > 0 ) {
                dReal* pX = NULL, *pY = NULL, *pZ = NULL;
                if( _robot->GetAffineDOF() & DOF_X ) {
                    pX = &dofvals.at(_robot->GetAffineDOFIndex(DOF_X));
                }
                if( _robot->GetAffineDOF() & DOF_Y ) {
                    pY = &dofvals.at(_robot->GetAffineDOFIndex(DOF_Y));
                }
                if( _robot->GetAffineDOF() & DOF_Z ) {
                    pZ = &dofvals.at(_robot->GetAffineDOFIndex(DOF_Z));
                }
                dReal fstandoff = _parameters->fstandoff;
                if( ct & CT_EnvironmentCollision ) {
                    dReal oldX=0, oldY=0, oldZ=0;
                    if( !!pX ) {
                        oldX = *pX;
                    }
                    if( !!pY ) {
                        oldY = *pY;
                    }
                    if( !!pZ ) {
                        oldZ = *pZ;
                    }
                    _MoveStraight(ptraj, vapproachdir, dofvals, CT_TargetCollision);
                    // hit the environment, so have to get the real standoff
                    dReal f = 0;
                    if( !!pX ) {
                        f += (*pX-oldX)*vapproachdir.x;
                    }
                    if( !!pY ) {
                        f += (*pY-oldY)*vapproachdir.y;
                    }
                    if( !!pZ ) {
                        f += (*pZ-oldZ)*vapproachdir.z;
                    }
                    // if robot moved more than standoff, than it means gripper is in collision with environment sooner, so have to change the standoff
                    if( fstandoff < f ) {
                        fstandoff = f;
                    }
                }
                Vector v = -vapproachdir*fstandoff;
                if( pX != NULL ) {
                    *pX += v.x;
                }
                if( pY != NULL ) {
                    *pY += v.y;
                }
                if( pZ != NULL ) {
                    *pZ += v.z;
                }
                _robot->SetActiveDOFValues(dofvals);
                if( (!_parameters->targetbody && GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) || (!!_parameters->targetbody && GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(_parameters->targetbody))) ) {
                    RAVELOG_DEBUG("grasp in collision from standoff, moving back\n");
                    if( pX != NULL ) {
                        *pX -= v.x;
                    }
                    if( pY != NULL ) {
                        *pY -= v.y;
                    }
                    if( pZ != NULL ) {
                        *pZ -= v.z;
                    }
                    _robot->SetActiveDOFValues(dofvals);
                }
            }

            // check that anything that should be avoided is not hit
            ct = 0;
            for(int q = 0; q < (int)_vlinks.size(); q++) {
                ct = _CheckCollision(_vlinks[q], KinBodyPtr());
                if( ct&CT_AvoidLinkHit ) {
                    RAVELOG_VERBOSE(str(boost::format("hit link that needed to be avoided %s: %s\n")%_vlinks.at(q)->GetName()%_report->__str__()));
                    return PS_Failed;
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
            for(size_t i = 0; i < _robot->GetActiveDOFIndices().size(); ++i) {
                FOREACHC(itmanip, _robot->GetManipulators()) {
                    BOOST_ASSERT((*itmanip)->GetClosingDirection().size() == (*itmanip)->GetGripperIndices().size());
                    vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
                    FOREACHC(itgripper,(*itmanip)->GetGripperIndices()) {
                        if(( *itclosing != 0) &&( *itgripper == _robot->GetActiveDOFIndices().at(i)) ) {
                            vclosingdir.at(i) = *itclosing;
                            break;
                        }
                        itclosing++;
                    }
                }
            }
        }

        RaveGetAffineDOFValuesFromTransform(trajpoint.begin(),_robot->GetTransform(),DOF_Transform);

        //close the fingers one by one
        for(size_t ifing = 0; ifing < _robot->GetActiveDOFIndices().size(); ifing++) {
            if( vclosingdir.at(ifing) == 0 ) {
                // not a real joint, so skip
                continue;
            }
            int nDOFIndex = _robot->GetActiveDOFIndices()[ifing];
            dReal fmult = 1;
            KinBody::JointPtr pjoint = _robot->GetJointFromDOFIndex(nDOFIndex);
            if( pjoint->IsPrismatic(nDOFIndex-pjoint->GetDOFIndex()) ) {
                fmult = _parameters->ftranslationstepmult;
            }
            dReal step_size = _parameters->fcoarsestep*fmult;

            bool collision = false;
            bool coarse_pass = true;     ///this parameter controls the coarseness of the step
            int num_iters = (int)((vupperlim[ifing] - vlowerlim[ifing])/step_size+0.5)+1;
            if( num_iters <= 1 ) {
                num_iters = 2; // need at least 2 iterations because of coarse/fine step tuning
            }
            bool bMoved = false;
            while(num_iters-- > 0) {
                // set manip joints that haven't been covered so far
                if( (( vclosingdir[ifing] > 0) &&( dofvals[ifing] > vupperlim[ifing]+step_size) ) || (( vclosingdir[ifing] < 0) &&( dofvals[ifing] < vlowerlim[ifing]-step_size) ) ) {
                    break;
                }
                _robot->SetActiveDOFValues(dofvals,true);
                _robot->GetActiveDOFValues(dofvals);

                for(int q = 0; q < (int)_vlinks.size(); q++) {
                    int ct;
                    if(_robot->DoesAffect(pjoint->GetJointIndex(),_vlinks[q]->GetIndex())  &&((ct = _CheckCollision(_vlinks[q], KinBodyPtr())) != CT_None) ) {
                        if( !coarse_pass && (ct & CT_AvoidLinkHit) ) {
                            RAVELOG_VERBOSE(str(boost::format("hit link that needed to be avoided: %s\n")%_report->__str__()));
                            return PS_Failed;
                        }
                        if(coarse_pass) {
                            //coarse step collided, back up and shrink step
                            coarse_pass = false;
                            //if it didn't start in collision, move back one step before switching to smaller step size
                            if(bMoved) {
                                dofvals[ifing] -= vclosingdir[ifing] * step_size;
                                _robot->SetActiveDOFValues(dofvals,true);
                                num_iters = (int)(step_size/(_parameters->ffinestep*fmult))+1;
                                step_size = _parameters->ffinestep*fmult;
                            }
                            else {
                                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                    RAVELOG_VERBOSE(str(boost::format("Collision (%d) of link %s using joint %s(%d) [%s]\n")%ct%_vlinks.at(q)->GetName()%pjoint->GetName()%nDOFIndex%_report->__str__()));
                                    stringstream ss; ss << "Transform: " << _vlinks.at(q)->GetTransform() << ", Joint Vals: ";
                                    for(int vi = 0; vi < _robot->GetActiveDOF(); vi++) {
                                        ss << dofvals[vi] << " ";
                                    }
                                    ss << endl;
                                    RAVELOG_VERBOSE(ss.str());
                                }
                                if( ct & CT_SelfCollision ) {
                                    RAVELOG_WARN(str(boost::format("robot in self collision even though nothing moved: %s!")%_report->__str__()));
                                }
                                ncollided++;
                                collision = true;
                                break;
                            }
                        }
                        else {
                            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                RAVELOG_VERBOSE(str(boost::format("Collision (%d) of link %s using joint %s(%d), value=%f [%s]\n")%ct%_vlinks.at(q)->GetName()%pjoint->GetName()%nDOFIndex%dofvals[ifing]%_report->__str__()));
                                stringstream ss; ss << "Transform: " << _vlinks.at(q)->GetTransform() << "Joint Vals: ";
                                for(int vi = 0; vi < _robot->GetActiveDOF(); vi++) {
                                    ss << dofvals[vi] << " ";
                                }
                                ss << endl;
                                RAVELOG_VERBOSE(ss.str());
                            }

                            if( (ct & CT_SelfCollision) || _parameters->bavoidcontact ) {
                                if(bMoved) {
                                    // don't want the robot to end up in self collision, so back up
                                    dofvals[ifing] -= vclosingdir[ifing] * step_size;
                                }
                                else {
                                    RAVELOG_WARN("robot in self collision even though nothing moved!\n");
                                }
                            }
                            ncollided++;
                            collision = true;
                            break;
                        }
                    }
                }

                std::copy(dofvals.begin(),dofvals.begin()+_robot->GetActiveDOFIndices().size(),trajpoint.begin()+_trajspec._vgroups[1].offset);
                if(_parameters->breturntrajectory) {
                    ptraj->Insert(ptraj->GetNumWaypoints(),trajpoint,_trajspec);
                }
                if(collision) {
                    break;
                }
                dofvals[ifing] += vclosingdir[ifing] * step_size;
                bMoved = true;
            }
        }

        bool bAddLastPoint = true;
        _robot->SetActiveDOFValues(dofvals,true);
        for(int q = 0; q < (int)_vlinks.size(); q++) {
            int ct = _CheckCollision(_vlinks[q], KinBodyPtr());
            if( ct & CT_AvoidLinkHit ) {
                return PS_Failed;
            }
            if( ct & CT_SelfCollision ) {
                bAddLastPoint = false;
            }
        }

        if( bAddLastPoint ) {
            // don't forget the final point!
            _robot->GetActiveDOFValues(dofvals);
            std::copy(dofvals.begin(),dofvals.begin()+_robot->GetActiveDOFIndices().size(),trajpoint.begin()+_trajspec._vgroups[1].offset);
            if(!_parameters->breturntrajectory) {
                ptraj->Insert(ptraj->GetNumWaypoints(),trajpoint,_trajspec);
            }
        }

        return ptraj->GetNumWaypoints() > 0 ? PS_HasSolution : PS_Failed;     // only return true if there is at least one valid pose!
    }

    virtual int _CheckCollision(KinBody::LinkConstPtr plink, KinBodyPtr targetbody)
    {
        int ct = 0;
        bool bcollision;
        if( !!targetbody ) {
            bcollision = GetEnv()->CheckCollision(plink, KinBodyConstPtr(targetbody),_report);
        }
        else {
            bcollision = GetEnv()->CheckCollision(plink,_report);
        }
        if( bcollision ) {
            if( (!!_report->plink1 && _report->plink1->GetParent() == _parameters->targetbody) || (!!_report->plink2 && _report->plink2->GetParent() == _parameters->targetbody) ) {
                ct |= CT_TargetCollision;
            }
            else {
                ct |= CT_EnvironmentCollision;
            }
            FOREACH(itavoid,_vAvoidLinkGeometry) {
                if(( *itavoid == _report->plink1) ||( *itavoid == _report->plink2) ) {
                    ct |= CT_AvoidLinkHit;
                    break;
                }
            }

            if( _parameters->bonlycontacttarget ) {
                // check if hit anything besides the target
                if( (!!_report->plink1 &&( _report->plink1->GetParent() != plink->GetParent()) &&( _report->plink1->GetParent() != _parameters->targetbody) ) || (!!_report->plink2 &&( _report->plink2->GetParent() != plink->GetParent()) &&( _report->plink2->GetParent() != _parameters->targetbody) ) ) {
                    ct |= CT_AvoidLinkHit;
                }
            }

            return ct;
        }

        if(_robot->CheckSelfCollision(_report)) {
            ct |= CT_SelfCollision;
        }
        return ct;
    }

    virtual RobotBasePtr GetRobot() {
        return _robot;
    }
    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

protected:
    virtual int _MoveStraight(TrajectoryBasePtr ptraj, const Vector& vapproachdir, vector<dReal>& dofvals, int checkcollisions)
    {
        dReal* pX = NULL, *pY = NULL, *pZ = NULL;
        if( _robot->GetAffineDOF() & DOF_X ) {
            pX = &dofvals.at(_robot->GetAffineDOFIndex(DOF_X));
        }
        if( _robot->GetAffineDOF() & DOF_Y ) {
            pY = &dofvals.at(_robot->GetAffineDOFIndex(DOF_Y));
        }
        if( _robot->GetAffineDOF() & DOF_Z ) {
            pZ = &dofvals.at(_robot->GetAffineDOFIndex(DOF_Z));
        }

        KinBodyPtr targetbody;
        if( !(checkcollisions & CT_EnvironmentCollision) && (checkcollisions&CT_TargetCollision) ) {
            targetbody = _parameters->targetbody;
        }
        vector<dReal> trajpoint(_trajspec.GetDOF(),0);
        std::copy(dofvals.begin(),dofvals.begin()+_robot->GetActiveDOFIndices().size(),trajpoint.begin()+_trajspec._vgroups[1].offset);
        Vector v = vapproachdir * (_parameters->fcoarsestep*_parameters->ftranslationstepmult);
        int ct = 0;
        while(1) {
            ct = 0;
            for(int q = 0; q < (int)_vlinks.size(); q++) {
                ct = _CheckCollision(_vlinks[q], targetbody);
                if( ct&checkcollisions ) {
                    break;
                }
            }
            if( ct&checkcollisions ) {
                break;
            }

            if(_parameters->breturntrajectory) {
                RaveGetAffineDOFValuesFromTransform(trajpoint.begin(),_robot->GetTransform(),DOF_Transform);
                ptraj->Insert(ptraj->GetNumWaypoints(),trajpoint);
            }

            if( pX != NULL ) {
                *pX += v.x;
            }
            if( pY != NULL ) {
                *pY += v.y;
            }
            if( pZ != NULL ) {
                *pZ += v.z;
            }
            _robot->SetActiveDOFValues(dofvals);

            // check if robot is already past all objects
            AABB abRobot = _robot->ComputeAABB();
            if( vapproachdir.dot(abRobot.pos-_vTargetCenter) > _fTargetRadius + RaveSqrt(abRobot.extents.lengthsqr3()) ) {
                RAVELOG_DEBUG("robot did not hit anything, planner failing...\n");
                return false;
            }
        }

        // move back and try again with a finer step
        if( pX != NULL ) {
            *pX -= v.x;
        }
        if( pY != NULL ) {
            *pY -= v.y;
        }
        if( pZ != NULL ) {
            *pZ -= v.z;
        }
        _robot->SetActiveDOFValues(dofvals);

        v = vapproachdir * (_parameters->ffinestep*_parameters->ftranslationstepmult);
        while(1) {
            ct = 0;
            for(int q = 0; q < (int)_vlinks.size(); q++) {
                ct = _CheckCollision(_vlinks[q], targetbody);
                if( ct&checkcollisions ) {
                    break;
                }
            }
            if( ct&checkcollisions ) {
                break;
            }
            if( pX != NULL ) {
                *pX += v.x;
            }
            if( pY != NULL ) {
                *pY += v.y;
            }
            if( pZ != NULL ) {
                *pZ += v.z;
            }
            _robot->SetActiveDOFValues(dofvals);

            if(_parameters->breturntrajectory) {
                RaveGetAffineDOFValuesFromTransform(trajpoint.begin(),_robot->GetTransform(),DOF_Transform);
                ptraj->Insert(ptraj->GetNumWaypoints(),trajpoint);
            }
        }
        return ct;
    }
    CollisionReportPtr _report;
    boost::shared_ptr<GraspParameters> _parameters;
    RobotBasePtr _robot;
    vector<KinBody::LinkPtr> _vAvoidLinkGeometry;
    std::vector<KinBody::LinkPtr> _vlinks;
    Vector _vTargetCenter;
    dReal _fTargetRadius;
    ConfigurationSpecification _trajspec;
};

PlannerBasePtr CreateGrasperPlanner(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new GrasperPlanner(penv,sinput));
}
