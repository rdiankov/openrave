// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

#include <sstream>
#include <boost/shared_ptr.hpp>

#define COARSE_STEP 0.1f  ///step for coarse planning
#define FINE_STEP (COARSE_STEP/(100.0f)) ///step for fine planning, THIS STEP MUST BE VERY SMALL [COARSE_STEP/(100.0f)] OR THE COLLISION CHECKER GIVES WILDLY BOGUS RESULTS
//#define play_it     false ///generate a trajectory or not
#define TRANSLATION_LIMIT ((int)(10.0f/step_size)) ///how far to translate before giving up

/// sets a new collision checker and resets to the old when destroyed
class CollisionCheckerMngr
{
public:
    CollisionCheckerMngr(EnvironmentBase* penv, const string& collisionchecker) : _penv(penv)
    {
        assert( _penv != NULL );
        _pprevchecker = _penv->GetCollisionChecker();
        _coloptions = _penv->GetCollisionOptions();

        if( collisionchecker.size() > 0 ) {
            _pnewchecker.reset(_penv->CreateCollisionChecker(collisionchecker.c_str()));
            if( !!_pnewchecker ) {
                RAVELOG_VERBOSEA("setting collision checker %s\n", collisionchecker.c_str());
                _penv->SetCollisionChecker(_pnewchecker.get());
            }
        }
    }
    ~CollisionCheckerMngr() {
        _penv->SetCollisionChecker(_pprevchecker);
        _penv->SetCollisionOptions(_coloptions);
    }
private:
    EnvironmentBase* _penv;
    boost::shared_ptr<CollisionCheckerBase> _pnewchecker;
    CollisionCheckerBase* _pprevchecker;
    int _coloptions;
};

bool GrasperPlanner::InitPlan(RobotBase* pbase,const PlannerParameters* pparams)
{
    if(pbase == NULL || pparams == NULL)
    {
        RAVEPRINT(L"GrasperPlanner::InitPlan - Error, robot and or parameters are NULL\n");
        return false;
    }
    
    robot = pbase;
    _parameters = *pparams;
    _initconfig = _parameters.vinitialconfig;
    
    if( _initconfig.empty() || _parameters.vParameters.empty())
    {
        RAVELOG_ERRORA("GrasperPlanner::InitPlan - Error, init config or vParameters(direction) is NULL\n");
        return false;
    }

    _bInit = true;
    return true;
}

bool GrasperPlanner::PlanPath(Trajectory *ptraj, std::ostream* pOutStream)
{
    if(!_bInit)
        return false;

    CollisionCheckerMngr checkermngr(GetEnv(),"");
    GetEnv()->SetCollisionOptions(0);

    // do not clear the trajectory because the user might want to append the grasp point to it
    if( ptraj->GetDOF() != robot->GetActiveDOF() )
        ptraj->Reset(robot->GetActiveDOF());

    //get translation direction vector

    dReal stand_off = _parameters.vParameters[0];

    bool bface_target;
    if(_parameters.vParameters[1] == 1.0f)
        bface_target = true;
    else
        bface_target = false;

    dReal roll_hand = _parameters.vParameters[2];
    
    Vector Vdirection;
    Vdirection.x = _parameters.vParameters[3];
    Vdirection.y = _parameters.vParameters[4];
    Vdirection.z = _parameters.vParameters[5];

    //get palm normal
    Vector Vnormalplane;
    Vnormalplane.x = _parameters.vParameters[6];
    Vnormalplane.y = _parameters.vParameters[7];
    Vnormalplane.z = _parameters.vParameters[8];  

    //get play_it option (returns a trajectory if true, gives only last point if false)
    bool play_it;
    if(_parameters.vParameters.size() > 9)
        play_it = _parameters.vParameters[9]>0;
    else
        play_it = false;

    std::vector<Transform> vtrans;
    robot->GetBodyTransformations(vtrans);

    //roll hand
    Transform rollhand;
    rollhand.rot.x = cos(roll_hand/2);
    rollhand.rot.y = Vnormalplane.x*sin(roll_hand/2);
    rollhand.rot.z = Vnormalplane.y*sin(roll_hand/2);
    rollhand.rot.w = Vnormalplane.z*sin(roll_hand/2);

    rollhand.trans.w = 0;
    rollhand.trans.x = 0;
    rollhand.trans.y = 0;
    rollhand.trans.z = 0;

    //rotate hand to face object
    Transform rothand;
    if(bface_target)
    {
        //get body transfroms (we only need the base one, which is index 0)

        //robot->GetBodyTransformations(vtrans);
        RAVELOG_DEBUGA("Direction: %f %f %f %f\n", Vdirection.w, Vdirection.x, Vdirection.y, Vdirection.z);
        RAVELOG_DEBUGA("Before Transform: %f %f %f %f\n", Vnormalplane.w, Vnormalplane.x, Vnormalplane.y, Vnormalplane.z);
        
        //don't need to divide by magnitude b/c they are unit vecs
        dReal rottheta = RaveAcos(dot3(Vdirection,Vnormalplane));

        
        RAVELOG(L"Rot Theta: %f\n",rottheta);

    //    Vector right; right.Cross(direction, normalplane);
    //    normalize3(right, right);
    //    Vector up; up.Cross(direction, right);
    //    TransformMatrix tmat;
    //    tmat.m[0] = right.x; tmat.m[1] = up.x; tmat.m[2] = direction.x;
    //    tmat.m[4] = right.y; tmat.m[5] = up.y; tmat.m[6] = direction.y;
    //    tmat.m[8] = right.z; tmat.m[9] = up.z; tmat.m[10] = direction.z;
        Vnormalplane.Cross(Vdirection);
        normalize4(Vnormalplane, Vnormalplane);
        Vector rotaxis = Vnormalplane;
        
        RAVELOG_DEBUGA("RotAxis: %f %f %f \n",rotaxis.x,rotaxis.y,rotaxis.z);

        rothand.rot.x = cos(rottheta/2);
        rothand.rot.y = rotaxis.x*sin(rottheta/2);
        rothand.rot.z = rotaxis.y*sin(rottheta/2);
        rothand.rot.w = rotaxis.z*sin(rottheta/2);

        rothand.trans.w = 0;
        rothand.trans.x = 0;
        rothand.trans.y = 0;
        rothand.trans.z = 0;    
    }

    robot->SetTransform(vtrans[0]*rothand*rollhand);
    
    int ncollided = 0; ///number of links that have collided with an obstacle
    //RAVELOG(L"Vector %f %f %f %f \n",Vdirection.w,Vdirection.x,Vdirection.y,Vdirection.z);

    std::vector<dReal> dofvals = _initconfig;

    const std::vector<KinBody::Link*>& vlinks = robot->GetLinks();

    // only reset when traj dof != active dof
    if( ptraj->GetDOF() != robot->GetActiveDOF() )
        ptraj->Reset(robot->GetActiveDOF());
    Trajectory::TPOINT ptemp;    
    ptemp.q.resize(robot->GetActiveDOF());
    ptemp.qdot.resize(robot->GetActiveDOF());

    for(int i = 0; i < robot->GetActiveDOF(); i++)
        UpdateDependents(i,&dofvals[0]);

    //move hand toward object until something collides
    dReal step_size = 0.25f*COARSE_STEP;
    bool collision = false;
    
    bool coarse_pass = true; ///this parameter controls the coarseness of the step
    
    int num_iters = TRANSLATION_LIMIT;
    bool bMoved = false;

    dReal* pX = NULL, *pY = NULL, *pZ = NULL;

    if( robot->GetAffineDOF() ) {
        // if using affine dofs, only get the translations
        dReal* p = &dofvals[0] + robot->GetActiveDOF();
        if( robot->GetAffineDOF() & RobotBase::DOF_Rotation3D ) p -= 3;
        else if( robot->GetAffineDOF() & RobotBase::DOF_RotationAxis ) p -= 1;

        if( robot->GetAffineDOF() & RobotBase::DOF_Z ) pZ = --p;
        if( robot->GetAffineDOF() & RobotBase::DOF_Y ) pY = --p;
        if( robot->GetAffineDOF() & RobotBase::DOF_X ) pX = --p;
    
        while(num_iters-- > 0)
        {
            for(int q = 0; q < (int)vlinks.size(); q++)
            {
                if(GetEnv()->CheckCollision(vlinks[q]))
                {   
                    if(coarse_pass)
                    {
                        //coarse step collided, back up and shrink step
                        if(bMoved)
                        {
                            coarse_pass = false;

                            if( pX != NULL )  *pX -= step_size*Vdirection.x;
                            if( pY != NULL )  *pY -= step_size*Vdirection.y;
                            if( pZ != NULL )  *pZ -= step_size*Vdirection.z;
                            step_size = 0.25f*FINE_STEP;
                            num_iters = (int)(COARSE_STEP/FINE_STEP)+1;
                            robot->SetActiveDOFValues(NULL, &dofvals[0]);
                            break;
                        }
                        else
                        {
                            collision = true;
                            break;
                        }
                    }
                    else
                    {
                        if(stand_off == 0.0f)
                            ncollided++;
                       
                        collision = true;
                        //move hand back by standoff
                        
                        if( pX != NULL )  *pX -= stand_off*Vdirection.x;
                        if( pY != NULL )  *pY -= stand_off*Vdirection.y;
                        if( pZ != NULL )  *pZ -= stand_off*Vdirection.z;

                        break;
                    }
                }
            }

            for(int i = 0; i < robot->GetActiveDOF(); i++)
                ptemp.q[i] = dofvals[i];

            if(play_it) {
               ptraj->AddPoint(ptemp); 
            }

            if(collision)
                break;

            if( pX != NULL )  *pX += step_size*Vdirection.x;
            if( pY != NULL )  *pY += step_size*Vdirection.y;
            if( pZ != NULL )  *pZ += step_size*Vdirection.z;

            robot->SetActiveDOFValues(NULL, &dofvals[0]);
            bMoved = true;
        }
    }
    
    RAVELOG(L"Closing Fingers\n");
    
    std::vector<dReal> vlowerlim(robot->GetActiveDOF()), vupperlim(robot->GetActiveDOF());
    robot->GetActiveDOFLimits(&vlowerlim[0],&vupperlim[0]);
//    RAVELOG(L"DOF limits: \n");
//    for(int i = 0 ; i < robot->GetActiveDOF(); i++)
//        RAVELOG(L"%.3f %.3f  ",vlowerlim[i],vupperlim[i]);
//    RAVELOG(L"\n");
    
//    for(int i = 0; i < robot->GetDOF(); i++)
//        for(int q = 0; q < vlinks.size(); q++)
//           RAVELOG(L"Joint %d, Link %d, Char: %d\n",i,q,(int)robot->DoesAffect(i,q));

    vector<dReal> vclosingsign;

    if( (int)_parameters.vgoalconfig.size() == robot->GetActiveDOF() ) {
        vclosingsign = _parameters.vgoalconfig;
    }
    else {
        // get closing direction from manipulator

        vector<dReal> vclosingsign_full; // sometimes the sign of closing the fingers can be positive
        vclosingsign_full.insert(vclosingsign_full.end(), robot->GetDOF(), 1);
        
        // extract the sign from each manipulator
        vector<RobotBase::Manipulator>::const_iterator itmanip;
        FORIT(itmanip, robot->GetManipulators()) {
            for(int i = 0; i < (int)itmanip->_vClosedGrasp.size(); ++i) {
                if( itmanip->_vClosedGrasp[i] < itmanip->_vOpenGrasp[i] )
                    vclosingsign_full[itmanip->_vecjoints[i]] = -1;
            }
        }
        vclosingsign.resize(robot->GetActiveDOF());
        for(int i = 0; i < robot->GetActiveDOF(); ++i) {
            int index = robot->GetActiveJointIndex(i);
            if( index >= 0 )
                vclosingsign[i] = vclosingsign_full[index];
        }
    }

    //close the fingers one by one
    dReal fmult;

    for(int ifing = 0; ifing < robot->GetActiveDOF(); ifing++) {
        int nJointIndex = robot->GetActiveJointIndex(ifing);
        if( nJointIndex < 0 )
            // not a real joint, so skip
            continue;

        switch(robot->GetJoints()[nJointIndex]->GetType()) {
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
        while(num_iters-- > 0)
        {
            // set manip joints that haven't been covered so far
            UpdateDependents(ifing,&dofvals[0]);

            if( (vclosingsign[ifing] > 0 && dofvals[ifing] >  vupperlim[ifing]) || (vclosingsign[ifing] < 0 && dofvals[ifing] < vlowerlim[ifing]) )
            {
                //RAVELOG(L"Finger Limit Reached: %f\n",dofvals[ifing]);
                break;

            }
            robot->SetActiveDOFValues(NULL, &dofvals[0]);
            
            for(int q = 0; q < (int)vlinks.size(); q++)
            {
                if((robot->DoesAffect(robot->GetActiveJointIndex(ifing),q)  && GetEnv()->CheckCollision(vlinks[q])) )
                {  
                    if(coarse_pass)
                    {
                        //coarse step collided, back up and shrink step
                        coarse_pass = false;
                        //if it didn't start in collision, move back one step before switching to smaller step size
                        if(bMoved)
                        {
                            dofvals[ifing] -= vclosingsign[ifing] * step_size;
                            UpdateDependents(ifing,&dofvals[0]);
                            robot->SetActiveDOFValues(NULL, &dofvals[0]);
                            step_size = FINE_STEP*fmult;
                            num_iters = (int)(COARSE_STEP/FINE_STEP)+1;
                        }
                        else {
                            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                RAVELOG_VERBOSEA("Collision of link %S using joint %d\n", robot->GetLinks()[q]->GetName(),robot->GetActiveJointIndex(ifing));
                                stringstream ss; ss << "Joint Vals: ";
                                for(int vi = 0; vi < robot->GetActiveDOF();vi++)
                                    ss << dofvals[vi] << " ";
                                ss << endl;
                                RAVELOG_VERBOSEA(ss.str().c_str());
                            }
                                //vbcollidedlinks[q] = true;
                            //vijointresponsible[q] = robot->GetActiveJointIndex(ifing);
                            ncollided++;
                            collision = true;
                            break;
                        }
                    }
                    else {
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            RAVELOG_VERBOSEA("Collision of link %S using joint %d\n", robot->GetLinks()[q]->GetName(),robot->GetActiveJointIndex(ifing));
                            stringstream ss; ss << "Joint Vals: ";
                            for(int vi = 0; vi < robot->GetActiveDOF();vi++)
                                ss << dofvals[vi];
                            ss << endl;
                            RAVELOG_VERBOSEA(ss.str().c_str());
                        }
                        
                        ncollided++;
                        collision = true;
                        break;
                    }
                }
            }

            for(int j = 0; j < robot->GetActiveDOF(); j++)
                ptemp.q[j] = dofvals[j];

            if(play_it)
               ptraj->AddPoint(ptemp); 

            if(collision)
                break;

            dofvals[ifing] += vclosingsign[ifing] * step_size;
            bMoved = true;
        }

        if(ncollided == (int)vlinks.size())
            break;
    }
    
    if(!play_it)
        ptraj->AddPoint(ptemp);

    return true;
}

void GrasperPlanner::UpdateDependents(int ifing, dReal * dofvals)
{
    for(int c = ifing; c < robot->GetActiveDOF(); ++c ) {
        int index = robot->GetActiveJointIndex(c);
        if( index < 0 )
            // not a real joint, so skip
            continue;
        KinBody::Joint* pmimicjoint = robot->GetJoint(index);
        //check that it's the right doff val
        if( pmimicjoint->GetMimicJointIndex() == robot->GetActiveJointIndex(ifing)) {
            // set accordingly
            dofvals[c] = pmimicjoint->GetMimicCoeffs()[0]*dofvals[ifing] + pmimicjoint->GetMimicCoeffs()[1];
        }
    }

}

//for use without the planner
void GrasperProblem::UpdateDependentJoints(RobotBase* robot,int ifing, dReal * dofvals)
{
    for(int c = ifing; c < robot->GetActiveDOF(); ++c ) {
        int index = robot->GetActiveJointIndex(c);
        if( index < 0 )
            // not a real joint, so skip
            continue;
        KinBody::Joint* pmimicjoint = robot->GetJoint(index);
        //check that it's the right doff val
        if( pmimicjoint->GetMimicJointIndex() == robot->GetActiveJointIndex(ifing)) {
            // set accordingly
            dofvals[c] = pmimicjoint->GetMimicCoeffs()[0]*dofvals[ifing] + pmimicjoint->GetMimicCoeffs()[1];
        }
    }

}


GrasperProblem::GrasperProblem(EnvironmentBase* penv) : ProblemInstance(penv), planner(NULL), robot(NULL), pbody(NULL)
{
    normalplane = Vector(0,0,1);
}

void GrasperProblem::Destroy()
{
    delete planner; planner = NULL;
}

GrasperProblem::~GrasperProblem()
{
    Destroy();
}

void GrasperProblem::SetActiveRobots(const std::vector<RobotBase*>& robots)
{
    robot = NULL;
    if( robots.size() > 0 )
        robot = robots.front();
}

int GrasperProblem::main(const char* cmd)
{
    delete planner;
    planner = GetEnv()->CreatePlanner(L"Grasper");
    if( planner == NULL ) {
        RAVELOG(L"Failed to create planner\n");
        return -1;
    }

    if( robot == NULL ) {
        if( GetEnv()->GetRobots().size() > 0 ) {
            robot = GetEnv()->GetRobots().front();
        }
    }

    return 0;
}

bool GrasperProblem::SendCommand(const char* cmd, std::string& response)
{
    // parse the command line for a direction (or actual body)
    Transform Tstart;
    bool randomize = false;
    const char* pFilename = NULL; // write results to this file
    bool bHasDirection = false;
    bool bNoise = false;
    bool bTriangulateObj = false;
    bool bTriangulateRobot = false;
    bool bTriangulateLink = false;
    bool bMakeShells = false;
    int linkindex=0;
    bool bComputeDistMap = false;
    bool bComputeDistMap2 = false;
    bool bComputeMultiResMap = false;
    bool bUsePoints = false;
    bool bGetClutter = false;
    bool bHasStepSize = false;
    bool bOpenFingers = false;
    bool bGetLinkCollisions = false;
    bool bCenterOnManipulator = false;

    bool bExecutePlanner=false;
    bool bTransRobot=true; // if false, don't move the robot
    bool bMoveOut = false; //move until just barely in collision along the specified direction

    PlannerBase::PlannerParameters params;
    dReal standoff=0;
    dReal fingerspread=0;
    dReal rotnoise=0;
    dReal transnoise=0;
    dReal handroll=0;
    unsigned int random_seed=0;
    Vector direction;
    Vector centeroffset; // center offset when aiming for pbody
    dReal conewidth = 45.0f;
    
    dReal mu = -1.0f;
    bool bcheck_stability = false;
    int nDistMapSamples = 60000;

    std::vector<GrasperProblem::SURFACEPOINT> vpoints_in;
    std::vector<dReal> vstepsizes;
    std::vector<int> vopenignorejoints;

    boost::shared_ptr<CollisionCheckerMngr> pcheckermngr;

    if( cmd != NULL ) {
        const char* delim = " \r\n\t";
        char* mycmd = strdup(cmd);
        char* p = strtok(mycmd, delim);
        while(p != NULL ) {
            if( stricmp(p, "body") == 0 ) {
                p = strtok(NULL, delim);
                pbody = GetEnv()->GetKinBody(_ravembstowcs(p).c_str());   
            }
            if( stricmp(p, "bodyid") == 0 ) {
                p = strtok(NULL, delim);
                if( p != NULL )
                    pbody = GetEnv()->GetBodyFromNetworkId(atoi(p));
            }
            else if( stricmp(p, "robot") == 0 ) {
                // specify which robot to use
                robot = NULL;
                int robotid = atoi(strtok(NULL, delim));
                vector<RobotBase*>::const_iterator itrobot;
                FORIT(itrobot, GetEnv()->GetRobots()) {
                    if( (*itrobot)->GetNetworkId() == robotid ) {
                        robot = *itrobot;
                        break;
                    }
                }

                if( robot == NULL ) {
                    RAVELOGA("Failed to find robot with id: %d\n", robotid);
                }
            }
            else if( stricmp(p, "direction") == 0 ) {
                direction.x = (dReal)atof(strtok(NULL, delim));
                direction.y = (dReal)atof(strtok(NULL, delim));
                direction.z = (dReal)atof(strtok(NULL, delim));
                bHasDirection = true;
            }
            else if( stricmp(p, "randomize") == 0 ) {
                random_seed = atoi(strtok(NULL, delim));
                randomize = true;
            }
            else if( stricmp(p, "computedistances") == 0 ) {
                bComputeDistMap = true;
            }
            else if( stricmp(p, "computedistances2") == 0 ) {
                bComputeDistMap2 = true;
            }
            else if( stricmp(p, "computemultiresmap") == 0 ) {
                bComputeMultiResMap = true;
            }
            else if( stricmp(p, "usepoints") == 0 ) {
                bUsePoints = true;
                int numpoints = atoi(strtok(NULL, delim));
                for(int i = 0; i < numpoints;i++)
                {
                    dReal px = (dReal)atof(strtok(NULL, delim));
                    dReal py = (dReal)atof(strtok(NULL, delim));
                    dReal pz = (dReal)atof(strtok(NULL, delim));
                    dReal nx = (dReal)atof(strtok(NULL, delim));
                    dReal ny = (dReal)atof(strtok(NULL, delim));
                    dReal nz = (dReal)atof(strtok(NULL, delim));

                    vpoints_in.push_back(SURFACEPOINT(px,py,pz,nx,ny,nz)); 
                    //RAVEPRINT(L"%f %f %f  %f %f %f  %f\n",vpoints_in[i].norm.x,vpoints_in[i].norm.y,vpoints_in[i].norm.z, vpoints_in[i].pos.x, vpoints_in[i].pos.y, vpoints_in[i].pos.z, vpoints_in[i].dist);
                }

            }
            else if( stricmp(p, "mapsamples") == 0 ) {
                nDistMapSamples = atoi(strtok(NULL, delim));
            }
            else if( stricmp(p, "file") == 0 ) {
                pFilename = strtok(NULL, delim);
            }
            else if( stricmp(p, "noise") == 0 ) {
                rotnoise = (dReal)atof(strtok(NULL, delim));
                transnoise = (dReal)atof(strtok(NULL, delim));
                bNoise = true;
            }
            else if( stricmp(p, "roll") == 0 ) {
                handroll = (dReal)atof(strtok(NULL, delim));
		        //RAVEPRINT(L"Hand Roll: %f\n",handroll);
            }
            else if( stricmp(p, "palmdir") == 0 ) {
                // set direction of palm
                normalplane.x = (dReal)atof(strtok(NULL, delim));
                normalplane.y = (dReal)atof(strtok(NULL, delim));
                normalplane.z = (dReal)atof(strtok(NULL, delim));
                normalize3(normalplane, normalplane);
            }
            else if( stricmp(p, "centeroffset") == 0 ) {
                centeroffset.x = (dReal)atof(strtok(NULL, delim));
                centeroffset.y = (dReal)atof(strtok(NULL, delim));
                centeroffset.z = (dReal)atof(strtok(NULL, delim));
            }   
            else if( stricmp(p, "centermanip") == 0 )
                bCenterOnManipulator = true;
            else if( stricmp(p, "fingerspread") == 0 ) {
                fingerspread = (dReal)atof(strtok(NULL, delim));
            }
            else if( stricmp(p, "standoff") == 0 ) {
                standoff = (dReal)atof(strtok(NULL, delim));
            }
            else if( stricmp(p, "getclutter") == 0 ) {
                bGetClutter = true;
            }
            else if( stricmp(p, "triangulateobject") == 0 ) {
                bTriangulateObj = true;
            }
            else if( stricmp(p, "triangulatehand") == 0 ) {
                bTriangulateRobot = true;
            }
            else if( stricmp(p, "triangulatelink") == 0 ) {
                bTriangulateLink = true;
                linkindex = (int)atof(strtok(NULL, delim));
            }
            else if( stricmp(p, "makeshells") == 0 ) {
                bMakeShells = true;
            }
            else if( stricmp(p, "stepsizes") == 0 ) {
                int numsizes = atoi(strtok(NULL, delim));
                vstepsizes.resize(numsizes);
                for(int i = 0; i < numsizes;i++)
                    vstepsizes[i] = (dReal)atof(strtok(NULL, delim)); 

                bHasStepSize = true;
            }
            else if( stricmp(p, "exec") == 0 ) {
                bExecutePlanner = true;
            }
            else if( stricmp(p, "notrans") == 0 ) {
                bTransRobot = false;
            }
            else if( stricmp(p, "conewidth") == 0 ) {
                conewidth = (dReal)atof(strtok(NULL, delim));
            }
            else if( stricmp(p, "friction") == 0 ) {
                mu = (dReal)atof(strtok(NULL, delim));
                bcheck_stability = true;
            }
            else if( stricmp(p, "moveout") == 0 ) {
                //should be used along with "notrans"
                bMoveOut = true;
            }
            else if( stricmp(p, "openfingers") == 0 ) {
                bOpenFingers = true;
            }            
            else if( stricmp(p, "openignorejoints") == 0 ) {
                int numjoints = atoi(strtok(NULL, delim));
                vopenignorejoints.resize(numjoints);
                for(int i = 0; i < numjoints;i++)
                    vopenignorejoints[i] = atoi(strtok(NULL, delim)); 
            }     
            else if( stricmp(p, "getlinkcollisions") == 0 ) {
                bGetLinkCollisions = true;
            }
            else if( stricmp(p, "collision") == 0 ) {
                pcheckermngr.reset(new CollisionCheckerMngr(GetEnv(), strtok(NULL, delim)));
            }
            p = strtok(NULL, delim);
        }
        
        free(p);
        dReal L = lengthsqr3(direction);
        if( L == 0 ) direction = Vector(0,0,1);
        else direction /= sqrtf(L);
    }
    
    boost::shared_ptr<KinBody::KinBodyStateSaver> bodysaver, robotsaver;
    if( pbody != NULL )
        bodysaver.reset(new KinBody::KinBodyStateSaver(pbody));
    //if( robot != NULL )
    //robotsaver.reset(new RobotBase::RobotStateSaver(robot));
    
    if(bMoveOut && bTransRobot)
    {
        RAVEPRINT(L"Error: 'Move until almost out of collision' and 'translate robot' cannot both be set. If you want to move out , use 'moveout notrans'.\n");       
        return false;
    }

    if( bMakeShells ) {

        if( robot != NULL ) {
    
            robot->SetTransform(Transform());
            KinBody::Link::TRIMESH trimesh;
            
            std::vector<KinBody::Link*> links = robot->GetLinks();

            vector<std::vector<Vector> > vpoints(links.size());
            Vector vtemp;
            Vector vprev(-1000000.0f,-1000000.0f,-1000000.0f);

            for(int i = 0; i < (int)links.size(); i++)
            {
                std::stringstream filename;
                filename << "link" << i << "points.txt";
                std::ifstream linkfile;

                linkfile.open(filename.str().c_str());

                if (linkfile.fail())
                {
                    RAVEPRINT(L"Can't open %s\n",filename.str().c_str());
                    return false;
                }
                
                while(!linkfile.eof())
                {

                    linkfile >> vtemp.x; linkfile >> vtemp.y; linkfile >> vtemp.z;
                    if(vtemp.x != vprev.x || vtemp.y != vprev.y || vtemp.z != vprev.z)
                    {
                        RAVELOG(L"%f %f %f\n",vtemp.x,vtemp.y,vtemp.z);
                        vpoints[i].push_back(vtemp );
                        vprev = vtemp;                    
                    }
//                    GetEnv()->plot3(vpoints[i][vpoints[i].size()-1], 1, 0, 0.004f, Vector(1,0,1) );
                }
            }
            //RAVEPRINT(L"Finished reading in points.\n");

            vector<dReal> closingdir;

            //make sure we get the right closing direction and don't look at irrelevant joints
            bool handjoint;
            RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
            for(int i = 0; i < robot->GetDOF(); ++i) 
            {
                handjoint = false;
                for(size_t j = 0; j < pmanip->_vecjoints.size(); j++) 
                {
                    if(i == pmanip->_vecjoints[j]) 
                    {
                        handjoint = true;
                        if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
                            closingdir.push_back(-1.0f);
                        else
                            closingdir.push_back(1.0f);

                        break;
                    }
                }
                
                //if(!handjoint)
                //    closingdir.push_back(0);
            }


            //GrasperProblem::SURFACEPOINT
            int numdof = robot->GetActiveDOF();
            vector<vector<vector<GrasperProblem::SURFACEPOINT> > > vjointvshells(numdof+1); //extra 1 is for immobile points (like on link 0)
            vector<GrasperProblem::SURFACEPOINT> dummy;
            std::vector<dReal> vlowerlimit(numdof), vupperlimit(numdof);
            robot->GetActiveDOFLimits(&vlowerlimit[0],&vupperlimit[0]);

            if((int)closingdir.size() != numdof)
            {

                RAVEPRINT(L"GrasperProblem Error - Active dofs (%d) can only be manipulator vecjoints dofs \n",numdof);
                return false;
            }
            

            if(bHasStepSize && ((int)vstepsizes.size() != numdof))
            {
                RAVELOG_ERRORA("GrasperProblem Error - Active dofs (%d) must be the same number as number of step sizes passed in (%"PRIdS")\n",numdof,vstepsizes.size());
                return false;

            }

            dReal defaultstepsize = 0.2f;
            dReal stepsize;

            vector<dReal> vcurrent(numdof); 
            vector<dReal> vstart(numdof);
            robot->GetActiveDOFValues(&vstart[0]);
            Transform Ttemp;
            std::vector<dReal> J(3*robot->GetDOF());
            Vector defaultdir;

            if(bHasDirection)
                defaultdir = direction;
            else
                defaultdir = Vector(0,0,1);
            Vector deltaxyz;
            dReal temp;
            GrasperProblem::SURFACEPOINT pointtemp;

            for(int i = 0; i < numdof; i++)
            {
                if(bHasStepSize)
                    stepsize = vstepsizes[i];
                else
                    stepsize = defaultstepsize;

                vcurrent[i] = vstart[i] - stepsize*closingdir[i];
                //RAVEPRINT(L"Starting DOF %d.\n",i);

                for(int j = 0; ((closingdir[i] == 1.0f) && (vcurrent[i] < vupperlimit[i])) || ((closingdir[i] == -1.0f) && (vcurrent[i] > vlowerlimit[i])); j++)
                {
                    //RAVEPRINT(L"Starting Shell %d.\n",j);

                    vcurrent[i] += stepsize*closingdir[i];
                    robot->SetActiveDOFValues(NULL,&vcurrent[0]);

                    vjointvshells[i].push_back(dummy);
                    for(int q = 0; q < (int)links.size(); q++)
                    {
                        //RAVEPRINT(L"Starting Link %d.\n",q);
                        if(robot->DoesAffect(robot->GetActiveJointIndex(i),q))
                        {
                            Ttemp = links[q]->GetTransform(); 
                            for(size_t qp = 0; qp < vpoints[q].size(); qp++)
                            {
                                pointtemp.pos = Ttemp * vpoints[q][qp];

                                memset(&J[0], 0, sizeof(dReal)*J.size());
                                robot->CalculateJacobian(q, pointtemp.pos, &J[0]);
                                //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                                for(int qj = 0; qj < 3; qj++)
                                {   


                                    temp = J[qj*robot->GetDOF() + robot->GetActiveJointIndex(i)]*0.01f*closingdir[i];
                                                                
                                    if( qj == 0)
                                        deltaxyz.x  = temp;
                                    else if( qj == 1)
                                        deltaxyz.y = temp;
                                    else if( qj == 2)
                                        deltaxyz.z = temp;
                                }

                                //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                                //so treat it as if it were part of the base link
                                if(lengthsqr3(deltaxyz) < 0.000000001f)
                                    deltaxyz = defaultdir;
                                
                                normalize3(deltaxyz,deltaxyz);

                                pointtemp.norm = deltaxyz;
                                vjointvshells[i][j].push_back(pointtemp);

                                GetEnv()->plot3(RaveVector<float>(pointtemp.pos), 1, 0, 7, RaveVector<float>(1,0,0) );

                                GetEnv()->drawarrow(pointtemp.pos,pointtemp.pos + 0.01*pointtemp.norm, 0.002f,Vector(0,1,0));
                            }
                        }
                    }
                }

            }
            //RAVEPRINT(L"Finished mobile links.\n");
            //add in immobile points in the last shell
            vjointvshells.back().push_back(dummy);
            bool affects;
            for(int i = 0; i < (int)links.size();i++)
            {
                affects = false;
                for(int j = 0; j < numdof; j++)
                    if(robot->DoesAffect(robot->GetActiveJointIndex(j),i))
                    {
                        affects = true;
                        break;
                    }
                if(!affects)
                {              
                    for(size_t qp = 0; qp < vpoints[i].size(); qp++)
                    {
                        Ttemp = links[i]->GetTransform(); 
                        pointtemp.pos = Ttemp * vpoints[i][qp];
                        pointtemp.norm = defaultdir;
                        //pointtemp.pos = vpoints[i][qp];
                        vjointvshells.back().back().push_back(pointtemp);
                        GetEnv()->plot3(RaveVector<float>(pointtemp.pos), 1, 0, 7, RaveVector<float>(1,0,0) );
                        GetEnv()->drawarrow(RaveVector<float>(pointtemp.pos),RaveVector<float>(pointtemp.pos + 0.01*pointtemp.norm), 0.002f,RaveVector<float>(0,1,0));
                    }
                }
            }
            //RAVEPRINT(L"Finished immobile links.\n");
            //robot->SetActiveDOFValues(NULL,&vlowerlimit[0]);

            std::stringstream outstream;
            //now print the shells to the return string

            outstream << vjointvshells.size() << "\n";
            for(size_t i = 0; i < vjointvshells.size(); i++)
            {
                outstream << vjointvshells[i].size() << "\n";
                for(size_t j = 0; j < vjointvshells[i].size(); j++)
                {
                    outstream << vjointvshells[i][j].size() << "\n";
                    for(size_t p = 0; p < vjointvshells[i][j].size(); p++)
                        outstream << vjointvshells[i][j][p].pos.x << " " << vjointvshells[i][j][p].pos.y << " " << vjointvshells[i][j][p].pos.z << " " << vjointvshells[i][j][p].norm.x << " " << vjointvshells[i][j][p].norm.y << " " << vjointvshells[i][j][p].norm.z << "\n";
                }
            }
            response = outstream.str();
            return true;
        }
        else RAVEPRINT(L"GrasperProblem: Robot is NULL!\n");    
    }
    
    if( bTriangulateLink ) {
        if( robot != NULL ) {

            KinBody::Link::TRIMESH trimesh;
            
            std::vector<KinBody::Link*> links = robot->GetLinks();
            if(linkindex >= (int)links.size())
            {   
                RAVELOG_ERRORA("GrasperProblem: Link index is out of range, max index is %"PRIdS"\n",links.size());    
                return false;
            }            
            
            trimesh = links[linkindex]->GetCollisionData();

            std::stringstream os;

            os << trimesh.indices.size()/3 << endl;


            for(size_t i = 0; i < trimesh.indices.size(); i++) {
                os << trimesh.indices[i] << " ";
            }

            for(size_t i = 0; i < trimesh.vertices.size(); i++) {
                os << trimesh.vertices[i].x << " " << trimesh.vertices[i].y << " " << trimesh.vertices[i].z << endl;
            }
            
            //robot->SetTransform(prevtrans);

            response = os.str();
        }
        else RAVEPRINT(L"GrasperProblem: Robot is NULL!\n");    

    }

    if( bTriangulateObj ) {
        if( pbody != NULL ) {
            Transform prevtrans = pbody->GetTransform();
            pbody->SetTransform(Transform());

            KinBody::Link::TRIMESH trimesh;
            GetEnv()->TriangulateScene(trimesh, EnvironmentBase::TO_Body, pbody->GetName());
            std::stringstream os;
            
            os << trimesh.indices.size()/3 << endl;


            for(size_t i = 0; i < trimesh.indices.size(); i++) {
                os << trimesh.indices[i] << " ";
            }

            for(size_t i = 0; i < trimesh.vertices.size(); i++) {
                os << trimesh.vertices[i].x << " " << trimesh.vertices[i].y << " " << trimesh.vertices[i].z << endl;
            }

            pbody->SetTransform(prevtrans);
            response = os.str();
            return true;
        }
        else RAVEPRINT(L"GrasperProblem: pbody is NULL!\n");            
    }

    if( bTriangulateRobot ) {
        if( robot != NULL ) {
            Transform prevtrans = robot->GetTransform();
            //robot->SetTransform(Transform());

            KinBody::Link::TRIMESH trimesh;
            GetEnv()->TriangulateScene(trimesh, EnvironmentBase::TO_Body, robot->GetName());
            std::stringstream os;

            os << trimesh.indices.size()/3 << endl;


            for(size_t i = 0; i < trimesh.indices.size(); i++) {
                os << trimesh.indices[i] << " ";
            }

            for(size_t i = 0; i < trimesh.vertices.size(); i++) {
                os << trimesh.vertices[i].x << " " << trimesh.vertices[i].y << " " << trimesh.vertices[i].z << endl;
            }
            
            //robot->SetTransform(prevtrans);

            response = os.str();
            return true;
        }
        else RAVEPRINT(L"GrasperProblem: Robot is NULL!\n");
    }

    if( bGetClutter ) {
        std::set<KinBody *> vbodyexcluded;
        std::set<KinBody::Link *> vlinkexcluded;
        vbodyexcluded.insert(pbody);
        
        //make sure the robot isn't colliding with anything except pbody
        std::stringstream clutterfile;

        if( GetEnv()->CheckCollision(robot,vbodyexcluded,vlinkexcluded,(COLLISIONREPORT *)NULL) ) {
        }
        else {
            COLLISIONREPORT colreport;
            GetEnv()->SetCollisionOptions(CO_Distance);

            GetEnv()->CheckCollision(robot->GetLinks()[0],vbodyexcluded,vlinkexcluded,&colreport);
            clutterfile << colreport.minDistance;

            GetEnv()->SetCollisionOptions(0);
        }

        response = clutterfile.str();
    }
    if( bComputeDistMap ) {
        if(pbody == NULL)
        {
           RAVEPRINT(L"Grasper Error - Target body is NULL.\n");
           return false;
        }

        // move the robot out of the way
        std::vector<RobotBase*> robots = GetEnv()->GetRobots();
        std::vector<Transform> Tbackup;
        std::vector<Transform> vtrans;
        for(size_t i = 0; i < robots.size(); i++)
        {
           robots[i]->GetBodyTransformations(vtrans);
           Tbackup.push_back(vtrans[0]);
           robots[i]->SetTransform(Transform(Vector(1,0,0,0),Vector(1000,1000,1000)));
        }
        //if( robot != NULL )
        //    robot->Enable(false);
        
        //get min distance data
        vector<GrasperProblem::SURFACEPOINT> vpoints;
        
        Vector graspcenter = centeroffset;
        if( pbody != NULL )
            graspcenter += pbody->GetCenterOfMass();


        BoxSample(pbody,vpoints,nDistMapSamples,graspcenter);
        //DeterministicallySample(pbody, vpoints, 4, graspcenter);

        vector<KinBody*> vbodies;
        GetEnv()->GetBodies(vbodies);
        if( pbody != NULL ) {
            for(size_t i = 0; i < vbodies.size(); ++i) {
                if( vbodies[i] == pbody ) {
                    vbodies.erase(vbodies.begin()+i);
                    break;
                }
            }
        }

        //compute distance map, last argument is width of cone around each ray
        ComputeDistanceMap(vbodies, vpoints, conewidth/180.0f*PI);

        std::stringstream distfile;
        for(size_t i = 0; i < vpoints.size(); ++i) {
            
            distfile << vpoints[i].dist << " " << vpoints[i].norm.x << " " << vpoints[i].norm.y << " " << vpoints[i].norm.z << " ";
            distfile << vpoints[i].pos.x - graspcenter.x << " " << vpoints[i].pos.y - graspcenter.y << " " << vpoints[i].pos.z - graspcenter.z << "\n";

            Vector v = vpoints[i].pos + vpoints[i].norm * vpoints[i].dist;
            
            //for visualizing the distance map

//            GetEnv()->plot3(vpoints[i].pos, 1, 0, 0.004f, Vector(1,0,0) );
//           if(vpoints[i].dist > 0.2f) GetEnv()->drawarrow(vpoints[i].pos, vpoints[i].pos + 0.02f*vpoints[i].norm);

        }
        
        RAVEPRINT(L"distance map computed\n");
        response = distfile.str();

        // reenable
        for(size_t i = 0; i < robots.size(); i++)
           robots[i]->SetTransform(Tbackup[i]);
    }


    if( bComputeDistMap2 ) {
        if(pbody == NULL)
        {
           RAVEPRINT(L"Grasper Error - Target body is NULL.\n");
           return false;
        }

        // move the robot out of the way
        std::vector<RobotBase*> robots = GetEnv()->GetRobots();
        std::vector<Transform> Tbackup;
        std::vector<Transform> vtrans;
        for(size_t i = 0; i < robots.size(); i++)
        {
           robots[i]->GetBodyTransformations(vtrans);
           Tbackup.push_back(vtrans[0]);
           robots[i]->SetTransform(Transform(Vector(1,0,0,0),Vector(1000,1000,1000)));
        }

        //get min distance data
        Vector graspcenter = centeroffset;
        graspcenter += pbody->GetCenterOfMass();

        if(!bUsePoints)
            BoxSample(pbody,vpoints_in,nDistMapSamples,graspcenter);

        for(size_t i = 0; i < vpoints_in.size(); ++i)
            vpoints_in[i].pos += 0.001f*vpoints_in[i].norm;


        vector<KinBody*> vbodies;
        GetEnv()->GetBodies(vbodies);

        ComputeDistanceMap(vbodies, vpoints_in, conewidth/180.0f*PI);

        std::stringstream distfile;
        
        for(size_t i = 0; i < vpoints_in.size(); ++i)  {

            distfile << vpoints_in[i].norm.x << " " << vpoints_in[i].norm.y << " " << vpoints_in[i].norm.z << " ";
            distfile << vpoints_in[i].pos.x << " " << vpoints_in[i].pos.y << " " << vpoints_in[i].pos.z << " ";
            distfile << vpoints_in[i].dist << "\n";
            //Vector v = vpoints_in[i].pos + vpoints_in[i].norm * vpoints_in[i].dist;
            
            //for visualizing the distance map
            //RAVEPRINT(L"%f %f %f  %f %f %f  %f\n",vpoints_in[i].norm.x,vpoints_in[i].norm.y,vpoints_in[i].norm.z, vpoints_in[i].pos.x, vpoints_in[i].pos.y, vpoints_in[i].pos.z, vpoints_in[i].dist);
//            dReal color;
//            if ( vpoints_in[i].dist >= 0.05)
//                color = 1.0f;
//            else
//                color = 0.0f;
//            GetEnv()->plot3(vpoints_in[i].pos, 1, 0, 0.004f, Vector(color,0,0) );
//            GetEnv()->drawarrow(vpoints_in[i].pos, vpoints_in[i].pos + (avg)*vpoints_in[i].norm);

        }
        
        RAVEPRINT(L"distance map computed\n");
        response = distfile.str();

        // reenable
        for(size_t i = 0; i < robots.size(); i++)
           robots[i]->SetTransform(Tbackup[i]);
    }



    if( bComputeMultiResMap ) {
        if(pbody == NULL)
        {
           RAVEPRINT(L"Grasper Error - Target body is NULL.\n");
           return false;
        }

        // move the robots out of the way
        std::vector<RobotBase*> robots = GetEnv()->GetRobots();
        std::vector<Transform> Tbackup;
        std::vector<Transform> vtrans;
        for(size_t i = 0; i < robots.size(); i++) {
           robots[i]->GetBodyTransformations(vtrans);
           Tbackup.push_back(vtrans[0]);
           robots[i]->SetTransform(Transform(Vector(1,0,0,0),Vector(1000,1000,1000)));
        }


        //vector<GrasperProblem::SURFACEPOINT> vpoints;
        Vector graspcenter = centeroffset;
        graspcenter += pbody->GetCenterOfMass();

        if(!bUsePoints)
            BoxSample(pbody,vpoints_in,nDistMapSamples,graspcenter);



        vector<KinBody*> vbodies;
        GetEnv()->GetBodies(vbodies);


        dReal stepsize = 0.01f;
        dReal maxradius = 0.04f;
        ComputeMultiResMap(vbodies, pbody, vpoints_in, stepsize,maxradius);

        std::stringstream distfile;

        distfile << stepsize << " " << (int)ceil(maxradius/stepsize) << "\n";

        for(size_t i = 0; i < vpoints_in.size(); ++i) {
            dReal avg = 0;

            distfile << vpoints_in[i].norm.x << " " << vpoints_in[i].norm.y << " " << vpoints_in[i].norm.z << " ";
            distfile << vpoints_in[i].pos.x << " " << vpoints_in[i].pos.y << " " << vpoints_in[i].pos.z << " ";
            for(size_t j = 0; j < vpoints_in[i].dists.size(); ++j) {
                distfile << vpoints_in[i].dists[j] << " ";    
                avg += vpoints_in[i].dists[j];
                    
            }
            avg = avg/vpoints_in[i].dists.size();

            distfile << "\n";
            //Vector v = vpoints_in[i].pos + vpoints_in[i].norm * vpoints_in[i].dist;
            
            //for visualizing the distance map

//            GetEnv()->plot3(vpoints_in[i].pos, 1, 0, 0.004f, Vector(min/2.0f,0,0) );
//            GetEnv()->drawarrow(vpoints_in[i].pos, vpoints_in[i].pos + (avg)*vpoints_in[i].norm);

        }
        
        RAVEPRINT(L"distance map computed\n");
        response = distfile.str();

        // reenable
        for(size_t i = 0; i < robots.size(); i++)
           robots[i]->SetTransform(Tbackup[i]);
    }


    if(bOpenFingers)
    {
        //WARNING: THIS FUNCTION ASSUMES ALL CLOSING DIRS ARE +1
        RAVELOG(L"Opening Fingers\n");
        RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
        if(pmanip == NULL)
        {
            RAVEPRINT(L"Error: Robot has no manipulator!\n");
            return false;
        }             
        if(pbody == NULL)
        {
            RAVEPRINT(L"Error: Need to specify body!\n");
            return false;
        }
        
        GetEnv()->SetCollisionOptions(0);
        
        vector<dReal> closingdir;

        //make sure we get the right closing direction and don't look at irrelevant joints
        int numdof = robot->GetActiveDOF();
        if(numdof != (int)pmanip->_vecjoints.size())
        {        
            RAVEPRINT(L"ERROR: Grasper - OpenFingers - the active DOF must be the same as manipulator vecjoints\n");
            return false;
        }

        bool handjoint;
        for(int i = 0; i < robot->GetActiveDOF(); ++i) 
        {
            handjoint = false;
            for(size_t j = 0; j < pmanip->_vecjoints.size(); j++) 
            {
                //RAVEPRINT(L"manip joint %d\n",pmanip->_vecjoints[j]);
                if(robot->GetActiveJointIndex(i) == pmanip->_vecjoints[j]) 
                {
                    handjoint = true;
                    if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
                        closingdir.push_back(-1.0f);
                    else
                        closingdir.push_back(1.0f);

                    break;
                }
            }
            
            if(!handjoint)
            {
                RAVEPRINT(L"ERROR: Grasper - OpenFingers - the active DOF must be the same as manipulator vecjoints\n");
                return false;
            }
            //    closingdir.push_back(0);
        }
       

        //for(int i = 0; i < numdof; i++)
        //    closingdir.push_back(1.0);
        
        std::vector<KinBody::Link*> links = robot->GetLinks();
        std::vector<dReal> vlowerlimit(numdof), vupperlimit(numdof);
        robot->GetActiveDOFLimits(&vlowerlimit[0],&vupperlimit[0]);
        
        vector<dReal> vcurrent(numdof); 
        vector<dReal> vstart(numdof);
        robot->GetActiveDOFValues(&vstart[0]);
        vcurrent = vstart;
        
        std::vector<KinBody *> vbodies;
        GetEnv()->GetBodies(vbodies);
        
        vector<KinBody*> vbodies_nopbody;
        
        

        for(size_t i = 0; i < vbodies.size(); ++i) {
            if( vbodies[i] != pbody && vbodies[i] != robot) {
                vbodies_nopbody.push_back(vbodies[i]);
            }
        }
        
        dReal stepsize = 0.02f;
        bool bbodyfree;
        bool bworldfree;
        bool outcol;
        bool ignorejoint;
        int firstfreej;
        for(int i = 0; i < numdof; i++)
        {
            ignorejoint = false;
            for(size_t j = 0; j < vopenignorejoints.size();j++)
                if( vopenignorejoints[j] == robot->GetActiveJointIndex(i))
                {
                    ignorejoint = true;
                    break;
                }
            if(ignorejoint)
                continue;
            
            outcol = false;
            firstfreej = 0;
            //vcurrent[i] = vstart[i];//+closingdir[i]*stepsize;//so that j = 0 is the start position
            for(int j = 0; true; j++)
            {
                
                
                bbodyfree = true;
                bworldfree = true;
                for(size_t q = 0; q < links.size(); q++)
                {
                    

                    if(robot->DoesAffect(robot->GetActiveJointIndex(i),q))
                    {
                        //if in free state, keep moving until collision if not, move until free
                        if(bbodyfree && GetEnv()->CheckCollision(links[q],pbody))
                            bbodyfree = false;
                            
                        
                        if(bworldfree)
                            for(size_t b = 0; b < vbodies_nopbody.size(); b++)
                                if(GetEnv()->CheckCollision(links[q],vbodies_nopbody[b]))
                                {
                                    bworldfree = false;
                                    break;
                                }
                    }
                    
                    
                }

                if(bbodyfree && bworldfree)
                {
                    //RAVEPRINT(L"finger is not touching anything, keep moving out i = %d  j = %d ffj = %d\n",i,j,firstfreej);
                    //if just reached free space, record j
                    if(!outcol)
                        firstfreej = j;
                    outcol = true;
                }
                else if(!bbodyfree && bworldfree && !outcol)
                {
                    //RAVEPRINT(L"finger is still inside object, keep moving out i = %d  j = %d ffj = %d\n",i,j,firstfreej);
                    
                }
                else if(!bbodyfree && bworldfree && outcol)
                {
                    //RAVEPRINT(L"finger has reached the end of concavity, move back half way i = %d  j = %d ffj = %d\n",i,j,firstfreej);
                    vcurrent[i] += ((dReal)j - (dReal)firstfreej)/2.0f * closingdir[i]*stepsize;
                    break;
                }
                else if(bbodyfree && !bworldfree)
                {
                    //RAVEPRINT(L"finger collided with world, move back half way i = %d  j = %d ffj = %d\n",i,j,firstfreej);
                    vcurrent[i] += ((dReal)j - (dReal)firstfreej)/2.0f * closingdir[i]*stepsize;
                    break;
                }
                else if(!bbodyfree && !bworldfree)
                {
                    //RAVEPRINT(L"finger is wedged between object and obstacle, stop moving i = %d  j = %d ffj = %d\n",i,j,firstfreej);        
                    break;
                }
                
                vcurrent[i] -= closingdir[i]*stepsize;
                UpdateDependentJoints(robot,i, &vcurrent[0]);
                robot->SetActiveDOFValues(NULL,&vcurrent[0]);
                if(!(((closingdir[i] == 1.0f) && (vcurrent[i] > vlowerlimit[i])) || ((closingdir[i] == -1.0f) && (vcurrent[i] < vupperlimit[i]))))
                {
                    //RAVEPRINT(L"joint limit reached i = %d  j = %d ffj = %d dir: %f lowerlim: %f upperlim: %f\n",i,j, firstfreej, closingdir[i], vlowerlimit[i], vupperlimit[i]);
                    vcurrent[i] += ((dReal)j - (dReal)firstfreej)/2.0f * closingdir[i]*stepsize;
                    break;
                }
            }
            UpdateDependentJoints(robot,i, &vcurrent[0]);
            robot->SetActiveDOFValues(NULL,&vcurrent[0]);
        }
        

        Trajectory* ptraj = GetEnv()->CreateTrajectory((int)robot->GetActiveDOF());

        Trajectory::TPOINT pt;
        //pt.q = vstart;
        //ptraj->AddPoint(pt);
        pt.q = vcurrent;
        ptraj->AddPoint(pt);
        
        ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
        robot->SetActiveMotion(ptraj);
        delete ptraj;
        
        GetEnv()->SetCollisionOptions(0);
    }
    
    if(bMoveOut) {

        if(!bHasDirection) {
            RAVEPRINT(L"Error: Need to specify direction to move along!\n");
            return false;
        }
        if(pbody == NULL) {
            RAVEPRINT(L"Error: Need to specify body!\n");
            return false;
        }
        
        if(GetEnv()->CheckCollision(robot,pbody)) {
            //backup the robot until it is no longer colliding with the object
            Transform Ti;
            Ti = robot->GetTransform();

            //GetEnv()->plot3(Ti.trans, 1, 0, 0.004f, Vector(0,1,0) );
            dReal step_size = 0.015f;
            while(1) {
                robot->SetTransform(Ti);
                if(!GetEnv()->CheckCollision(robot))
                    break;
                Ti.trans = Ti.trans + step_size*direction; //note: moves positively along direction
            }               
            //move back into collision
            step_size = step_size/15.0f;
            while(1) {
                robot->SetTransform(Ti);
                if(GetEnv()->CheckCollision(robot,pbody))
                    break;
                Ti.trans = Ti.trans - step_size*direction; 
            }
        }

    }

    if( !bExecutePlanner )
        return true;

    // execute the planner
    if( robot == NULL ) {
        RAVELOG(L"Error: need to specify robot\n");
        return false;
    }
    
    if( pbody == NULL ) {
        RAVELOG(L"Error: need to specify body\n");
        return false;
    }

    robot->Enable(true);

    Vector graspcenter = pbody->GetCenterOfMass() + centeroffset;
    //GetEnv()->plot3(graspcenter, 1, sizeof(Vector), 0.01f);
    
    Tstart.trans = Tstart.trans + graspcenter;

    if( bTransRobot ) {
//        if( bCenterOnManipulator && robot->GetActiveManipulator() != NULL && robot->GetActiveManipulator()->pEndEffector != NULL ) {
//            Transform tee = robot->GetActiveManipulator()->GetEndEffectorTransform();
//            Tstart.rot = tee.rot;
//            Tstart.trans -= (tee.trans-robot->GetTransform().trans);
//        }
//        else
        Tstart.rot = robot->GetTransform().rot;

        robot->SetTransform(Tstart);
    }

    std::vector<Transform > vtrans;
    std::vector<KinBody *> vecbodies;

    bool bremove_obstacles = false;
    if(bremove_obstacles)
    {   
        GetEnv()->GetBodies(vecbodies); 
        for(size_t i = 0; i < vecbodies.size(); i++)
            if( vecbodies[i] != pbody && vecbodies[i] != robot)
                GetEnv()->RemoveKinBody(vecbodies[i]);

        pbody->SetTransform(Transform());
    }

    if( !bHasDirection ) {
        //RAVELOGA("Grasper ERROR: Invalid approach direction, setting direction to (0,0,1)\n");
        direction = Vector(0,0,1);
        /*
        direction = pbody->GetCenterOfMass() - robot->GetLinks().front()->GetCentroid();// + Vector(0,0.1,0);
        if( lengthsqr3(direction) < 0.001 )
        {
        direction = Vector(0,0,1);
            RAVEPRINT(L"Grasper ERROR: Invalid approach direction, setting direction to (0,0,1)\n");
        }
        else
            direction.normalize();
        */
    }

    if(randomize)
    {
        srand(random_seed);
        GetEnv()->GetBodies(vecbodies); 

        dReal variance;

        do
        {
            srand(random_seed);
            for(int i = 0; i < (int)vecbodies.size(); i++)
            {
                if( vecbodies[i] != pbody && vecbodies[i] != robot)
                {
                    if(i <= 8)
                        variance = 0.2f;
                    else
                        variance = 1.0f;

                    Transform Trand;
                    Trand.trans.x = variance*(2*RANDOM_FLOAT()-1);
                    Trand.trans.y = variance*(2*RANDOM_FLOAT()-1);
                    Trand.trans.z = variance*(2*RANDOM_FLOAT()-1);
                    Trand.rot = GetRandomQuat();
                    vecbodies[i]->GetBodyTransformations(vtrans);
            
                    Trand = vtrans[0]*Trand;

                    vecbodies[i]->SetTransform(Trand);
                }

            }
        }while(GetEnv()->CheckCollision(pbody));
    }

    
    

    
    params.vParameters.resize(0);
    params.vParameters.push_back(standoff); ///start closing fingers when at this distance
    
    dReal bface_target = 0.0f; ///point the hand at the target or not (1 = yes, else no)
    params.vParameters.push_back(bface_target);

    dReal roll_hand = 0.0f; /// rotate the hand about the palm normal by this many radians
    params.vParameters.push_back(roll_hand);

    params.vParameters.push_back(direction.x);
    params.vParameters.push_back(direction.y);
    params.vParameters.push_back(direction.z);

    if( bTransRobot ) {

        // There are some complications with the 0 angle of the roll
        // In order to make things consistent even when the body rotates, have to 
        // first take the direction with respect to the identity body transformation
        Transform tBodyTrans = pbody->GetTransform();
        tBodyTrans.trans = Vector(0,0,0);
        Vector vnewdir = tBodyTrans.inverse().rotate(direction);
        
        // set the robot so it always points in the direction of the object
        Vector vup = Vector(0,1,0);
        vup = vup - vnewdir  * dot3(vnewdir, vup);
        if( vup.y < 0.01 ) {
            vup.x = 1;
            vup = vup - vnewdir * dot3(vnewdir, vup);
        }
        normalize3(vup, vup);
        
        Vector vright; cross3(vright, vup, vnewdir);
        
        TransformMatrix mtrans;
        mtrans.m[0] = vright.x;     mtrans.m[1] = vup.x;     mtrans.m[2] = vnewdir.x;
        mtrans.m[4] = vright.y;     mtrans.m[5] = vup.y;     mtrans.m[6] = vnewdir.y;
        mtrans.m[8] = vright.z;     mtrans.m[9] = vup.z;     mtrans.m[10] = vnewdir.z;
        mtrans.trans = Tstart.trans;//robot->GetLinks().front()->GetCentroid();

        if( bCenterOnManipulator && robot->GetActiveManipulator() != NULL ) {
            Transform t = robot->GetActiveManipulator()->tGrasp.inverse();
            mtrans = mtrans * robot->GetActiveManipulator()->tGrasp.inverse();
            //RAVELOG_INFOA("center manip: %f %f %f\n",t.trans.x,t.trans.y,t.trans.z);
        }
        
        Transform Tfinal(mtrans);
        Transform newrot;
        newrot.rotfromaxisangle(Vector(0,0,1),handroll);

        // rotate back to world by tBodyTrans
        Tfinal = tBodyTrans*Tfinal*newrot;
        
        // set the robot so that its palm is facing normalplane
        // find the closest rotation
        Vector vrot;
        cross3(vrot, normalplane, Vector(0,0,1));

        dReal fsin = sqrtf(lengthsqr3(vrot));
        dReal fcos = dot3(normalplane, Vector(0,0,1));

        if( fsin > 0.001f ) {
            vrot /= fsin;

            Transform talign;
            talign.rotfromaxisangle(vrot, RaveAtan2(fsin, fcos));
            Tfinal = Tfinal * talign;
        }
        else if( fcos < 0 ) {
            // hand is flipped 180, rotate around x axis
            vrot = Vector(1,0,0);
            //vrot -= vnewdir * dot3(vnewdir, vrot); // not necessary?
            vrot -= normalplane * dot3(normalplane, vrot);
            normalize3(vrot, vrot);

            Transform talign;
            talign.rotfromaxisangle(vrot, RaveAtan2(fsin, fcos));
            Tfinal = Tfinal * talign;
        }

        robot->SetTransform(Tfinal);    

        //backup the robot until it is no longer colliding with the object
        Transform Ti;
        Ti.trans = graspcenter;
        //GetEnv()->plot3(Ti.trans, 1, 0, 0.004f, Vector(0,1,0) );
        Ti.rot = Tfinal.rot;
        dReal step_size = 0.05f;
        while(1) {
            robot->SetTransform(Ti);
            if(!GetEnv()->CheckCollision(robot,pbody))
                break;
            Ti.trans = Ti.trans - step_size*direction;
        }
    }
    
    if(bNoise) {    
        Transform Trand;
        Trand.trans.x = transnoise*(2.0f*RANDOM_FLOAT()-1.0f);
        Trand.trans.y = transnoise*(2.0f*RANDOM_FLOAT()-1.0f);
        Trand.trans.z = transnoise*(2.0f*RANDOM_FLOAT()-1.0f);
        if(rotnoise > 0)
        {
            //this is too slow
//            Trand.rot = GetRandomQuat();
//            while( 2*atan2(sqrt(Trand.rot.w*Trand.rot.w + Trand.rot.y*Trand.rot.y + Trand.rot.z*Trand.rot.z),Trand.rot.x) > rotnoise)
//            {
//                RAVEPRINT(L"Trying Quat\n");
//                Trand.rot = GetRandomQuat();
//            }
            Vector rand;
            //get random axis
            while(1) {
                rand.x = 2.0f*RANDOM_FLOAT()-1.0f;
                rand.y = 2.0f*RANDOM_FLOAT()-1.0f;
                rand.z = 2.0f*RANDOM_FLOAT()-1.0f;
                if( sqrt(rand.lengthsqr3()) <= 1.0f)
                    break;
            }
            
            
            Trand.rotfromaxisangle(rand,(dReal)RANDOM_FLOAT(rotnoise));
        }
        else
            RAVELOGA("Rot Noise below threshold, no rotation noise added\n");
        
        pbody->ApplyTransform(Trand);
    }

    params.vinitialconfig.resize(robot->GetActiveDOF());
    robot->GetActiveDOFValues(&params.vinitialconfig[0]);  ///get starting configuration of hand

    params.vParameters.push_back(normalplane.x);
    params.vParameters.push_back(normalplane.y);
    params.vParameters.push_back(normalplane.z);
    
    //planner = GetEnv()->CreatePlanner(L"Grasper");
    if( planner != NULL )
        RAVELOG(L"grasping planner created!\n");

    if( !planner->InitPlan(robot, &params) ) {
        RAVELOG(L"InitPlan failed\n");
        return false;
    }

    Trajectory* ptraj = GetEnv()->CreateTrajectory(robot->GetActiveDOF());
    if( !planner->PlanPath(ptraj) || ptraj->GetPoints().size() == 0 ) {
        RAVELOG(L"PlanPath failed\n");
        return false;
    }

    if( pFilename != NULL )
        ptraj->Write(pFilename, 0);

    std::stringstream colfile;

    std::set<KinBody *> vbodyexcluded;
    std::set<KinBody::Link *> vlinkexcluded;
    vbodyexcluded.insert(pbody);

    Transform transBodyOriginal = pbody->GetTransform();

    //make sure the robot isn't colliding with anything except pbody
    COLLISIONREPORT report, *preport=NULL;
    if(  RaveGetDebugLevel() )
         preport = &report;

    robot->SetActiveDOFValues(NULL,&ptraj->GetPoints().back().q[0]);
    
    if( bNoise ) {
        pbody->SetTransform(transBodyOriginal);
    }

    bool get_contacts = !GetEnv()->CheckCollision(robot,vbodyexcluded,vlinkexcluded,&report) && !robot->CheckSelfCollision();

    if( get_contacts ) 
    {
        if(bcheck_stability)
            GetStableContacts(&colfile, pbody,direction, mu);
        else
        {
            // calculate the contact normals
            GetEnv()->SetCollisionOptions(CO_Contacts);
            COLLISIONREPORT report;
            vector<KinBody::Link*>::const_iterator itlink;
            int icont=0;
            FORIT(itlink, robot->GetLinks()) {
                
                if( GetEnv()->CheckCollision(*itlink, pbody, &report) ) {
                    RAVELOG_DEBUGA("contact %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());

                    for(size_t i = 0; i < report.contacts.size(); i++) {
                        Vector pos = report.contacts[i].pos;
                        Vector norm = report.contacts[i].norm;
                        if( report.plink1 != *itlink )
                            norm = -norm;
                        colfile << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z <<" ";
                        if(bGetLinkCollisions)
                            colfile << (*itlink)->GetIndex();
                        colfile << endl;
                        icont++;
                        //GetEnv()->plot3(pos, 1, 0);
                    }
                }
            }
            RAVELOG(L"number of contacts: %d\n", icont);
            GetEnv()->SetCollisionOptions(0);
        }
    }
    else if( RaveGetDebugLevel() ) {

        if( report.plink1 != NULL && report.plink2 != NULL )
            RAVELOG(L"collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
    }

    response = colfile.str();

    ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
    robot->SetActiveMotion(ptraj);

    delete ptraj;
    return true;
}


void GrasperProblem::SampleObject(KinBody* pbody, vector<GrasperProblem::SURFACEPOINT>& vpoints, int N, Vector graspcenter)
{
    RAY r;
    Vector com = graspcenter;
    COLLISIONREPORT report;

    GetEnv()->SetCollisionOptions(CO_Contacts|CO_Distance);

    vpoints.resize(N);
    int i = 0;

    while(i < N) {
        r.dir.z = 2*RANDOM_FLOAT()-1;
        dReal R = sqrtf(1 - r.dir.x * r.dir.x);
        dReal U2 = 2 * PI * RANDOM_FLOAT();
        r.dir.x = R * cos(U2);
        r.dir.y = R * sin(U2);

        r.pos = com - 10.0f*r.dir;
        r.dir *= 1000;

        if( GetEnv()->CheckCollision(r, pbody, &report) ) {
            vpoints[i].norm = report.contacts.front().norm;
            vpoints[i].pos = report.contacts.front().pos + 0.001f * vpoints[i].norm; // extrude a little
            vpoints[i].dist = 0;
            i++;
        }
    }

    GetEnv()->SetCollisionOptions(0);
}

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */ \
  (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */ \
  (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

// generate a sphere triangulation starting with an icosahedron
// all triangles are oriented counter clockwise
void GenerateSphereTriangulation(KinBody::Link::TRIMESH& tri, int levels)
{
    KinBody::Link::TRIMESH temp, temp2;

    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));

    int indices[] = {
        0, 1, 2,
        1, 3, 4,
        3, 5, 6,
        2, 4, 7,
        5, 6, 8,
        2, 7, 9,
        0, 5, 8,
        7, 9, 10,
        0, 1, 5,
        7, 10, 11,
        1, 3, 5,
        6, 10, 11,
        3, 6, 11,
        9, 10, 8,
        3, 4, 11,
        6, 8, 10,
        4, 7, 11,
        1, 2, 4,
        0, 8, 9,
        0, 2, 9
    };

    Vector v1, v2, v3, v12, v13, v23;
    
    // make sure oriented CCCW 
    for(size_t i = 0; i < ARRAYSIZE(indices); i += 3 ) {
        v1 = temp.vertices[indices[i]];
        v2 = temp.vertices[indices[i+1]];
        v3 = temp.vertices[indices[i+2]];
        if( dot3(v1, cross3(v12, v2-v1, v3-v1)) < 0 )
            swap(indices[i], indices[i+1]);
    }

    temp.indices.resize(ARRAYSIZE(indices));
    memcpy(&temp.indices[0], indices, sizeof(indices));

    KinBody::Link::TRIMESH* pcur = &temp;
    KinBody::Link::TRIMESH* pnew = &temp2;
    while(levels-- > 0) {

        pnew->vertices.resize(0);
        pnew->vertices.resize(2*pcur->indices.size());
        pnew->indices.resize(0);
        pnew->indices.reserve(4*pcur->indices.size());
        for(int i = 0; i < (int)pcur->indices.size(); i += 3) {
            // for ever tri, create 3 new vertices and 4 new triangles.
            v1 = pcur->vertices[pcur->indices[i]];
            v2 = pcur->vertices[pcur->indices[i+1]];
            v3 = pcur->vertices[pcur->indices[i+2]];
            normalize3(v12, v1+v2);
            normalize3(v13, v1+v3);
            normalize3(v23, v2+v3);

            int offset = (int)pnew->vertices.size();

            pnew->vertices.push_back(v1);
            pnew->vertices.push_back(v2);
            pnew->vertices.push_back(v3);
            pnew->vertices.push_back(v12);
            pnew->vertices.push_back(v13);
            pnew->vertices.push_back(v23);
            pnew->indices.push_back(offset);    pnew->indices.push_back(offset+3);    pnew->indices.push_back(offset+4);
            pnew->indices.push_back(offset+3);    pnew->indices.push_back(offset+1);    pnew->indices.push_back(offset+5);
            pnew->indices.push_back(offset+4);    pnew->indices.push_back(offset+3);    pnew->indices.push_back(offset+5);
            pnew->indices.push_back(offset+4);    pnew->indices.push_back(offset+5);    pnew->indices.push_back(offset+2);
        }

        swap(pnew,pcur);
    }

    tri = *pcur;
}

// generates samples of the object surface using a sampling box, num samples must be a multiple of 12
void GrasperProblem::BoxSample(KinBody* pbody, vector<SURFACEPOINT>& vpoints, int num_samples, Vector center)
{
    RAY r;
    COLLISIONREPORT report;
    KinBody::Link::TRIMESH tri;
    SURFACEPOINT p;
    //Vector com = graspcenter;
    //GenerateSphereTriangulation(tri,levels);
    dReal ffar = 1.0f;

    GetEnv()->SetCollisionOptions(CO_Contacts|CO_Distance);

    vpoints.reserve(num_samples);

    /*
    if(num_samples%12)
    {
        RAVEPRINT(L"GrasperProblem Error - Num samples must be a multiple of 12/n");
        return;
    }
    */

    dReal counter = ffar/sqrt((dReal)num_samples/12);
    for(int k = 0; k < 6; k++)
        for(dReal i = -ffar/2.0f; i < ffar/2.0f; i+=counter)
            for(dReal j = -ffar/2.0f; j < ffar/2.0f; j+=counter)
            {
                switch(k){
                    case 0:
                        r.pos = Vector(center.x-ffar,center.y+i,center.z+j);
                        r.dir = Vector(1000,0,0);
                        break;
                    case 1:
                        r.pos = Vector(center.x+ffar,center.y+i,center.z+j);
                        r.dir = Vector(-1000,0,0);
                        break;
                    case 2:
                        r.pos = Vector(center.x+i,center.y-ffar,center.z+j);
                        r.dir = Vector(0,1000,0);
                        break;
                    case 3:
                        r.pos = Vector(center.x+i,center.y+ffar,center.z+j);
                        r.dir = Vector(0,-1000,0);
                        break;
                    case 4:
                        r.pos = Vector(center.x+i,center.y+j,center.z-ffar);
                        r.dir = Vector(0,0,1000);
                        break;
                    case 5:
                        r.pos = Vector(center.x+i,center.y+j,center.z+ffar);
                        r.dir = Vector(0,0,-1000);
                        break;


                }
                
                if( GetEnv()->CheckCollision(r, pbody, &report) ) {
                    p.norm = -report.contacts.front().norm;//-r.dir//report.contacts.front().norm1;
                    p.pos = report.contacts.front().pos;// + 0.001f * p.norm; // extrude a little
                    p.dist = 0;
                    vpoints.push_back(p);
                }
                //GetEnv()->plot3(r.pos, 1, 0, 0.004f);
            }

   

/*
    for(int i = 0; i < (int)tri.indices.size(); i += 3) {
        r.dir = 0.33333f * (tri.vertices[tri.indices[i]] + tri.vertices[tri.indices[i+1]] + tri.vertices[tri.indices[i+2]]);
        normalize3(r.dir, r.dir);
        r.dir *= 1000;
        r.pos = com - 10*r.dir;
        POINT p;
        if( GetEnv()->CheckCollision(r, pbody, &report) ) {
            p.norm = -report.contacts.front().norm;//-r.dir//report.contacts.front().norm1;
            p.pos = report.contacts.front().pos + 0.001f * p.norm; // extrude a little
            p.dist = 0;
            vpoints.push_back(p);
        }
    }
*/
    GetEnv()->SetCollisionOptions(0);
}


// generates samples across a geodesic sphere (the higher the level, the higher the number of points
void GrasperProblem::DeterministicallySample(KinBody* pbody, vector<SURFACEPOINT>& vpoints, int levels, Vector graspcenter)
{
    RAY r;
    COLLISIONREPORT report;
    KinBody::Link::TRIMESH tri;
    Vector com = graspcenter;
    GenerateSphereTriangulation(tri,levels);

    GetEnv()->SetCollisionOptions(CO_Contacts|CO_Distance);

    // take the mean across every tri
    vpoints.reserve(tri.indices.size()/3);
    for(int i = 0; i < (int)tri.indices.size(); i += 3) {
        r.dir = 0.33333f * (tri.vertices[tri.indices[i]] + tri.vertices[tri.indices[i+1]] + tri.vertices[tri.indices[i+2]]);
        normalize3(r.dir, r.dir);
        r.dir *= 1000;
        
        r.pos = com - 10.0f*r.dir;
        SURFACEPOINT p;
        if( GetEnv()->CheckCollision(r, pbody, &report) ) {
            p.norm = -report.contacts.front().norm;//-r.dir//report.contacts.front().norm1;
            p.pos = report.contacts.front().pos + 0.001f * p.norm; // extrude a little
            p.dist = 0;
            vpoints.push_back(p);
        }
    }

    GetEnv()->SetCollisionOptions(0);
}


void GrasperProblem::ComputeMultiResMap(vector<KinBody*>& vbodies, KinBody* pbody, vector<SURFACEPOINT>& vpoints, dReal stepsize,dReal maxradius)
{
    
    Vector orthvec1;

    //sweap out circles of increasing radius
    int numsteps = (int)ceil(maxradius/stepsize);
    int numangles = 24;
    int _numangles;
    Transform tm;

    RAY r;
    COLLISIONREPORT report;
    dReal maxfreeradius = maxradius;
    dReal fMinDist = 2.0f;
    dReal thispointdist = 2.0f;

    dReal testnorm;
    Vector pointpos;

    vector<KinBody*> vbodies_nopbody;
    vbodies_nopbody = vbodies;

    for(size_t i = 0; i < vbodies_nopbody.size(); ++i) {
        if( vbodies_nopbody[i] == pbody ) {
            vbodies_nopbody.erase(vbodies_nopbody.begin()+i);
            break;
        }
    }

    RAVEPRINT(L"Done computing distmap\n");

    for(size_t i = 0;i < vpoints.size();i++) {
        pointpos = vpoints[i].pos + 0.001f*vpoints[i].norm;
        //if(i % 20 == 0) GetEnv()->drawarrow(vpoints[i].pos,vpoints[i].pos+0.03*vpoints[i].norm);

        testnorm = sqrt(lengthsqr3(vpoints[i].norm));
        if(testnorm < 0.9999 || testnorm > 1.0001)
            RAVEPRINT(L"Error, point %d has non-unit norm (%f)\n",i,testnorm);


        //get some point on the plane by rotating the normal by 90
        orthvec1 = Vector(1,0,0);
        if( fabsf(vpoints[i].norm.x) > 0.9 ) orthvec1.y = 1;
        orthvec1 -= vpoints[i].norm * dot3(orthvec1,vpoints[i].norm);




        //if(i % 20 == 0) GetEnv()->drawarrow(vpoints[i].pos,vpoints[i].pos+0.03*orthvec1,0.003f,Vector(0,0,1));

        fMinDist = 2.0f;
        thispointdist = 2.0f;

        //to prevent undesirable results if the point starts slightly inside another object
        //check on first iteration if circle collides, then change maxfreeradius
        //do not consider pbody in this test
        tm.trans.x = tm.trans.y = tm.trans.z = 0;            
        for(int k = 0; k < numangles; k++)
        {
            thispointdist = 2.0f;
            tm.rotfromaxisangle(vpoints[i].norm,(dReal)(k*2*PI/numangles));
                    
            r.pos = pointpos;
            r.dir = 1000.0*(tm*(orthvec1));
            
            for(int b = 0; b < (int)vbodies_nopbody.size(); ++b) {
                if( GetEnv()->CheckCollision(r, vbodies_nopbody[b], &report) ) {
                    if( report.minDistance < fMinDist )
                        fMinDist = report.minDistance;
                    if( report.minDistance < thispointdist )
                        thispointdist = report.minDistance;
                }
            }

            //if(i % 20 == 0) GetEnv()->plot3(r.pos+tm*(0.001f*orthvec1), 1, 0, 0.004f, Vector(thispointdist,0,0) );
        }
        maxfreeradius = fMinDist;

        //sweap out circles of increasing radius
        for(int j = 0; j < numsteps; j++)
        {
            //check if outside of maxfreeradius, if you are, fill rest of point.dists vector with 0s
            if(j*stepsize >= maxfreeradius)
            {
                vpoints[i].dists.push_back(0);
                continue;
            }


            if(j == 0)
                _numangles = 1;
            else
                _numangles = numangles;

            fMinDist = 2.0f;
            for(int k = 0; k < _numangles; k++)
            {

                thispointdist = 2.0f;
                
                tm.rotfromaxisangle(vpoints[i].norm,(dReal)(k*2*PI/_numangles));
                r.pos = tm*(j*stepsize*orthvec1) + pointpos;
                r.dir = 1000.0f*vpoints[i].norm;
            

                for(int b = 0; b < (int)vbodies.size(); ++b) {
                    if( GetEnv()->CheckCollision(r, vbodies[b], &report) ) {

                        //special case if ray collides with pbody, to ignore small curvatures:
                        //if ray distance is small, ignore collision
                        //NOTE: this will not work if point is in a very narrow pocket
                        if( vbodies[b] == pbody && report.minDistance < (numsteps-j)*0.001f)
                            continue;

                        if( report.minDistance < fMinDist )
                            fMinDist = report.minDistance;
                        if( report.minDistance < thispointdist )
                            thispointdist = report.minDistance;
                    }
                }
                //if(i % 20 == 0) GetEnv()->plot3(r.pos, 1, 0, 0.004f, Vector(thispointdist,0,0) );
            }

            vpoints[i].dists.push_back(fMinDist);
        }
    }

}

void GrasperProblem::ComputeDistanceMap(const vector<KinBody*>& vbodies, vector<SURFACEPOINT>& vpoints, dReal fTheta)
{
    dReal fCosTheta = cosf(fTheta);
    int N;
    if(fTheta < 0.01)
        N = 1;
    
    RAY r;
    COLLISIONREPORT report;

    GetEnv()->SetCollisionOptions(CO_Distance);

    // set number of rays to randomly sample
    if( fTheta < 1e-2 )
        N = 1;
    else
        N = (int)ceil(fTheta * (64.0f/(PI/12.0f))); // sample 64 points when at pi/12
    for(int i = 0; i < (int)vpoints.size(); ++i) {

        Vector vright = Vector(1,0,0);
        if( fabsf(vpoints[i].norm.x) > 0.9 ) vright.y = 1;
        vright -= vpoints[i].norm * dot3(vright,vpoints[i].norm);
        normalize3(vright,vright);
        Vector vup;
        cross3(vup, vpoints[i].norm, vright);

        dReal fMinDist = 2;
        for(int j = 0; j < N; ++j) {
            // sample around a cone
            dReal fAng = fCosTheta + (1-fCosTheta)*RANDOM_FLOAT();
            dReal R = sqrtf(1 - fAng * fAng);
            dReal U2 = 2 * PI * RANDOM_FLOAT();
            r.dir = 1000.0f*(fAng * vpoints[i].norm + R * RaveCos(U2) * vright + R * RaveSin(U2) * vup);

            r.pos = vpoints[i].pos;

            for(int k = 0; k < (int)vbodies.size(); ++k) {
                if( GetEnv()->CheckCollision(r, vbodies[k], &report) ) {
                    if( report.minDistance < fMinDist )
                        fMinDist = report.minDistance;
                }
            }
        }

        vpoints[i].dist = fMinDist;
    }

    GetEnv()->SetCollisionOptions(0);
}

void GrasperProblem::GetStableContacts(std::stringstream* colfile, KinBody* ptarget,Vector& direction, dReal mu)
{
    RAVELOG(L"Starting GetStableContacts...\n");
    wstringstream s; 

    bool bdraw = false;

    if(ptarget == NULL)
    {
        RAVEPRINT(L"GrasperProblem::GetStableContacts - Error: Target not specified.\n");
        return;
    }

    if(!GetEnv()->CheckCollision(robot,ptarget))
    {
        RAVEPRINT(L"GrasperProblem::GetStableContacts - Error: Robot is not colliding with the target.\n");
        return;
    }

    if(mu < 0)
    {
        RAVEPRINT(L"GrasperProblem::GetStableContacts - Error: Friction coefficient is invalid.\n");
        return;
    }

    RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();

    if( pmanip == NULL ) {
        RAVEPRINT(L"GetStableContacts, no active manipulator\n");
        return;
    }

    int ibaselink = 0;
    if(pmanip != NULL && pmanip->pBase != NULL)
    {
        ibaselink = pmanip->pBase->GetIndex();
    }
    RAVELOG(L"Using link %d as base link.\n",ibaselink);




    vector<dReal> closingdir;

    //make sure we get the right closing direction and don't look at irrelevant joints
    bool handjoint;
    for(int i = 0; i < robot->GetDOF(); ++i) 
    {
        handjoint = false;
        for(size_t j = 0; j < pmanip->_vecjoints.size(); j++) 
        {
            if(i == pmanip->_vecjoints[j]) 
            {
                handjoint = true;
                if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
                    closingdir.push_back(-1.0f);
                else
                    closingdir.push_back(1.0f);

                break;
            }
        }
        
        if(!handjoint)
            closingdir.push_back(0);
    }

    s.str() = L"closing dirs: ";
    for(size_t q = 0; q < closingdir.size(); q++)
        s << closingdir[q] << " ";
    s << endl;
    RAVELOG(s.str().c_str());

    // calculate the contact normals using the Jacobian
    int numdofs = robot->GetDOF();
    std::vector<dReal> J(3*numdofs);
    Vector deltaxyz;
    dReal temp;
    Vector vnormalpart;
    Vector vtangentpart;
   

    
    COLLISIONREPORT report;
    std::vector<KinBody::Link*> vbodies = robot->GetLinks();
    
    for(int ilink = 0; ilink < (int)vbodies.size(); ilink++) {
        if( GetEnv()->CheckCollision(vbodies[ilink],ptarget, &report) )  {
         
            RAVELOG(L"contact %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());

            Transform linkTm = vbodies[ilink]->GetTransform();
            //s.str() = L"Link Tm: ";
            //s << linkTm.trans.x << " " << linkTm.trans.y << " " << linkTm.trans.z << " " << linkTm.rot.x << " " << linkTm.rot.y << " " << linkTm.rot.z << " " << linkTm.rot.w << " ";
            //s << endl;
            //RAVEPRINT(s.str().c_str());


            Transform pointTm;
            int icont=0;
            
            for(size_t i = 0; i < report.contacts.size(); i++) {  
                icont++;
               
                report.contacts[i].norm = -report.contacts[i].norm;
                //check if this link is the base link, if so there will be no Jacobian
                if(ilink == ibaselink)  {
                    deltaxyz = direction;
                }
                else {   
                    //calculate the jacobian for the contact point as if were part of the link
                    pointTm.trans = report.contacts[i].pos;
                    //pointTm.trans = pointTm.trans - linkTm.trans; //+ robot->GetTransform().trans;
                    //pointTm = pointTm*linkTm.inverse();
                    //s.str() = L"Point Tm: ";
                    //s << pointTm.trans.x << " " << pointTm.trans.y << " " << pointTm.trans.z << " " << pointTm.rot.x << " " << pointTm.rot.y << " " << pointTm.rot.z << " " << pointTm.rot.w << " ";
                    //s << endl;
                    //RAVEPRINT(s.str().c_str());
                    

                    
                    //report.contacts[i].pos.y +=0.5f;
                    //robot->SetTransform(temp2);
                    
                    //memset(&J[0], 0, sizeof(dReal)*J.size());
                    //robot->CalculateJacobian(vbodies[ilink]->GetIndex(), temppointtm.trans, &J[0]);
                    //GetEnv()->plot3(temppointtm.trans, 1, 0, 0.004f, Vector(0,0,0) );

                    //s.clear();
                    //s << L"Jacobian 1: ";
                    //for(int q = 0; q < J.size(); q++)
                    //        s << J[q] << " ";
                    //s << endl;
                    //RAVEPRINT(s.str().c_str());                   

                    //robot->SetTransform(temp1);

                    memset(&J[0], 0, sizeof(dReal)*J.size());
                    robot->CalculateJacobian(vbodies[ilink]->GetIndex(), pointTm.trans, &J[0]);
                    //GetEnv()->plot3(pointTm.trans, 1, 0, 0.004f, Vector(0.5,0.5,0.5) );
                    
                    //s.str() = L"Jacobian 2: ";
                    //for(int q = 0; q < J.size(); q++)
                    //        s << J[q] << " ";
                    //s << endl;
                    //RAVEPRINT(s.str().c_str());

                    //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                    for(int j = 0; j < 3; j++) {   
                        temp = 0;
                        for(int k = 0; k < numdofs; k++)
                            temp += J[j*numdofs + k]*0.01f*closingdir[k];
                                                    
                        if( j == 0)
                            deltaxyz.x  = temp;
                        else if( j == 1)
                            deltaxyz.y = temp;
                        else if( j == 2)
                            deltaxyz.z = temp;
                    }
                }
                
                //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                //so treat it as if it were part of the base link
                if(lengthsqr3(deltaxyz) < 0.000000001f)
                    deltaxyz = direction;
                
                normalize3(deltaxyz,deltaxyz);

                s.str().erase();
                s << L"link " << ilink << " delta XYZ: ";
                for(int q = 0; q < 3; q++)
                    s << deltaxyz[q] << " ";
                s << endl;
                RAVELOG(s.str().c_str());

                RAVELOG(L"number of contacts: %d\n", icont);

                //determine if contact is stable
                bool bstable = true;
                //if angle is obtuse, can't be in friction cone
                if (RaveAcos(dot3(report.contacts[i].norm,deltaxyz)) > PI/2.0f)
                    bstable = false;
                else {
                    vnormalpart = dot3(report.contacts[i].norm,deltaxyz)*report.contacts[i].norm;
                    vtangentpart = deltaxyz - vnormalpart;
                    //check if tangent force is outside friction cone
                    if( mu*sqrt(lengthsqr3(vnormalpart)) < sqrt(lengthsqr3(vtangentpart)) )
                        bstable = false;
  
                }


                if(bdraw) {
                    if(bstable)
                        GetEnv()->plot3( RaveVector<float>(report.contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(0,1,0) );
                    else
                        GetEnv()->plot3( RaveVector<float>(report.contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );

                    GetEnv()->plot3(RaveVector<float>(report.contacts[i].pos + 0.02f*report.contacts[i].norm), 1, 0, 0.004f, RaveVector<float>(0,0,1) );
                    GetEnv()->plot3(RaveVector<float>(report.contacts[i].pos + 0.02f*deltaxyz), 1, 0, 0.004f, RaveVector<float>(1,1,0) );
                }
                
                if(bstable)
                    *colfile << report.contacts[i].pos.x <<"\t" << report.contacts[i].pos.y <<"\t" << report.contacts[i].pos.z <<"\t" << report.contacts[i].norm.x <<"\t" << report.contacts[i].norm.y <<"\t" << report.contacts[i].norm.z << endl;
            }
        }
    }
}

