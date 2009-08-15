// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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

// Functions to plan with caging grasps. See
// Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
// Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.
#include "plugindefs.h"

#include "taskcagingproblem.h"

#include <queue>

//////////////////
// Task Problem //
//////////////////
TaskCagingProblem::TaskCagingProblem(EnvironmentBase* penv) : CmdProblemInstance(penv), robot(NULL)
{
    RegisterCommand("graspset",(CommandFn)&TaskCagingProblem::CreateGraspSet,
                    "Creates a caging grasp set\n"
                    "Options: step exploreprob size target targetjoint contactconfigdelta cagedconfig");
    RegisterCommand("taskconstraintplan", (CommandFn)&TaskCagingProblem::TaskConstrainedPlanner,
                    "Invokes the relaxed task constrained planner");
    RegisterCommand("simpleconstraintplan", (CommandFn)&TaskCagingProblem::SimpleConstrainedPlanner,
                    "Invokes a simple one grasp planner");
    RegisterCommand("bodytraj",(CommandFn)&TaskCagingProblem::BodyTrajectory,
                    "Starts a body to follow a trajectory. The trajrectory must contain timestamps\n"
                    "Options: target targettraj");
    RegisterCommand("help", (CommandFn)&TaskCagingProblem::Help,"Help message");
}

void TaskCagingProblem::Destroy()
{
    robot = NULL;
}

TaskCagingProblem::~TaskCagingProblem()
{
    Destroy();
}

/// cmd format:
/// robot_name
/// 
/// robot_name is the name of the robot that will be controlled by this problem
/// Can add more than one sensor systems by specifying 'sensor' then the above parameters.
int TaskCagingProblem::main(const char* cmd)
{
    if( cmd == NULL )
        return 0;

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = _ravembstowcs(p);
    
    SetActiveRobots(GetEnv()->GetRobots());

    return 0;
}

bool TaskCagingProblem::SimulationStep(dReal fElapsedTime)
{
    FOREACH_NOINC(itbody, _listBodyTrajs) {
        Trajectory::TPOINT tp;
        itbody->ptraj->SampleTrajectory(itbody->time, tp);

        assert( (int)tp.q.size() == itbody->ptarget->GetDOF());

        if( tp.q.size() > 0 )
            itbody->ptarget->SetJointValues(NULL, NULL, &tp.q[0]);
        itbody->ptarget->SetTransform(tp.trans);

        if( itbody->time > itbody->ptraj->GetTotalDuration() ) {
            itbody = _listBodyTrajs.erase(itbody);
        }
        else {
            itbody->time += fElapsedTime;
            ++itbody;
        }
    }

    return false;
}

void TaskCagingProblem::SetActiveRobots(const std::vector<RobotBase*>& robots)
{
    //RAVELOGA("Starting SetupRobot...\n");
    robot = NULL;

    vector<RobotBase*>::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( wcsicmp((*itrobot)->GetName(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot == NULL ) {
        RAVEPRINT(L"Failed to find %S\n", _strRobotName.c_str());
    }
}

bool TaskCagingProblem::SendCommand(const char* pcmd, string& response)
{
    SetActiveRobots(GetEnv()->GetRobots());
    return CmdProblemInstance::SendCommand(pcmd, response);
}

// used to prune grasps that are not caging a target object
class GraspConstraintFn : public PlannerBase::ConstraintFunction
{
public:
    GraspConstraintFn(EnvironmentBase* penv) : probot(NULL), plink(NULL), _penv(penv) {
        Nrandconfigs = 10;
        fRandPerturb.resize(7);
        // 0.01 for translation and 0.07 for quaterions (~8-10 degrees)
        fRandPerturb[0] = fRandPerturb[1] = fRandPerturb[2] = 0.01f;
        fRandPerturb[3] = fRandPerturb[4] = fRandPerturb[5] = fRandPerturb[6] = 0.07f;
    }
    
    virtual bool Constraint(const dReal* pSrcConf, dReal* pDestConf, Transform* ptrans, int settings)
    {
        assert( pDestConf != NULL );
        if( plink == NULL || vtargetjoints.size() == 0)
            return true;
        
        probot->SetActiveDOFValues(NULL, pDestConf);

        if( _penv->CheckCollision(probot) || probot->CheckSelfCollision() )
            return false;

        bool bCaged = false;
        vector<dReal> vrobotconfig;
        probot->GetActiveDOFValues(vrobotconfig);

        // find N noncolliding grasps and check if they still cage the object
        for(int iter = 0; iter < Nrandconfigs; ++iter) {

            if( iter > 0 ) {
                
                plink->GetParent()->SetBodyTransformations(_vTargetTransforms);

                while(1) {
                    for(int i = 0; i < probot->GetActiveDOF(); ++i)
                        vrobotconfig[i] = pDestConf[i] + fRandPerturb[i]*(_penv->RandomFloat()-0.5f);

                    probot->SetActiveDOFValues(NULL, &vrobotconfig[0]);
                    if( !_penv->CheckCollision(probot) && !probot->CheckSelfCollision() )
                        break;
                }
            }

            FOREACH(itvv, _vvvCachedTransforms) {
                
                bCaged = false;
                FOREACH(itv, *itvv) {
                    plink->GetParent()->SetBodyTransformations(*itv);
                    
                    if( _penv->CheckCollision(probot, plink->GetParent()) ) {
                        bCaged = true;
                        break;
                    }
                }
                
                if( !bCaged )
                    break;
            }

            if( !bCaged )
                break;
        }

        probot->SetActiveDOFValues(NULL, pDestConf);
        plink->GetParent()->SetBodyTransformations(_vTargetTransforms);
        return bCaged;
    }

    // \param vTargetSides - -1 for only negative sides, 0 for both, 1 for only positive sides)
    void CacheTransforms(const vector<int>& vTargetSides)
    {
        static vector<dReal> values;
        values = vtargetvalues;
        
        _vvvCachedTransforms.resize(vtargetjoints.size()*2);
        vector< vector< vector<Transform> > >::iterator itvv = _vvvCachedTransforms.begin();

        plink->GetParent()->SetJointValues(NULL, NULL, &vtargetvalues[0]);
        plink->GetParent()->GetBodyTransformations(_vTargetTransforms);
        vector<int>::const_iterator itside = vTargetSides.begin();

        FOREACH(itjoint, vtargetjoints) {
            

            if( *itside <= 0 ) {
                itvv->resize(0);
                for(dReal finc = fIncrement; finc < fCagedConfig; finc += fIncrement) {
                    values[*itjoint] = vtargetvalues[*itjoint] - finc;
                    plink->GetParent()->SetJointValues(NULL, NULL, &values[0]);
                    
                    itvv->push_back(vector<Transform>());
                    plink->GetParent()->GetBodyTransformations(itvv->back());
                }
                ++itvv;
            }
            
            if( *itside >= 0 ) {
                itvv->resize(0);
                for(dReal finc = fIncrement; finc < fCagedConfig; finc += fIncrement) {
                    values[*itjoint] = vtargetvalues[*itjoint] + finc;
                    plink->GetParent()->SetJointValues(NULL, NULL, &values[0]);

                    itvv->push_back(vector<Transform>());
                    plink->GetParent()->GetBodyTransformations(itvv->back());
                                
                }
                ++itvv;
            }
            
            values[*itjoint] = vtargetvalues[*itjoint]; // revert
            ++itside;
        }

        _vvvCachedTransforms.erase(itvv,_vvvCachedTransforms.end());
        plink->GetParent()->SetJointValues(NULL, NULL, &vtargetvalues[0]);
    }

    dReal fIncrement, fCagedConfig;
    RobotBase* probot;
    KinBody::Link* plink;
    vector<int> vtargetjoints;
    vector<dReal> vtargetvalues;
    int Nrandconfigs;
    vector<dReal> fRandPerturb;

private:
    vector< vector< vector<Transform> > > _vvvCachedTransforms;
    vector<Transform> _vTargetTransforms;
    EnvironmentBase* _penv;
};

bool TaskCagingProblem::CreateGraspSet(ostream& sout, istream& sinput)
{
    dReal fStep = 0.01f;
    dReal fExploreProb = 0.02f;

    dReal fContactConfigDelta = 0.01f; // how much to move in order to find the contact grasp set
    GraspConstraintFn graspfn(GetEnv());

    KinBody* ptarget = NULL;
    graspfn.fCagedConfig = 0.05f;
    graspfn.fIncrement = 0.005f;
    graspfn.probot = robot;
    
    PlannerBase::PlannerParameters params;
    params.nMaxIterations = 1000;
    params.pconstraintfn = &graspfn;

    vector<int> vTargetSides;
    
    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "step") == 0 )
            sinput >> fStep;
        else if( stricmp(cmd.c_str(), "exploreprob") == 0 )
            sinput >> fExploreProb;
        else if( stricmp(cmd.c_str(), "size") == 0 ) {
            sinput >> params.nMaxIterations;
        }
        else if( stricmp(cmd.c_str(), "target") == 0 ) {
            string name;
            int linkindex;
            sinput >> name >> linkindex;

            ptarget = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
            if( ptarget == NULL || linkindex >= (int)ptarget->GetLinks().size() ) {
                RAVEPRINT(L"invalid target %s, index %d\n", name.c_str(), linkindex);
                return false;
            }
            graspfn.plink = ptarget->GetLinks()[linkindex];
        }
        else if( stricmp(cmd.c_str(), "targetjoint") == 0 ) {
            int joint; sinput >> joint;
            graspfn.vtargetjoints.push_back(joint);
        }
        else if( stricmp(cmd.c_str(), "contactconfigdelta") == 0 ) {
            sinput >> fContactConfigDelta;
        }
        else if( stricmp(cmd.c_str(), "cagedconfig") == 0 ) {
            sinput >> graspfn.fCagedConfig;
        }
        else if( stricmp(cmd.c_str(), "cagesides") == 0 ) {
            vTargetSides.resize(graspfn.vtargetjoints.size());
            FOREACH(it, vTargetSides)
                sinput >> *it;
        }
        else break;

        if( !sinput ) {
            RAVELOG(L"failed\n");
            return false;
        }
    }

    if( ptarget == NULL || graspfn.plink == NULL ) {
        RAVEPRINT(L"invalid target\n");
        return false;
    }

    if( vTargetSides.size() == 0 )
        vTargetSides.resize(graspfn.vtargetjoints.size(),0);

    const wchar_t* plannername = L"GraspExploration";
    PlannerBase* planner = GetEnv()->CreatePlanner(plannername);
    if( planner == NULL ) {
        RAVEPRINT(L"failed to find planner %S\n", plannername);
        return false;
    }

    RobotBase::RobotStateSaver state(robot);

    Transform tlinkorig, tlinknew;
    vector<dReal> vtargetvalues, vorigvalues;
    ptarget->GetJointValues(vorigvalues);
    tlinkorig = graspfn.plink->GetTransform();

    vector<dReal> upper, lower;
    ptarget->GetJointLimits(lower, upper);
    graspfn.vtargetvalues = lower;
    for(size_t i = 0; i < lower.size(); ++i)
        graspfn.vtargetvalues[i] = 0.5f*(lower[i]+upper[i]);

    ptarget->SetJointValues(NULL, NULL, &graspfn.vtargetvalues[0]);
    tlinknew = graspfn.plink->GetTransform();

    graspfn.CacheTransforms(vTargetSides);

    // move the robot according to the way the target object moved
    robot->ApplyTransform(tlinknew*tlinkorig.inverse());
    
    if( !JitterTransform(robot, 0.004f) ) {
        RAVEPRINT(L"failed to jitter\n");
        return false;
    }

    robot->SetActiveDOFs(vector<int>(), RobotBase::DOF_X|RobotBase::DOF_Y|RobotBase::DOF_Z|RobotBase::DOF_RotationQuat);
    robot->GetActiveDOFValues(params.vinitialconfig);
    
    Transform trobot = robot->GetTransform();
    robot->SetAffineTranslationLimits(trobot.trans-Vector(0.5f,0.5f,0.5f),trobot.trans+Vector(0.5f,0.5f,0.5f));

    params.vParameters.push_back(fStep);
    params.vParameters.push_back(fExploreProb);

    if( !planner->InitPlan(robot, &params) ) {
        RAVEPRINT(L"failed to initplan\n");
        return false;
    }

    uint32_t basetime = timeGetTime();
    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

    if( !planner->PlanPath(ptraj.get()) ) {
        RAVEPRINT(L"failed to plan\n");
        return false;
    }
    

    RAVELOG_INFOA("finished computing grasp set: pts=%"PRIdS" in %dms\n", ptraj->GetPoints().size(), timeGetTime()-basetime);
    
    vtargetvalues = graspfn.vtargetvalues;
    
    FOREACHC(itp, ptraj->GetPoints()) {
        
        int nContact = 0;
        
        if( graspfn.vtargetjoints.size() > 0 ) {
            robot->SetActiveDOFValues(NULL, &itp->q[0], false);

            // test 2^dof configurations of the target object
            int nTest = 1<<graspfn.vtargetjoints.size();
            for(int i = 0; i < nTest; ++i) {
                for(int j = 0; j < (int)graspfn.vtargetjoints.size(); ++j) {
                    dReal fadd = (i & (1<<j)) ? fContactConfigDelta : -fContactConfigDelta;
                    vtargetvalues[graspfn.vtargetjoints[j]] = graspfn.vtargetvalues[graspfn.vtargetjoints[j]] + fadd;
                }

                ptarget->SetJointValues(NULL, NULL, &vtargetvalues[0], true);

                if( GetEnv()->CheckCollision(robot, ptarget) ) {
                    // in collision, so part of contact set
                    nContact |= (1<<i);
                }
            }
        }

        // convert the active dofs to a transform and save
        robot->SetActiveDOFValues(NULL, &itp->q[0], false);
        sout << tlinknew.inverse() * robot->GetTransform() << " "  << nContact << " ";
    }

    if( ptarget != NULL )
        ptarget->SetJointValues(NULL, NULL, &vorigvalues[0]);
    return true;
}

TaskCagingProblem::ConstrainedTaskData::ConstrainedTaskData(EnvironmentBase* penv)
    : fGraspThresh(0.1f), fConfigThresh(0.2f), ptarget(NULL), ptargetlink(NULL),
      nMaxIkIterations(8), nMaxSamples(1), fGoalThresh(0.04f), _penv(penv)
{
    _robot = NULL;
    fWeights[0] = fWeights[1] = fWeights[2] = 0;
    bSortSolutions = false;
    bGenerateFeatures = false;
    bCheckFullCollision = false;
    _permdata.ptaskdata = this;
    _permdata.pgrasps = NULL;
    _permdata.psolution = NULL;
}

TaskCagingProblem::ConstrainedTaskData::~ConstrainedTaskData()
{
    // destroy all
    FOREACH(it, mapgrasps) {
        delete (FINDGRASPDATA*)it->second->_userdata;
        delete it->second;
    }
}

void TaskCagingProblem::ConstrainedTaskData::SetRobot(RobotBase* robot)
{
    assert( robot != NULL );
    _robot = robot;
    robot->GetActiveDOFLimits(_lower, _upper);
    _J.resize(3*robot->GetActiveDOF());
    _JJt.resize(9);

    if( ptarget != NULL ) {
        vector<dReal> vl, vu;
        ptarget->GetJointLimits(vl, vu);
        for(size_t i = 0; i < _vtargetjoints.size(); ++i) {
            _lower.push_back(vl[_vtargetjoints[i]]);
            _upper.push_back(vu[_vtargetjoints[i]]);
        }

        ptarget->GetJointValues(vtargvalues);
    }
    
    _vsample.resize(GetDOF());

    _vRobotWeights.resize(GetDOF());
    FOREACH(itw, _vRobotWeights)
        *itw = 1;

    int index = 0;
    FOREACHC(it, _robot->GetActiveJointIndices())
        _vRobotWeights[index++] = _robot->GetJointWeight(*it);
}

void TaskCagingProblem::ConstrainedTaskData::SetState(const dReal* pstate)
{
    _robot->SetActiveDOFValues(NULL, pstate);
    pstate += _robot->GetActiveDOF();

    for(size_t i = 0; i < _vtargetjoints.size(); ++i)
        vtargvalues[_vtargetjoints[i]] = pstate[i];
    ptarget->SetJointValues(NULL, NULL, &vtargvalues[0]);
}

void TaskCagingProblem::ConstrainedTaskData::GetState(dReal* pstate)
{
    _robot->GetActiveDOFValues(pstate);
    pstate += _robot->GetActiveDOF();

    for(size_t i = 0; i < _vtargetjoints.size(); ++i)
        pstate[i] = vtargvalues[_vtargetjoints[i]];
}

void TaskCagingProblem::ConstrainedTaskData::GetLimits(dReal* plower, dReal* pupper)
{
    memcpy(plower, &_lower[0], GetDOF()*sizeof(dReal));
    memcpy(pupper, &_upper[0], GetDOF()*sizeof(dReal));
}       

void TaskCagingProblem::ConstrainedTaskData::GenerateFeatures(const dReal* q, dReal* pfeatures)
{
    dReal f = 0;
    dReal fdiff = 0.3f;
    for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
        dReal flower = _lower[i]+fdiff-q[i];
        if( flower > 0 )
            f += flower;
        else {
            dReal fupper = q[i]-_upper[i]+fdiff;
            if( fupper > 0 )
                f += fupper;
        }
    }
    pfeatures[0] = f;

    f = 0;
    if( _robot->GetActiveManipulator() != NULL ) {
        assert( _robot->GetActiveManipulator()->pEndEffector != NULL );
        int linkindex = _robot->GetActiveManipulator()->pEndEffector->GetIndex();
        Transform tEE = _robot->GetActiveManipulator()->GetEndEffectorTransform();
        _robot->CalculateActiveJacobian(linkindex, tEE.trans, &_J[0]);
        //_robot->CalculateActiveRotationalJacobian(linkindex, tEE.trans, &_J[3*robot->GetActiveDOF()]);
        
        multtrans_to2<dReal, dReal, dReal>(&_J[0], &_J[0], 3, _robot->GetActiveDOF(), 3, &_JJt[0], false);
        
        // find the determinant
        pfeatures[1] = RaveSqrt(RaveFabs(matrixdet3(&_JJt[0], 3)));
    }
    else
        pfeatures[1] = 0;

    f = 0;
    for(size_t i = 0; i < vtargvalues.size(); ++i)
        f += SQR(vtargvalues[i]-vtargettraj.front()[i]);
    pfeatures[2] = RaveSqrt(f);
}

TaskCagingProblem::ConstrainedTaskData::FEATURES TaskCagingProblem::ConstrainedTaskData::EvalWithFeatures(const void* pConfiguration)
{
    assert( pConfiguration != NULL );
    
    ptarget->GetJointValues(vtargvalues);

    const dReal* q = (const dReal*)pConfiguration;
    _robot->SetActiveDOFValues(NULL, q);
    
    FEATURES f;
    GenerateFeatures(q, f.features);
    f.ftotal = expf(fWeights[0] *f.features[0]) + expf(fWeights[1] * f.features[1]) + expf(fWeights[2] * f.features[2]);
    
    return f;
}

float TaskCagingProblem::ConstrainedTaskData::Eval(const void* pConfiguration)
{
    const dReal* ptargetconfig = (const dReal*)pConfiguration + _robot->GetActiveDOF();
    dReal f = 0;
    for(size_t i = 0; i < _vtargetjoints.size(); ++i)
        f += SQR(ptargetconfig[i]-vtargettraj.front()[_vtargetjoints[i]]);

    return sqrtf(f);//expf(fWeights[0]*sqrtf(f));
}

// distance metric
float TaskCagingProblem::ConstrainedTaskData::Eval(const void* c0, const void* c1)
{
    assert( GetDOF() == (int)_vRobotWeights.size() );

    float out = 0;
    for(int i=0; i < GetDOF(); i++)
        out += _vRobotWeights[i] * (((dReal *)c0)[i]-((dReal *)c1)[i])*(((dReal *)c0)[i]-((dReal *)c1)[i]);
    
    return sqrtf(out);
}

dReal TaskCagingProblem::ConstrainedTaskData::GraspDist(const Transform& tprev, const dReal* preshapeprev, const Transform& tnew)
{
    dReal frotweight = 0.4f;
    dReal frotdist1 = (tprev.rot - tnew.rot).lengthsqr4();
    dReal frotdist2 = (tprev.rot + tnew.rot).lengthsqr4();
    return (tprev.trans-tnew.trans).lengthsqr3() + frotweight * min(frotdist1,frotdist2);
}

bool TaskCagingProblem::ConstrainedTaskData::AcceptConfig(const dReal* qprev, const vector<dReal>& qnew)
{
    dReal d = 0;
    for(int i = 0; i < (int)qnew.size(); ++i)
        d += (qnew[i]-qprev[i])*(qnew[i]-qprev[i]);
    return d < fConfigThresh*fConfigThresh;
}

bool TaskCagingProblem::ConstrainedTaskData::SampleIkPermutation(void* userdata, unsigned int index)
{
    PERMUTATIONDATA* pdata = (PERMUTATIONDATA*)userdata;
    assert( pdata != NULL && pdata->ptaskdata != NULL && pdata->pgrasps != NULL && pdata->psolution != NULL );
    assert( index < pdata->pgrasps->size() );

    return pdata->ptaskdata->SampleIkSolution( (*pdata->pgrasps)[index], pdata->pcursolution, pdata->psolution);
}

bool TaskCagingProblem::ConstrainedTaskData::FindGraspPermutation(void* userdata, unsigned int index)
{
    FINDGRASPDATA* pdata = (FINDGRASPDATA*)userdata;
    return pdata->ptaskdata->GraspDist(pdata->tcurgrasp, NULL, pdata->tlink * (*pdata->pgrasps)[index]) < pdata->fThresh2;
}

void TaskCagingProblem::ConstrainedTaskData::Sample(dReal* pNewSample)
{
    while(1) {
        assert( pNewSample != NULL );
        for(int i = _robot->GetActiveDOF(); i < GetDOF(); ++i) {
            pNewSample[i] = _lower[i] + (_upper[i]-_lower[i])*GetEnv()->RandomFloat();
            vtargvalues[_vtargetjoints[i]] = pNewSample[i];
        }
        
        ptarget->SetJointValues(NULL, NULL, &vtargvalues[0]);
        
        // sample the grasp and execute in random permutation order
        vector<Transform>& vgrasps = Eval(pNewSample) < 2*thresh ? vGraspContactSet : vGraspSet;
        if( SampleIkSolution(vgrasps[GetEnv()->RandomInt()%vgrasps.size()], NULL, pNewSample))
            return;
        //_permexecutor.SetUserData(&_permdata);
        //_permexecutor.SetFunction(SampleIkPermutation);
        //_permdata.pgrasps = Eval(pNewSample) < 2*thresh ? &vGraspContactSet : &vGraspSet;
//        _permdata.psolution = pNewSample;
        //_permdata.pcursolution = NULL;
//        if( _permexecutor.Permute(_permdata.pgrasps->size()) )
//            return;
    }
}

inline int GetTargetValue(int val)
{
    return (val>1)?(val-1):(val-2);
}

bool TaskCagingProblem::ConstrainedTaskData::Sample(dReal* pNewSample, const dReal* pCurSample, dReal fRadius)
{
    map<int, RandomPermuationExecutor*>::iterator itgrasp;

    FOREACH(it, mapgrasps)
        ((FINDGRASPDATA*)it->second->_userdata)->status = 0;

    // sample the neighborhood
    int failures = 0;
    while(1) {
        
        // sample the target object values
        dReal fbest = 10000;
        uint32_t bestid = 0;
        for(int iter = 0; iter < 3; ++iter) {
            uint32_t id = 0;
            for(uint32_t i = (uint32_t)_robot->GetActiveDOF(); i < (uint32_t)GetDOF(); ++i) {
                int val = GetEnv()->RandomInt()&3;
                id |= val<<(2*(i-_robot->GetActiveDOF()));
                _vsample[i] = pCurSample[i] + fRadius*(float)GetTargetValue(val);
                _vsample[i] = CLAMP_ON_RANGE(_vsample[i], _lower[i], _upper[i]);
            }

            float fval = Eval(&_vsample[0]);
            if( fval < fbest ) {
                fbest = fval;
                bestid = id;
            }
        }

        for(uint32_t i = (uint32_t)_robot->GetActiveDOF(); i < (uint32_t)GetDOF(); ++i) {
            _vsample[i] = pCurSample[i] + fRadius*(float)GetTargetValue((bestid>>(2*(i-_robot->GetActiveDOF())))&3);
            vtargvalues[_vtargetjoints[i-_robot->GetActiveDOF()]] = _vsample[i];
        }
        
        ptarget->SetJointValues(NULL, NULL, &vtargvalues[0]);
        
        // choose a grasp
        
        RandomPermuationExecutor* pexecutor;
        itgrasp = mapgrasps.find(bestid);
        if( itgrasp == mapgrasps.end() ) {
            // create a new permuter
            FINDGRASPDATA* pdata = new FINDGRASPDATA();
            pdata->status = 0;

            pexecutor = new RandomPermuationExecutor();
            pexecutor->_userdata = pdata;
            pexecutor->SetFunction(FindGraspPermutation);
            mapgrasps[bestid] = pexecutor;
        }
        else
            pexecutor = itgrasp->second;
        
        FINDGRASPDATA* pdata = (FINDGRASPDATA*)pexecutor->_userdata;
        if( pdata->status == 2 ) {
            continue;
        }

        if( pdata->status == 0 ) {
            pdata->ptaskdata = this;
            pdata->tcurgrasp = _robot->GetActiveManipulator()->GetEndEffectorTransform();
            pdata->fThresh2 = fGraspThresh*fGraspThresh;
            pdata->tlink = ptargetlink->GetTransform();

            if( 0 && Eval(pNewSample) < 2*thresh ) {
                pdata->pgrasps = &vGraspContactSet;
                //RAVEPRINT(L"contact grasp\n");
            }
            else
                pdata->pgrasps = &vGraspSet;
            
            pdata->status = 1;
            pexecutor->PermuteStart(pdata->pgrasps->size());
        }

        assert( pdata->status == 1 );
        
        int index = pexecutor->PermuteContinue();
        if( index < 0 ) {
            // couldn't find a grasp, check if we've run out of possible grasps
            ++failures;
            if( failures == pow(3.0f,(float)_vtargetjoints.size()) )
                return false;

            pdata->status = 2;
            continue; // sample again
        }
        
        _robot->SetActiveDOFValues(NULL, pCurSample);
        if( !SampleIkSolution(pdata->tlink * (*pdata->pgrasps)[index], pCurSample, &_vsample[0]) )
            continue;
        
        // evaluate
        memcpy(pNewSample, &_vsample[0], GetDOF()*sizeof(dReal));
        break;
    }

    return true;
}

bool TaskCagingProblem::ConstrainedTaskData::SampleIkSolution(const Transform& tgrasp, const dReal* pCurSolution, dReal* psolution)
{
    vector< dReal> solution;
    const RobotBase::Manipulator* pmanip = _robot->GetActiveManipulator();
    assert( pmanip != NULL );
    
    if( nMaxIkIterations == 0 || pmanip->GetNumFreeParameters() == 0 ) {
        if( pmanip->FindIKSolution(tgrasp, solution, true) ) {
            
            if( pCurSolution != NULL ) {
                // make sure solution is close to current solution
                if( !AcceptConfig(pCurSolution, solution) )
                    return false;
            }

            memcpy(psolution, &solution[0], sizeof(dReal)*solution.size());
            return true;
        }

        return false;
    }

    // have a classifier that knows when ik solution is impossible
    if( pmanip->GetNumFreeParameters() > 0 ) {
        _vfreeparams.resize(pmanip->GetNumFreeParameters());
        _vcurfreeparams.resize(pmanip->GetNumFreeParameters());
        pmanip->GetFreeParameters(&_vcurfreeparams[0]);
    }

    dReal startphi = _vcurfreeparams[0];
    dReal upperphi = 1, lowerphi = 0, deltaphi = 0;
    int iter = 0;
    dReal incphi = 0.01f;

    while(iter < nMaxIkIterations) {

        dReal curphi = startphi;
        if( iter & 1 ) { // increment
            curphi += deltaphi;
            if( curphi > upperphi ) {

                if( startphi-deltaphi < lowerphi)
                    break; // reached limit
                ++iter;
                continue;
            }
        }
        else { // decrement
            curphi -= deltaphi;
            if( curphi < lowerphi ) {

                if( startphi+deltaphi > upperphi )
                    break; // reached limit
                ++iter;
                deltaphi += incphi; // increment
                continue;
            }

            deltaphi += incphi; // increment
        }

        iter++;
        _vfreeparams[0] = curphi;
        
        if( pmanip->FindIKSolution(tgrasp, _vfreeparams.size() > 0 ? &_vfreeparams[0] : NULL, solution, true) ) {

            if( pCurSolution != NULL ) {
                // make sure solution is close to current solution
                if( !AcceptConfig(pCurSolution, solution) ) {
                    //RAVEPRINT(L"no accept\n");
                    continue;
                }
            }

            memcpy(psolution, &solution[0], sizeof(dReal)*solution.size());
            return true;
        }
    }

    //int iter = nMaxIkIterations;
//    while(iter-- > 0) {
//        // randomly sample params
//        for(int j = 0; j < pmanip->GetNumFreeParameters(); ++j)
//            _vfreeparams[j] = CLAMP_ON_RANGE(_vcurfreeparams[j] + 0.5f*fConfigThresh*(GetEnv()->RandomFloat()-0.5f),(dReal)0,(dReal)1);
//        
//        if( pmanip->FindIKSolution(tgrasp, _vfreeparams.size() > 0 ? &_vfreeparams[0] : NULL, solution, true) ) {
//
//            if( pCurSolution != NULL ) {
//                // make sure solution is close to current solution
//                if( !AcceptConfig(pCurSolution, solution) ) {
//                    //RAVEPRINT(L"no accept\n");
//                    continue;
//                }
//            }
//
//            memcpy(psolution, &solution[0], sizeof(dReal)*solution.size());
//            return true;
//        }
//    }

    //RAVEPRINT(L"failed\n");
    return false;
}

void TaskCagingProblem::ConstrainedTaskData::FillIkSolutions(TaskCagingProblem::ConstrainedTaskData::GRASP& g,
                                                                 const vector<vector<dReal> >& solutions)
{
    g.iksolutions.clear();
    
    if( bSortSolutions ) {
        // fill the list
        FOREACHC(itsol, solutions)
            g.iksolutions.push_back(ConstrainedTaskData::IKSOL(*itsol, EvalWithFeatures(&(*itsol)[0])));
        
        g.iksolutions.sort(IkSolutionCompare());
        //RAVEPRINT(L"%f %f\n", g.iksolutions.front().second, g.iksolutions.back().second);
    }
    else {
        // fill the list
        FOREACHC(itsol, solutions)
            g.iksolutions.push_back(ConstrainedTaskData::IKSOL(*itsol, FEATURES()));
    }
}

void TaskCagingProblem::ConstrainedTaskData::SetGenerateFeatures(const string& filename)
{
    bGenerateFeatures = true;
    flog.open(filename.c_str());
}

void TaskCagingProblem::ConstrainedTaskData::Log(TaskCagingProblem::ConstrainedTaskData::IKSOL& iksol)
{
    for(size_t i = 0; i < ARRAYSIZE(iksol.second.features); ++i)
        flog << iksol.second.features[i] << " ";
    flog << iksol.second.bSuccess << " ";
    FOREACH(it, iksol.first)
        flog << *it << " ";
    flog << endl;
}

bool TaskCagingProblem::ConstrainedTaskData::_CheckCollision(const dReal *pQ0, const dReal *pQ1, IntervalType interval)
{
    // set the bounds based on the interval type
    int start;
    bool bCheckEnd;
    switch (interval) {
        case OPEN:
            start = 1;  bCheckEnd = false;
            break;
        case OPEN_START:
            start = 1;  bCheckEnd = true;
            break;
        case OPEN_END:
            start = 0;  bCheckEnd = false;
            break;
        case CLOSED:
            start = 0;  bCheckEnd = true;
            break;
        default:
            assert(0);
    }

    // first make sure the end is free
    if (bCheckEnd) {
        SetState(pQ1);
        if (GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            return true;
    }

    // compute  the discretization
    int i, numSteps = 1;
    for (i = 0; i < GetDOF(); i++) {
        int steps = (int)(fabs(pQ1[i] - pQ0[i]) * 60.0f);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    static vector<dReal> _jointIncrement;
    _jointIncrement.resize(GetDOF());
    for (i = 0; i < GetDOF(); i++)
        _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

    static vector<dReal> v;
    v.resize(GetDOF());

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    for (int f = start; f < numSteps; f++) {
        for (i = 0; i < GetDOF(); i++)
            v[i] = pQ0[i] + (_jointIncrement[i] * f);

        SetState(&v[0]);
        if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            return true;
    }

    return false;
}

bool TaskCagingProblem::FindAllRelaxedForward(const dReal* qprev, int j, Trajectory* ptraj,
                                                  TaskCagingProblem::ConstrainedTaskData& data)
{
    //RAVEPRINT(L"%d\n", j);
    RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    assert( pmanip != NULL && pmanip->HasIKSolver() );
    
    data.ptarget->SetJointValues(NULL, NULL, &data.vtargettraj[j][0]);
    
    robot->SetActiveDOFValues(NULL, qprev);
    Transform tprevgrasp = pmanip->GetEndEffectorTransform();
    
    vector<dReal> preshape;
    bool bFoundAtLeastOne = false;

    typedef pair<list<ConstrainedTaskData::GRASP>::iterator, dReal> GRASPPAIR;
    priority_queue<GRASPPAIR , deque<GRASPPAIR>, ConstrainedTaskData::GraspCompare> pqAcceptedGrasps;

    FOREACH(itgrasp, data.vlistGraspSet[j]) {

        dReal fdist = data.GraspDist(tprevgrasp, NULL, itgrasp->tgrasp);

        if( fdist < data.fGraspThresh*data.fGraspThresh ) {
            // accept the grasp
            pqAcceptedGrasps.push( pair<list<ConstrainedTaskData::GRASP>::iterator,dReal>(itgrasp, fdist));
        }
    }

    while(!pqAcceptedGrasps.empty()) {
        ConstrainedTaskData::GRASP& grasp = *pqAcceptedGrasps.top().first;

        if( grasp.iksolutions.size() == 0 ) {
            vector< vector<dReal> > solutions;
            if( !pmanip->FindIKSolutions(grasp.tgrasp, solutions, true) ) {
                // could not find ik solutions for this grasp, so remove it permanently
                data.vlistGraspSet[j].erase(pqAcceptedGrasps.top().first);
                pqAcceptedGrasps.pop();
                continue;
            }
            
            //RAVEPRINT(L"iksol\n", j);
            data.FillIkSolutions(grasp, solutions);
            if( data.bCheckFullCollision ) {
                FOREACH(itsol, grasp.iksolutions) {
                    FOREACH(itindex, data._vtargetjoints)
                        itsol->first.push_back(data.vtargettraj[j][*itindex]);
                }
            }

            robot->SetActiveDOFValues(NULL, qprev);
        }

        FOREACH_NOINC(itsol, grasp.iksolutions) {
            if( data.AcceptConfig(qprev, itsol->first) ) {

                // check for intermediate collisions
                if( data.bCheckFullCollision ) {
                    if( data._CheckCollision(qprev, &itsol->first[0], ConstrainedTaskData::OPEN) ) {
                        ++itsol;
                        continue;
                    }
                }
                
                // go to next
                if( itsol->second.bSuccess ) {
                    // already know it will succeed, no need to recurse
                    bFoundAtLeastOne = true;
                    ++itsol;
                    continue;
                }

                if( data.bCheckFullCollision )
                    data.ptarget->SetJointValues(NULL, NULL, &data.vtargettraj[j][0]);
                if( j == 0 || FindAllRelaxedForward(&itsol->first[0], j-1, ptraj, data)) {
                    
                    bFoundAtLeastOne = true;
                    itsol->second.bSuccess = true;

                    if( !data.IsGenerateFeatures() ) {

                        if( !data.bCheckFullCollision ) {
                            FOREACH(itindex, data._vtargetjoints)
                                itsol->first.push_back(data.vtargettraj[j][*itindex]);
                        }
                        ptraj->AddPoint(Trajectory::TPOINT(itsol->first, 0));
                        return true;
                    }
                    else {
                        // go on
                        data.Log(*itsol);
                    }
                }
                else {

                    if( data.IsGenerateFeatures() )
                        data.Log(*itsol);
                    
                    if( data.vlistGraspSet[j-1].size() == 0 ) {
                        // no more grasps at next level, so fail
                        data.vlistGraspSet[j].clear();
                        return false;
                    }
                    
                    // remove itsol from consideration in the future
                    itsol = grasp.iksolutions.erase(itsol);
                    continue;
                }
            }
            ++itsol;
        }

        if( data.bCheckFullCollision )
            data.ptarget->SetJointValues(NULL, NULL, &data.vtargettraj[j][0]);

        if( grasp.iksolutions.size() == 0 )
            data.vlistGraspSet[j].erase(pqAcceptedGrasps.top().first); // remove

        pqAcceptedGrasps.pop();
    }

    return bFoundAtLeastOne;
}

string getfilename_withseparator(istream& sinput, char separator)
{
    string filename;
    if( !getline(sinput, filename, separator) ) {
        // just input directly
        RAVEPRINT(L"graspset filename not terminated with ';'\n");
        sinput >> filename;
    }

    // trim leading spaces
    size_t startpos = filename.find_first_not_of(" \t");
    size_t endpos = filename.find_last_not_of(" \t");

    // if all spaces or empty return an empty string  
    if(( string::npos == startpos ) || ( string::npos == endpos))
        return "";

    filename = filename.substr( startpos, endpos-startpos+1 );
    return filename;
}

bool TaskCagingProblem::TaskConstrainedPlanner(ostream& sout, istream& sinput)
{
    ConstrainedTaskData taskdata(GetEnv());

    int nLinkIndex=-1;
    string strsavetraj, strbodytraj;

    // randomized algorithm parameters
    wstring plannername;
    RAStarParameters params;
    params.fDistThresh = 0.03f;
    params.fRadius = 0.1f;
    params.fGoalCoeff = 1;
    params.nMaxChildren = 5;
    params.nMaxSampleTries = 2;
    params.nMaxIterations = 1001;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "target") == 0 ) {
            string name; sinput >> name;
            taskdata.ptarget = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
            if( taskdata.ptarget == NULL )
                RAVEPRINT(L"invalid target %s\n", name.c_str());
        }
        else if( stricmp(cmd.c_str(), "planner") == 0 ) {
            string name; sinput >> name;
            plannername = _ravembstowcs(name.c_str());
        }
        else if( stricmp(cmd.c_str(), "distthresh") == 0 )
            sinput >> params.fDistThresh;
        else if( stricmp(cmd.c_str(), "sampleradius") == 0 )
            sinput >> params.fRadius;
        else if( stricmp(cmd.c_str(), "goalcoeff") == 0 )
            sinput >> params.fGoalCoeff;
        else if( stricmp(cmd.c_str(), "numchildren") == 0 )
            sinput >> params.nMaxChildren;
        else if( stricmp(cmd.c_str(), "goalthresh") == 0 )
            sinput >> taskdata.fGoalThresh;
        else if( stricmp(cmd.c_str(), "targetlink") == 0 )
            sinput >> nLinkIndex;
        else if( stricmp(cmd.c_str(), "savetraj") == 0 )
            strsavetraj = getfilename_withseparator(sinput,';');
        else if( stricmp(cmd.c_str(), "savebodytraj") == 0 )
            strbodytraj = getfilename_withseparator(sinput,';');
        else if( stricmp(cmd.c_str(), "graspthresh") == 0 )
            sinput >> taskdata.fGraspThresh;
        else if( stricmp(cmd.c_str(), "configthresh") == 0 )
            sinput >> taskdata.fConfigThresh;
        else if( stricmp(cmd.c_str(), "maxsamples") == 0 )
            sinput >> taskdata.nMaxSamples;
        else if( stricmp(cmd.c_str(), "maxiterations") == 0 )
            sinput >> params.nMaxIterations;
        else if( stricmp(cmd.c_str(), "maxikiterations") == 0 )
            sinput >> taskdata.nMaxIkIterations;
        else if( stricmp(cmd.c_str(), "targetjoints") == 0 ) {
            int num=0;
            sinput >> num;
            taskdata._vtargetjoints.resize(num);
            FOREACH(it, taskdata._vtargetjoints)
                sinput >> *it;
        }
        else if(stricmp(cmd.c_str(), "graspset") == 0 ) {
            taskdata.vGraspSet.clear(); taskdata.vGraspSet.reserve(200);
            string filename = getfilename_withseparator(sinput,';');
            ifstream fgrasp(filename.c_str());
            while(!fgrasp.eof()) {
                Transform t;
                fgrasp >> t;
                if( !fgrasp ) {
                    break;
                }
                taskdata.vGraspSet.push_back(t);
            }
            RAVELOG_DEBUGA("grasp set size = %"PRIdS"\n", taskdata.vGraspSet.size());
        }
        else if(stricmp(cmd.c_str(), "graspcontactset") == 0 ) {
            taskdata.vGraspContactSet.clear(); taskdata.vGraspContactSet.reserve(200);
            string filename = getfilename_withseparator(sinput,';');
            ifstream fgrasp(filename.c_str());
            while(!fgrasp.eof()) {
                Transform t;
                fgrasp >> t;
                if( !fgrasp ) {
                    break;
                }
                taskdata.vGraspContactSet.push_back(t);
            }
            RAVELOG_DEBUGA("grasp contact set size = %"PRIdS"\n", taskdata.vGraspContactSet.size());
        }
        else if(stricmp(cmd.c_str(), "graspstartset") == 0 ) {
            taskdata.vGraspStartSet.clear(); taskdata.vGraspStartSet.reserve(200);
            string filename; sinput >> filename;
            ifstream fgrasp(filename.c_str());
            while(!fgrasp.eof()) {
                Transform t;
                fgrasp >> t;
                if( !fgrasp ) {
                    break;
                }
                taskdata.vGraspStartSet.push_back(t);
            }
            RAVELOG_DEBUGA("grasp start set size = %"PRIdS"\n", taskdata.vGraspStartSet.size());
        }
        else if( stricmp(cmd.c_str(), "targettraj") == 0 ) {

             if( taskdata.ptarget == NULL ) {
                RAVEPRINT(L"target cannot be null when specifying trajectories!\n");
                return false;
            }

            int nPoints; sinput >> nPoints;
            taskdata.vtargettraj.resize(nPoints);
            
            FOREACH(itp, taskdata.vtargettraj) {
                itp->resize(taskdata.ptarget->GetDOF());
                FOREACH(it, *itp)
                    sinput >> *it;
            }
        }
        else if( stricmp(cmd.c_str(), "usegoal") == 0 ) {
            sinput >> taskdata.fWeights[0] >> taskdata.fWeights[1] >> taskdata.fWeights[2];
            taskdata.bSortSolutions = true;
        }
        else if( stricmp(cmd.c_str(), "fullcol") == 0 ) {
            taskdata.bCheckFullCollision = true;
        }
        else if( stricmp(cmd.c_str(), "features") == 0 ) {
            string filename;
            sinput >> filename;
            taskdata.SetGenerateFeatures(filename);
        }
        else break;

        if( !sinput ) {
            RAVELOG(L"failed\n");
            return false;
        }
    }

    if( taskdata.ptarget == NULL ) {
        RAVEPRINT(L"need to specify target!\n");
        return false;
    }

    if( robot->GetActiveManipulator() == NULL ) {
        RAVEPRINT(L"no manipulator!\n");
        return false;
    }

    if(nLinkIndex < 0 || nLinkIndex >= (int)taskdata.ptarget->GetLinks().size() ) {
        RAVEPRINT(L"need target link name\n");
        return false;
    }

    if( taskdata.vGraspSet.size() == 0 ) {
        RAVEPRINT(L"error, graspset size 0\n");
        return false;
    }
    if( taskdata.vGraspContactSet.size() == 0 ) {
        RAVEPRINT(L"error, graspset size 0\n");
        return false;
    }

    taskdata.ptargetlink = taskdata.ptarget->GetLinks()[nLinkIndex];

    RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL || !pmanip->HasIKSolver() ) {
        RAVEPRINT(L"need to select a robot manipulator with ik solver\n");
        return false;
    }

    RobotBase::RobotStateSaver saver(robot);
    KinBody::KinBodyStateSaver targetsaver(taskdata.ptarget);

    robot->SetActiveDOFs(pmanip->_vecarmjoints);
    
    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    
    uint32_t basetime = timeGetTime(), finaltime;
    taskdata.SetRobot(robot);

    boost::shared_ptr<Trajectory> ptrajtemp(GetEnv()->CreateTrajectory(taskdata.GetDOF()));
    bool bReverseTrajectory = false;

    if( plannername.size() > 0 ) {

        if( taskdata.vtargettraj.size() < 2 ) {
            RAVEPRINT(L"not enough target trajectory points (need at least 2 for initial and goal\n");
            return false;
        }

        bReverseTrajectory = true;

        // create RA* planner
        params.pgoalfn = &taskdata;
        params.pSampleFn = &taskdata;
        params.pConfigState = &taskdata;
        params.pdistmetric = &taskdata;

        vector<dReal> vtargetinit;
        taskdata.ptarget->GetJointValues(vtargetinit);

        taskdata.ptarget->SetJointValues(NULL, NULL, &taskdata.vtargettraj.back()[0]);
        Transform tlink  = taskdata.ptargetlink->GetTransform();
        
        bool bSucceed = false;
        FOREACH(it, taskdata.vGraspContactSet) {
            if( pmanip->FindIKSolution(tlink * *it, params.vinitialconfig, true) ) {
                robot->SetActiveDOFValues(NULL, &params.vinitialconfig[0]);
                bSucceed = true;
                break;
            }
        }

        if( !bSucceed ) {
            RAVEPRINT(L"failed to find goal configuration = %dms\n", timeGetTime()-basetime);
            return false;
        }
        
        FOREACH(it, taskdata._vtargetjoints)
            params.vinitialconfig.push_back(taskdata.vtargettraj.back()[*it]);

        //// pick any solution at the initial config of the target object
//        bool bSucceed = false;
//        Transform tlink = taskdata.ptargetlink->GetTransform();
//        vector<dReal> solution;
//        FOREACH(itgrasp, taskdata.vGraspSet) {
//            if( pmanip->FindIKSolution(tlink * *itgrasp, solution, true) ) {
//                robot->SetActiveDOFValues(NULL, &solution[0]);
//                bSucceed = true;
//                break;
//            }
//        }
//
//        if( !bSucceed ) {
//            RAVEPRINT(L"failed to find initial configuration = %dms\n", timeGetTime()-basetime);
//            return false;
//        }


        // try to find an IK solution for every point on the trajectory (Optional!!) 
        bool bHasIK = false;
        vector<dReal> solution;
        
        for(int ivgrasp = 0; ivgrasp < (int)taskdata.vtargettraj.size()-1; ++ivgrasp) {
            taskdata.ptarget->SetJointValues(NULL, NULL, &taskdata.vtargettraj[ivgrasp][0]);
            tlink = taskdata.ptargetlink->GetTransform();
            bHasIK = false;
            
            FOREACH(it, taskdata.vGraspSet) {
                if( pmanip->FindIKSolution(tlink * *it, solution, true) ) {
                    bHasIK = true;
                    break;
                }
            }
            
            if( !bHasIK ) {
                // no ik solution found for this grasp, so quit
                RAVELOG_ERRORA("failure, due to ik time=%dms, %d/%"PRIdS"\n", timeGetTime()-basetime, ivgrasp, taskdata.vtargettraj.size());
                break;
            }
        }
        
        if( !bHasIK )
            return false;

        taskdata.SetState(&params.vinitialconfig[0]);

        boost::shared_ptr<PlannerBase> pra(GetEnv()->CreatePlanner(plannername.c_str()));
        if( pra == NULL ) {
            RAVEPRINT(L"could not find %S planner\n", plannername.c_str());
            return false;
        }

        if( !pra->InitPlan(robot,  &params) )
            return false;
        
        RAVEPRINT(L"planning a caging grasp...\n");
        if( !pra->PlanPath(ptrajtemp.get()) ) {
            RAVEPRINT(L"planner failure, time = %dms\n", timeGetTime()-basetime);
            return false;
        }
    }
    else {

        if( taskdata.vtargettraj.size() < 2 ) {
            RAVEPRINT(L"not enough trajectory points\n");
            return false;
        }

        // Discretized Search
        vector<vector<dReal> > solutions;
        
        // initialize the grasp sets
        taskdata.vlistGraspSet.resize(taskdata.vtargettraj.size());
        
        bool bHasIK = false;
        
        for(int ivgrasp = 0; ivgrasp < (int)taskdata.vlistGraspSet.size(); ++ivgrasp) {
            int realindex = (ivgrasp+taskdata.vlistGraspSet.size()-1)%taskdata.vlistGraspSet.size();

            taskdata.ptarget->SetJointValues(NULL, NULL, &taskdata.vtargettraj[realindex][0]);
            Transform Ttarget = taskdata.ptargetlink->GetTransform();
            bHasIK = false;
            
            vector< Transform >& vGraspSet = (realindex == 0 && taskdata.vGraspStartSet.size()>0)?taskdata.vGraspStartSet:taskdata.vGraspSet;
            FOREACH(it, vGraspSet) {
                Transform tgrasp = Ttarget * *it;
                
                // make sure that there exists at least 1 IK solution for every
                // trajectory point of the target configruation space
                if( !bHasIK ) {
                    
                    if( pmanip->FindIKSolutions(tgrasp, solutions, true) ) {
                        bHasIK = true;
                        taskdata.vlistGraspSet[realindex].push_back(ConstrainedTaskData::GRASP(tgrasp));
                        taskdata.FillIkSolutions(taskdata.vlistGraspSet[realindex].back(), solutions);
                        if( taskdata.bCheckFullCollision ) {
                            FOREACH(itsol, taskdata.vlistGraspSet[realindex].back().iksolutions) {
                                FOREACH(itindex, taskdata._vtargetjoints)
                                    itsol->first.push_back(taskdata.vtargettraj[realindex][*itindex]);
                            }
                        }
                        
                    }
                }
                else {
                    // add without testing ik (lazy computation)
                    taskdata.vlistGraspSet[realindex].push_back(ConstrainedTaskData::GRASP(tgrasp));
                }
            }
            
            if( !bHasIK ) {
                // no ik solution found for this grasp, so quit
                RAVELOG_ERRORA("failure, due to ik time=%dms, %d/%"PRIdS"\n", timeGetTime()-basetime, realindex, taskdata.vtargettraj.size());
                break;
            }
        }
        
        if( !bHasIK ) {
            return false;
        }
        
        taskdata.ptarget->SetJointValues(NULL, NULL, &taskdata.vtargettraj.back()[0]);
        Transform Ttarget = taskdata.ptargetlink->GetTransform();
    
        // start planning backwards    
        bool bSuccess = false;
        FOREACHC(itgrasp, taskdata.vGraspContactSet) {
            if( pmanip->FindIKSolutions(Ttarget * *itgrasp, solutions, true) ) {
                FOREACH(itsol, solutions) {
                    FOREACH(itindex, taskdata._vtargetjoints)
                        itsol->push_back(taskdata.vtargettraj.back()[*itindex]);
                    
                    if( FindAllRelaxedForward(&(*itsol)[0], taskdata.vtargettraj.size()-2, ptrajtemp.get(), taskdata) ) {
                        ptrajtemp->AddPoint(Trajectory::TPOINT(*itsol, 0));
                        bSuccess = true;
                        break;
                    }
                }
                
                if( bSuccess )
                    break;
            }
        }

        if( !bSuccess ) {
            RAVEPRINT(L"failure, time=%dms\n", timeGetTime()-basetime);
            return false;
        }
    }

    finaltime = timeGetTime()-basetime;

    Trajectory::TPOINT tp;
    if( bReverseTrajectory ) {
        FOREACHR(itpoint, ptrajtemp->GetPoints()) {
            tp.q.resize(0);
            tp.q.insert(tp.q.end(), itpoint->q.begin(), itpoint->q.begin()+robot->GetActiveDOF());
            ptraj->AddPoint(tp);
        }
    }
    else {
        FOREACH(itpoint, ptrajtemp->GetPoints()) {
            tp.q.resize(0);
            tp.q.insert(tp.q.end(), itpoint->q.begin(), itpoint->q.begin()+robot->GetActiveDOF());
            ptraj->AddPoint(tp);
        }
    }
    
    ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
    
    if( strbodytraj.size() > 0 ) {
        
        boost::shared_ptr<Trajectory> pbodytraj(GetEnv()->CreateTrajectory(taskdata.ptarget->GetDOF()));
        
        vector<Trajectory::TPOINT>::const_iterator itrobottraj = ptraj->GetPoints().begin();
        taskdata.ptarget->GetJointValues(tp.q);
        Transform ttarget = taskdata.ptarget->GetTransform();

        if( bReverseTrajectory ) {
            FOREACHR(itpoint, ptrajtemp->GetPoints()) {
                for(size_t i = 0; i < taskdata._vtargetjoints.size(); ++i)
                    tp.q[taskdata._vtargetjoints[i]] = itpoint->q[robot->GetActiveDOF()+i];
                tp.time = itrobottraj++->time;
                tp.trans = ttarget;
                pbodytraj->AddPoint(tp);
            }
        }
        else {
            FOREACH(itpoint, ptrajtemp->GetPoints()) {
                for(size_t i = 0; i < taskdata._vtargetjoints.size(); ++i)
                    tp.q[taskdata._vtargetjoints[i]] = itpoint->q[robot->GetActiveDOF()+i];
                tp.time = itrobottraj++->time;
                tp.trans = ttarget;
                pbodytraj->AddPoint(tp);
            }
        }
        
        pbodytraj->Write(strbodytraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    }

    RAVEPRINT(L"success, time=%dms\n", finaltime);
    sout << finaltime << " ";

    // write first and last points
    vector<dReal> values;
    robot->SetActiveDOFValues(NULL, &ptraj->GetPoints().front().q[0]);
    robot->GetJointValues(values);
    FOREACH(it, values)
        sout << *it << " ";
    robot->SetActiveDOFValues(NULL, &ptraj->GetPoints().back().q[0]);
    robot->GetJointValues(values);
    FOREACH(it, values)
        sout << *it << " ";
    
    if( strsavetraj.size() ) {
        boost::shared_ptr<Trajectory> pfulltraj(GetEnv()->CreateTrajectory(robot->GetDOF()));
        robot->GetFullTrajectoryFromActive(pfulltraj.get(), ptraj.get());
        pfulltraj->Write(strsavetraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    }

    return true;
}

bool TaskCagingProblem::FindAllSimple(const dReal* qprev, int j, list<vector<dReal> >& vtrajectory,
                                          dReal fConfigThresh2, vector<list<vector<dReal> > >& vsolutions,
                                          TaskCagingProblem::ConstrainedTaskData& data)
{
    FOREACH_NOINC(itsol, vsolutions[j]) {

        dReal d = 0;
        for(size_t i = 0; i < (*itsol).size(); ++i)
        d += ((*itsol)[i]-qprev[i])*((*itsol)[i]-qprev[i]);

        if( d > fConfigThresh2 ) {
            ++itsol;
            continue;
        }

        if( data.bCheckFullCollision ) {
            if( data._CheckCollision(qprev, &(*itsol)[0], ConstrainedTaskData::OPEN) ) {
                ++itsol;
                continue;
            }
        }

        if( j+1 >= (int)vsolutions.size() ||
            FindAllSimple(&(*itsol)[0], j+1, vtrajectory, fConfigThresh2, vsolutions, data) ) {
            vtrajectory.push_back(*itsol);
            return true;
        }
     
        // no point in continuing
        if( vsolutions[j+1].size() == 0 ) {
            vsolutions[j].clear();
            return false;
        }
        
        itsol = vsolutions[j].erase(itsol);
    }

    return false;
}

bool TaskCagingProblem::SimpleConstrainedPlanner(ostream& sout, istream& sinput)
{
    RAVELOG(L"SimpleConstrainedPlanner\n");
    
    vector<vector<dReal> > vtargettraj;
    dReal fConfigThresh2 = 0.1f*0.1f;

    list< Transform > listGraspSet;

    TaskCagingProblem::ConstrainedTaskData taskdata(GetEnv());

    int nLinkIndex=-1;
    string strsavetraj, strbodytraj;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "target") == 0 ) {
            string name; sinput >> name;
            taskdata.ptarget = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
            if( taskdata.ptarget == NULL )
                RAVEPRINT(L"invalid target %s\n", name.c_str());
        }
        else if( stricmp(cmd.c_str(), "targetlink") == 0 )
            sinput >> nLinkIndex;
        else if( stricmp(cmd.c_str(), "savetraj") == 0 )
            sinput >> strsavetraj;
        else if( stricmp(cmd.c_str(), "savebodytraj") == 0 )
            sinput >> strbodytraj;
        else if( stricmp(cmd.c_str(), "configthresh") == 0 ) {
            sinput >> fConfigThresh2;
            fConfigThresh2 *= fConfigThresh2;
        }
        else if(stricmp(cmd.c_str(), "graspset") == 0 ) {
            listGraspSet.clear();
            string filename = getfilename_withseparator(sinput,';');
            ifstream fgrasp(filename.c_str());
            while(!fgrasp.eof()) {
                Transform t;
                fgrasp >> t;
                if( !fgrasp ) {
                    RAVELOG(L"grasp set file corrupted\n");
                    break;
                }
                listGraspSet.push_back(t);
            }
        }
        else if( stricmp(cmd.c_str(), "targettraj") == 0 ) {

            if( taskdata.ptarget == NULL ) {
                RAVEPRINT(L"target cannot be null when specifying trajectories!\n");
                return false;
            }

            int nPoints; sinput >> nPoints;

            vtargettraj.resize(nPoints);
            FOREACH(itp, vtargettraj) {
                itp->resize(taskdata.ptarget->GetDOF());
                FOREACH(it, *itp)
                    sinput >> *it;
            }
        }
        else if( stricmp(cmd.c_str(), "fullcol") == 0 ) {
            taskdata.bCheckFullCollision = true;
        }
        else break;

        if( !sinput ) {
            RAVELOG(L"failed\n");
            return false;
        }
    }

    if( taskdata.ptarget == NULL ) {
        RAVEPRINT(L"need to specify target!\n");
        return false;
    }

    if(nLinkIndex < 0 || nLinkIndex >= (int)taskdata.ptarget->GetLinks().size() ) {
        RAVEPRINT(L"need target link name\n");
        return false;
    }
    taskdata.ptargetlink = taskdata.ptarget->GetLinks()[nLinkIndex];

    RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL || !pmanip->HasIKSolver() ) {
        RAVEPRINT(L"need to select a robot manipulator with ik solver\n");
        return false;
    }

    if( vtargettraj.size() == 0 )
        return false;

    for(int i = 0; i < (int)vtargettraj.front().size(); ++i)
        taskdata._vtargetjoints.push_back(i);

    RobotBase::RobotStateSaver saver(robot);
    KinBody::KinBodyStateSaver targetsaver(taskdata.ptarget);

    robot->SetActiveDOFs(pmanip->_vecarmjoints);
    taskdata.SetRobot(robot);

    uint32_t basetime = timeGetTime();

    vector<vector<dReal> > solutions;
    list<vector<dReal> > vtrajectory;
    bool bSuccess = false;

    vector<list<vector<dReal> > > vsolutions(vtargettraj.size());

    FOREACHC(itgrasp, listGraspSet) {

        bool bNoIK = false;

        // get the IK solutions
        for(int i = 0; i < (int)vtargettraj.size(); ++i) {
            int realindex = (i+vtargettraj.size()-1)%vtargettraj.size();
            vsolutions[realindex].clear();
            taskdata.ptarget->SetJointValues(NULL, NULL, &vtargettraj[realindex][0]);
            Transform Ttarget = taskdata.ptargetlink->GetTransform();
            
            pmanip->FindIKSolutions(Ttarget * *itgrasp, solutions, true);

            if( solutions.size() == 0 ) {
                bNoIK = true;
                break;
            }
            
            FOREACH(itsol, solutions) {
                if( taskdata.bCheckFullCollision )
                    itsol->insert(itsol->end(), vtargettraj[realindex].begin(), vtargettraj[realindex].end());
                vsolutions[realindex].push_back(*itsol);
            }
        }

        if( bNoIK )
            continue;

        FOREACH(itsol, vsolutions.front()) {
            if( FindAllSimple(&(*itsol)[0], 1, vtrajectory, fConfigThresh2, vsolutions, taskdata) ) {
                vtrajectory.push_back(*itsol);
                bSuccess = true;
                break;
            }
        }
            
        if( bSuccess )
            break;
    }

    uint32_t finaltime = timeGetTime()-basetime;

    if( !bSuccess ) {
        RAVEPRINT(L"failure, time=%dms\n", finaltime);
        return false;
    }

    RAVEPRINT(L"success, time=%dms\n", finaltime);
    sout << finaltime << " ";

    // write the last point
    vector<dReal> values;

    robot->SetActiveDOFValues(NULL, &vtrajectory.front()[0]);    
    robot->GetJointValues(values);
    FOREACH(it, values)
        sout << *it << " ";
    robot->SetActiveDOFValues(NULL, &vtrajectory.back()[0]);    
    robot->GetJointValues(values);
    FOREACH(it, values)
        sout << *it << " ";

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    Trajectory::TPOINT tp;
    FOREACHR(itsol, vtrajectory) {
        tp.q.resize(0);
        tp.q.insert(tp.q.end(), itsol->begin(), itsol->begin()+robot->GetActiveDOF());
        ptraj->AddPoint(tp);
    }

    ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);

    assert( vtargettraj.size() == ptraj->GetPoints().size() );

    if( strbodytraj.size() > 0 ) {

        boost::shared_ptr<Trajectory> pbodytraj(GetEnv()->CreateTrajectory(taskdata.ptarget->GetDOF()));
        vector<Trajectory::TPOINT>::const_iterator itrobottraj = ptraj->GetPoints().begin();
        
        taskdata.ptarget->GetJointValues(tp.q);
        Transform ttarget = taskdata.ptarget->GetTransform();

        FOREACH(itv, vtargettraj) {
            for(size_t i = 0; i < taskdata._vtargetjoints.size(); ++i)
                tp.q[taskdata._vtargetjoints[i]] = (*itv)[i];
            tp.time = itrobottraj++->time;
            tp.trans = ttarget;
            pbodytraj->AddPoint(tp);
        }
        
        pbodytraj->Write(strbodytraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    }

    if( strsavetraj.size() ) {
        boost::shared_ptr<Trajectory> pfulltraj(GetEnv()->CreateTrajectory(robot->GetDOF()));
        robot->GetFullTrajectoryFromActive(pfulltraj.get(), ptraj.get());
        pfulltraj->Write(strsavetraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    }
   
    return true;
}

bool TaskCagingProblem::BodyTrajectory(ostream& sout, istream& sinput)
{
    BODYTRAJ body;
    string strtraj;
    string cmd;
    char sep = ' ';
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "targettraj") == 0 ) {
            strtraj = getfilename_withseparator(sinput,sep);
        }
        else if( stricmp(cmd.c_str(), "sep") == 0 )
            sinput >> sep;
        else if( stricmp(cmd.c_str(), "target") == 0 ) {
            string name; sinput >> name;
            body.ptarget = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
        }
        else break;

        if( !sinput ) {
            RAVELOG(L"failed\n");
            return false;
        }
    }

    if( body.ptarget == NULL )
        return false;

    body.ptraj.reset(GetEnv()->CreateTrajectory(body.ptarget->GetDOF()));
    if( !body.ptraj->Read(strtraj.c_str(), NULL) ) {
        RAVELOG(L"failed to read %s\n", strtraj.c_str());
        return false;
    }
    _listBodyTrajs.push_back(body);
    return true;
}

bool TaskCagingProblem::JitterTransform(KinBody* pbody, dReal fJitter)
{
    // randomly add small offset to the body until it stops being in collision
    Transform transorig = pbody->GetTransform();
    Transform transnew = transorig;
    int iter = 0;
    while(GetEnv()->CheckCollision(pbody) ) {

        if( iter > 1000 ) {
            return false;
        }

        transnew.trans = transorig.trans + fJitter * Vector(RANDOM_FLOAT()-0.5f, RANDOM_FLOAT()-0.5f, RANDOM_FLOAT()-0.5f);
        pbody->SetTransform(transnew);
        ++iter;
    }

    return true;
}

//TaskCagingFn::TaskCagingFn()
//{
//    _nConstrainedLink = -1;
//    _probot = NULL;
//}
//
//void TaskCagingFn::SetHingeConstraint(RobotBase* probot, int linkindex, const Vector& vanchor, const Vector& vaxis)
//{
//    _probot = probot;
//    _nConstrainedLink = linkindex;
//    assert( _probot != NULL && _nConstrainedLink >= 0 && _nConstrainedLink < _probot->GetLinks().size() );
//
//    _tLinkInitialInv = _probot->GetLinks()[_nConstrainedLink]->GetTransform().inverse();
//    TransformMatrix thinge;
//    thinge.trans = vanchor;
//    
//    Vector vright, vup;
//    if( fabsf(vaxis.x) < 0.9 ) vup.x = 1;
//    else if( fabsf(vaxis.y) < 0.9 ) vup.y = 1;
//    else vup.z = 1;
//    vup = vup - vaxis * dot3(vaxis,vup); vup.normalize3();
//    cross3(vright,vup,vaxis);
//
//    thinge.m[0] = vright.x; thinge.m[1] = vup.x; thinge.m[2] = vaxis.x; thinge.m[3] = 0;
//    thinge.m[4] = vright.y; thinge.m[5] = vup.y; thinge.m[6] = vaxis.y; thinge.m[7] = 0;
//    thinge.m[8] = vright.z; thinge.m[9] = vup.z; thinge.m[10] = vaxis.z; thinge.m[11] = 0;
//
//    _tTaskFrame = thinge;
//    _tTaskFrameInv = thinge.inverse();
//
//    _vConstraintTrans.x = _vConstraintTrans.y = _vConstraintTrans.z = 1; // no translation
//    _vConstraintRot.x = _vConstraintRot.w = 0; // rotate on z only
//    _vConstraintRot.y = _vConstraintRot.z = 1;
//
//    vconfig.resize(_probot->GetActiveDOF());
//    J.resize(_probot->GetActiveDOF()*7);
//    Jinv.resize(_probot->GetActiveDOF()*7);
//    JJt.setbounds(1,7,1,7); // 7x7
//}
//
//bool TaskCagingFn::Constraint(const dReal* pSrcConf, dReal* pDestConf, Transform* ptrans, int settings)
//{
//    assert(J.size() == _probot->GetActiveDOF()*7);
//    assert( pSrcConf != NULL && pDestConf != NULL );
//    const dReal flambda = 1e-4f;
//    static dReal ferror[7];
//
//    memcpy(&vconfig[0], pDestConf, sizeof(dReal)*_probot->GetActiveDOF());
//    dReal fthresh = 0;
//    for(int i = 0; i < _probot->GetActiveDOF(); ++i)
//        fthresh += (pDestConf[i]-pSrcConf[i])*(pDestConf[i]-pSrcConf[i]);
//
//    while(1) {
//        Transform terror; // error need to compensate for
//        KinBody::Link* plink = _probot->GetLinks()[_nConstrainedLink];
//        terror = _tTaskFrameInv * plink->GetTransform() * _tLinkInitialInv * _tTaskFrame;
//        terror.trans *= _vConstraintTrans; terror.rot *= _vConstraintRot; // constrain
//        terror.rot.normalize4(); // normalize
//        terror = _tTaskFrame * terror * _tTaskFrameInv;
//
//        dReal flength2 = terror.trans.lengthsqr3() + terror.rot.lengthsqr4();
//        if( flength2 < 1e-7 )
//            return true;
//
//        _probot->CalculateActiveJacobian(_nConstrainedLink, Vector(0,0,0), &J[0]);
//        _probot->CalculateActiveRotationJacobian(_nConstrainedLink, Vector(0,0,0), &J[3*_probot->GetActiveDOF()]);
//
//        multtrans_to2<dReal, dReal, double>(&J[0], &J[0], 7, _probot->GetActiveDOF(), 7, JJt.getcontent(), false);
//        for(int i = 0; i < 7; ++i) JJt(i+1,i+1) += flambda*flambda;
//
//        bool bSingular = inverse(JJt, 7); // take inverse via LU decomposition
//
//        if( bSingular ) {
//            RAVEPRINT(L"matrix singular, exiting");
//            return false;
//        }
//
//        multtrans<dReal, double, dReal>(&J[0], JJt.getcontent(), 7, _probot->GetActiveDOF(), 7, &Jinv[0], false);
//
//        // add the error to the new conf q_new -= J^(-1) * terror
//        ferror[0] = -terror.trans.x; ferror[1] = -terror.trans.y; ferror[2] = -terror.trans.z;
//        *(Vector*)&ferror[3] = -terror.rot;
//        mult<dReal, dReal, dReal>(&Jinv[0], ferror, _probot->GetActiveDOF(), 7, 1, pDestConf, true);
//
//        dReal fdist = 0;
//        for(int i = 0; i < _probot->GetActiveDOF(); ++i)
//            fdist += (pDestConf[i]-vconfig[i])*(pDestConf[i]-vconfig[i]);
//        if( fdist > fthresh )
//            return false;
//
//        _probot->SetActiveDOFValues(NULL, pDestConf, true);
//    }
//
//    if( GetEnv()->CheckCollision(_probot) || _probot->CheckSelfCollision() )
//        return false;
//
//    _probot->GetActiveDOFValues(pDestConf); // get with joint limits 
//    return true;
//}

bool TaskCagingProblem::Help(ostream& sout, istream& sinput)
{
    sout << "------------------------" << endl
         << "TaskCaging Problem Commands:" << endl;
    GetCommandHelp(sout);
    sout << "------------------------" << endl;
    return true;
}
