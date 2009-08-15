// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "stdafx.h"

#ifndef _WIN32
#include <alloca.h>
#define _alloca alloca
#endif

//---------------------------------------------------------------------
//
// CONSTANTS
//
//---------------------------------------------------------------------

// planning
const dReal STEP_LENGTH        = 0.05f; // radians

const int   NUM_OPT_ITERATIONS = 300;  // optimization iterations
const int   MAX_JUMP           = 5;   // max nodes to attempt to skip

const int   MONITOR_INTERVAL   = 500; // progress monitoring
const dReal DEFAULT_BIAS_PROB  = 0.15f; // probability of selecting goal

const int   UNDEFINED          = -1;


const int   MAX_NODES          = 5000;
const int   MAX_FRAMES         = 500;


//const bool  USE_PQP            = true; //use pqp or use ode collision checker
//---------------------------------------------------------------------
//                            BirrtPlanner
//
// Method:  Default Constructor
//
//---------------------------------------------------------------------


//BirrtPlanner::BirrtPlanner(dReal* pPathArray, RrtNode *pNodeArray, RrtNode** pPathNodes)
BirrtPlanner::BirrtPlanner(EnvironmentBase* penv) : PlannerBase(penv)
{
    _pRobot = NULL;
    _numNodes = 0;
    _pActiveNode = NULL;
    _pConnectNode = NULL;
    _pTargetConfig = NULL;
    _numFrames = 0;

    _pathLength = 0;
    _pOptCheckRecords = new ColCheckRecord[NUM_OPT_ITERATIONS];
    _iNumOptRec = 0;


    _towardBiasProb = DEFAULT_BIAS_PROB;

    //these shouldn't be de-allocated so that multiple calls of this planner are faster
    _pForwardTree = new NodeTree(false);
    _pBackwardTree = new NodeTree(true);


}


//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  Destructor
//
//---------------------------------------------------------------------
BirrtPlanner::~BirrtPlanner()
{
  if (_pOptCheckRecords)
    delete [] _pOptCheckRecords;
}

//clean up and return
bool BirrtPlanner::CleanUpReturn(bool retval)
{
    if(bdelete_distmetric)
        delete _parameters.pdistmetric;

    delete _pForwardTree->_pMakeNext;
    delete _pBackwardTree->_pMakeNext;

    _pForwardTree->DeleteNodes();
    _pBackwardTree->DeleteNodes();

    return retval;
}

//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  InitPlan
//
//! initialize the planner
//
//---------------------------------------------------------------------
bool BirrtPlanner::InitPlan(RobotBase* pbase, const PlannerParameters* pparams)
{
    RAVELOG(L"Initializing Planner\n");
    if( pparams != NULL )
        _parameters = *pparams;
    else
    {
        RAVELOGA("BiRRT::InitPlan - Error: No parameters passed in to initialization");
        return false;
    }

    _pRobot = pbase;

    // get the initial and goal configs
    _pInitConfig = _parameters.vinitialconfig;


    assert(_pInitConfig.size() != 0);
    if(_CheckCollision(&_pInitConfig[0]))
    {
      RAVELOGA("BirrtPlanner::InitPlan - Error: Initial configuration in collision\n");
      return false;
    }

    if( _parameters.pdistmetric == NULL )
    {
        RAVELOG(L"RrtPlanner: Using Default Distance Function\n");
        bdelete_distmetric = true;
        _parameters.pdistmetric = new BirrtDistanceMetric(_pRobot,0.0001f);
    }
    else
        bdelete_distmetric = false;

    if( _pRobot == NULL || _parameters.pdistmetric == NULL ) {
        _pRobot = NULL;
        return false;
    }

    // initialize the jointResolutions
    _pRobot->GetActiveDOFResolutions(_jointResolution);
    if( _vnodes.capacity() < 512 )
        _vnodes.reserve(512);

    _vectrans.resize(_pRobot->GetLinks().size());

    // invert for speed
    _jointResolutionInv.resize(_jointResolution.size());
    
    for (int i = 0; i < _pRobot->GetActiveDOF(); i++) {
        _jointResolutionInv[i] = (dReal)1.0/_jointResolution[i];
        RAVELOG(L"   joint Resolution %d : %f   joint Resolution inv: %f\n", i, _jointResolution[i], _jointResolutionInv[i]);
    }

    _pRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);

    checkConfig.resize(_pRobot->GetActiveDOF());

    _randomConfig.resize(_pRobot->GetActiveDOF());
    _validRange.resize(_pRobot->GetActiveDOF());
    _jointIncrement.resize(_pRobot->GetActiveDOF());


    if( (int)_parameters.vinitialconfig.size() != _pRobot->GetActiveDOF() )
    {
        RAVELOGA("BirrtPlanner::InitPlan - Error: Initial configuration size is different from number of active dofs.\n");
	return false; //_pRobot->GetActiveDOFValues(_parameters.vinitialconfig);

    }
    for (int i = 0; i < _pRobot->GetActiveDOF(); i++) {
        _validRange[i] = _upperLimit[i] - _lowerLimit[i];
        assert(_validRange[i] > 0);
    }

 
    // reset all RRT parameters
    _numNodes = 0;
    _pConnectNode = NULL;

    // set up the initial state
    _pActiveNode = new RrtNode((int)_parameters.vinitialconfig.size(),false,0);
    _pActiveNode->SetConfig(&_pInitConfig[0]);
    _pForwardTree->AddNode(*_pActiveNode);
    delete _pActiveNode;


    RAVELOG(L"Initial State: \n");
    _pForwardTree->GetNode(0)->Print();
    RAVELOG(L"Initial State Node Created\n");



    _pGoalConfig.resize(_pRobot->GetActiveDOF());
    //read in all goals
    int goal_index = 0;
    int num_goals = 0;
    while(1)
    {
        for(int i = 0 ; i < _pRobot->GetActiveDOF(); i++)
        {
            if(goal_index < (int)_parameters.vgoalconfig.size())
                _pGoalConfig[i] = _parameters.vgoalconfig[goal_index];
            else
            {
                RAVELOG(L"BirrtPlanner::InitPlan - Error: goals are improperly specified:\n");
                return false;
            }
            goal_index++;
        }



        if(_CheckCollision(&_pGoalConfig[0]))
        {
            RAVELOG(L"BirrtPlanner::InitPlan - Error: Goal configuration in collision:\n");
            for(int j = 0 ; j < _pRobot->GetActiveDOF(); j++)
            RAVELOG(L"%f ", _pGoalConfig[j]);
            RAVELOG(L"\n");
            return false;
        }





        // set up the goal states
        _pActiveNode = new RrtNode(_pRobot->GetActiveDOF(),true,num_goals);
        _pActiveNode->SetConfig(&_pGoalConfig[0]);
        num_goals++;

        _pBackwardTree->AddNode(*_pActiveNode);


        RAVELOG(L"Goal State(s): \n");
        _pActiveNode->Print();
        delete _pActiveNode;

        if(goal_index == (int)_parameters.vgoalconfig.size())
            break;

    }
  
    RAVELOG(L"Goal State Node(s) Created\n");

    _pForwardTree->_pMakeNext = new MakeNext(_pForwardTree->GetFromGoal(),_pRobot);
    _pBackwardTree->_pMakeNext = new MakeNext(_pBackwardTree->GetFromGoal(),_pRobot);



    bInit = true;
    RAVELOG(L"RrtPlanner::InitPlan - RRT Planner Initialized\n");
    return true;
}


//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  ExecutePlan
//
//! attempt to plan a path
//
//---------------------------------------------------------------------
bool BirrtPlanner::PlanPath(Trajectory* ptraj, std::ostream* pOutStream)
{
    if(!bInit)
      RAVELOG(L"BirrtPlanner::PlanPath - Error, planner not initialized\n");


    // the main planning loop
    RAVELOG(L"Starting PlanPath\n");

    bool bContinuePlan = true;
    _bTowardBias = false;

    NodeTree* TreeA = _pForwardTree;
    NodeTree* TreeB = _pBackwardTree;
    NodeTree* ptreetemp = NULL;
    int iclosest = -1;
    int iConnectedA = 1;
    int iConnectedB = -1;
    std::vector<RrtNode> NewNodes;
    NewNodes.reserve(512);

    while(bContinuePlan)
    {
        _PickRandomConfig();

        //Tree A
        iclosest = _FindClosest(TreeA);
        TreeA->_pMakeNext->MakeNodes(TreeA->GetSize(),TreeA->GetNode(iclosest),&_randomConfig,&NewNodes,this);
        
        if(NewNodes.size() > 0)
        {
            iConnectedA = NewNodes.back().GetID();
            RAVELOG(L"TreeA MakeNodes Has Generated: \n");
            for(unsigned int i = 0; i < NewNodes.size(); i++)
                NewNodes[i].Print();
            TreeA->AddNodes(&NewNodes);
            
            //set up randomConfig for treeB
            _randomConfig = *NewNodes.back().GetDataVector();

        }
        else
        {
            RAVELOG(L"TreeA MakeNodes Has Generated No New Nodes\n");
            RAVELOG(L"Using Node %d as target\n",iclosest);
            
            _randomConfig = *TreeA->GetNode(iclosest)->GetDataVector();
            iConnectedA = iclosest;

        }
        //Tree B
        iclosest = _FindClosest(TreeB);
        if(TreeB->_pMakeNext->MakeNodes(TreeB->GetSize(),TreeB->GetNode(iclosest),&_randomConfig,&NewNodes,this))
        {
            RAVELOG(L"TreeB MakeNodes Has Generated: \n");
            for(unsigned int i = 0; i < NewNodes.size(); i++)
                NewNodes[i].Print();
            

            if( NewNodes.size() > 0 )
                iConnectedB = NewNodes.back().GetID();
            else
                iConnectedB = -1;
        }
        else
        {
            RAVELOG(L"TreeB Has Not Reached Target\n");
            iConnectedB = -1;
        }



        TreeB->AddNodes(&NewNodes);
        
        if(iConnectedB != -1)
            bContinuePlan = false;
        else
        {
            ptreetemp = TreeA;        
            TreeA = TreeB;
            TreeB = ptreetemp;
        }

    }

    //figure out which tree is which
    if(TreeB->GetFromGoal())
    {
        _pForwardTree->_iConnected = iConnectedA;
        _pBackwardTree->_iConnected = iConnectedB;
    }
    else
    {
        _pForwardTree->_iConnected = iConnectedB;
        _pBackwardTree->_iConnected = iConnectedA;

    }

    //construct optimized trajectory
    _JoinPathHalves();
    _OptimizePath();
//    _OptimizePathPerDimension(128);
    _CreateTraj(ptraj);


    return CleanUpReturn(true);

}



//////////////////////
// PRIVATE METHODS
//



//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  _CheckCollision
//
//! check collision between chain and environment
//
//---------------------------------------------------------------------
bool BirrtPlanner::_CheckCollision(const dReal *pConfig)
{
    _pRobot->SetActiveDOFValues(NULL, pConfig);//(&_vectrans, NULL, pConfig);
    COLLISIONREPORT report;
    if( GetEnv()->CheckCollision(_pRobot, &report) ) {
        RAVELOG(L"birrt collision: %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
                    
        return true;
    }
    return _pRobot->CheckSelfCollision();
}

//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  _CheckCollision
//
//! check collision along a path segment
//          returns true if there is a collision
//
//---------------------------------------------------------------------
bool BirrtPlanner::_CheckCollision(dReal *pQ0, dReal *pQ1, 
					IntervalType interval)
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
    cerr << "RrtPlanner: ERROR - unknown interval type." << endl;
  }

  // first make sure the end is free
  if (bCheckEnd)
    if (_CheckCollision(pQ1))
      return true;

  // compute  the discretization
  int i, numSteps = 1;
  for (i = 0; i < _pRobot->GetActiveDOF(); i++) {
    int steps = (int)(fabs(pQ1[i] - pQ0[i]) * _jointResolutionInv[i]);
    if (steps > numSteps)
      numSteps = steps;
  }
  //cerr << "CheckCollision: number of steps: " << numSteps << endl;

  // compute joint increments
  for (i = 0; i < _pRobot->GetActiveDOF(); i++)
    _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

  // check for collision along the straight-line path
  // NOTE: this does not check the end config, and may or may
  // not check the start based on the value of 'start'
  //dReal checkConfig[MAX_DOFS];
  for (int f = start; f < numSteps; f++) {
    for (i = 0; i < _pRobot->GetActiveDOF(); i++)
      checkConfig[i] = pQ0[i] + (_jointIncrement[i] * f);
    
    if (_CheckCollision(&checkConfig[0]))
      return true;
  }

  return false;
}


//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  _PickRandomConfig
//
//! pick a random configuration in the configuration space
//
//---------------------------------------------------------------------
void BirrtPlanner::_PickRandomConfig()
{

    for (int i = 0; i < _pRobot->GetActiveDOF(); i++)
        _randomConfig[i] = _lowerLimit[i] + RANDOM_FLOAT(_validRange[i]);
}


//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  _FindClosest
//
//! Find the previously explored state that is closest in
//          terms of some metric to the goal state.
//
//---------------------------------------------------------------------
int BirrtPlanner::_FindClosest(NodeTree* pNodeTree)
{
  int iBest = UNDEFINED;
  dReal minDist = 100000000;
  dReal dist;

  for (int i = 0; i < pNodeTree->GetSize(); i++) 
  {
      dist = _parameters.pdistmetric->Eval(&_randomConfig[0],pNodeTree->GetNode(i)->GetData());
      //RAVELOG(L"dist: %f\n",dist);
      if (dist < minDist) {
        minDist = dist;
        iBest = i;
      }
  }
  
  assert(iBest != UNDEFINED);

  return iBest;

}
//---------------------------------------------------------------------
//                            RrtPlanner
// Method:  _OptimizePath
//
//! optimize the computed path over a number of iterations
//
//---------------------------------------------------------------------
bool BirrtPlanner::_OptimizePath()
{
    bool bResult = false;
    int numSuccessful = 0;
    int origLength = (int)vecpath.size();
    
    // clear the check records
    _iNumOptRec = 0;
    
    std::vector<RrtNode> NewNodes;
    std::vector<RrtNode> temp;
    temp.reserve(origLength);
    int iStart=-1;
    int iGoal=-1;
    int j;
    int iLimit = NUM_OPT_ITERATIONS;


    for (int i = 0; i < iLimit  ; i++)
    {
        temp.clear();
        if((int)vecpath.size() <= 2)
            break;
        iGoal = 1+RANDOM_INT((int)vecpath.size()-1);
        iStart = RANDOM_INT(iGoal);
        //RAVELOG(L"Start: %d   Goal: %d\n",iStart,iGoal);
        if (_pForwardTree->_pMakeNext->MakeNodes(0,&vecpath[iStart],vecpath[iGoal].GetDataVector(),&NewNodes,this)) 
        {
            /*
            RAVELOG(L"New Nodes: \n");
            for(int q = 0; q < (int) NewNodes.size();q++)
                NewNodes[q].Print();

            RAVELOG(L"\n");
            */
            for(j = 0; j <= iStart; j++)
                temp.push_back(vecpath[j]);
            for(j = 0; j < (int)NewNodes.size(); j++)
                temp.push_back(NewNodes[j]);
            for(j = iGoal+1; j < (int)vecpath.size(); j++)
                temp.push_back(vecpath[j]);

            vecpath = temp;

            numSuccessful++;
            bResult = true;
        }
    }
          /*      
            RAVELOG(L"Traj: \n");
            for(j = 0;j< (int)vecpath.size();j++)
            {
                RAVELOG(L"%d: ",j);
                vecpath[j].Print();        
            }
            RAVELOG(L"\n");
         */
        // statistics
        RAVELOG(L"Path Length:  original=%d  final=%d\n", origLength, vecpath.size());
        
        return bResult;
}

bool BirrtPlanner::_OptimizePathPerDimension(int iterations)
{
    bool bResult = false;
    
    // clear the check records
    _iNumOptRec = 0;
    
    vector<dReal> tempvals(_pRobot->GetActiveDOF());
    
    int iStart=-1;
    int iGoal=-1;
    
    for (int i = 0; i < iterations; i++)
    {
        if((int)vecpath.size() <= 2)
            break;
        iGoal = 1+RANDOM_INT((int)vecpath.size()-1);
        iStart = RANDOM_INT(iGoal);

        int iDim = RANDOM_INT(_pRobot->GetActiveDOF());

        dReal dimvalue = vecpath[iStart].GetData()[iDim];
        dReal deltavalue = (vecpath[iGoal].GetData()[iDim]-dimvalue) / (dReal)(iGoal-iStart);
        
        RrtNode prevnode = vecpath[iStart];
        int inode;
        for(inode = iStart; inode < iGoal; ++inode) {
            // check path between inode and inode+1
            RrtNode newnode = vecpath[inode+1];
            newnode.GetData()[iDim] = dimvalue;
            
            for(int j = 0; j < _pRobot->GetActiveDOF(); ++j)
                tempvals[j] = 0.5f * (newnode.GetData()[j] + prevnode.GetData()[j]);
            
            if( _CheckCollision(newnode.GetData()) || _CheckCollision(&tempvals[0]) )
                break;

            prevnode = newnode;
            dimvalue += deltavalue;
        }

        if( inode < iGoal )
            continue;

        dimvalue = vecpath[iStart].GetData()[iDim];
        for(inode = iStart; inode < iGoal; ++inode) {
            vecpath[inode].GetData()[iDim] = dimvalue;
            dimvalue += deltavalue;
        }
    }
    
    return bResult;
}

//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  _CreateTraj
//
//! generate the Trajectory
//
//---------------------------------------------------------------------
bool BirrtPlanner::_CreateTraj(Trajectory* traj)
{
  
    Trajectory::TPOINT p;
    p.q.resize(_pRobot->GetActiveDOF());
    p.qdot.resize(_pRobot->GetActiveDOF());
   
    vector<dReal> pvectemp;


//    RAVELOG(L"Init active config: \n");
//    for(int i = 0; i < currValues.size(); i++)
//       RAVELOG(L"%f ",currValues[i]);
//    RAVELOG(L"\n");


    //RAVELOG(L"Trajectory Points: \n");
    for (unsigned int f = 0; f < vecpath.size(); f++) {
        pvectemp = *vecpath[f].GetDataVector();
        for (int i = 0; i < _pRobot->GetActiveDOF(); i++)
        {
             p.q[i] = pvectemp[i];
             //RAVELOG(L"%f ",p.q[i]);
        }

        traj->AddPoint(p);
        //RAVELOG(L"\n");
    }


    return true;
}


//---------------------------------------------------------------------
//                            BirrtPlanner
// Method:  _JoinPathHalves
//
//! join the two halves of the solution path
//
//---------------------------------------------------------------------
bool BirrtPlanner::_JoinPathHalves()
{
    vector<RrtNode> vectemp;
    //get forward path
    int index = -1;

    
    _pActiveNode = _pForwardTree->GetNode(_pForwardTree->_iConnected);
    while(true)
    {
        //RAVELOG(L"Node Added\n");
        //_pActiveNode->Print();
        vectemp.push_back(*_pActiveNode);
        index = _pActiveNode->GetParent();
        if(index == -1)
            break;
        _pActiveNode = _pForwardTree->GetNode(index);
    }
    


    for(int i = (int)vectemp.size()-1; i >= 0; i--)
        vecpath.push_back(vectemp[i]);

    
    
    if(_pBackwardTree->GetNode(_pBackwardTree->_iConnected)->GetParent() == -1)
        _pActiveNode = _pBackwardTree->GetNode(_pBackwardTree->_iConnected);
    else
    {
        _pActiveNode = _pBackwardTree->GetNode(_pBackwardTree->GetNode(_pBackwardTree->_iConnected)->GetParent());
        
        while(true)
        {
            //RAVELOG(L"Node Added\n");
            //_pActiveNode->Print();
            vecpath.push_back(*_pActiveNode);
            index = _pActiveNode->GetParent();
            if(index == -1)
                break;
            _pActiveNode = _pBackwardTree->GetNode(index);
        }
    }
    /*
    RAVELOG(L"Found Path: \n");
    for(unsigned int i = 0; i < vecpath.size();i++)
        vecpath[i].Print();
    RAVELOG(L"\n");
    */

    RAVELOG(L"Joined Halves\n");


    return true;
}


//---------------------------------------------------------------------
//                            RobotSimulator
// Method:  _InterpolateNodes
//
//! path node interpolation (returns num frames added)
//
//---------------------------------------------------------------------
void BirrtPlanner::_InterpolateNodes(const dReal* pQ0, const dReal* pQ1, Trajectory* traj)
{
    // compute  the discretization
    int i, numSteps = 1;
    for (i = 0; i < _pRobot->GetActiveDOF(); i++) {
        int steps = (int)(fabs(pQ1[i] - pQ0[i]) * _jointResolutionInv[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < _pRobot->GetActiveDOF(); i++)
        _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

    Trajectory::TPOINT p;
    p.q.resize(_pRobot->GetActiveDOF());

    // compute the straight-line path
    for (int f = 1; f <= numSteps; f++) {
        for (i = 0; i < _pRobot->GetActiveDOF(); i++)
            p.q[i] = pQ0[i] + (_jointIncrement[i] * f);
        traj->AddPoint(p);
    }
}


float BirrtPlanner::BirrtGoalMetric::Eval(const void* pConfiguration)
{
   
   dReal dist = sqrt(lengthsqr((dReal*)pConfiguration, (dReal*)pGoalConfig,_robot->GetActiveDOF()));

   if(dist < thresh)
       dist = 0.0f;

   return dist;
 
}


float BirrtPlanner::BirrtDistanceMetric::Eval(const void* c0, const void* c1)
{
    return lengthsqr((dReal*)c0, (dReal*)c1,_robot->GetActiveDOF());
}


BirrtPlanner::MakeNext::MakeNext(bool bFromGoal, RobotBase* robot)
{
    _bFromGoal = bFromGoal;
    _probot = robot; 
    _probot->GetActiveDOFLimits(_lowerLimit,_upperLimit);

}

bool BirrtPlanner::MakeNext::MakeNodes(int TreeSize, RrtNode* StartNode, std::vector<dReal>* targetConfig,std::vector<RrtNode>* voutput,BirrtPlanner* planner)
{
   
    assert(StartNode != NULL);

    bExtendConnect = true;

    startConfig = StartNode->GetData();

    numDOF = (unsigned int)targetConfig->size(); 
    
    success = false;
    normDiff = 0;
    configDiff = 0;
    normDist =0;
    targetReached = false;

    unsigned int i;
    
    diff = (dReal*)_alloca(sizeof(dReal)*numDOF);
    newConfig = (dReal*)_alloca(sizeof(dReal)*numDOF);
    oldConfig = (dReal*)_alloca(sizeof(dReal)*numDOF);
    dist = (dReal*)_alloca(sizeof(dReal)*numDOF);


    voutput->clear();
    
/*
    RAVELOG(L"Starting From Config: \n");
    for(i = 0; i < numDOF; i++)
        RAVELOG(L"%.3f ",startConfig[i]);
    RAVELOG(L"\n");


    RAVELOG(L"Going Toward Target Config: \n");
    for(i = 0; i < numDOF; i++)
        RAVELOG(L"%.3f ",targetConfig->at(i));
    RAVELOG(L"\n");
*/




 
    // calculate diffence vector

    /*
    RAVELOG(L"Diff: \n");
    RAVELOG(L"%.3f ",diff[i]);
    RAVELOG(L"\n");  
    */



    for (i = 0; i < numDOF; i++)
    {
        oldConfig[i] = startConfig[i];
        diff[i] = targetConfig->at(i) - oldConfig[i];
        normDiff += diff[i]*diff[i];
    }
    
    normDiff = sqrt(normDiff);
    //RAVELOG(L"Diff: \n");
    for (i = 0; i < numDOF; i++)
    {
        diff[i] = diff[i]/normDiff;
        //RAVELOG(L"%f ", diff[i]);
    }
    //RAVELOG(L"\n");
    //can have different function depending on _bFromGoal


    do
    {

        normDist =0;
        //int DOFsMatched = 0;
        // when bExtendConnect is false, this loop executes only once
        //RAVELOG(L"Dist: \n");
        for (i = 0; i < numDOF; i++)
        {
            dist[i] = targetConfig->at(i) - oldConfig[i]; 
            //RAVELOG(L"%f ", dist[i]);
            normDist += dist[i]*dist[i]; 
        }
        //RAVELOG(L"\n");
        normDist = sqrt(normDist);

        //get new config
        if(normDist < STEP_LENGTH)
        {
            targetReached = true;
            for (i = 0; i < numDOF; i++)
                newConfig[i] = targetConfig->at(i);
        }
        else
            for (i = 0; i < numDOF; i++)
                newConfig[i] = oldConfig[i] + (STEP_LENGTH * diff[i]);

        if(planner->_CheckCollision(oldConfig,newConfig,OPEN_START))
        {
            RAVELOG(L"Coll!");
            break;
        }
        configDiff = 0.0f;
        for (i = 0; i < numDOF; i++)
        {
            configDiff += fabs(oldConfig[i]-newConfig[i]);
            oldConfig[i] = newConfig[i];
            
        }

        //if theres a significant configuration difference
        if(configDiff != 0.0)
        {

            //create a node with this configuration
            RrtNode* pnewnode = new RrtNode(numDOF,StartNode->GetFromGoal(),TreeSize++);
            if(voutput->size() == 0)
                pnewnode->SetParent(StartNode->GetID());
            else
                pnewnode->SetParent(voutput->back().GetID());


            pnewnode->SetConfig(newConfig);
            voutput->push_back(*pnewnode);
            delete pnewnode;
        }

        if(targetReached)
        {
            success = true;
            break;
        }

    }while(bExtendConnect);  



    return success;
}
