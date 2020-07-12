// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
#ifndef  BIRRT_PLANNER_H
#define  BIRRT_PLANNER_H

#include "rplanners.h"
#include <boost/algorithm/string.hpp>

static const dReal g_fEpsilonDotProduct = RavePow(g_fEpsilon,0.8);

template <typename Node>
class RrtPlanner : public PlannerBase
{
public:

    RrtPlanner(EnvironmentBasePtr penv) : PlannerBase(penv), _treeForward(0)
    {
        __description = "\
:Interface Author:  Rosen Diankov\n\n\
Uses the Rapidly-Exploring Random Trees Algorithm.\n\
";
        RegisterCommand("GetGoalIndex",boost::bind(&RrtPlanner<Node>::GetGoalIndexCommand,this,_1,_2),
                        "returns the goal index of the plan");
        RegisterCommand("GetInitGoalIndices",boost::bind(&RrtPlanner<Node>::GetInitGoalIndicesCommand,this,_1,_2),
                        "returns the start and goal indices");
        _filterreturn.reset(new ConstraintFilterReturn());
    }
    virtual ~RrtPlanner() {
    }

    virtual bool _InitPlan(RobotBasePtr pbase, PlannerParametersPtr params)
    {
        params->Validate();
        _goalindex = -1;
        _startindex = -1;
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        }
        _robot = pbase;

        _uniformsampler->SetSeed(params->_nRandomGeneratorSeed);
        FOREACH(it, params->_listInternalSamplers) {
            (*it)->SetSeed(params->_nRandomGeneratorSeed);
        }

        PlannerParameters::StateSaver savestate(params);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        if( (int)params->vinitialconfig.size() % params->GetDOF() ) {
            RAVELOG_ERROR_FORMAT("env=%d, initial config wrong dim: %d %% %d != 0", GetEnv()->GetId()%params->vinitialconfig.size()%params->GetDOF());
            return false;
        }

        _vecInitialNodes.resize(0);
        _sampleConfig.resize(params->GetDOF());
        // TODO perhaps distmetricfn should take into number of revolutions of circular joints
        _treeForward.Init(shared_planner(), params->GetDOF(), params->_distmetricfn, params->_fStepLength, params->_distmetricfn(params->_vConfigLowerLimit, params->_vConfigUpperLimit));
        std::vector<dReal> vinitialconfig(params->GetDOF());
        for(size_t index = 0; index < params->vinitialconfig.size(); index += params->GetDOF()) {
            std::copy(params->vinitialconfig.begin()+index,params->vinitialconfig.begin()+index+params->GetDOF(),vinitialconfig.begin());
            _filterreturn->Clear();
            if( params->CheckPathAllConstraints(vinitialconfig,vinitialconfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart, CFO_FillCollisionReport, _filterreturn) != 0 ) {
                RAVELOG_DEBUG_FORMAT("env=%d, initial configuration for rrt does not satisfy constraints: %s", GetEnv()->GetId()%_filterreturn->_report.__str__());
                continue;
            }
            _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitialconfig, _vecInitialNodes.size()));
        }

        if( _treeForward.GetNumNodes() == 0 && !params->_sampleinitialfn ) {
            RAVELOG_WARN_FORMAT("env=%d, no initial configurations", GetEnv()->GetId());
            return false;
        }

        return true;
    }

//    /// \brief simple path optimization given a path of dof values
//    virtual void _SimpleOptimizePath(std::deque<dReal>& path, int numiterations)
//    {
//        if( path.size() <= 2 ) {
//            return;
//        }
//        PlannerParametersConstPtr params = GetParameters();
//
//        typename list<Node*>::iterator startNode, endNode;
//        if( !_filterreturn ) {
//            _filterreturn.reset(new ConstraintFilterReturn());
//        }
//        int dof = GetParameters()->GetDOF();
//        int nrejected = 0;
//        int i = numiterations;
//        while(i > 0 && nrejected < (int)path.size()+4 ) {
//            --i;
//
//            // pick a random node on the path, and a random jump ahead
//            int endIndex = 2+(_uniformsampler->SampleSequenceOneUInt32()%((int)path.size()-2));
//            int startIndex = _uniformsampler->SampleSequenceOneUInt32()%(endIndex-1);
//
//            startNode = path.begin();
//            advance(startNode, startIndex);
//            endNode = startNode;
//            advance(endNode, endIndex-startIndex);
//            nrejected++;
//
//            // check if the nodes can be connected by a straight line
//            _filterreturn->Clear();
//            if ( params->CheckPathAllConstraints(*startNode, *endNode, std::vector<dReal>(), std::vector<dReal>(), 0, IT_Open, 0xffff|CFO_FillCheckedConfiguration, _filterreturn) != 0 ) {
//                if( nrejected++ > (int)path.size()+8 ) {
//                    break;
//                }
//                continue;
//            }
//
//            ++startNode;
//            OPENRAVE_ASSERT_OP(_filterreturn->_configurations.size()%dof,==,0);
//            for(std::vector<dReal>::iterator itvalues = _filterreturn->_configurations.begin(); itvalues != _filterreturn->_configurations.end(); itvalues += dof) {
//                path.insert(startNode, std::vector<dReal>(itvalues,itvalues+dof));
//            }
//            // splice out in-between nodes in path
//            path.erase(startNode, endNode);
//            nrejected = 0;
//
//            if( path.size() <= 2 ) {
//                return;
//            }
//        }
//    }

    /// \brief simple path optimization given a path of dof values. Every _parameters->GetDOF() are one point in the path
    virtual void _SimpleOptimizePath(std::deque<dReal>& path, int numiterations)
    {
        PlannerParametersConstPtr params = GetParameters();
        const int dof = params->GetDOF();
        if( (int)path.size() <= 2*dof ) {
            return;
        }

        deque<dReal>::iterator startNode, endNode, itconfig;
        if( !_filterreturn ) {
            _filterreturn.reset(new ConstraintFilterReturn());
        }
        int nrejected = 0;
        int curiter = numiterations;
        std::vector<dReal> vstart(dof), vend(dof), vdiff0(dof), vdiff1(dof);
        while(curiter > 0 && nrejected < (int)path.size()+4*dof ) {
            --curiter;

            // pick a random node on the path, and a random jump ahead
            int endIndex = 2+(_uniformsampler->SampleSequenceOneUInt32()%((int)path.size()/dof-2));
            int startIndex = _uniformsampler->SampleSequenceOneUInt32()%(endIndex-1);

            startNode = path.begin();
            advance(startNode, startIndex*dof);
            endNode = startNode;
            advance(endNode, (endIndex-startIndex)*dof);
            nrejected++;

            // check if the nodes are in a straight line and if yes, then check different node
            std::copy(startNode, startNode+dof, vstart.begin());
            std::copy(endNode-dof, endNode, vend.begin());
            vdiff0 = vend;
            params->_diffstatefn(vdiff0, vstart);
            bool bcolinear = true;
            // take the midpoint since that is the most stable and check if it is collinear
            {
                itconfig = startNode + ((endIndex-startIndex)/2)*dof;
                std::copy(itconfig, itconfig+dof, vdiff1.begin());
                params->_diffstatefn(vdiff1, vstart);
                dReal dotproduct=0,x0length2=0,x1length2=0;
                for(int idof = 0; idof < dof; ++idof) {
                    dotproduct += vdiff0[idof]*vdiff1[idof];
                    x0length2 += vdiff0[idof]*vdiff0[idof];
                    x1length2 += vdiff1[idof]*vdiff1[idof];
                }
                if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) > g_fEpsilonDotProduct ) {
                    //RAVELOG_INFO_FORMAT("env=%d, colinear: %.15e, %.15e", GetEnv()->GetId()%RaveFabs(dotproduct * dotproduct - x0length2*x1length2)%(dotproduct/RaveSqrt(x0length2*x1length2)));
                    bcolinear = false;
                    break;
                }
            }

            if( bcolinear ) {
                continue;
            }

            // check if the nodes can be connected by a straight line
            _filterreturn->Clear();
            if ( params->CheckPathAllConstraints(vstart, vend, std::vector<dReal>(), std::vector<dReal>(), 0, IT_Open, 0xffff|CFO_FillCheckedConfiguration, _filterreturn) != 0 ) {
                if( nrejected++ > (int)path.size()+8 ) {
                    break;
                }
                continue;
            }

            startNode += dof;
            OPENRAVE_ASSERT_OP(_filterreturn->_configurations.size()%dof,==,0);
            // need to copy _filterreturn->_configurations between startNode and endNode
            size_t ioffset=endNode-startNode;
            if( ioffset > 0 ) {
                if( ioffset <= _filterreturn->_configurations.size() ) {
                    std::copy(_filterreturn->_configurations.begin(), _filterreturn->_configurations.begin()+ioffset, startNode);
                }
                else {
                    std::copy(_filterreturn->_configurations.begin(), _filterreturn->_configurations.end(), startNode);
                    // have to remove nodes
                    path.erase(startNode+_filterreturn->_configurations.size(), endNode);
                }
            }
            if( ioffset < _filterreturn->_configurations.size() ) {
                // insert the rest of the continue
                path.insert(endNode, _filterreturn->_configurations.begin()+ioffset, _filterreturn->_configurations.end());
            }

            nrejected = 0;
            if( (int)path.size() <= 2*dof ) {
                return;
            }
        }
    }

    bool GetGoalIndexCommand(std::ostream& os, std::istream& is)
    {
        os << _goalindex;
        return !!os;
    }

    bool GetInitGoalIndicesCommand(std::ostream& os, std::istream& is)
    {
        os << _startindex << " " << _goalindex;
        return !!os;
    }

protected:
    RobotBasePtr _robot;
    std::vector<dReal> _sampleConfig;
    int _goalindex, _startindex;
    SpaceSamplerBasePtr _uniformsampler;
    ConstraintFilterReturnPtr _filterreturn;
    std::deque<dReal> _cachedpath;

    SpatialTree< Node > _treeForward;
    std::vector< NodeBase* > _vecInitialNodes;

    inline boost::shared_ptr<RrtPlanner> shared_planner() {
        return boost::static_pointer_cast<RrtPlanner>(shared_from_this());
    }
    inline boost::shared_ptr<RrtPlanner const> shared_planner_const() const {
        return boost::static_pointer_cast<RrtPlanner const>(shared_from_this());
    }
};

class BirrtPlanner : public RrtPlanner<SimpleNode>
{
public:
    BirrtPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv), _treeBackward(1)
    {
        __description += "Bi-directional RRTs. See\n\n\
- J.J. Kuffner and S.M. LaValle. RRT-Connect: An efficient approach to single-query path planning. In Proc. IEEE Int'l Conf. on Robotics and Automation (ICRA'2000), pages 995-1001, San Francisco, CA, April 2000.";
        RegisterCommand("DumpTree", boost::bind(&BirrtPlanner::_DumpTreeCommand,this,_1,_2),
                        "dumps the source and goal trees to $OPENRAVE_HOME/birrtdump.txt. The first N values are the DOF values, the last value is the parent index.\n\
Some python code to display data::\n\
\n\
  sourcetree=loadtxt(os.path.join(RaveGetHomeDirectory(),'sourcetree.txt'),delimiter=' ,')\n\
  hs=env.plot3(sourcetree,5,[1,0,0])\n\
  sourcedist = abs(sourcetree[:,0]-x[0]) + abs(sourcetree[:,1]-x[1])\n\
  robot.SetActiveDOFValues(sourcetree[argmin(sourcedist)])\n\
\n\
");
        _nValidGoals = 0;
    }
    virtual ~BirrtPlanner() {
    }

    struct GOALPATH
    {
        GOALPATH() : startindex(-1), goalindex(-1), length(0) {
        }
        vector<dReal> qall;
        int startindex, goalindex;
        dReal length;
    };

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new RRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        _fGoalBiasProb = dReal(0.01);
        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        // TODO perhaps distmetricfn should take into number of revolutions of circular joints
        _treeBackward.Init(shared_planner(), _parameters->GetDOF(), _parameters->_distmetricfn, _parameters->_fStepLength, _parameters->_distmetricfn(_parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit));

        //read in all goals
        if( (_parameters->vgoalconfig.size() % _parameters->GetDOF()) != 0 ) {
            RAVELOG_ERROR_FORMAT("env=%d, BirrtPlanner::InitPlan - Error: goals are improperly specified", GetEnv()->GetId());
            _parameters.reset();
            return false;
        }

        vector<dReal> vgoal(_parameters->GetDOF());
        _vecGoalNodes.resize(0);
        _nValidGoals = 0;
        for(size_t igoal = 0; igoal < _parameters->vgoalconfig.size(); igoal += _parameters->GetDOF()) {
            std::copy(_parameters->vgoalconfig.begin()+igoal,_parameters->vgoalconfig.begin()+igoal+_parameters->GetDOF(),vgoal.begin());
            int ret = _parameters->CheckPathAllConstraints(vgoal,vgoal,std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart);
            if( ret == 0 ) {
                _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
                _nValidGoals++;
            }
            else {
                RAVELOG_WARN_FORMAT("env=%d, goal %d fails constraints with 0x%x", GetEnv()->GetId()%igoal%ret);
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    int ret = _parameters->CheckPathAllConstraints(vgoal,vgoal,std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart);
                }
                _vecGoalNodes.push_back(NULL); // have to push back dummy or else indices will be messed up
            }
        }

        if( _treeBackward.GetNumNodes() == 0 && !_parameters->_samplegoalfn ) {
            RAVELOG_WARN_FORMAT("env=%d, no goals specified", GetEnv()->GetId());
            _parameters.reset();
            return false;
        }

        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 10000;
        }

        _vgoalpaths.resize(0);
        if( _vgoalpaths.capacity() < _parameters->_minimumgoalpaths ) {
            _vgoalpaths.reserve(_parameters->_minimumgoalpaths);
        }
        RAVELOG_DEBUG_FORMAT("env=%d, BiRRT Planner Initialized, initial=%d, goal=%d, step=%f", GetEnv()->GetId()%_vecInitialNodes.size()%_treeBackward.GetNumNodes()%_parameters->_fStepLength);
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _goalindex = -1;
        _startindex = -1;
        if(!_parameters) {
            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, BirrtPlanner::PlanPath - Error, planner not initialized")%GetEnv()->GetId()), PS_Failed);
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint64_t basetimeus = utils::GetMonotonicTime();
        
        int constraintFilterOptions = 0xffff|CFO_FillCheckedConfiguration;
        if (planningoptions & PO_AddCollisionStatistics) {
            constraintFilterOptions = constraintFilterOptions|CFO_FillCollisionReport;
        }

        // the main planning loop
        PlannerStatus planningstatus;
        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        SpatialTreeBase* TreeA = &_treeForward;
        SpatialTreeBase* TreeB = &_treeBackward;
        NodeBase* iConnectedA=NULL, *iConnectedB=NULL;
        int iter = 0;

        bool bSampleGoal = true;
        PlannerProgress progress;
        PlannerAction callbackaction=PA_None;
        while(_vgoalpaths.size() < _parameters->_minimumgoalpaths && iter < 3*_parameters->_nMaxIterations) {
            RAVELOG_VERBOSE_FORMAT("env=%d, iter=%d, forward=%d, backward=%d", GetEnv()->GetId()%(iter/3)%_treeForward.GetNumNodes()%_treeBackward.GetNumNodes());
            ++iter;

            // have to check callbacks at the beginning since code can continue
            callbackaction = _CallCallbacks(progress);
            if( callbackaction ==  PA_Interrupt ) {
                return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
            }
            else if( callbackaction == PA_ReturnWithAnySolution ) {
                if( _vgoalpaths.size() > 0 ) {
                    break;
                }
            }

            if( _parameters->_nMaxPlanningTime > 0 ) {
                uint64_t elapsedtime = utils::GetMonotonicTime()-basetimeus;
                if( elapsedtime >= 1000*_parameters->_nMaxPlanningTime ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, time exceeded (%d[us] > %d[us]) so breaking. iter=%d < %d", GetEnv()->GetId()%elapsedtime%(1000*_parameters->_nMaxPlanningTime)%(iter/3)%_parameters->_nMaxIterations);
                    break;
                }
            }


            if( !!_parameters->_samplegoalfn ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE(str(boost::format("env=%d, inserting new goal index %d")%GetEnv()->GetId()%_vecGoalNodes.size()));
                    _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
                    _nValidGoals++;
                }
            }
            if( !!_parameters->_sampleinitialfn ) {
                vector<dReal> vinitial;
                if( _parameters->_sampleinitialfn(vinitial) ) {
                    RAVELOG_VERBOSE(str(boost::format("env=%d, inserting new initial %d")%GetEnv()->GetId()%_vecInitialNodes.size()));
                    _vecInitialNodes.push_back(_treeForward.InsertNode(NULL,vinitial, _vecInitialNodes.size()));
                }
            }

            _sampleConfig.resize(0);
            if( (bSampleGoal || _uniformsampler->SampleSequenceOneReal() < _fGoalBiasProb) && _nValidGoals > 0 ) {
                bSampleGoal = false;
                // sample goal as early as possible
                uint32_t bestgoalindex = -1;
                for(size_t testiter = 0; testiter < _vecGoalNodes.size()*3; ++testiter) {
                    uint32_t sampleindex = _uniformsampler->SampleSequenceOneUInt32();
                    uint32_t goalindex = sampleindex%_vecGoalNodes.size();
                    if( !_vecGoalNodes.at(goalindex) ) {
                        continue; // dummy
                    }
                    // make sure goal is not already found
                    bool bfound = false;
                    FOREACHC(itgoalpath,_vgoalpaths) {
                        if( goalindex == (uint32_t)itgoalpath->goalindex ) {
                            bfound = true;
                            break;
                        }
                    }
                    if( !bfound) {
                        bestgoalindex = goalindex;
                        break;
                    }
                }
                if( bestgoalindex != uint32_t(-1) ) {
                    _treeBackward.GetVectorConfig(_vecGoalNodes.at(bestgoalindex), _sampleConfig);
                }
            }

            if( _sampleConfig.size() == 0 ) {
                if( !_parameters->_samplefn(_sampleConfig) ) {
                    continue;
                }
            }

            // extend A
            ExtendType et = TreeA->Extend(_sampleConfig, iConnectedA, false, constraintFilterOptions);

            if (et == ET_Failed && (constraintFilterOptions&CFO_FillCollisionReport)) {
                planningstatus.AddCollisionReport(_treeForward.GetConstraintReport()->_report);
            }

            // although check isn't necessary, having it improves running times
            if( et == ET_Failed ) {
                // necessary to increment iterator in case spaces are not connected
                if( iter > 3*_parameters->_nMaxIterations ) {
                    RAVELOG_WARN_FORMAT("env=%d, iterations exceeded", GetEnv()->GetId());
                    break;
                }
                continue;
            }

            et = TreeB->Extend(TreeA->GetVectorConfig(iConnectedA), iConnectedB, false, constraintFilterOptions);     // extend B toward A

            if (et == ET_Failed && (constraintFilterOptions&CFO_FillCollisionReport)) {
                planningstatus.AddCollisionReport(_treeBackward.GetConstraintReport()->_report);
            }

            if( et == ET_Connected ) {
                // connected, process goal
                _vgoalpaths.push_back(GOALPATH());
                _ExtractPath(_vgoalpaths.back(), TreeA == &_treeForward ? iConnectedA : iConnectedB, TreeA == &_treeBackward ? iConnectedA : iConnectedB);
                int goalindex = _vgoalpaths.back().goalindex;
                int startindex = _vgoalpaths.back().startindex;
                if( IS_DEBUGLEVEL(Level_Debug) ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    ss << "env=" << GetEnv()->GetId() << ", found a goal, start index=" << startindex << " goal index=" << goalindex << ", path length=" << _vgoalpaths.back().length << ", startvalues=[";
                    for(int i = 0; i < _parameters->GetDOF(); ++i) {
                        ss << _vgoalpaths.back().qall.at(i) << ", ";
                    }
                    ss << "]; goalvalues=[";
                    for(int i = 0; i < _parameters->GetDOF(); ++i) {
                        ss << _vgoalpaths.back().qall.at(_vgoalpaths.back().qall.size()-_parameters->GetDOF()+i) << ", ";
                    }
                    ss << "];";
                    RAVELOG_DEBUG(ss.str());
                }
                if( _vgoalpaths.size() >= _parameters->_minimumgoalpaths || _vgoalpaths.size() >= _nValidGoals ) {
                    break;
                }
                bSampleGoal = true;
                // more goals requested, so make sure to remove all the nodes pointing to the current found goal
                _treeBackward.InvalidateNodesWithParent(_vecGoalNodes.at(goalindex));
            }

            swap(TreeA, TreeB);
            iter += 3;
            if( iter > 3*_parameters->_nMaxIterations ) {
                RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d", GetEnv()->GetId()%_parameters->_nMaxIterations);
                break;
            }

            progress._iteration = iter/3;
        }

        if( _vgoalpaths.size() == 0 ) {
            uint64_t elapsedtimeus = utils::GetMonotonicTime()-basetimeus;
            std::string description = str(boost::format(_("env=%d, plan failed in %u[us], iter=%d, nMaxIterations=%d"))%GetEnv()->GetId()%(elapsedtimeus)%(iter/3)%_parameters->_nMaxIterations);
            RAVELOG_WARN(description);
            return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
        }

        vector<GOALPATH>::iterator itbest = _vgoalpaths.begin();
        FOREACH(itpath,_vgoalpaths) {
            if( itpath->length < itbest->length ) {
                itbest = itpath;
            }
        }
        _goalindex = itbest->goalindex;
        _startindex = itbest->startindex;
        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        ptraj->Insert(ptraj->GetNumWaypoints(), itbest->qall, _parameters->_configurationspecification);
        uint64_t elapsedtimeus = utils::GetMonotonicTime()-basetimeus;
        std::string description = str(boost::format(_("env=%d, plan success, iters=%d, path=%d points, computation time=%u[us]\n"))%GetEnv()->GetId()%progress._iteration%ptraj->GetNumWaypoints()%(elapsedtimeus));
        RAVELOG_DEBUG(description);
        PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
        //TODO should use accessor to change description
        //status.description = description; //?

        // Special case. No need to transfer collision information?
        return status;
    }

    virtual void _ExtractPath(GOALPATH& goalpath, NodeBase* iConnectedForward, NodeBase* iConnectedBackward)
    {
//        list< std::vector<dReal> > vecnodes;
//
//        // add nodes from the forward tree
//        SimpleNode* pforward = (SimpleNode*)iConnectedForward;
//        goalpath.startindex = -1;
//        while(1) {
//            vecnodes.push_front(pforward);
//            if(!pforward->rrtparent) {
//                goalpath.startindex = pforward->_userdata;
//                break;
//            }
//            pforward = pforward->rrtparent;
//        }
//
//        // add nodes from the backward tree
//        goalpath.goalindex = -1;
//        SimpleNode *pbackward = (SimpleNode*)iConnectedBackward;
//        while(1) {
//            vecnodes.push_back(pbackward);
//            if(!pbackward->rrtparent) {
//                goalpath.goalindex = pbackward->_userdata;
//                break;
//            }
//            pbackward = pbackward->rrtparent;
//        }
//
//        BOOST_ASSERT( goalpath.goalindex >= 0 && goalpath.goalindex < (int)_vecGoalNodes.size() );
//        _SimpleOptimizePath(vecnodes,10);
//        const int dof = _parameters->GetDOF();
//        goalpath.qall.resize(vecnodes.size()*dof);
//        list<SimpleNode*>::iterator itprev = vecnodes.begin();
//        list<SimpleNode*>::iterator itnext = itprev; itnext++;
//        goalpath.length = 0;
//        vector<dReal>::iterator itq = goalpath.qall.begin();
//        std::copy((*itprev)->q, (*itprev)->q+dof, itq);
//        itq += dof;
//        vector<dReal> vivel(dof,1.0);
//        for(size_t i = 0; i < vivel.size(); ++i) {
//            if( _parameters->_vConfigVelocityLimit.at(i) != 0 ) {
//                vivel[i] = 1/_parameters->_vConfigVelocityLimit.at(i);
//            }
//        }
//
//        while(itnext != vecnodes.end()) {
//            std::copy((*itnext)->q, (*itnext)->q+dof, itq);
//            itprev=itnext;
//            ++itnext;
//            itq += dof;
//        }

        const int dof = _parameters->GetDOF();
        _cachedpath.resize(0);

        // add nodes from the forward tree
        SimpleNode* pforward = (SimpleNode*)iConnectedForward;
        goalpath.startindex = -1;
        while(1) {
            _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
            //vecnodes.push_front(pforward);
            if(!pforward->rrtparent) {
                goalpath.startindex = pforward->_userdata;
                break;
            }
            pforward = pforward->rrtparent;
        }

        // add nodes from the backward tree
        goalpath.goalindex = -1;
        SimpleNode *pbackward = (SimpleNode*)iConnectedBackward;
        while(1) {
            //vecnodes.push_back(pbackward);
            _cachedpath.insert(_cachedpath.end(), pbackward->q, pbackward->q+dof);
            if(!pbackward->rrtparent) {
                goalpath.goalindex = pbackward->_userdata;
                break;
            }
            pbackward = pbackward->rrtparent;
        }

        BOOST_ASSERT( goalpath.goalindex >= 0 && goalpath.goalindex < (int)_vecGoalNodes.size() );
        _SimpleOptimizePath(_cachedpath,10);
        goalpath.qall.resize(_cachedpath.size());
        std::copy(_cachedpath.begin(), _cachedpath.end(), goalpath.qall.begin());
        goalpath.length = 0;
        vector<dReal> vivel(dof,1.0);
        for(size_t i = 0; i < vivel.size(); ++i) {
            if( _parameters->_vConfigVelocityLimit.at(i) != 0 ) {
                vivel[i] = 1/_parameters->_vConfigVelocityLimit.at(i);
            }
        }

        // take distance scaled with respect to velocities with the first and last points only!
        // this is because rrt paths can initially be very complex but simplify down to something simpler.
        std::vector<dReal> vdiff(goalpath.qall.begin(), goalpath.qall.begin()+dof);
        _parameters->_diffstatefn(vdiff, std::vector<dReal>(goalpath.qall.end()-dof, goalpath.qall.end()));
        for(size_t i = 0; i < vdiff.size(); ++i) {
            goalpath.length += RaveFabs(vdiff.at(i))*vivel.at(i);
        }
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is) {
        std::string filename = RaveGetHomeDirectory() + string("/birrtdump.txt");
        getline(is, filename);
        boost::trim(filename);
        RAVELOG_VERBOSE(str(boost::format("dumping rrt tree to %s")%filename));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _treeForward.DumpTree(f);
        _treeBackward.DumpTree(f);
        return true;
    }

protected:
    RRTParametersPtr _parameters;
    SpatialTree< SimpleNode > _treeBackward;
    dReal _fGoalBiasProb;
    std::vector< NodeBase* > _vecGoalNodes;
    size_t _nValidGoals; ///< num valid goals
    std::vector<GOALPATH> _vgoalpaths;
};

class BasicRrtPlanner : public RrtPlanner<SimpleNode>
{
public:
    BasicRrtPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description = "Rosen's Basic RRT planner";
        _fGoalBiasProb = dReal(0.05);
        _bOneStep = false;
        RegisterCommand("DumpTree", boost::bind(&BasicRrtPlanner::_DumpTreeCommand,this,_1,_2),
                        "dumps the source and goal trees to $OPENRAVE_HOME/basicrrtdump.txt. The first N values are the DOF values, the last value is the parent index.\n");
    }
    virtual ~BasicRrtPlanner() {
    }

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new BasicRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        //read in all goals
        int goal_index = 0;
        vector<dReal> vgoal(_parameters->GetDOF());
        _vecGoals.resize(0);
        while(_parameters->vgoalconfig.size() > 0) {
            for(int i = 0; i < _parameters->GetDOF(); i++) {
                if(goal_index < (int)_parameters->vgoalconfig.size())
                    vgoal[i] = _parameters->vgoalconfig[goal_index];
                else {
                    RAVELOG_ERROR_FORMAT("env=%d, BasicRrtPlanner::InitPlan - Error: goals are improperly specified", GetEnv()->GetId());
                    _parameters.reset();
                    return false;
                }
                goal_index++;
            }

            if( GetParameters()->CheckPathAllConstraints(vgoal,vgoal, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart) == 0 ) {
                _vecGoals.push_back(vgoal);
            }
            else {
                RAVELOG_WARN_FORMAT("env=%d, goal in collision", GetEnv()->GetId());
            }

            if(goal_index == (int)_parameters->vgoalconfig.size()) {
                break;
            }
        }

        if(( _vecGoals.size() == 0) && !_parameters->_goalfn ) {
            RAVELOG_WARN_FORMAT("env=%d, no goals or goal function specified", GetEnv()->GetId());
            _parameters.reset();
            return false;
        }

        _bOneStep = _parameters->_nRRTExtentType == 1;
        RAVELOG_DEBUG_FORMAT("env=%d, BasicRrtPlanner initialized _nRRTExtentType=%d", GetEnv()->GetId()%_parameters->_nRRTExtentType);
        return true;
    }

    PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        if(!_parameters) {
            std::string description = str(boost::format("env=%d, BasicRrtPlanner::PlanPath - Error, planner not initialized")%GetEnv()->GetId());
            RAVELOG_WARN(description);
            return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = utils::GetMilliTime();

        NodeBasePtr lastnode; // the last node visited by the RRT
        NodeBase* bestGoalNode = NULL; // the best goal node found already by the RRT. If this is not NULL, then RRT succeeded
        dReal fBestGoalNodeDist = 0; // configuration distance from initial position to the goal node

        // the main planning loop
        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        std::vector<dReal> vtempinitialconfig;
        PlannerAction callbackaction = PA_None;
        PlannerProgress progress;
        int iter = 0;
        _goalindex = -1; // index into vgoalconfig if the goal is found
        _startindex = -1;

        int numfoundgoals = 0;

        while(iter < _parameters->_nMaxIterations) {
            iter++;
            if( !!bestGoalNode && iter >= _parameters->_nMinIterations ) {
                break;
            }
            if( !!_parameters->_samplegoalfn ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, found goal", GetEnv()->GetId());
                    _vecGoals.push_back(vgoal);
                }
            }
            if( !!_parameters->_sampleinitialfn ) {
                vector<dReal> vinitial;
                if( _parameters->_sampleinitialfn(vinitial) ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, found initial", GetEnv()->GetId());
                    _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitial, _vecInitialNodes.size()));
                }
            }

            if( (iter == 1 || RaveRandomFloat() < _fGoalBiasProb ) && _vecGoals.size() > 0 ) {
                _sampleConfig = _vecGoals[RaveRandomInt()%_vecGoals.size()];
            }
            else if( !_parameters->_samplefn(_sampleConfig) ) {
                continue;
            }

            // extend A
            ExtendType et = _treeForward.Extend(_sampleConfig, lastnode, _bOneStep);

            if( et == ET_Connected ) {
                FOREACH(itgoal, _vecGoals) {
                    if( _parameters->_distmetricfn(*itgoal, _treeForward.GetVectorConfig(lastnode)) < 2*_parameters->_fStepLength ) {
                        SimpleNode* pforward = (SimpleNode*)lastnode;
                        while(1) {
                            if(!pforward->rrtparent) {
                                break;
                            }
                            pforward = pforward->rrtparent;
                        }
                        vtempinitialconfig = _treeForward.GetVectorConfig(pforward); // GetVectorConfig returns the same reference, so need to make a copy
                        dReal fGoalNodeDist = _parameters->_distmetricfn(vtempinitialconfig, _treeForward.GetVectorConfig(lastnode));
                        if( !bestGoalNode || fBestGoalNodeDist > fGoalNodeDist ) {
                            bestGoalNode = lastnode;
                            fBestGoalNodeDist = fGoalNodeDist;
                            _goalindex = (int)(itgoal-_vecGoals.begin());
                        }
                        if( iter >= _parameters->_nMinIterations ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, found goal index: %d", GetEnv()->GetId()%_goalindex);
                            break;
                        }
                    }
                }
            }

            // check the goal heuristic more often
            if(( et != ET_Failed) && !!_parameters->_goalfn ) {
                // have to check all the newly created nodes since anyone could be already in the goal (do not have to do this with _vecGoals since that is being sampled)
                bool bfound = false;
                SimpleNode* ptestnode = (SimpleNode*)lastnode;
                while(!!ptestnode && ptestnode->_userdata==0) { // when userdata is 0, then it hasn't been checked for goal yet
                    if( _parameters->_goalfn(_treeForward.GetVectorConfig(ptestnode)) <= 1e-4f ) {
                        bfound = true;
                        numfoundgoals++;
                        ptestnode->_userdata = 1;
                        SimpleNode* pforward = ptestnode;
                        while(1) {
                            if(!pforward->rrtparent) {
                                break;
                            }
                            pforward = pforward->rrtparent;
                        }
                        vtempinitialconfig = _treeForward.GetVectorConfig(pforward); // GetVectorConfig returns the same reference, so need to make a copy
                        dReal fGoalNodeDist = _parameters->_distmetricfn(vtempinitialconfig, _treeForward.GetVectorConfig(ptestnode));
                        if( !bestGoalNode || fBestGoalNodeDist > fGoalNodeDist ) {
                            bestGoalNode = ptestnode;
                            fBestGoalNodeDist = fGoalNodeDist;
                            _goalindex = -1;
                            RAVELOG_DEBUG_FORMAT("env=%d, found node at goal at dist=%f at %d iterations, computation time=%fs", GetEnv()->GetId()%fBestGoalNodeDist%iter%(0.001f*(float)(utils::GetMilliTime()-basetime)));
                        }
                    }

                    ptestnode->_userdata = 1;
                    ptestnode = ptestnode->rrtparent;
                }
                if( bfound ) {
                    if( iter >= _parameters->_nMinIterations ) {
                        // check how many times we've got a goal?
                        if( numfoundgoals >= (int)_parameters->_minimumgoalpaths ) {
                            break;
                        }
                    }
                }
            }

            // check if reached any goals
            if( iter > _parameters->_nMaxIterations ) {
                RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d\n", GetEnv()->GetId()%_parameters->_nMaxIterations);
                break;
            }

            if( !!bestGoalNode && _parameters->_nMaxPlanningTime > 0 ) {
                uint32_t elapsedtime = utils::GetMilliTime()-basetime;
                if( elapsedtime >= _parameters->_nMaxPlanningTime ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, time exceeded (%dms) so breaking with bestdist=%f", GetEnv()->GetId()%elapsedtime%fBestGoalNodeDist);
                    break;
                }
            }

            progress._iteration = iter;
            callbackaction = _CallCallbacks(progress);
            if( callbackaction ==  PA_Interrupt ) {
                return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
            }
            else if( callbackaction == PA_ReturnWithAnySolution ) {
                if( !!bestGoalNode ) {
                    break;
                }
            }
        }

        if( !bestGoalNode ) {
            std::string description = str(boost::format("env=%d, plan failed, %fs")%GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
            RAVELOG_DEBUG(description);
            return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
        }

        const int dof = _parameters->GetDOF();
        _cachedpath.resize(0);

        // add nodes from the forward tree
        SimpleNode* pforward = (SimpleNode*)bestGoalNode;
        while(1) {
            _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
            if(!pforward->rrtparent) {
                break;
            }
            pforward = pforward->rrtparent;
        }

        _SimpleOptimizePath(_cachedpath,10);
        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        std::vector<dReal> vinsertvalues(_cachedpath.begin(), _cachedpath.end());
        ptraj->Insert(ptraj->GetNumWaypoints(), vinsertvalues, _parameters->_configurationspecification);

        PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
        RAVELOG_DEBUG_FORMAT("env=%d, plan success, path=%d points computation time=%fs, maxPlanningTime=%f", GetEnv()->GetId()%ptraj->GetNumWaypoints()%((0.001f*(float)(utils::GetMilliTime()-basetime)))%(0.001*_parameters->_nMaxPlanningTime));
        return status;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is) {
        std::string filename = RaveGetHomeDirectory() + string("/basicrrtdump.txt");
        getline(is, filename);
        boost::trim(filename);
        RAVELOG_VERBOSE(str(boost::format("dumping rrt tree to %s")%filename));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _treeForward.DumpTree(f);
        return true;
    }
protected:
    boost::shared_ptr<BasicRRTParameters> _parameters;
    dReal _fGoalBiasProb;
    bool _bOneStep;
    std::vector< std::vector<dReal> > _vecGoals;
    int _nValidGoals; ///< num valid goals
};

class ExplorationPlanner : public RrtPlanner<SimpleNode>
{
public:
    ExplorationPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nRRT-based exploration planner";
    }
    virtual ~ExplorationPlanner() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ExplorationParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, ExplorationPlanner::InitPlan - RRT Planner Initialized", GetEnv()->GetId());
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _goalindex = -1;
        _startindex = -1;
        if( !_parameters ) {
            return OPENRAVE_PLANNER_STATUS(PS_Failed);
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        vector<dReal> vSampleConfig;

        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        int iter = 0;
        while(iter < _parameters->_nMaxIterations && _treeForward.GetNumNodes() < _parameters->_nExpectedDataSize ) {
            ++iter;

            if( RaveRandomFloat() < _parameters->_fExploreProb ) {
                // explore
                int inode = RaveRandomInt()%_treeForward.GetNumNodes();
                NodeBase* pnode = _treeForward.GetNodeFromIndex(inode);

                if( !_parameters->_sampleneighfn(vSampleConfig, _treeForward.GetVectorConfig(pnode), _parameters->_fStepLength) ) {
                    continue;
                }
                if( GetParameters()->CheckPathAllConstraints(_treeForward.GetVectorConfig(pnode), vSampleConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart) == 0 ) {
                    _treeForward.InsertNode(pnode, vSampleConfig, 0);
                    GetEnv()->UpdatePublishedBodies();
                    RAVELOG_DEBUG_FORMAT("env=%d, size %d", GetEnv()->GetId()%_treeForward.GetNumNodes());
                }
            }
            else {     // rrt extend
                if( !_parameters->_samplefn(vSampleConfig) ) {
                    continue;
                }
                NodeBasePtr plastnode;
                if( _treeForward.Extend(vSampleConfig, plastnode, true) == ET_Connected ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, size %d", GetEnv()->GetId()%_treeForward.GetNumNodes());
                }
            }
        }

        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        // save nodes to trajectory
        std::vector<NodeBase*> vnodes;
        _treeForward.GetNodesVector(vnodes);
        FOREACH(itnode, vnodes) {
            ptraj->Insert(ptraj->GetNumWaypoints(), _treeForward.GetVectorConfig(*itnode), _parameters->_configurationspecification);
        }
        return OPENRAVE_PLANNER_STATUS(PS_HasSolution);
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

private:
    boost::shared_ptr<ExplorationParameters> _parameters;

};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(BirrtPlanner::GOALPATH)
#endif

#endif
