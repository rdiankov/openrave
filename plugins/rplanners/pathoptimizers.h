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
#ifndef  PATH_OPTIMIZER_H
#define  PATH_OPTIMIZER_H

class ShortcutLinearPlanner : public PlannerBase
{
public:
ShortcutLinearPlanner(EnvironmentBasePtr penv) : PlannerBase(penv)
    {
        __description = "path optimizer using linear shortcuts (author: rosen diankov)";
    }
    virtual ~ShortcutLinearPlanner() {}

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new PlannerParameters());
        _parameters->copy(params);
        if( _parameters->_nMaxIterations <= 0 )
            _parameters->_nMaxIterations = 100;
        if( _parameters->_fStepLength <= 0 )
            _parameters->_fStepLength = 0.04;
        _robot = pbase;
        return true;
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new PlannerParameters());
        isParameters >> *_parameters;
        if( _parameters->_nMaxIterations <= 0 )
            _parameters->_nMaxIterations = 100;
        if( _parameters->_fStepLength <= 0 )
            _parameters->_fStepLength = 0.04;
        _robot = pbase;
        return true;
    }

    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }
    
    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        if( !_parameters || !ptraj || ptraj->GetPoints().size() < 2 )
            return false;
        uint32_t basetime = timeGetTime();
        PlannerParametersConstPtr parameters = GetParameters();

        // subsample trajectory and add to list
        list< vector<dReal> > path;
        _SubsampleTrajectory(ptraj,path);
        
        list< vector<dReal> >::iterator startNode, endNode;
        vector< vector<dReal> > vconfigs;

        int nrejected = 0;
        int i = parameters->_nMaxIterations;
        while(i > 0 && nrejected < (int)path.size()+4 && path.size() > 2 ) {

            --i;

            // pick a random node on the path, and a random jump ahead
            int endIndex = 2+(RaveRandomInt()%((int)path.size()-2));
            int startIndex = RaveRandomInt()%(endIndex-1);
        
            startNode = path.begin();
            advance(startNode, startIndex);
            endNode = startNode;
            advance(endNode, endIndex-startIndex);
            nrejected++;

            // check if the nodes can be connected by a straight line
            vconfigs.resize(0);
            if (CollisionFunctions::CheckCollision(parameters,_robot,*startNode, *endNode, OPEN, &vconfigs)) {

                if( nrejected++ > (int)path.size()+8 )
                    break;
                continue;
            }

            ++startNode;
            FOREACHC(itc, vconfigs) {
                path.insert(startNode, *itc);
            }
            // splice out in-between nodes in path
            path.erase(startNode, endNode);
            nrejected = 0;

            if( path.size() <= 2 )
                break;
        }

        ptraj->Clear();
        FOREACH(it, path) {
            ptraj->AddPoint(Trajectory::TPOINT(*it,0));
        }
        RAVELOG_DEBUG(str(boost::format("path optimizing - time=%fs\n")%(0.001f*(float)(timeGetTime()-basetime))));
        return true;
    }

protected:
    void _SubsampleTrajectory(TrajectoryBasePtr ptraj, list< vector<dReal> >& listpoints) const
    {
        PlannerParametersConstPtr parameters = _parameters;
        // subsample trajectory and add to list (have to do this since trajectory waypoints may not be linear)
        if( ptraj->GetTotalDuration() > 0 ) {
            listpoints.push_back(ptraj->GetPoints().at(0).q);
            TrajectoryBase::TPOINT tp;
            float fsampledelta = 0.01f;
            BOOST_ASSERT((int)parameters->_vConfigResolution.size()==parameters->GetDOF());
            for(float fsampletime = fsampledelta; fsampletime < ptraj->GetTotalDuration(); fsampletime += fsampledelta ) {
                ptraj->SampleTrajectory(fsampletime,tp);
                // check whether to add
                for(size_t i = 0; i < tp.q.size(); ++i) {
                    if( RaveFabs(tp.q[i]-listpoints.back()[i]) > parameters->_vConfigResolution[i] ) {
                        listpoints.push_back(tp.q);
                        break;
                    }
                }
            }
        }
        else {
            vector<dReal> q(parameters->GetDOF()), dq(parameters->GetDOF());
            for(size_t ipoint = 1; ipoint < ptraj->GetPoints().size(); ++ipoint) {
                vector<dReal>& q0 = ptraj->GetPoints()[ipoint-1].q;
                vector<dReal>& q1 = ptraj->GetPoints()[ipoint].q;

                dq = q1;
                parameters->_diffstatefn(dq,q0);
                int i, numSteps = 1;
                vector<dReal>::const_iterator itres = parameters->_vConfigResolution.begin();
                for (i = 0; i < parameters->GetDOF(); i++,itres++) {
                    int steps;
                    if( *itres != 0 )
                        steps = (int)(fabs(dq[i]) / *itres);
                    else
                        steps = (int)(fabs(dq[i]) * 100);
                    if (steps > numSteps)
                        numSteps = steps;
                }
                dReal fisteps = dReal(1.0f)/numSteps;
                FOREACH(it,dq) {
                    *it *= fisteps;
                }
                for (int f = 0; f < numSteps; f++) {
                    for (i = 0; i < parameters->GetDOF(); i++)
                        q[i] = q0[i] + dq[i]*f;
                    listpoints.push_back(q);
                }
            }
            listpoints.push_back(ptraj->GetPoints().back().q);
        }
    }

    RobotBasePtr _robot;
    PlannerParametersPtr _parameters;
};
    
//    virtual void _OptimizePathSingle(list<Node*>& path, int numiterations)
//    {
//        if( path.size() <= 2 )
//            return;
//        RAVELOG_DEBUG("optimizing path - path shortcut single dimension\n");
//        PlannerParametersConstPtr params = GetParameters();
//
//        typename list<Node*>::iterator startNode, prevNode, nextNode, endNode;
//        vector< vector<dReal> > vconfigs;
//        vector<dReal> qprev, qnext;
//        vector<dReal> vdists;
//
//        int nrejected = 0;
//        int i = numiterations;
//        while(i > 0 && nrejected < (int)path.size()+4 ) {
//            --i;
//            
//            // pick a random node on the path, and a random jump ahead
//            int endIndex = 2+(RaveRandomInt()%((int)path.size()-2));
//            int startIndex = RaveRandomInt()%(endIndex-1);
//            int dim = RaveRandomInt()%params->GetDOF();
//            
//            nrejected++;
//            startNode = path.begin(); advance(startNode, startIndex);
//            endNode = startNode;
//            advance(endNode, endIndex-startIndex);
//            
//            prevNode = startNode;
//            nextNode = prevNode; ++nextNode;
//            vdists.resize(0);
//            vdists.push_back(0);
//            for(int j = 1; j < endIndex-startIndex; ++j,++nextNode) {
//                vdists.push_back(vdists.back()+params->_distmetricfn((*prevNode)->q,(*nextNode)->q));
//                prevNode = nextNode;
//            }
//
//            if( vdists.back() <= 0 ) {
//                nrejected = 0;
//                ++startNode;
//                path.erase(startNode, endNode);
//                continue;
//            }
//
//            // normalize distances and start checking collision
//            dReal itotaldist = 1.0f/vdists.back();
//
//            // check if the nodes can be connected by a linear interpolation of dim
//            vconfigs.resize(0);
//            prevNode = startNode;
//            nextNode = prevNode; ++nextNode;
//            qprev = (*prevNode)->q;
//            bool bCanConnect = true;
//            for(int j = 1; j < endIndex-startIndex; ++j,++nextNode) {
//                qnext = (*nextNode)->q;
//                qnext[dim] = vdists[j]*itotaldist*((*endNode)->q[dim]-(*startNode)->q[dim])+(*startNode)->q[dim];
//                if (CollisionFunctions::CheckCollision(params,_robot,qprev, qnext, OPEN_START, &vconfigs)) {
//                    bCanConnect = false;
//                    break;
//                }
//                qprev = qnext;
//                prevNode = nextNode;
//            }
//
//            if( !bCanConnect ) {
//                if( nrejected++ > (int)path.size()+8 )
//                    break;
//                continue;
//            }
//
//            ++startNode;
//            FOREACHC(itc, vconfigs)
//                path.insert(startNode, _treeForward._nodes.at(_treeForward.AddNode(-1,*itc)));
//            // splice out in-between nodes in path
//            path.erase(startNode, ++endNode);
//            nrejected = 0;
//
//            if( path.size() <= 2 )
//                return;
//        }
//    }

#endif
