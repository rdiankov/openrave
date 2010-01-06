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
#ifndef  GRASP_GRADIENT_PLANNER_H
#define  GRASP_GRADIENT_PLANNER_H

class GraspGradientPlanner : public PlannerBase
{
public:
    struct GRASP
    {
    GRASP() : fgoaldist(-1),bChecked(false), bProcessed(false) {}

        bool operator <(const GRASP& r) const { return fgraspdist < r.fgraspdist; }
        dReal fgraspdist, fgoaldist;
        Transform tgrasp;
        vector<dReal> qgoal; ///< ik solution that achieves tgrasp
        bool bChecked; ///< set to true if grasp is checked for ik solution
        bool bProcessed; ///< set to true if grasp has already been used in gradient descend
    };

 GraspGradientPlanner(EnvironmentBasePtr penv) : PlannerBase(penv), _parameters(penv), _bInit(false) {
        __description = "Grasp Planning with Stochastic Gradient Descent";
        _report.reset(new COLLISIONREPORT());
    }
    virtual ~GraspGradientPlanner() {}
    
    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.copy(pparams);
        _robot = pbase;
        RobotBase::RobotStateSaver savestate(_robot);

        if( (int)_parameters.vinitialconfig.size() != _parameters.GetDOF() ) {
            RAVELOG_ERRORA(str(boost::format("initial config wrong dim: %d\n")%_parameters.vinitialconfig.size()));
            return false;
        }

        if(CollisionFunctions::CheckCollision(_parameters,_robot,_parameters.vinitialconfig, _report)) {
            RAVELOG_DEBUGA("BirrtPlanner::InitPlan - Error: Initial configuration in collision\n");
            return false;
        }

        if( _parameters._vgrasps.size() == 0 ) {
            RAVELOG_ERRORA("no goal sampler specified\n");
            return false;
        }
        if( !_parameters._ptarget ) {
            RAVELOG_ERRORA("no target specified\n");
            return false;
        }

        if( (int)_parameters.vinitialconfig.size() != _robot->GetActiveDOF() ) {
            RAVELOG_ERRORA(str(boost::format("initial config wrong dim: %d\n")%_parameters.vinitialconfig.size()));
            return false;
        }

        _randomConfig.resize(_parameters.GetDOF());

        _pmanip = _robot->GetActiveManipulator();
        if( (int)_pmanip->GetArmJoints().size() != _robot->GetActiveDOF()) {
            RAVELOG_ERRORA("active dof not equal to arm joints\n");
            return false;
        }

        for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
            if( _pmanip->GetArmJoints()[i] != _robot->GetActiveJointIndex(i) ) {
                RAVELOG_ERRORA("active dof not equal to arm joints\n");
                return false;
            }
        }
            
        // set up the initial state
        if( !!_parameters._constraintfn ) {
            // filter
            _parameters._setstatefn(_parameters.vinitialconfig);
            if( !_parameters._constraintfn(_parameters.vinitialconfig, _parameters.vinitialconfig,0) ) {
                // failed
                RAVELOG_WARNA("initial state rejected by constraint fn\n");
                //return false;
            }
        }

        if( _parameters._nMaxIterations <= 0 )
            _parameters._nMaxIterations = 10000;

        _bInit = true;
        return true;
    }


    /// \param pOutStream returns which goal was chosen
    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        if(!_bInit) {
            RAVELOG_ERRORA("GraspGradientPlanner::PlanPath - Error, planner not initialized\n");
            return false;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());    
        uint32_t basetime = timeGetTime();    
        RobotBase::RobotStateSaver savestate(_robot);

        list<vector<dReal> > listbestpath, listpath;
        dReal bestgraspdist = 1e37f;
    
        bool bSuccess = false;

        // prioritize the grasps and go through each one
        _robot->SetActiveDOFValues(_parameters.vinitialconfig);
        Transform tcurgrasp = _pmanip->GetEndEffectorTransform();

        Transform tobject = _parameters._ptarget->GetTransform();
        vector<GRASP> vgrasps; vgrasps.reserve(_parameters._vgrasps.size());
        for(size_t i = 0; i < _parameters._vgrasps.size(); ++i) {
            Transform tgrasp = tobject * _parameters._vgrasps[i];
            dReal fgraspdist = min((tgrasp.rot-tcurgrasp.rot).lengthsqr4(),(tgrasp.rot+tcurgrasp.rot).lengthsqr4());
            //+0.0f*(tgrasp.trans-tcurgrasp.trans).lengthsqr3();
            if( fgraspdist < _parameters._fGraspDistThresh ) {
                vgrasps.push_back(GRASP());
                vgrasps.back().tgrasp = tgrasp;
                vgrasps.back().fgraspdist = fgraspdist;
            }
        }
    
        if( vgrasps.size() == 0 )
            return false;

        sort(vgrasps.begin(),vgrasps.end());
    
        dReal fConfigThresh = 2.5f; // first search for all grasps whose ik solutions pass this thresh
        bool bContinue = true;
        while(bContinue) {
            bContinue = false;
            FOREACH(itgrasp, vgrasps) {
                if( itgrasp->bProcessed || (itgrasp->bChecked && itgrasp->fgoaldist < 0) )
                    continue;

                RAVELOG_DEBUGA("attempting grasp %d, %f\n", (int)(itgrasp-vgrasps.begin()), itgrasp->fgraspdist);
                RAVELOG_DEBUGA("trans: %f, %f, %f, %f, %f, %f, %f\n",
                               itgrasp->tgrasp.rot.x,itgrasp->tgrasp.rot.y,itgrasp->tgrasp.rot.z,itgrasp->tgrasp.rot.w,itgrasp->tgrasp.trans.x,itgrasp->tgrasp.trans.y,itgrasp->tgrasp.trans.z);
                
                if( StochasticGradientDescent(*itgrasp, fConfigThresh, listpath) ) {
                    listbestpath.swap(listpath);
                    bSuccess = true;
                    break;
                }

                if( itgrasp->bProcessed ) {
                    // find the grasp distance
                    Transform t = _pmanip->GetEndEffectorTransform();
                    dReal graspdist = TransformDistance2(t,itgrasp->tgrasp,0.2f);
                    if( bestgraspdist > graspdist ) {
                        bestgraspdist = graspdist;
                        listbestpath.swap(listpath);
                    }
                }
                else if( itgrasp->bChecked && itgrasp->fgoaldist >= 0 )
                    bContinue = true;
            }

            if( bSuccess )
                break;

            fConfigThresh *= 1.5f;
        }

        FOREACH(it, listbestpath)
            ptraj->AddPoint(Trajectory::TPOINT(*it,0));

        RAVELOG_DEBUGA(str(boost::format("plan %s, path=%d points in %fs\n")%(bSuccess?"success":"failure")%ptraj->GetPoints().size()%(0.001f*(float)(timeGetTime()-basetime))));
    
        return bSuccess;
    }

    virtual RobotBasePtr GetRobot() { return _robot; }

private:

    bool StochasticGradientDescent(GraspGradientPlanner::GRASP& g, dReal fGoalThresh, list<vector<dReal> >& listpath)
    {
        vector<dReal> qbest, q(_robot->GetActiveDOF()),qgoaldir;
        listpath.clear();
        
        _robot->SetActiveDOFValues(_parameters.vinitialconfig);

        if( g.bChecked ) {
            if( g.fgoaldist < 0 )
                return false;
        }
        else {
            g.bChecked = true;

            if( _pmanip->CheckEndEffectorCollision(g.tgrasp, _report) ) {
                RAVELOG_DEBUGA("gripper collision: (%s:%s)x(%s:%s).\n",
                               !!_report->plink1?_report->plink1->GetParent()->GetName().c_str():"",
                               !!_report->plink1?_report->plink1->GetName().c_str():"",
                               !!_report->plink2?_report->plink2->GetParent()->GetName().c_str():"",
                               !!_report->plink2?_report->plink2->GetName().c_str():"");
                if( !(!!_report->plink1 && _report->plink1->GetParent() == _parameters._ptarget) &&
                    !(!!_report->plink2 && _report->plink2->GetParent() == _parameters._ptarget) )
                    return false;
            }

            bool bGetFirstSolution=true;
            if( bGetFirstSolution ) {
                if( !_pmanip->FindIKSolution(g.tgrasp,g.qgoal,true) ) {
                    RAVELOG_DEBUGA("failed to find ik solution\n");
                    return false;
                }
            }
            else {
                // get all solutions and find the closest to initial config
                if( !_pmanip->FindIKSolutions(g.tgrasp,_viksolutions,true) ) {
                    RAVELOG_DEBUGA("failed to find ik solutions\n");
                    return false;
                }

                dReal bestdist=1e30f;
                g.qgoal.resize(0);
                FOREACH(itq,_viksolutions) {
                    dReal dist = _parameters._distmetricfn(_parameters.vinitialconfig,*itq);
                    if( bestdist > dist  ) {
                        bestdist = dist;
                        g.qgoal = *itq;
                    }
                }

                BOOST_ASSERT(g.qgoal.size()>0);
            }

            g.fgoaldist = _parameters._distmetricfn(g.qgoal,_parameters.vinitialconfig);
        }

        if( g.fgoaldist > fGoalThresh )
            return false;

        RAVELOG_INFOA("goaldist: %f\n",g.fgoaldist);
        g.bProcessed = true;

        vector< vector<dReal> > vpath;
        qbest.resize(0);
        dReal bestdist=0;

        listpath.push_back(_parameters.vinitialconfig);
        dReal fGoalStep = 0.25f*g.qgoal.size();
        dReal fdistmult = _parameters._distmetricfn(_parameters.vinitialconfig,g.qgoal);
        if( fdistmult > fGoalStep )
            fdistmult = fGoalStep/fdistmult;
        else
            fdistmult = 1.0f;

        qgoaldir.resize(g.qgoal.size());
        for(size_t i = 0; i < g.qgoal.size(); ++i)
            qgoaldir[i] = (g.qgoal[i] - _parameters.vinitialconfig[i])*fdistmult;
    

        for(int iter = 0; iter < _parameters._nMaxIterations; ++iter) {
            dReal fRadius = 0.2f;
            for(int giter = 0; giter < _parameters._nGradientSamples; ++giter, fRadius*=0.96f) {
                if( giter == 0 ) {
                    dReal fcurdist = _parameters._distmetricfn(listpath.back(),g.qgoal);
                    if( fcurdist < fGoalStep )
                        q = g.qgoal;
                    else {
                        for(size_t i = 0; i < qgoaldir.size(); ++i)
                            q[i] = listpath.back()[i]+qgoaldir[i];
                    }
                }
                else
                    _parameters._sampleneighfn(q,listpath.back(),fRadius);

                vpath.resize(0);
                if( !CollisionFunctions::CheckCollision(_parameters,_robot,listpath.back(),q,OPEN_START,&vpath) ) {
                    BOOST_ASSERT(vpath.size()>0);
                    if( !!_parameters._constraintfn ) {
                        vector<dReal> qnew(_robot->GetActiveDOF());
                        int goodind = -1;
                        FOREACH(itq,vpath) {
                            _robot->SetActiveDOFValues(*itq);

                            // check if grasp is closer than threshold
                            dReal graspdist = TransformDistance2(_pmanip->GetEndEffectorTransform(),g.tgrasp,0.2f);
                            //RAVELOG_DEBUGA("graspdist: %f\n",RaveSqrt(graspdist));
                            if( graspdist > _parameters._fVisibiltyGraspThresh*_parameters._fVisibiltyGraspThresh ) {
                                if( !_parameters._constraintfn(*itq, qnew, 0) )
                                    break;
                                q = qnew;
                            }
                            else
                                q = *itq;

                            ++goodind;
                        }

                        if( goodind < 0 )
                            continue;
                    }
                    else
                        q = vpath.back();

                    // if new sample is closer than the best, accept it
                    dReal dist = _parameters._distmetricfn(q,g.qgoal);
                    if( qbest.size() == 0 || dist < bestdist ) {
                        RAVELOG_DEBUGA("dist: %f\n",dist);
                        qbest = q;
                        bestdist = dist;
                        if( giter == 0 ) // goal reached
                            break;
                    }
                }
            }

            if( qbest.size() == 0 )
                break;
            listpath.push_back(qbest);
            if( bestdist < 0.0001f ) {// we're done
                RAVELOG_INFOA("done after %d iters\n", iter);
                return true;
            }
        }

        return false;
    }

    RobotBase::ManipulatorPtr _pmanip;
    GraspSetParameters _parameters;
    RobotBasePtr         _robot;
    CollisionReportPtr _report;

    std::vector<dReal>         _randomConfig;
    std::vector<std::vector<dReal> > _viksolutions;

    bool _bInit;
};

#endif
