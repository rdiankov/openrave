// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "rplanners.h"

class GraspGradientPlanner : public PlannerBase
{
public:
    struct GRASP
    {
        GRASP() : fgoaldist(-1),bChecked(false), bProcessed(false) {
        }

        bool operator <(const GRASP& r) const {
            return fgraspdist < r.fgraspdist;
        }
        dReal fgraspdist, fgoaldist;
        Transform tgrasp;
        vector<dReal> qgoal;     ///< ik solution that achieves tgrasp
        bool bChecked;     ///< set to true if grasp is checked for ik solution
        bool bProcessed;     ///< set to true if grasp has already been used in gradient descend
    };

    GraspGradientPlanner(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nGrasp Planning with Stochastic Gradient Descent";
        _report.reset(new CollisionReport());
    }
    virtual ~GraspGradientPlanner() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset();
        boost::shared_ptr<GraspSetParameters> parameters(new GraspSetParameters(GetEnv()));
        parameters->copy(pparams);
        _robot = pbase;
        RobotBase::RobotStateSaver savestate(_robot);

        if( (int)parameters->vinitialconfig.size() != parameters->GetDOF() ) {
            RAVELOG_ERROR(str(boost::format("initial config wrong dim: %d\n")%parameters->vinitialconfig.size()));
            return false;
        }

        if( !_parameters->_checkpathconstraintsfn(parameters->vinitialconfig,parameters->vinitialconfig,IT_OpenStart,ConfigurationListPtr()) ) {
            RAVELOG_DEBUG("BirrtPlanner::InitPlan - Error: Initial configuration not in free space\n");
            return false;
        }

        if( parameters->_vgrasps.size() == 0 ) {
            RAVELOG_ERROR("no goal sampler specified\n");
            return false;
        }
        if( !parameters->_ptarget ) {
            RAVELOG_ERROR("no target specified\n");
            return false;
        }

        if( (int)parameters->vinitialconfig.size() != _robot->GetActiveDOF() ) {
            RAVELOG_ERROR(str(boost::format("initial config wrong dim: %d\n")%parameters->vinitialconfig.size()));
            return false;
        }

        _randomConfig.resize(parameters->GetDOF());

        _pmanip = _robot->GetActiveManipulator();
        if( (int)_pmanip->GetArmIndices().size() != _robot->GetActiveDOF()) {
            RAVELOG_ERROR("active dof not equal to arm joints\n");
            return false;
        }

        if(( _robot->GetActiveDOF() != (int)_pmanip->GetArmIndices().size()) ||( _robot->GetActiveDOFIndices().size() != _pmanip->GetArmIndices().size()) ) {
            RAVELOG_ERROR("active dof not equal to arm joints\n");
            return false;
        }
        for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
            if( _pmanip->GetArmIndices().at(i) != _robot->GetActiveDOFIndices().at(i) ) {
                RAVELOG_ERROR("active dof not equal to arm joints\n");
                return false;
            }
        }

        // set up the initial state
        if( !parameters->_checkpathconstraintsfn(parameters->vinitialconfig, parameters->vinitialconfig,IT_OpenStart,ConfigurationListPtr()) ) {
            // failed
            RAVELOG_WARN("initial state rejected by constraint fn\n");
            //return false;
        }

        if( parameters->_nMaxIterations <= 0 ) {
            parameters->_nMaxIterations = 10000;
        }
        _parameters = parameters;
        return true;
    }

    /// \param pOutStream returns which goal was chosen
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        if(!_parameters) {
            RAVELOG_ERROR("GraspGradientPlanner::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = utils::GetMilliTime();
        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        list<vector<dReal> > listbestpath, listpath;
        dReal bestgraspdist = 1e37f;

        bool bSuccess = false;

        // prioritize the grasps and go through each one
        _parameters->_setstatefn(_parameters->vinitialconfig);
        Transform tcurgrasp = _pmanip->GetTransform();

        Transform tobject = _parameters->_ptarget->GetTransform();
        vector<GRASP> vgrasps; vgrasps.reserve(_parameters->_vgrasps.size());
        for(size_t i = 0; i < _parameters->_vgrasps.size(); ++i) {
            Transform tgrasp = tobject * _parameters->_vgrasps[i];
            dReal fgraspdist = min((tgrasp.rot-tcurgrasp.rot).lengthsqr4(),(tgrasp.rot+tcurgrasp.rot).lengthsqr4());
            //+0.0f*(tgrasp.trans-tcurgrasp.trans).lengthsqr3();
            if( fgraspdist < _parameters->_fGraspDistThresh ) {
                vgrasps.push_back(GRASP());
                vgrasps.back().tgrasp = tgrasp;
                vgrasps.back().fgraspdist = fgraspdist;
            }
        }

        if( vgrasps.size() == 0 )
            return PS_Failed;

        sort(vgrasps.begin(),vgrasps.end());

        dReal fConfigThresh = 2.5f;     // first search for all grasps whose ik solutions pass this thresh
        bool bContinue = true;
        while(bContinue) {
            bContinue = false;
            FOREACH(itgrasp, vgrasps) {
                if( itgrasp->bProcessed || (itgrasp->bChecked &&( itgrasp->fgoaldist < 0) ) )
                    continue;

                RAVELOG_DEBUG("attempting grasp %d, %f\n", (int)(itgrasp-vgrasps.begin()), itgrasp->fgraspdist);
                RAVELOG_DEBUG("trans: %f, %f, %f, %f, %f, %f, %f\n",
                              itgrasp->tgrasp.rot.x,itgrasp->tgrasp.rot.y,itgrasp->tgrasp.rot.z,itgrasp->tgrasp.rot.w,itgrasp->tgrasp.trans.x,itgrasp->tgrasp.trans.y,itgrasp->tgrasp.trans.z);

                if( StochasticGradientDescent(*itgrasp, fConfigThresh, listpath) ) {
                    listbestpath.swap(listpath);
                    bSuccess = true;
                    break;
                }

                if( itgrasp->bProcessed ) {
                    // find the grasp distance
                    Transform t = _pmanip->GetTransform();
                    dReal graspdist2 = TransformDistance2(t,itgrasp->tgrasp,0.2f);
                    if( bestgraspdist > graspdist2 ) {
                        bestgraspdist = graspdist2;
                        listbestpath.swap(listpath);
                    }
                }
                else if( itgrasp->bChecked &&(itgrasp->fgoaldist >= 0))
                    bContinue = true;
            }

            if( bSuccess )
                break;

            fConfigThresh *= 1.5f;
        }

        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        FOREACH(it, listbestpath) {
            ptraj->Insert(ptraj->GetNumWaypoints(),*it);
        }

        RAVELOG_DEBUG(str(boost::format("plan %s, path=%d points in %fs\n")%(bSuccess ? "success" : "failure")%ptraj->GetNumWaypoints()%(0.001f*(float)(utils::GetMilliTime()-basetime))));

        return bSuccess ? PS_HasSolution : PS_Failed;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

private:
    bool StochasticGradientDescent(GraspGradientPlanner::GRASP& g, dReal fGoalThresh, list<vector<dReal> >& listpath)
    {
        vector<dReal> qbest, q(_robot->GetActiveDOF()),qgoaldir;
        listpath.clear();

        _parameters->_setstatefn(_parameters->vinitialconfig);

        if( g.bChecked ) {
            if( g.fgoaldist < 0 )
                return false;
        }
        else {
            g.bChecked = true;

            if( _pmanip->CheckEndEffectorCollision(g.tgrasp, _report) ) {
                RAVELOG_DEBUG("gripper collision: (%s:%s)x(%s:%s).\n",
                              !!_report->plink1 ? _report->plink1->GetParent()->GetName().c_str() : "",
                              !!_report->plink1 ? _report->plink1->GetName().c_str() : "",
                              !!_report->plink2 ? _report->plink2->GetParent()->GetName().c_str() : "",
                              !!_report->plink2 ? _report->plink2->GetName().c_str() : "");
                if( !(!!_report->plink1 &&( _report->plink1->GetParent() == _parameters->_ptarget) ) &&
                    !(!!_report->plink2 &&( _report->plink2->GetParent() == _parameters->_ptarget) ) )
                    return false;
            }

            bool bGetFirstSolution=true;
            if( bGetFirstSolution ) {
                if( !_pmanip->FindIKSolution(g.tgrasp,g.qgoal,true) ) {
                    RAVELOG_DEBUG("failed to find ik solution\n");
                    return false;
                }
            }
            else {
                // get all solutions and find the closest to initial config
                if( !_pmanip->FindIKSolutions(g.tgrasp,_viksolutions,true) ) {
                    RAVELOG_DEBUG("failed to find ik solutions\n");
                    return false;
                }

                dReal bestdist=1e30f;
                g.qgoal.resize(0);
                FOREACH(itq,_viksolutions) {
                    dReal dist = _parameters->_distmetricfn(_parameters->vinitialconfig,*itq);
                    if( bestdist > dist  ) {
                        bestdist = dist;
                        g.qgoal = *itq;
                    }
                }

                BOOST_ASSERT(g.qgoal.size()>0);
            }

            g.fgoaldist = _parameters->_distmetricfn(g.qgoal,_parameters->vinitialconfig);
        }

        if( g.fgoaldist > fGoalThresh )
            return false;

        RAVELOG_INFOA("goaldist: %f\n",g.fgoaldist);
        g.bProcessed = true;
        qbest.resize(0);
        dReal bestdist=0;

        listpath.push_back(_parameters->vinitialconfig);
        dReal fGoalStep = 0.25f*g.qgoal.size();
        dReal fdistmult = _parameters->_distmetricfn(_parameters->vinitialconfig,g.qgoal);
        if( fdistmult > fGoalStep )
            fdistmult = fGoalStep/fdistmult;
        else
            fdistmult = 1.0f;

        qgoaldir.resize(g.qgoal.size());
        for(size_t i = 0; i < g.qgoal.size(); ++i)
            qgoaldir[i] = (g.qgoal[i] - _parameters->vinitialconfig[i])*fdistmult;


        for(int iter = 0; iter < _parameters->_nMaxIterations; ++iter) {
            dReal fRadius = 0.2f;
            for(int giter = 0; giter < _parameters->_nGradientSamples; ++giter, fRadius*=0.96f) {
                if( giter == 0 ) {
                    dReal fcurdist = _parameters->_distmetricfn(listpath.back(),g.qgoal);
                    if( fcurdist < fGoalStep )
                        q = g.qgoal;
                    else {
                        for(size_t i = 0; i < qgoaldir.size(); ++i)
                            q[i] = listpath.back()[i]+qgoaldir[i];
                    }
                }
                else {
                    _parameters->_sampleneighfn(q,listpath.back(),fRadius);
                }

                if( _parameters->_checkpathconstraintsfn(listpath.back(),q,IT_OpenStart,ConfigurationListPtr()) ) {
                    // if new sample is closer than the best, accept it
                    dReal dist = _parameters->_distmetricfn(q,g.qgoal);
                    if(( qbest.size() == 0) ||( dist < bestdist) ) {
                        RAVELOG_DEBUG("dist: %f\n",dist);
                        qbest = q;
                        bestdist = dist;
                        if( giter == 0 ) {     // goal reached
                            break;
                        }
                    }
                }
            }
            if( qbest.size() == 0 ) {
                break;
            }
            listpath.push_back(qbest);
            if( bestdist < 0.0001f ) {    // we're done
                RAVELOG_INFOA("done after %d iters\n", iter);
                return true;
            }
        }

        return false;
    }

    RobotBase::ManipulatorPtr _pmanip;
    boost::shared_ptr<GraspSetParameters> _parameters;
    RobotBasePtr _robot;
    CollisionReportPtr _report;

    std::vector<dReal>         _randomConfig;
    std::vector<std::vector<dReal> > _viksolutions;
};

PlannerBasePtr CreateGraspGradientPlanner(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new GraspGradientPlanner(penv, sinput));
}
