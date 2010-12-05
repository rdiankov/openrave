// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef  WORKSPACE_TRAJECTORY_PLANNER_H
#define  WORKSPACE_TRAJECTORY_PLANNER_H

#include "rplanners.h"

class WorkspaceTrajectoryTracker : public PlannerBase
{
public:
    class WorkspaceTrajectoryParameters : public PlannerBase::PlannerParameters {
    public:
    WorkspaceTrajectoryParameters() : _fMaxDeviationAngle(0.15*PI), _bMaintainTiming(false), _bProcessing(false) {
            _vXMLParameters.push_back("maxdeviationangle");
            _vXMLParameters.push_back("maintaintiming");
        }
        
        dReal _fMaxDeviationAngle;
        bool _bMaintainTiming; ///< maintain timing with input trajectory

    protected:
        bool _bProcessing;
        // save the extra data to XML
        virtual bool serialize(std::ostream& O) const
        {
            if( !PlannerParameters::serialize(O) ) {
                return false;
            }
            O << "<maxdeviationangle>" << _fMaxDeviationAngle << "</maxdeviationangle>" << endl;
            O << "<maintaintiming>" << _bMaintainTiming << "</maintaintiming>" << endl;
            return !!O;
        }

        ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
        {
            if( _bProcessing ) {
                return PE_Ignore;
            }
            switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }    
            _bProcessing = name=="maxdeviationangle" || name=="maintaintiming";
            return _bProcessing ? PE_Support : PE_Pass;
        }
        
        // called at the end of every XML tag, _ss contains the data 
        virtual bool endElement(const std::string& name)
        {
            // _ss is an internal stringstream that holds the data of the tag
            if( _bProcessing ) {
                if( name == "maxdeviationangle") {
                    _ss >> _fMaxDeviationAngle;
                }
                else if( name == "maintaintiming" ) {
                    _ss >> _bMaintainTiming;
                }
                else {
                    RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
                }
                _bProcessing = false;
                return false;
            }
            // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        }
    };
    
 WorkspaceTrajectoryTracker(EnvironmentBasePtr penv) : PlannerBase(penv)
    {
        __description = "\
:Interface Author:  Rosen Diankov\n\
Given a workspace trajectory of the end effector of a manipulator (active manipulator of the robot), finds a configuration space trajectory that tracks it using analytical inverse kinematics.\n\
Options can be specified to prioritize trajectory time, trajectory smoothness, and planning time\n\
In the simplest case, the workspace trajectory can be a straight line from one point to another.\n\
";
        _report.reset(new CollisionReport());
    }
    virtual ~WorkspaceTrajectoryTracker() {}

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new WorkspaceTrajectoryParameters());
        _parameters->copy(params);
        _robot = pbase;
        _fMaxCosDeviationAngle = RaveCos(_parameters->_fMaxDeviationAngle);

        if( _parameters->_bMaintainTiming ) {
            RAVELOG_WARN("currently do not support maintaining timing\n");
        }

        RobotBase::RobotStateSaver savestate(_robot);
        if( (int)_parameters->vinitialconfig.size() != _parameters->GetDOF() ) {
            RAVELOG_ERROR(str(boost::format("initial config wrong dim: %d\n")%_parameters->vinitialconfig.size()));
            return false;
        }

        if(CollisionFunctions::CheckCollision(_parameters,_robot,_parameters->vinitialconfig, _report)) {
            RAVELOG_DEBUG("BirrtPlanner::InitPlan - Error: Initial configuration in collision\n");
            return false;
        }
    
        // set up the initial state
        if( !!_parameters->_constraintfn ) {
            _parameters->_setstatefn(_parameters->vinitialconfig);
            if( !_parameters->_constraintfn(_parameters->vinitialconfig, _parameters->vinitialconfig,0) ) {
                RAVELOG_WARN("initial state rejected by constraint fn\n");
                return false;
            }
        }

        return true;
    }

    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        if(!_parameters) {
            RAVELOG_ERROR("BirrtPlanner::PlanPath - Error, planner not initialized\n");
            return false;
        }
        if( ptraj->GetDOF() > 0 || ptraj->GetTotalDuration() == 0 ) {
            RAVELOG_ERROR("input trajectory needs to be 0 DOF and initialized with interpolation information\n");
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = timeGetTime();
 
        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        RAVELOG_DEBUG(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetPoints().size()%((0.001f*(float)(timeGetTime()-basetime)))));
        return true;
    }

    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }

protected:
    RobotBasePtr         _robot;
    CollisionReportPtr _report;
    boost::shared_ptr<WorkspaceTrajectoryParameters> _parameters;
    dReal _fMaxCosDeviationAngle;
};

#endif
