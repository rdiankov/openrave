// -*- coding: utf-8 -*-
// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu), Carnegie Mellon University
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
#ifndef COMMON_MANIPULATION_H
#define COMMON_MANIPULATION_H

#include "plugindefs.h"
#include <openrave/planningutils.h>

class CM
{
public:
    //Constraint function for maintaining relative orientations of the EEFs of both manipulators
    template <typename T>
    class DualArmManipulation {
public:
        DualArmManipulation(RobotBasePtr probot, RobotBase::ManipulatorPtr pmanipA, RobotBase::ManipulatorPtr pmanipI) : _probot(probot), _pmanipA(pmanipA), _pmanipI(pmanipI) {
            _tOriginalEEI = _pmanipI->GetTransform();
            _tOriginalEEA = _pmanipA->GetTransform();
            _diff = _tOriginalEEA.inverse()*_tOriginalEEI;

        }
        virtual ~DualArmManipulation() {
        }

        bool DualArmConstrained(std::vector<dReal>& vprev, const std::vector<dReal>& vdelta)
        {
            std::vector<dReal> vcur = vprev;
            for(size_t i = 0; i < vcur.size(); ++i) {
                vcur[i] += vdelta.at(i);
            }
            std::vector<dReal> vnew = vcur;
            bool pstatus=true;
            double errorRot=0.1;
            double errorTrans=.010;
            vector<int> JointIndicesI;

            std::vector<dReal> vold, vsolution;
            _probot->GetDOFValues(vold);          //gets the current robot config
            Transform tA = _pmanipA->GetTransform();

            Transform tInew= tA*_diff;          //this is (wTRANSl)*(lTRANSr)
            bool a= _pmanipI->FindIKSolution(tInew,vsolution, false);

            if(a) {
                vector<int> JointIndicesI = _pmanipI->GetArmIndices();

                for (size_t i=0; i<JointIndicesI.size(); i++) {      //this check is important to make sure the IK solution does not fly too far away since there are multiple Ik solutions possible
                    if(fabs(vsolution.at(i)-vprev[JointIndicesI.at(i)])<errorRot*2) {
                        vprev[JointIndicesI.at(i)]=vsolution[i];
                    }
                    else {
                        return false;
                    }
                }
                pstatus=true;
            }
            else  {
                return false;
            }

            //now checking
            _probot->SetActiveDOFValues(vprev);
            Transform tI = _pmanipI->GetTransform();
            tA = _pmanipA->GetTransform();
            Transform tnew = tA.inverse()*tI;

            for(int i = 0; i < 4; ++i) {
                if (!(RaveFabs(tnew.rot[i]-_diff.rot[i])<errorRot)) {
                    pstatus=false;
                    return false;
                }
            }
            for(int i = 0; i < 3; ++i) {
                if (!(fabs(tnew.trans[i]-_diff.trans[i])<errorTrans)) {
                    pstatus=false;
                    return false;
                }
            }

            return pstatus;
        }

        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

protected:
        RobotBasePtr _probot;
        RobotBase::ManipulatorPtr _pmanipA;
        RobotBase::ManipulatorPtr _pmanipI;
        Transform _tOriginalEEI,_tOriginalEEA, _diff;
        double _diffX,_diffY,_diffZ;

    };

    static bool SetActiveTrajectory(RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout,dReal fMaxVelMult=1)
    {
        if( pActiveTraj->GetNumWaypoints() == 0 ) {
            return false;
        }
        if( pActiveTraj->GetDuration() == 0 && pActiveTraj->GetNumWaypoints() > 1 ) {
            planningutils::RetimeActiveDOFTrajectory(pActiveTraj,robot,false,fMaxVelMult);
        }
        bool bExecuted = false;
        if( bExecute ) {
            if( pActiveTraj->GetNumWaypoints() > 1 ) {
                if( !!robot->GetController() ) {
                    robot->GetController()->SetPath(pActiveTraj);
                    bExecute = true;
                }
            }
            // have to set anyway since calling script will query ControllerBase::IsDone
            else if( !!robot->GetController() ) {
                vector<dReal> robotvalues;
                pActiveTraj->GetWaypoint(0,robotvalues,robot->GetConfigurationSpecification());
                robotvalues.resize(robot->GetDOF());
                if( robot->GetController()->SetDesired(robotvalues)) {
                    bExecuted = true;
                }
            }
        }

        if( strsavetraj.size() || !!pout ) {
            if( strsavetraj.size() > 0 ) {
                ofstream f(strsavetraj.c_str());
                pActiveTraj->serialize(f);
            }
            if( !!pout ) {
                pActiveTraj->serialize(*pout);
            }
        }

        return bExecuted;
    }
};

#endif
