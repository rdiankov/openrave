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
            _errorRot = 0.1;
            _errorTrans = .010;
        }
        virtual ~DualArmManipulation() {
        }

        int DualArmConstrained(std::vector<dReal>& vprev, const std::vector<dReal>& vdelta)
        {
            std::vector<dReal> vnew = vprev;
            for(size_t i=0; i<vnew.size(); ++i) {
                vnew.at(i) += vdelta.at(i);
            }

            if( _CheckConstraint(vnew) ) {
                // vprev+vdelta already fulfill the constraints
                vprev = vnew;
                return NSS_Reached;
            }

            KinBody::KinBodyStateSaver saver(_probot, KinBody::Save_LinkTransformation);

            std::vector<dReal> vcur;
            _probot->SetActiveDOFValues(vprev);
            _probot->GetDOFValues(vcur);

            _probot->SetActiveDOFValues(vnew);
            _probot->GetDOFValues(vnew);
            Transform tA = _pmanipA->GetTransform();
            Transform tInew= tA*_diff;          //this is (wTRANSl)*(lTRANSr)

            std::vector<dReal> vsolution;
            if( _pmanipI->FindIKSolution(tInew,vsolution, false) ) {
                vector<int> jointIndicesI = _pmanipI->GetArmIndices();
                vector<dReal> vdiff;
                for (size_t i=0; i<jointIndicesI.size(); ++i) {
                    vdiff.push_back(vcur[jointIndicesI.at(i)]);
                }
                _probot->SubtractDOFValues(vdiff, vsolution, jointIndicesI);

                //this check is important to make sure the IK solution does not fly too far away since there are multiple Ik solutions possible
                for (size_t i=0; i<vdiff.size(); ++i) {
                    if( fabs(vdiff[i]) < _errorRot*2.0 ) {
                        vnew[jointIndicesI.at(i)] =  vcur[jointIndicesI.at(i)] - vdiff[i];
                    }
                    else {
                        return NSS_Failed;
                    }
                }
            }
            else  {
                return NSS_Failed;
            }

            _probot->SetDOFValues(vnew);
            _probot->GetActiveDOFValues(vprev);
            return _CheckConstraint(vprev) ? NSS_SuccessfulWithDeviation : NSS_Failed;
        }

        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

protected:
        RobotBasePtr _probot;
        RobotBase::ManipulatorPtr _pmanipA;
        RobotBase::ManipulatorPtr _pmanipI;
        Transform _tOriginalEEI,_tOriginalEEA, _diff;
        double _errorRot, _errorTrans;

        bool _CheckConstraint(std::vector<dReal>& v) {
            KinBody::KinBodyStateSaver saver(_probot, KinBody::Save_LinkTransformation);

            _probot->SetActiveDOFValues(v);
            Transform tI = _pmanipI->GetTransform();
            Transform tA = _pmanipA->GetTransform();
            Transform tnew = tA.inverse()*tI;

            for(int i = 0; i < 4; ++i) {
                if (!(RaveFabs(tnew.rot[i]-_diff.rot[i]) < _errorRot)) {
                    return false;
                }
            }
            for(int i = 0; i < 3; ++i) {
                if (!(fabs(tnew.trans[i]-_diff.trans[i]) < _errorTrans)) {
                    return false;
                }
            }

            return true;
        }
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
