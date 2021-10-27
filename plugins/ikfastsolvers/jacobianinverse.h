// -*- coding: utf-8 -*-
// Copyright (C) 2006-2016 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_JACOBIANINVERSE_H
#define OPENRAVE_JACOBIANINVERSE_H

//#define OPENRAVE_DISPLAY_CONVERGENCESTATS

#include "plugindefs.h"

#ifdef OPENRAVE_HAS_LAPACK

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace ikfastsolvers {

/// \brief inverse jacobian solver. although uses RobotBase::Manipulator, should not hold a shared pointer of it
template <typename T>
class JacobianInverseSolver
{
    class ValueSaver
    {
public:
        ValueSaver(int *src, int *dst) : _src(src), _dst(dst) {
        }
        ~ValueSaver() {
            *_dst = *_src;
        }
private:
        int* _src, *_dst;
    };

public:
    JacobianInverseSolver() {
        _errorthresh2 = 1e-12;
        _lastiter = -1;
        _nMaxIterations = 100;
    }

    /// \brief initializes with the manipulator, but doesn't store it!
    ///
    /// \param errorthresh the threshold of the error on the constraints
    void Init(const RobotBase::Manipulator& manip)
    {
        RobotBasePtr probot = manip.GetRobot();

        _J.resize(6,probot->GetActiveDOF());
        _invJJt.resize(6,6);
        _error.resize(6,1);

        _J3d.resize(3,probot->GetActiveDOF());
        _invJJt3d.resize(3,3);
        _error3d.resize(3,1);

        _viweights.resize(manip.GetArmIndices().size(),0);
        for(size_t i = 0; i < _viweights.size(); ++i) {
            int dof = manip.GetArmIndices().at(i);
            dReal fweight = probot->GetJointFromDOFIndex(dof)->GetWeight(dof-probot->GetJointFromDOFIndex(dof)->GetDOFIndex());
            if( fweight > 0 ) {
                _viweights.at(i) = 1/fweight;
            }
            _viweights[i] = 1;
        }
    }

    void SetErrorThresh(T errorthresh)
    {
        _errorthresh2 = errorthresh*errorthresh;
    }

    void SetMaxIterations(int nMaxIterations)
    {
        _nMaxIterations = nMaxIterations;
    }

    T GetErrorThresh() const
    {
        return RaveSqrt(_errorthresh2);
    }

    int GetMaxIterations() const
    {
        return _nMaxIterations;
    }

    /// \brief computes the jacobian inverse solution. Supports different ik param types, and computes error vector and jacobian accordingly
    ///
    /// robot is at the starting solution and solution should already be very close to the goal.
    /// assumes the robot's active dof is already set to the manipulator arm indices
    /// \param ikgoal the goal ik parameterization in the manipulator's base frame
    /// \param vsolution output if successful. vsolution should be initialized to current robot dof
    /// \return -1 if not changed, 0 if failed, 1 if changed and new succeeded in getting new position
    int ComputeSolution(const IkParameterization& ikgoal, const RobotBase::Manipulator& manip, std::vector<dReal>& vsolution, bool bIgnoreJointLimits=false)
    {
        // check if manip's iksolver suppports this ikgoal type
        if (! manip.GetIkSolver()->Supports(ikgoal.GetType())) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("iksolver %s of manipulator %s do not support "),manip.GetIkSolver()%manip.GetName(),ORE_InvalidArguments);
        }

        RobotBasePtr probot = manip.GetRobot();
        uint32_t checklimits = bIgnoreJointLimits ? OpenRAVE::KinBody::CLA_Nothing : OpenRAVE::KinBody::CLA_CheckLimitsSilent; // if not ignoring limits, silently clamp the values to their limits.
        const int ikdof = ikgoal.GetDOF();
        BOOST_ASSERT(6 >= ikdof && ikdof > 0);
        _J.resize(ikdof,probot->GetActiveDOF());
        _invJJt.resize(ikdof, ikdof);
        _error.resize(ikdof,1);

        _goalIkp = ikgoal;
        //_vGoalQuat = tgoal.rot;
        //_vGoalAxisAngle = axisAngleFromQuat(tgoal.rot);
        //_vGoalPosition = tgoal.trans;
        
        KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation);
        Transform tbase = manip.GetBase()->GetTransform();
        Transform trobot = probot->GetTransform();
        probot->SetTransform(tbase.inverse()*trobot); // transform so that the manip's base is at the identity and matches tgoal

        const IkParameterization ikpprev = manip.GetIkParameterization(ikgoal);
        T totalerror2 = _ComputeConstraintError(ikpprev, _error, _nMaxIterations);
        if( totalerror2 <= _errorthresh2 ) {
            return -1;
        }

        const T lambda2 = 1e-12;         // normalization constant, changes the rate of convergence, but also improves convergence stability
        using namespace boost::numeric::ublas;

        T firsterror2 = totalerror2;
        T besterror2 = totalerror2;
        _lasterror2 = totalerror2;
        int armdof = manip.GetArmDOF();
        std::vector<dReal>& vbest = _cachevbest; vbest = vsolution;
        std::vector<dReal>& vnew = _cachevnew; vnew = vsolution;
        bool bSuccess = false;
        int iter = 0;
        // setup a class so its destructor saves the last iter used in _lastiter
        ValueSaver valuesaver(&iter, &_lastiter);
        for(iter = 0; iter < _nMaxIterations; ++iter) {
            const IkParameterization ikpmanip = manip.GetIkParameterization(ikgoal);
            T totalerror2 = _ComputeConstraintError(ikpmanip, _error, _nMaxIterations-iter);
            //dReal ratio = totalerror2/_lasterror2;
            //RAVELOG_VERBOSE_FORMAT("%s:%s iter=%d, totalerror %.15e (%f)", probot->GetName()%manip.GetName()%iter%RaveSqrt(totalerror2)%RaveSqrt(totalerror2/_lasterror2));
            if( totalerror2 < besterror2 ) {
                besterror2 = totalerror2;
                vbest = vnew;
            }
            if( totalerror2 <= _errorthresh2 ) {
                bSuccess = true;
                break;
            }
//            if( ratio > 0.98 ) {
//                // probably won't get better...
//                break;
//            }

            // 2.0 is an arbitrary number...
            if( totalerror2 > 10.0*firsterror2 ) {// && fdistcur2 > 4.0*fdistprev2 ) {
                // last adjustment was greater than total distance (jacobian was close to being singular)
                RAVELOG_VERBOSE_FORMAT("last adjustment on iter %d was greater than total distance (jacobian was close to being singular?): %.15e > %.15e", iter%totalerror2%_lasterror2);//%fdistcur2%fdistprev2);
                iter = -1;
                break;
            }
            _lasterror2 = totalerror2;

            // calculate jacobians
            _CalculateJacobian(manip, ikpmanip);

            // pseudo inverse of jacobian
            _Jt = trans(_J);
            _invJJt = prod(_J,_Jt);
            for(int i = 0; i < ikdof; ++i) {
                _invJJt(i,i) += lambda2;
            }

#ifdef OPENRAVE_DISPLAY_CONVERGENCESTATS
            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+2);
            ss << std::endl << "J=array([" << std::endl;
            for(int i = 0; i < armdof; ++i) {
                ss << "[";
                for(int j = 0; j < ikdof; ++j) {
                    ss << _J(i,j) << ", ";
                }
                ss << "]," << std::endl;
            }
            ss << "]);" << std::endl;
            ss << "error=array([";
            for(int i = 0; i < ikdof; ++i) {
                ss << _error(i,0) << ", ";
            }
            ss << "]);" << std::endl;
#endif
            try {
                if( !_InvertMatrix(_invJJt,_invJJt) ) {
                    RAVELOG_VERBOSE("failed to invert matrix\n");
                    iter = -1;
                    break;
                }
            }
            catch(const std::exception& ex) {
                RAVELOG_VERBOSE_FORMAT("got exception during constraining, perhaps matix is singular: %s", ex.what());
                iter = -1;
                break;
            }
            catch(...) {
                RAVELOG_WARN("got unknown exception during constraining\n");
                iter = -1;
                break;
            }
            _invJ = prod(_Jt,_invJJt);
            _qdelta = prod(_invJ,_error);

#ifdef OPENRAVE_DISPLAY_CONVERGENCESTATS
            ss << std::endl << "invJJt=array([" << std::endl;
            for(int i = 0; i < armdof; ++i) {
                ss << "[";
                for(int j = 0; j < armdof; ++j) {
                    ss << _invJJt(i,j) << ", ";
                }
                ss << "]," << std::endl;
            }
            ss << "]);" << std::endl;

            ss << "qdelta=array([";
            for(size_t i = 0; i < vnew.size(); ++i) {
                ss << _qdelta(i,0) << ", ";
            }
            ss << "]);" << std::endl;
            ss << "vprev=array([";
            for(size_t i = 0; i < vnew.size(); ++i) {
                ss << vnew[i] << ", ";
            }
            ss << "]);" << std::endl;

            RAVELOG_VERBOSE(ss.str());
#endif

            dReal fmindeltascale = 1; // depending on
            bool baddelta = false;
            for(size_t i = 0; i < vnew.size(); ++i) {
                if(!isfinite(_qdelta(i,0))) { // don't assert since it is frequent and could destroy the entire plan
                    RAVELOG_WARN_FORMAT("inverse matrix produced a non-finite value: %e", _qdelta(i,0));
                    baddelta = true;
                    break;
                }

                dReal f = _qdelta(i,0)*_viweights.at(i);
                _qdelta(i,0) = f;
            }
            if( baddelta ) {
                break;
            }

            if( fmindeltascale < 1 ) {
                for(size_t i = 0; i < vnew.size(); ++i) {
                    vnew.at(i) += _qdelta(i,0)*fmindeltascale;
                }
            }
            else {
                for(size_t i = 0; i < vnew.size(); ++i) {
                    vnew.at(i) += _qdelta(i,0);
                }
            }

            probot->SetActiveDOFValues(vnew, checklimits);
            if( checklimits == OpenRAVE::KinBody::CLA_CheckLimitsSilent ) {
                probot->GetActiveDOFValues(vnew);
            }
        }

        int retcode = 0;
        if( bSuccess || besterror2 < firsterror2 ) {
            // revert to real values
            probot->SetActiveDOFValues(vbest, checklimits);
            probot->GetActiveDOFValues(vsolution); // have to re-get the joint values since joint limits are involved
            probot->SetTransform(trobot);
            saver.Release(); // finished successfully, so use the new state
            if( bSuccess || besterror2 <= 10*_errorthresh2 ) { // if close enough to error, just return as being close. user should take this in account when setting the error threshold
                retcode = 1;
            }
            else {
                retcode = 2;
            }
        }
        else if( iter >= _nMaxIterations ) {
            iter = -1;
            RAVELOG_VERBOSE_FORMAT("constraint function exceeded %d iterations, first error^2 is %.15e, final error^2 is %.15e > %.15e", _nMaxIterations%firsterror2%_lasterror2%_errorthresh2);
        }
        return retcode;
    }

    int ComputeSolutionTranslation(const IkParameterization& ikgoal, const RobotBase::Manipulator& manip, std::vector<dReal>& vsolution, bool bIgnoreJointLimits=false)
    {
        // check if manip's iksolver suppports this ikgoal type
        if (! manip.GetIkSolver()->Supports(ikgoal.GetType())) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("iksolver %s of manipulator %s do not support "),manip.GetIkSolver()%manip.GetName(),ORE_InvalidArguments);
        }

        _goalIkp = ikgoal;
        
        RobotBasePtr probot = manip.GetRobot();
        uint32_t checklimits = bIgnoreJointLimits ? OpenRAVE::KinBody::CLA_Nothing : OpenRAVE::KinBody::CLA_CheckLimitsSilent; // if not ignoring limits, silently clamp the values to their limits.

        KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation);
        Transform tbase = manip.GetBase()->GetTransform();
        Transform trobot = probot->GetTransform();
        probot->SetTransform(tbase.inverse()*trobot); // transform so that the manip's base is at the identity and matches tgoal

        Transform tprev = manip.GetTransform();
        T totalerror2 = _ComputeConstraintError(tprev, _error3d, _nMaxIterations, false);
        if( totalerror2 <= _errorthresh2 ) {
            return -1;
        }

        const T lambda2 = 1e-12;         // normalization constant, changes the rate of convergence, but also improves convergence stability
        using namespace boost::numeric::ublas;

        T firsterror2 = totalerror2;
        T besterror2 = totalerror2;
        _lasterror2 = totalerror2;
        int armdof = manip.GetArmDOF();
        std::vector<dReal>& vbest = _cachevbest; vbest = vsolution;
        std::vector<dReal>& vnew = _cachevnew; vnew = vsolution;
        bool bSuccess = false;
        int iter = 0;
        // setup a class so its destructor saves the last iter used in _lastiter
        ValueSaver valuesaver(&iter, &_lastiter);
        for(iter = 0; iter < _nMaxIterations; ++iter) {
            Transform tmanip = manip.GetTransform();
            T totalerror2 = _ComputeConstraintError(tmanip, _error3d, _nMaxIterations-iter, false);
            //dReal ratio = totalerror2/_lasterror2;
            //RAVELOG_VERBOSE_FORMAT("%s:%s iter=%d, totalerror %.15e (%f)", probot->GetName()%manip.GetName()%iter%RaveSqrt(totalerror2)%RaveSqrt(totalerror2/_lasterror2));
            if( totalerror2 < besterror2 ) {
                besterror2 = totalerror2;
                vbest = vnew;
            }
            if( totalerror2 <= _errorthresh2 ) {
                bSuccess = true;
                break;
            }
//            if( ratio > 0.98 ) {
//                // probably won't get better...
//                break;
//            }

            // 2.0 is an arbitrary number...
            if( totalerror2 > 10.0*firsterror2 ) {// && fdistcur2 > 4.0*fdistprev2 ) {
                // last adjustment was greater than total distance (jacobian was close to being singular)
                RAVELOG_VERBOSE_FORMAT("last adjustment on iter %d was greater than total distance (jacobian was close to being singular?): %.15e > %.15e", iter%totalerror2%_lasterror2);//%fdistcur2%fdistprev2);
                iter = -1;
                break;
            }
            _lasterror2 = totalerror2;

            // compute jacobians, make sure to transform by the world frame
            manip.CalculateJacobian(_vjacobian);
            for(size_t j = 0; j < _viweights.size(); ++j) {
                Vector v = Vector(_vjacobian[j],_vjacobian[armdof+j],_vjacobian[2*armdof+j]);
                _J3d(0,j) = v[0]*_viweights[j];
                _J3d(1,j) = v[1]*_viweights[j];
                _J3d(2,j) = v[2]*_viweights[j];
            }
            // pseudo inverse of jacobian
            _Jt3d = trans(_J3d);
            _invJJt3d = prod(_J3d,_Jt3d);
            for(int i = 0; i < 3; ++i) {
                _invJJt3d(i,i) += lambda2;
            }

#ifdef OPENRAVE_DISPLAY_CONVERGENCESTATS
            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+2);
            ss << std::endl << "J=array([" << std::endl;
            for(int i = 0; i < 3; ++i) {
                ss << "[";
                for(int j = 0; j < 3; ++j) {
                    ss << _J3d(i,j) << ", ";
                }
                ss << "]," << std::endl;
            }
            ss << "]);" << std::endl;
            ss << "error=array([";
            for(int i = 0; i < 3; ++i) {
                ss << _error3d(i,0) << ", ";
            }
            ss << "]);" << std::endl;
#endif
            try {
                if( !_InvertMatrix(_invJJt3d,_invJJt3d) ) {
                    RAVELOG_VERBOSE("failed to invert matrix\n");
                    iter = -1;
                    break;
                }
            }
            catch(const std::exception& ex) {
                RAVELOG_VERBOSE_FORMAT("got exception during constraining, perhaps matix is singular: %s", ex.what());
                iter = -1;
                break;
            }
            catch(...) {
                RAVELOG_WARN("got unknown exception during constraining\n");
                iter = -1;
                break;
            }
            _invJ3d = prod(_Jt3d,_invJJt3d);
            _qdelta = prod(_invJ3d,_error3d);

#ifdef OPENRAVE_DISPLAY_CONVERGENCESTATS
            ss << std::endl << "invJJt=array([" << std::endl;
            for(int i = 0; i < 3; ++i) {
                ss << "[";
                for(int j = 0; j < 3; ++j) {
                    ss << _invJJt3d(i,j) << ", ";
                }
                ss << "]," << std::endl;
            }
            ss << "]);" << std::endl;

            ss << "qdelta=array([";
            for(size_t i = 0; i < vnew.size(); ++i) {
                ss << _qdelta(i,0) << ", ";
            }
            ss << "]);" << std::endl;
            ss << "vprev=array([";
            for(size_t i = 0; i < vnew.size(); ++i) {
                ss << vnew[i] << ", ";
            }
            ss << "]);" << std::endl;

            RAVELOG_VERBOSE(ss.str());
#endif

            dReal fmindeltascale = 1; // depending on
            bool baddelta = false;
            for(size_t i = 0; i < vnew.size(); ++i) {
                if(!isfinite(_qdelta(i,0))) { // don't assert since it is frequent and could destroy the entire plan
                    RAVELOG_WARN_FORMAT("inverse matrix produced a non-finite value: %e", _qdelta(i,0));
                    baddelta = true;
                    break;
                }

                dReal f = _qdelta(i,0)*_viweights.at(i);
                _qdelta(i,0) = f;
            }
            if( baddelta ) {
                break;
            }

            if( fmindeltascale < 1 ) {
                for(size_t i = 0; i < vnew.size(); ++i) {
                    vnew.at(i) += _qdelta(i,0)*fmindeltascale;
                }
            }
            else {
                for(size_t i = 0; i < vnew.size(); ++i) {
                    vnew.at(i) += _qdelta(i,0);
                }
            }

            probot->SetActiveDOFValues(vnew, checklimits);
            if( checklimits == OpenRAVE::KinBody::CLA_CheckLimitsSilent ) {
                probot->GetActiveDOFValues(vnew);
            }
        }

        int retcode = 0;
        if( bSuccess || besterror2 < firsterror2 ) {
            // revert to real values
            probot->SetActiveDOFValues(vbest, checklimits);
            probot->GetActiveDOFValues(vsolution); // have to re-get the joint values since joint limits are involved
            probot->SetTransform(trobot);
            saver.Release(); // finished successfully, so use the new state
            if( bSuccess || besterror2 < 10*_errorthresh2 ) { // if close enough to error, just return as being close. user should take this in account when setting the error threshold
                retcode = 1;
            }
            else {
                retcode = 2;
            }
        }
        else if( iter >= _nMaxIterations ) {
            iter = -1;
            RAVELOG_VERBOSE_FORMAT("constraint function exceeded %d iterations, first error^2 is %.15e, final error^2 is %.15e > %.15e", _nMaxIterations%firsterror2%_lasterror2%_errorthresh2);
        }
        return retcode;
    }

    /// \brief computes the jacobian inverse solution.
    ///
    /// robot is at the starting solution and solution should already be very close to the goal.
    /// assumes the robot's active dof is already set to the manipulator arm indices
    /// \param tgoal the goal in the manipulator's base frame
    /// \param vsolution output if successful
    /// \return -1 if not changed, 0 if failed, 1 if changed and new succeeded in getting new position
    int ComputeSolution(const Transform& tgoal, const RobotBase::Manipulator& manip, std::vector<dReal>& vsolution, bool bIgnoreJointLimits=false)
    {
        return ComputeSolution(IkParameterization(tgoal, IKP_Transform6D), manip, vsolution, bIgnoreJointLimits);
    }
    
    int ComputeSolutionTranslation(const Transform& tgoal, const RobotBase::Manipulator& manip, std::vector<dReal>& vsolution)
    {
        return ComputeSolutionTranslation(IkParameterization(tgoal, IKP_Translation3D), manip, vsolution);
    }

    virtual T _ComputeConstraintError(const IkParameterization& ikpcur, boost::numeric::ublas::matrix<T>& error, int nMaxIterations, bool bAddRotation=true)
    {
        T totalerror2=0;
        int transoffset = 0;
        switch (ikpcur.GetType()) {
        case IKP_Transform6D:
            {
                const Transform tcur = ikpcur.GetTransform6D();
                const Transform tgoal = _goalIkp.GetTransform6D();
                if( bAddRotation ) {
                    const Vector axisangleerror = axisAngleFromQuat(quatMultiply(tgoal.rot, quatInverse(tcur.rot)));
                    for(int i = 0; i < 3; ++i) {
                        error(i,0) = axisangleerror[i];
                        totalerror2 += error(i,0)*error(i,0);
                    }
                    transoffset += 3;
                }

                for(int i = 0; i < 3; ++i) {
                    error(i+transoffset,0) = (tgoal.trans[i]-tcur.trans[i]);
                    totalerror2 += error(i+transoffset,0)*error(i+transoffset,0);
                }
        
                dReal fallowableerror2 = 0.03; // arbitrary... since solutions are close, is this step necessary?
                if( totalerror2 > _errorthresh2 && totalerror2 > fallowableerror2+1e-7 ) {
                    // have to reduce the error or else the jacobian will not converge to the correct place and diverge too much from the current solution
                    // depending on how many iterations there is left, have to adjust the error
                    T fscale = sqrt(totalerror2/fallowableerror2);
                    if( fscale > nMaxIterations ) {
                        fscale = nMaxIterations;
                    }
                    T fiscale = 1/fscale;
                    RAVELOG_VERBOSE_FORMAT("fiscale=%f", fiscale);
                    for(int i = 0; i < (int)error.size1(); ++i) {
                        error(i,0) *= fiscale;
                    }
                }
                break;
            }
        case IKP_TranslationZAxisAngle4D:
            {
                const std::pair<Vector,dReal> cur = ikpcur.GetTranslationZAxisAngle4D();
                const std::pair<Vector,dReal> goal = _goalIkp.GetTranslationZAxisAngle4D();
                if( bAddRotation ) {
                    error(0,0) = goal.second - cur.second;
                    totalerror2 += error(0,0)*error(0,0);
                    transoffset = 1;
                }

                for(int i = 0; i < 3; ++i) {
                    error(i+transoffset,0) = (goal.first[i]-cur.first[i]);
                    totalerror2 += error(i+transoffset,0)*error(i+transoffset,0);
                }
                break;
            }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unsupported ikparam %s."),ikpcur.GetName(),ORE_InvalidArguments);
            break;
        };
        return totalerror2;
    }

    // Calculates jacobian depending on ikparameter type. jacobian and error vector has to be consistent
    virtual void _CalculateJacobian(const RobotBase::Manipulator& manip, const IkParameterization& ikp)
    {
        const int armdof = manip.GetArmDOF();
        switch (ikp.GetType()) {
        case IKP_Transform6D:
            {
                manip.CalculateAngularVelocityJacobian(_vjacobian); // doesn't work well...
                for(size_t j = 0; j < _viweights.size(); ++j) {
                    Vector v = Vector(_vjacobian[j],_vjacobian[armdof+j],_vjacobian[2*armdof+j]);
                    _J(0,j) = v[0]*_viweights[j];
                    _J(1,j) = v[1]*_viweights[j];
                    _J(2,j) = v[2]*_viweights[j];
                }
                manip.CalculateJacobian(_vjacobian);
                for(size_t j = 0; j < _viweights.size(); ++j) {
                    Vector v = Vector(_vjacobian[j],_vjacobian[armdof+j],_vjacobian[2*armdof+j]);
                    _J(0+3,j) = v[0]*_viweights[j];
                    _J(1+3,j) = v[1]*_viweights[j];
                    _J(2+3,j) = v[2]*_viweights[j];
                }
                break;
            }
        case IKP_TranslationZAxisAngle4D:
            {
                const RobotBasePtr probot = manip.GetRobot();
                const TransformMatrix robotrot = matrixFromQuat(probot->GetTransform().rot);
                const Vector robotZDir(robotrot.rot(0, 2), robotrot.rot(1, 2), robotrot.rot(2, 2));

                const TransformMatrix maniprot = matrixFromQuat(manip.GetTransform().rot);
                const Vector manipZDir(maniprot.rot(0, 2), maniprot.rot(1, 2), maniprot.rot(2, 2));
                const double rzzSq = maniprot.rot(2, 2)*maniprot.rot(2, 2);
                // angle part, definition is arccos(Rzz(q)) where Rzz is the z-z component of manip's 3x3 rotation matrix
                // a = f(h(q)) where q is the joint angle, h is the forward kinematics for Rzz and f is the arccos
                // using chain rule, da/dq = df/dh times dh/dq
                // df/dh is -1 / sqrt(1-h^2) and
                // dh/dq is cross product b/w joint axis and manipulator direction

                manip.CalculateAngularVelocityJacobian(_vjacobian);
                for(size_t j = 0; j < _viweights.size(); ++j) {
                    // better to get joint axis from angular jacobian than, directly getting it from joint->getaxis to handle prismatic joint properly
                    const Vector jointAxis = Vector(_vjacobian[j],_vjacobian[armdof+j],_vjacobian[2*armdof+j]);
                    //int dof = manip.GetArmIndices().at(j);
                    //const Vector jointAxis = probot->GetJointFromDOFIndex(dof)->GetAxis();

                    if (1.0 - rzzSq > 1.0e-9) { // non-singular
                        const double dfdh = -1.0 / sqrt(1.0 - rzzSq);
                        const double dhdq = jointAxis.cross(manipZDir)[2];
                        _J(0,j) = dfdh * dhdq * _viweights[j];
                    }
                    else { // singular, base z axis and manip z axis are almost parallel
                        // positive value means error increases if joint is rotated positively
                        const double jacobianZDot = (robotZDir.cross(manipZDir)).dot(jointAxis);
                        const double jacobianSign = jacobianZDot > 0 ? 1 : -1;

                        const double jacobianZAngle = (jointAxis.cross(manipZDir)).lengthsqr2();
                        _J(0,j) = jacobianSign * jacobianZAngle *_viweights[j];
                    }
                }
            
                // position part
                manip.CalculateJacobian(_vjacobian);
                for(size_t j = 0; j < _viweights.size(); ++j) {
                    Vector v = Vector(_vjacobian[j],_vjacobian[armdof+j],_vjacobian[2*armdof+j]);
                    _J(0+1,j) = v[0]*_viweights[j];
                    _J(1+1,j) = v[1]*_viweights[j];
                    _J(2+1,j) = v[2]*_viweights[j];
                }
                break;
            }
        default:
            break;
        };
    }

    // Matrix inversion routine. Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
    static bool _InvertMatrix(const boost::numeric::ublas::matrix<T>& input, boost::numeric::ublas::matrix<T>& inverse) {
        using namespace boost::numeric::ublas;
        matrix<T> A(input);         // create a working copy of the input
        permutation_matrix<std::size_t> pm(A.size1());         // create a permutation matrix for the LU-factorization
        int res = lu_factorize(A,pm);         // perform LU-factorization
        if( res != 0 )
            return false;

        inverse.assign(identity_matrix<T>(A.size1()));         // create identity matrix of "inverse"
        for(size_t i = 0; i < A.size1(); ++i) {
            if( RaveFabs(A(i,i)) < 1e-9 ) {
                RAVELOG_VERBOSE_FORMAT("most likely matrix is singular %.15e, so fail!", RaveFabs(A(i,i)));
                return false;
            }
        }
        lu_substitute(A, pm, inverse);         // backsubstitute to get the inverse
        return true;
    }

    /// \brief computes squared distance
//    void _ComputeConfigurationDistance2(const std::vector<dReal>& v0, const std::vector<dReal>& v1)
//    {
//        dReal dist2 = 0;
//        for(size_t i = 0; i < v0.size(); ++i) {
//            dReal f = (v0[i] - v1[i])*_viweights[i];
//            dist2 += f*f;
//        }
//        return dist2;
//    }

    // statistics about last run
    int _lastiter;
    double _lasterror2; // error squared
    int _nMaxIterations;

protected:
    //Vector _vGoalQuat, _vGoalAxisAngle, _vGoalPosition;
    IkParameterization _goalIkp;
    std::vector<dReal> _viweights, _vcachevalues;
    T _errorthresh2;
    std::vector<dReal> _vjacobian;
    boost::numeric::ublas::matrix<T> _J, _Jt, _invJJt, _invJ, _error, _qdelta;

    boost::numeric::ublas::matrix<T> _J3d, _Jt3d, _invJJt3d, _invJ3d, _error3d; // for translation

    std::vector<dReal> _cachevnew, _cachevbest; ///< cache
    dReal _fTighterCosAngleThresh; ///< if _pdirthresh is used, then this is a smaller angle than the one used in _pdirthresh->fCosAngleThresh
};

} // end namespace ikfastsolvers

#endif

#endif
