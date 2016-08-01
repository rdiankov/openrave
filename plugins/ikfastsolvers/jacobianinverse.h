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

    /// \brief computes the jacobian inverse solution.
    ///
    /// robot is at the starting solution and solution should already be very close to the goal.
    /// assumes the robot's active dof is already set to the manipulator arm indices
    /// \param tgoal the goal in the manipulator's base frame
    /// \param vsolution output if successful
    /// \return -1 if not changed, 0 if failed, 1 if changed and new succeeded in getting new position
    int ComputeSolution(const Transform& tgoal, const RobotBase::Manipulator& manip, std::vector<dReal>& vsolution)
    {
        _vGoalQuat = tgoal.rot;
        _vGoalAxisAngle = axisAngleFromQuat(tgoal.rot);
        _vGoalPosition = tgoal.trans;
        
        RobotBasePtr probot = manip.GetRobot();

        KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation);
        Transform tbase = manip.GetBase()->GetTransform();
        Transform trobot = probot->GetTransform();
        probot->SetTransform(tbase.inverse()*trobot); // transform so that the manip's base is at the identity and matches tgoal

        Transform tprev = manip.GetTransform();
        T totalerror2 = _ComputeConstraintError(tprev, _error, _nMaxIterations);
        if( totalerror2 <= _errorthresh2 ) {
            return -1;
        }
        
        const T lambda2 = 1e-16;         // normalization constant
        using namespace boost::numeric::ublas;

        T firsterror2 = totalerror2;
        _lasterror2 = totalerror2;
        int armdof = manip.GetArmDOF();
        std::vector<dReal>& vnew = _cachevnew; vnew = vsolution;
        bool bSuccess = false;
        int iter = 0;
        // setup a class so its destructor saves the last iter used in _lastiter
        ValueSaver valuesaver(&iter, &_lastiter);
        for(iter = 0; iter < _nMaxIterations; ++iter) {
            Transform tmanip = manip.GetTransform();
            T totalerror2 = _ComputeConstraintError(tmanip, _error, _nMaxIterations-iter);
            //dReal ratio = totalerror2/_lasterror2;
            //RAVELOG_VERBOSE_FORMAT("%s:%s iter=%d, totalerror %.15e (%f)", probot->GetName()%manip.GetName()%iter%RaveSqrt(totalerror2)%RaveSqrt(ratio));
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
                iter = -1;
                RAVELOG_VERBOSE_FORMAT("last adjustment was greater than total distance (jacobian was close to being singular?): %.15e > %.15e", totalerror2%_lasterror2);//%fdistcur2%fdistprev2);
                break;
            }
            _lasterror2 = totalerror2;

            // compute jacobians, make sure to transform by the world frame
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
            // pseudo inverse of jacobian
            _Jt = trans(_J);
            _invJJt = prod(_J,_Jt);
            for(int i = 0; i < 6; ++i) {
                _invJJt(i,i) += lambda2;
            }
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
            dReal fmindeltascale = 1.0; // depending on
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

            probot->SetActiveDOFValues(vnew,0);
        }

        int retcode = 0;
        if( bSuccess || _lasterror2 < firsterror2 ) {
            // revert to real values
            probot->GetActiveDOFValues(vnew); // have to re-get the joint values since joint limits are involved
            vsolution = vnew;
            probot->SetTransform(trobot);
            saver.Release(); // finished successfully, so use the new state
            if( bSuccess ) {
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

    virtual T _ComputeConstraintError(const Transform& tcur, boost::numeric::ublas::matrix<T>& error, int nMaxIterations)
    {
        T totalerror2=0;
        const Vector axisangleerror = axisAngleFromQuat(quatMultiply(_vGoalQuat, quatInverse(tcur.rot)));
        for(int i = 0; i < 3; ++i) {
            error(i,0) = axisangleerror[i];
            totalerror2 += error(i,0)*error(i,0);
        }
        for(int i = 0; i < 3; ++i) {
            error(i+3,0) = (_vGoalPosition[i]-tcur.trans[i]);
            totalerror2 += error(i+3,0)*error(i+3,0);
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
            for(int i = 0; i < 6; ++i) {
                error(i,0) *= fiscale;
            }
        }
        return totalerror2;
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
            if( RaveFabs(A(i,i)) < 1e-8 ) {
                // most likely matrix is singular, so fail!
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
    Vector _vGoalQuat, _vGoalAxisAngle, _vGoalPosition;
    std::vector<dReal> _viweights, _vcachevalues;
    T _errorthresh2;
    std::vector<dReal> _vjacobian;
    boost::numeric::ublas::matrix<T> _J, _Jt, _invJJt, _invJ, _error, _qdelta;
    std::vector<dReal> _cachevnew; ///< cache
    dReal _fTighterCosAngleThresh; ///< if _pdirthresh is used, then this is a smaller angle than the one used in _pdirthresh->fCosAngleThresh
};

} // end namespace ikfastsolvers

#endif

#endif
