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
#ifndef COMMON_MANIPULATION_H
#define COMMON_MANIPULATION_H

#include "plugindefs.h"
#include "plannerparameters.h"

#include <openrave/planningutils.h>

// used for inverse jacobian computation
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

class CM
{
public:
    class MoveUnsync
    {
public:
        MoveUnsync() : _maxdivision(10) {
        }
        virtual ~MoveUnsync() {
        }

        virtual void SetRobot(RobotBasePtr robot) {
            _robot = robot; thresh = 0;
        }
        virtual float GetGoalThresh() {
            return thresh;
        }

        virtual float Eval(const std::vector<dReal>& pConfiguration)
        {
            // check if there's a collision when hand moves to final config
            RobotBase::RobotStateSaver saver(_robot);
            _robot->SetActiveDOFValues(pConfiguration);

            _robot->SetActiveDOFs(vhandjoints);
            _robot->GetActiveDOFValues(vhandvalues);

            int numiter = _maxdivision;
            vhanddelta.resize(vhandjoints.size());
            for(size_t i = 0; i < vhandjoints.size(); ++i)
                vhanddelta[i] = (vhandgoal[i]-vhandvalues[i])/(dReal)numiter;

            while(numiter-- > 0) {

                for(size_t i = 0; i < vhandjoints.size(); ++i)
                    vhandvalues[i] += vhanddelta[i];

                _robot->SetActiveDOFValues(vhandvalues);

                if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) {
                    return 1000;
                }
                // don't check self collisions for multiple DOF since don't know how dof will actually get to the final config!!!!
                if(( vhandvalues.size() == 1) && _robot->CheckSelfCollision()) {
                    return 1000;
                }
            }

            // check self collision with the final config
            if( _robot->CheckSelfCollision() )
                return 1000;

            // check that final goal is away from obstacles
            newvalues.resize(vhandgoal.size());
            numiter = 10;
            while(numiter > 0) {
                for(size_t i = 0; i < newvalues.size(); ++i) {
                    newvalues[i] = vhandgoal[i] + 0.2f*(RaveRandomFloat()-0.5f);
                }
                _robot->SetActiveDOFValues(newvalues, true);
                if( _robot->CheckSelfCollision() ) {
                    continue;
                }
                if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) ) {
                    return 1000;
                }
                --numiter;
            }

            // not in collision so returnt true
            return 0;
        }

        vector<dReal> vhandgoal;
        vector<int> vhandjoints;
        float thresh;

        static bool _MoveUnsyncJoints(EnvironmentBasePtr penv, RobotBasePtr robot, TrajectoryBasePtr ptraj, const vector<int>& vhandjoints, const vector<dReal>& vhandgoal, const std::string& pplannername="BasicRRT",int maxdivision=10)
        {
            if( vhandjoints.size() == 0 ||vhandjoints.size() != vhandgoal.size() ) {
                return false;
            }
            BOOST_ASSERT(!!ptraj);

            if( robot->GetActiveConfigurationSpecification() != ptraj->GetConfigurationSpecification() ) {
                ptraj->Init(robot->GetActiveConfigurationSpecification());
            }
            boost::shared_ptr<MoveUnsync> pgoalfn(new MoveUnsync());
            pgoalfn->thresh = 0;
            pgoalfn->vhandjoints = vhandjoints;
            pgoalfn->vhandgoal = vhandgoal;
            pgoalfn->_maxdivision=maxdivision;
            pgoalfn->SetRobot(robot);

            PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
            params->SetRobotActiveJoints(robot);
            params->_goalfn = boost::bind(&MoveUnsync::Eval,pgoalfn,_1);
            params->_nMaxIterations = 5000;
            robot->GetActiveDOFValues(params->vinitialconfig);
            params->_fStepLength = 0.04f;

            if( pgoalfn->Eval(params->vinitialconfig) <= pgoalfn->GetGoalThresh() ) {
                // already done
                ptraj->Insert(0,params->vinitialconfig);
                return true;
            }

            boost::shared_ptr<PlannerBase> planner(RaveCreatePlanner(penv,pplannername));
            if( !planner ) {
                RAVELOG_WARN(str(boost::format("failed to find planner %s\n")%pplannername));
                return false;
            }

            if( !planner->InitPlan(robot, params) ) {
                return false;
            }
            if( !planner->PlanPath(ptraj) ) {
                return false;
            }

            return true;
        }

protected:
        vector<dReal> vhandvalues, vhanddelta, values, newvalues;
        RobotBasePtr _robot;
        int _maxdivision;
    };

    template <typename T>
    class GripperJacobianConstrains {
public:
        GripperJacobianConstrains(RobotBase::ManipulatorPtr pmanip, const Transform& tTargetWorldFrame, const boost::array<T,6>& vfreedoms, T errorthresh=1e-3) : _pmanip(pmanip), _vfreedoms(vfreedoms) {
            _errorthresh2 = errorthresh*errorthresh;
            _probot = _pmanip->GetRobot();
            _tOriginalEE = _pmanip->GetTransform();
            _tTargetFrameLeft = tTargetWorldFrame*_tOriginalEE.inverse();
            _tTargetFrameRight = tTargetWorldFrame.inverse();
            _J.resize(6,_probot->GetActiveDOF());
            _invJJt.resize(6,6);
            _error.resize(6,1);
            _nMaxIterations = 40;
            _pmanip->GetRobot()->GetActiveDOFLimits(_vlower,_vupper);

        }
        virtual ~GripperJacobianConstrains() {
        }

        bool RetractionConstraint(std::vector<dReal>& vprev, const std::vector<dReal>& vdelta)
        {
            const T lambda2 = 1e-8;         // normalization constant
            using namespace boost::numeric::ublas;
            std::vector<dReal> vnew = vprev;
            for(size_t i = 0; i < vnew.size(); ++i) {
                vnew[i] += vdelta.at(i);
                if( vnew[i] < _vlower[i] ) {
                    vnew[i] = _vlower[i];
                }
                else if( vnew[i] > _vupper[i] ) {
                    vnew[i] = _vupper[i];
                }
            }
            dReal fdistprev = _distmetricfn(vprev,vnew), fdistcur=0;
            _lasterror=0;
            _probot->SetActiveDOFValues(vnew);
            for(_iter = 0; _iter < _nMaxIterations; ++_iter) {
                Transform tEE = _pmanip->GetTransform();
                Transform t = _tTargetFrameLeft * tEE * _tTargetFrameRight;

                T totalerror=0;
                for(int i = 0; i < 3; ++i) {
                    dReal ang = 2.0f*RaveAtan2(t.rot[i+1],t.rot[0]);
                    if( ang > PI ) {
                        ang -= 2*PI;
                    }
                    if( ang < -PI ) {
                        ang += 2*PI;
                    }
                    _error(i,0) = _vfreedoms[i]*ang;
                    _error(i+3,0) = _vfreedoms[i+3]*t.trans[i];
                    totalerror += _error(i,0)*_error(i,0) + _error(3+i,0)*_error(3+i,0);
                }
                if( totalerror < _errorthresh2 ) {
                    vprev = vnew;
                    return true;
                }
                // 4.0 is an arbitrary number...
                if(( totalerror > 4.0*_lasterror) &&( fdistcur > 4.0*fdistprev) ) {
                    // last adjustment was greater than total distance (jacobian was close to being singular)
                    _iter = -1;
                    //RAVELOG_INFO(str(boost::format("%f > %f && %f > %f\n")%totalerror%_lasterror%fdistcur%fdistprev));
                    return false;
                }
                _lasterror = totalerror;

                // compute jacobian
                _pmanip->CalculateAngularVelocityJacobian(_vjacobian);
                for(size_t i = 0; i < 3; ++i) {
                    std::copy(_vjacobian[i].begin(),_vjacobian[i].end(),_J.find2(0,i,0));
                }
                _pmanip->CalculateJacobian(_vjacobian);
                for(size_t i = 0; i < 3; ++i) {
                    std::copy(_vjacobian[i].begin(),_vjacobian[i].end(),_J.find2(0,3+i,0));
                }
                // pseudo inverse of jacobian
                _Jt = trans(_J);
                _invJJt = prod(_J,_Jt);
                for(int i = 0; i < 6; ++i) {
                    _invJJt(i,i) += lambda2;
                }
                try {
                    if( !InvertMatrix(_invJJt,_invJJt) ) {
                        RAVELOG_VERBOSE("failed to invert matrix\n");
                        _iter = -1;
                        return false;
                    }
                }
                catch(...) {
                    _iter = -1;
                    return false;
                }
                _invJ = prod(_Jt,_invJJt);
                _qdelta = prod(_invJ,_error);
                for(size_t i = 0; i < vnew.size(); ++i) {
                    vnew.at(i) += _qdelta(i,0);
                }
                fdistcur = _distmetricfn(vprev,vnew);
                _probot->SetActiveDOFValues(vnew);         // for next iteration
            }

            _iter = -1;
            RAVELOG_DEBUG("constraint function exceeded iterations\n");
            return false;
        }

        // Matrix inversion routine. Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
        static bool InvertMatrix(const boost::numeric::ublas::matrix<T>& input, boost::numeric::ublas::matrix<T>& inverse) {
            using namespace boost::numeric::ublas;
            matrix<T> A(input);         // create a working copy of the input
            permutation_matrix<std::size_t> pm(A.size1());         // create a permutation matrix for the LU-factorization
            int res = lu_factorize(A,pm);         // perform LU-factorization
            if( res != 0 )
                return false;

            inverse.assign(identity_matrix<T>(A.size1()));         // create identity matrix of "inverse"
            lu_substitute(A, pm, inverse);         // backsubstitute to get the inverse
            return true;
        }

        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

        // statistics about last run
        int _iter;
        double _lasterror;
        int _nMaxIterations;

protected:
        RobotBasePtr _probot;
        RobotBase::ManipulatorPtr _pmanip;
        Transform _tTargetFrameLeft, _tTargetFrameRight,_tOriginalEE;
        vector<dReal> _vlower, _vupper;
        boost::array<T,6> _vfreedoms;
        T _errorthresh2;
        boost::multi_array<dReal,2> _vjacobian;
        boost::numeric::ublas::matrix<T> _J, _Jt, _invJJt, _invJ, _error, _qdelta;
    };

    static bool SetActiveTrajectory(RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout,dReal fMaxVelMult=1)
    {
        if( pActiveTraj->GetNumWaypoints() == 0 ) {
            return false;
        }
        if( pActiveTraj->GetDuration() == 0 && pActiveTraj->GetNumWaypoints() > 1 ) {
            planningutils::RetimeActiveDOFTrajectory(pActiveTraj,robot);
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

    inline static dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
    {
        dReal facos = RaveAcos(min(dReal(1),RaveFabs(t1.rot.dot(t2.rot))));
        return (t1.trans-t2.trans).lengthsqr3() + frotweight*facos*facos;
    }

    Vector SampleQuaternion()
    {
        Vector v;
        while(1) {
            v.x = 2*RaveRandomFloat()-1;
            v.y = 2*RaveRandomFloat()-1;
            v.z = 2*RaveRandomFloat()-1;
            v.w = 2*RaveRandomFloat()-1;
            dReal flen = v.lengthsqr4();
            if( flen > 1 )
                continue;
            flen = 1.0f/RaveSqrt(flen);
            return v*(1.0f/RaveSqrt(flen));
        }
        return Vector();
    }

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */   \
    (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */   \
    (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

    // generate a sphere triangulation starting with an icosahedron
    // all triangles are oriented counter clockwise
    static void GenerateSphereTriangulation(KinBody::Link::TRIMESH& tri, int levels)
    {
        KinBody::Link::TRIMESH temp, temp2;

        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));

        const int nindices=60;
        int indices[nindices] = {
            0, 1, 2,
            1, 3, 4,
            3, 5, 6,
            2, 4, 7,
            5, 6, 8,
            2, 7, 9,
            0, 5, 8,
            7, 9, 10,
            0, 1, 5,
            7, 10, 11,
            1, 3, 5,
            6, 10, 11,
            3, 6, 11,
            9, 10, 8,
            3, 4, 11,
            6, 8, 10,
            4, 7, 11,
            1, 2, 4,
            0, 8, 9,
            0, 2, 9
        };

        Vector v[3];

        // make sure oriented CCW
        for(int i = 0; i < nindices; i += 3 ) {
            v[0] = temp.vertices[indices[i]];
            v[1] = temp.vertices[indices[i+1]];
            v[2] = temp.vertices[indices[i+2]];
            if( v[0].dot3((v[1]-v[0]).cross(v[2]-v[0])) < 0 ) {
                swap(indices[i], indices[i+1]);
            }
        }

        temp.indices.resize(nindices);
        std::copy(&indices[0],&indices[nindices],temp.indices.begin());

        KinBody::Link::TRIMESH* pcur = &temp;
        KinBody::Link::TRIMESH* pnew = &temp2;
        while(levels-- > 0) {

            pnew->vertices.resize(0);
            pnew->vertices.reserve(2*pcur->vertices.size());
            pnew->vertices.insert(pnew->vertices.end(), pcur->vertices.begin(), pcur->vertices.end());
            pnew->indices.resize(0);
            pnew->indices.reserve(4*pcur->indices.size());

            map< uint64_t, int > mapnewinds;
            map< uint64_t, int >::iterator it;

            for(size_t i = 0; i < pcur->indices.size(); i += 3) {
                // for ever tri, create 3 new vertices and 4 new triangles.
                v[0] = pcur->vertices[pcur->indices[i]];
                v[1] = pcur->vertices[pcur->indices[i+1]];
                v[2] = pcur->vertices[pcur->indices[i+2]];

                int inds[3];
                for(int j = 0; j < 3; ++j) {
                    uint64_t key = ((uint64_t)pcur->indices[i+j]<<32)|(uint64_t)pcur->indices[i + ((j+1)%3) ];
                    it = mapnewinds.find(key);

                    if( it == mapnewinds.end() ) {
                        inds[j] = mapnewinds[key] = mapnewinds[(key<<32)|(key>>32)] = (int)pnew->vertices.size();
                        pnew->vertices.push_back((v[j]+v[(j+1)%3 ]).normalize3());
                    }
                    else {
                        inds[j] = it->second;
                    }
                }

                pnew->indices.push_back(pcur->indices[i]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[2]);
                pnew->indices.push_back(inds[0]);    pnew->indices.push_back(pcur->indices[i+1]);    pnew->indices.push_back(inds[1]);
                pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[1]);
                pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[1]);    pnew->indices.push_back(pcur->indices[i+2]);
            }

            swap(pnew,pcur);
        }

        tri = *pcur;
    }
};

/// returns a random sequence of permuations
template <class T> void PermutateRandomly(vector<T>& vpermutation)
{
    if( vpermutation.size() <= 1 ) {
        return;
    }
    for(size_t i = 0; i < vpermutation.size()-1; ++i) {
        std::swap(vpermutation[i], vpermutation[i+(rand()%(vpermutation.size()-i))]);
    }
}

/// permute a sequence of n numbers
/// and execute a function for each number in that sequence
/// if the function returns true, break from executing further
/// functions, otherwise continue
class RandomPermutationExecutor
{
public:
    RandomPermutationExecutor() : nextindex(-1) {
    }
    RandomPermutationExecutor(const boost::function<bool(int)>& fn) : _fn(fn), nextindex(-1) {
    }

    /// returns the index of the permutation that the function returned true in
    /// or -1 if function never returned true
    void PermuteStart(unsigned int permutationsize) {
        BOOST_ASSERT( permutationsize > 0);
        vpermutation.resize(permutationsize);
        for(unsigned int i = 0; i < permutationsize; ++i) {
            vpermutation[i] = i;
        }
        nextindex = 0;
    }

    /// continue from last time
    int PermuteContinue() {
        if(( nextindex < 0) ||( nextindex >= vpermutation.size()) ) {
            return -1;
        }
        for(unsigned int i = nextindex; i < vpermutation.size(); ++i) {
            std::swap(vpermutation[i], vpermutation[i+(rand()%(vpermutation.size()-i))]);
            if( _fn(vpermutation[i]) ) {
                nextindex = i+1;
                return vpermutation[i];
            }
        }

        nextindex = -1;
        return -1;
    }

    boost::function<bool(int)> _fn;

private:
    std::vector<unsigned int> vpermutation;
    unsigned int nextindex;
};

class RealVectorCompare
{
public:
    RealVectorCompare(dReal thresh) : _thresh(thresh) {
    }
    bool operator()(const vector<dReal> & v1, const vector<dReal>& v2) const
    {
        if( v1.size() != v2.size() )
            return true;

        for(size_t i = 0; i < v1.size(); ++i) {
            if( v1[i] < v2[i]-_thresh )
                return true;
            else if( v1[i] > v2[i]+_thresh )
                return false;
        }

        return false;
    }

    dReal _thresh;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RandomPermutationExecutor)
#endif

#endif
