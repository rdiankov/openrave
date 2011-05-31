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
    class SimpleDistMetric
    {
    public:
    SimpleDistMetric(RobotBasePtr robot) : _robot(robot) {
            _robot->GetActiveDOFWeights(weights);
        }
        virtual dReal Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1)
        {
            std::vector<dReal> c = c0;
            _robot->SubtractActiveDOFValues(c,c1);
            dReal dist = 0;
            for(int i=0; i < _robot->GetActiveDOF(); i++) {
                dist += weights.at(i)*c.at(i)*c.at(i);
            }
            return RaveSqrt(dist);
        }

    protected:
        RobotBasePtr _robot;
        vector<dReal> weights;
    };

    /// \return 0 if jitter failed and robot is in collision, -1 if robot originally not in collision, 1 if jitter succeeded
    typedef boost::function<bool(const std::vector<dReal>&, std::vector<dReal>&, int)> ConstraintFn;
    static int JitterActiveDOF(RobotBasePtr robot,int nMaxIterations=5000,dReal fRand=0.03f,const ConstraintFn& constraintfn = ConstraintFn())
    {
        RAVELOG_VERBOSE("starting jitter active dof...\n");
        vector<dReal> curdof, newdof;
        robot->GetActiveDOFValues(curdof);
        int iter = 0;
        CollisionReport report;
        CollisionReportPtr preport(&report,null_deleter());
        bool bCollision = false;
        bool bConstraint = !!constraintfn;
        if( !bConstraint || constraintfn(curdof,curdof,0) ) {
            if( bConstraint ) {
                // curdof might have changed from constraint function
                robot->SetActiveDOFValues(curdof);
                //robot->GetActiveDOFValues(curdof); // necessary?
            }
            if(robot->CheckSelfCollision(preport)) {
                bCollision = true;
                RAVELOG_DEBUG(str(boost::format("JitterActiveDOFs: initial config in self collision: %s!\n")%report.__str__()));
            }
            if( robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot),preport) ) {
                bCollision = true;
                RAVELOG_DEBUG(str(boost::format("JitterActiveDOFs: initial config in collision: %s!\n")%report.__str__()));
            }
            if( !bCollision ) {
                return -1;
            }
        }
        else {
            RAVELOG_DEBUG("constraint function failed\n");
        }

        newdof.resize(curdof.size());
        do {
            if( iter++ > nMaxIterations ) {
                RAVELOG_WARN("Failed to find noncolliding position for robot\n");
                robot->SetActiveDOFValues(curdof,true);
                return 0;
            }
            for(int j = 0; j < robot->GetActiveDOF(); j++) {
                newdof[j] = curdof[j] + fRand * (RaveRandomFloat()-0.5f);
            }
            robot->SetActiveDOFValues(newdof,true);
            robot->GetActiveDOFValues(newdof);
            if( bConstraint && !constraintfn(curdof,newdof,0) ) {
                continue;
            }
        } while(robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || robot->CheckSelfCollision() );
    
        return 1;
    }

    static bool JitterTransform(KinBodyPtr pbody, float fJitter, int nMaxIterations=1000)
    {
        RAVELOG_VERBOSE("starting jitter transform...\n");

        // randomly add small offset to the body until it stops being in collision
        Transform transorig = pbody->GetTransform();
        Transform transnew = transorig;
        int iter = 0;
        while(pbody->GetEnv()->CheckCollision(KinBodyConstPtr(pbody)) ) {
            if( iter > nMaxIterations ) {
                return false;
            }
            transnew.trans = transorig.trans + fJitter * Vector(RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f);
            pbody->SetTransform(transnew);
            ++iter;
        }

        return true;
    }

    /// Samples numsamples of solutions and each solution to vsolutions
    /// \return number of ik solutions sampled
    static int SampleIkSolutions(RobotBasePtr robot, const IkParameterization& ikp, int numsamples, vector<dReal>& vsolutions)
    {
        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        if( numsamples <= 0 ) {
            return 0;
        }
        // quickly prune grasp is end effector is in collision
        if( ikp.GetType() == IkParameterization::Type_Transform6D ) {
            CollisionReportPtr report(new CollisionReport());
            if( pmanip->CheckEndEffectorCollision(ikp.GetTransform6D(),report) ) {
                RAVELOG_VERBOSE(str(boost::format("sampleiksolutions gripper in collision: %s.\n")%report->__str__()));
                return 0;
            }
        }

        int _numsamples = numsamples;

        vector< vector<dReal> > viksolutions;
        vector<dReal> vfree(pmanip->GetIkSolver()->GetNumFreeParameters());
        for(int iter = 0; iter < 50*numsamples; ++iter) {
            for(int i = 0; i < (int)vfree.size(); ++i) {
                vfree[i] = RaveRandomFloat();
            }
            if( pmanip->FindIKSolutions(ikp, vfree, viksolutions, true) ) {
                FOREACH(itsol, viksolutions) {
                    vsolutions.insert(vsolutions.end(), itsol->begin(), itsol->end());
                    if( --_numsamples <= 0 ) {
                        return numsamples;
                    }
                }
            }
        }
        
        bool bSuccess = pmanip->FindIKSolutions(ikp, viksolutions, true);
        if( !bSuccess || viksolutions.size() == 0 ) {
            //RAVELOG_WARN("findiksolutions returns nothing\n");
            return false;
        }

        while(1) {
            FOREACH(itsol, viksolutions) {
                vsolutions.insert(vsolutions.end(), itsol->begin(), itsol->end());
                if( --_numsamples <= 0 ) {
                    return numsamples;
                }
            }
        }

        return numsamples-_numsamples;
    }

    class MoveUnsync
    {
    public:
    MoveUnsync() : _maxdivision(10) {}
        virtual ~MoveUnsync() {}

        virtual void SetRobot(RobotBasePtr robot) { _robot = robot; thresh = 0; }
        virtual float GetGoalThresh() { return thresh; }
        
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
                if( vhandvalues.size() == 1 && _robot->CheckSelfCollision()) {
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
            if( vhandjoints.size() == 0 || vhandjoints.size() != vhandgoal.size() || !ptraj )
                return false;
    
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
                Trajectory::TPOINT pt; pt.q = params->vinitialconfig;
                ptraj->AddPoint(pt);
                return true;
            }

            boost::shared_ptr<PlannerBase> planner(RaveCreatePlanner(penv,pplannername));
            if( !planner ) {
                RAVELOG_WARN(str(boost::format("failed to find planner %s\n")%pplannername));
                return false;
            }
    
            if( !planner->InitPlan(robot, params) )
                return false;
            if( !planner->PlanPath(ptraj) )
                return false;

            //ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
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
            _nMaxIterations = 20;
            
        }
        virtual ~GripperJacobianConstrains() {}

        bool RetractionConstraint(const std::vector<dReal>& vprev, std::vector<dReal>& vcur, int settings)
        {
            const T lambda2 = 1e-8; // normalization constant
            using namespace boost::numeric::ublas;
            std::vector<dReal> vnew = vcur;
            dReal fdistprev = _distmetricfn(vprev,vcur), fdistcur=0;
            _lasterror=0;
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
                    vcur = vnew;
                    return true;
                }
                // 4.0 is an arbitrary number...
                if( totalerror > 4.0*_lasterror && fdistcur > 4.0*fdistprev ) {
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
                fdistcur = _distmetricfn(vcur,vnew);
                _probot->SetActiveDOFValues(vnew); // for next iteration
            }
            
            _iter = -1;
            RAVELOG_WARN("constraint function exceeded iterations\n");
            return false;
        }

        // Matrix inversion routine. Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
        static bool InvertMatrix(const boost::numeric::ublas::matrix<T>& input, boost::numeric::ublas::matrix<T>& inverse) {
            using namespace boost::numeric::ublas;
            matrix<T> A(input); // create a working copy of the input
            permutation_matrix<std::size_t> pm(A.size1()); // create a permutation matrix for the LU-factorization
            int res = lu_factorize(A,pm); // perform LU-factorization
            if( res != 0 )
                return false;

            inverse.assign(identity_matrix<T>(A.size1())); // create identity matrix of "inverse"
            lu_substitute(A, pm, inverse); // backsubstitute to get the inverse
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
        boost::array<T,6> _vfreedoms;
        T _errorthresh2;
        boost::multi_array<dReal,2> _vjacobian;
        boost::numeric::ublas::matrix<T> _J, _Jt, _invJJt, _invJ, _error, _qdelta;
    };

    static bool SetActiveTrajectory(RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout,dReal fMaxVelMult=1)
    {
        if( pActiveTraj->GetPoints().size() == 0 )
            return false;

        pActiveTraj->CalcTrajTiming(robot, pActiveTraj->GetInterpMethod(), true, true,fMaxVelMult);

        bool bExecuted = false;
        if( bExecute ) {
            if( pActiveTraj->GetPoints().size() > 1 ) {
                robot->SetActiveMotion(pActiveTraj);
                bExecute = true;
            }
            // have to set anyway since calling script will orEnvWait!
            else if( !!robot->GetController() ) {
                TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(robot->GetEnv(),robot->GetDOF());
                robot->GetFullTrajectoryFromActive(pfulltraj, pActiveTraj);

                if( robot->GetController()->SetDesired(pfulltraj->GetPoints()[0].q))
                    bExecuted = true;
            }
        }

        if( strsavetraj.size() || !!pout ) {
            TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(robot->GetEnv(),robot->GetDOF());
            robot->GetFullTrajectoryFromActive(pfulltraj, pActiveTraj);

            if( strsavetraj.size() > 0 ) {
                ofstream f(strsavetraj.c_str());
                pfulltraj->Write(f, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            }
            if( !!pout )
                pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
        }
    
        return bExecuted;
    }

    static bool SetFullTrajectory(RobotBasePtr robot, TrajectoryBasePtr pfulltraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout)
    {
        if( pfulltraj->GetPoints().size() == 0 )
            return false;

        bool bExecuted = false;
        if( bExecute ) {
            if( pfulltraj->GetPoints().size() > 1 ) {
                robot->SetMotion(pfulltraj);
                bExecute = true;
            }
            // have to set anyway since calling script will orEnvWait!
            else if( !!robot->GetController() ) {
                if( robot->GetController()->SetDesired(pfulltraj->GetPoints()[0].q))
                    bExecuted = true;
            }
        }

        if( strsavetraj.size() || !!pout ) {
            if( strsavetraj.size() > 0 ) {
                ofstream f(strsavetraj.c_str());
                pfulltraj->Write(f, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            }
            if( !!pout )
                pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
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
    if( vpermutation.size() <= 1 )
        return;
    for(size_t i = 0; i < vpermutation.size()-1; ++i)
        std::swap(vpermutation[i], vpermutation[i+(rand()%(vpermutation.size()-i))]);
}

/// permute a sequence of n numbers
/// and execute a function for each number in that sequence
/// if the function returns true, break from executing further
/// functions, otherwise continue
class RandomPermuationExecutor
{
public:
 RandomPermuationExecutor() : nextindex(-1) {}
 RandomPermuationExecutor(const boost::function<bool(int)>& fn) : _fn(fn), nextindex(-1) {}

    /// returns the index of the permutation that the function returned true in
    /// or -1 if function never returned true
    void PermuteStart(unsigned int permutationsize) {
        BOOST_ASSERT( permutationsize > 0);
        vpermutation.resize(permutationsize);
        for(unsigned int i = 0; i < permutationsize; ++i)
            vpermutation[i] = i;

        nextindex = 0;
    }

    /// continue from last time
    int PermuteContinue() {
        if( nextindex < 0 || nextindex >= vpermutation.size() )
            return -1;
        
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
 RealVectorCompare(dReal thresh) : _thresh(thresh) {}
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

#ifndef MATH_RANDOM_FLOAT
#define MATH_RANDOM_FLOAT (rand()/((T)RAND_MAX))
#endif

/// \brief Generate a uniformly distributed random quaternion.
template <typename T> inline RaveVector<T> GetRandomQuat()
{
    RaveVector<T> q;
    while(1) {
        q.x = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        q.y = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        q.z = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        q.w = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        T norm = q.lengthsqr4();
        if(norm <= 1) {
            q = q * (1 / RaveSqrt(norm));
            break;
        }
    }
    return q;
}

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RandomPermuationExecutor)
#endif

#endif
