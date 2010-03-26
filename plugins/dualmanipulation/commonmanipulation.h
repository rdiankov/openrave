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

class CM
{
 public:
    static bool JitterActiveDOF(RobotBasePtr robot,int nMaxIterations=5000)
    {
        RAVELOG_VERBOSEA("starting jitter active dof...\n");
        vector<dReal> curdof, newdof, lower, upper;
        robot->GetActiveDOFValues(curdof);
        robot->GetActiveDOFLimits(lower, upper);
        newdof = curdof;

        dReal fRand = 0.03f;
        int iter = 0;

        if(robot->CheckSelfCollision())
            RAVELOG_WARNA("JitterActiveDOFs: initial config in self collision!\n");

        while(robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || robot->CheckSelfCollision() ) {
            if( iter > nMaxIterations ) {
                RAVELOG_WARNA("Failed to find noncolliding position for robot\n");

                robot->SetActiveDOFValues(curdof);

                // display collision report
                COLLISIONREPORT report;
                if( robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot), CollisionReportPtr(&report,null_deleter())) ) {
                    if( !!report.plink1 && !!report.plink2 ) {
                        RAVELOG_WARNA(str(boost::format("Jitter collision %s:%s with %s:%s\n")%report.plink1->GetParent()->GetName()%report.plink1->GetName()%report.plink2->GetParent()->GetName()%report.plink2->GetName()));
                    }
                }
            
                return false;        
            }

            for(int j = 0; j < robot->GetActiveDOF(); j++)
                newdof[j] = CLAMP_ON_RANGE(curdof[j] + fRand * (RaveRandomFloat()-0.5f), lower[j], upper[j]);

            /// dangerous
            //        if( (iter%1000) == 499 )
            //            fRand *= 2;

            robot->SetActiveDOFValues(newdof);
            ++iter;
        }
    
        return true;
    }

    static bool JitterTransform(KinBodyPtr pbody, float fJitter, int nMaxIterations=1000)
    {
        RAVELOG_VERBOSEA("starting jitter transform...\n");

        // randomly add small offset to the body until it stops being in collision
        Transform transorig = pbody->GetTransform();
        Transform transnew = transorig;
        int iter = 0;
        while(pbody->GetEnv()->CheckCollision(KinBodyConstPtr(pbody)) ) {
            if( iter > nMaxIterations )
                return false;

            transnew.trans = transorig.trans + fJitter * Vector(RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f);
            pbody->SetTransform(transnew);
            ++iter;
        }

        return true;
    }

    /// Samples numsamples of solutions and each solution to vsolutions
    /// \return number of ik solutions sampled
    static int SampleIkSolutions(RobotBasePtr robot, const Transform& tgrasp, int numsamples, vector<dReal>& vsolutions)
    {
        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        if( numsamples <= 0 )
            return 0;
        // quickly prune grasp is end effector is in collision
        if( pmanip->CheckEndEffectorCollision(tgrasp) ) {
            RAVELOG_VERBOSEA("sampleiksolutions: gripper in collision\n");
            return 0;
        }

        int _numsamples = numsamples;

        vector< vector<dReal> > viksolutions;
        vector<dReal> vfree(pmanip->GetNumFreeParameters());
        for(int iter = 0; iter < 50*numsamples; ++iter) {//it puts random values in vfree params and checks to see if there is an IK solution for that config..and returns the numspamples as soon as adequate number of soulutions have been found..thats the reson why we use 50*numsamples
            for(int i = 0; i < (int)vfree.size(); ++i)
                vfree[i] = RaveRandomFloat();
            
            if( pmanip->FindIKSolutions(tgrasp, vfree, viksolutions, true) ) {
                FOREACH(itsol, viksolutions) {
                    vsolutions.insert(vsolutions.end(), itsol->begin(), itsol->end());
                    if( --_numsamples <= 0 )
                        return numsamples;
                }
            }
        }
        
        bool bSuccess = pmanip->FindIKSolutions(tgrasp, viksolutions, true);
        if( !bSuccess || viksolutions.size() == 0 )
            return false;

        while(1) {
            FOREACH(itsol, viksolutions) {
                vsolutions.insert(vsolutions.end(), itsol->begin(), itsol->end());
                if( --_numsamples <= 0 )
                    return numsamples;
            }
        }

        return numsamples-_numsamples;
    }

    //Constraint function for maintaining relative orientations of the EEFs of both manipulators
    template <typename T>
    class DualArmManipulation {
    public:
    DualArmManipulation(RobotBasePtr probot, RobotBase::ManipulatorPtr pmanipA, RobotBase::ManipulatorPtr pmanipI) : _probot(probot), _pmanipA(pmanipA), _pmanipI(pmanipI) {
            _tOriginalEEI = _pmanipI->GetEndEffectorTransform();
            _tOriginalEEA = _pmanipA->GetEndEffectorTransform();
            _diff = _tOriginalEEA.inverse()*_tOriginalEEI;
                           
        }
        virtual ~DualArmManipulation() {}

        bool DualArmConstrained(const std::vector<dReal>& vprev, std::vector<dReal>& vcur, int settings)
        {
            std::vector<dReal> vnew = vcur;
            bool pstatus=true;
            double errorRot=0.1;
            double errorTrans=.010;
            vector<int> JointIndicesI;
        
            std::vector<dReal> vold, vsolution;
            _probot->GetJointValues(vold);  //gets the current robot config
            Transform tA = _pmanipA->GetEndEffectorTransform();
        
            Transform tInew= tA*_diff;  //this is (wTRANSl)*(lTRANSr)
            bool a= _pmanipI->FindIKSolution(tInew,vsolution, false);

            if(a){
                vector<int> JointIndicesI = _pmanipI->GetArmJoints();

                for (size_t i=0;i<JointIndicesI.size();i++) {//this check is important to make sure the IK solution does not fly too far away since there are multiple Ik solutions possible
                    if(fabs(vsolution.at(i)-vprev[JointIndicesI.at(i)])<errorRot*2)
                        vcur[JointIndicesI.at(i)]=vsolution[i];
                    else 
                        return false;
                }
                pstatus=true;
            }
            else 
                return false;

            //now checking
            //  vnew=vcur;
            _probot->SetActiveDOFValues(vcur);
            Transform tI = _pmanipI->GetEndEffectorTransform();
            tA = _pmanipA->GetEndEffectorTransform();
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

    static bool SetActiveTrajectory(RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout)
    {
        if( pActiveTraj->GetPoints().size() == 0 )
            return false;

        pActiveTraj->CalcTrajTiming(robot, pActiveTraj->GetInterpMethod(), true, true);

        bool bExecuted = false;
        if( bExecute ) {
            if( pActiveTraj->GetPoints().size() > 1 ) {
                robot->SetActiveMotion(pActiveTraj);
                bExecute = true;
            }
            // have to set anyway since calling script will orEnvWait!
            else if( !!robot->GetController() ) {
                TrajectoryBasePtr pfulltraj = robot->GetEnv()->CreateTrajectory(robot->GetDOF());
                robot->GetFullTrajectoryFromActive(pfulltraj, pActiveTraj);

                if( robot->GetController()->SetDesired(pfulltraj->GetPoints()[0].q))
                    bExecuted = true;
            }
        }

        if( strsavetraj.size() || !!pout ) {
            TrajectoryBasePtr pfulltraj = robot->GetEnv()->CreateTrajectory(robot->GetDOF());
            robot->GetFullTrajectoryFromActive(pfulltraj, pActiveTraj);

            if( strsavetraj.size() > 0 )
                pfulltraj->Write(strsavetraj, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);

            if( !!pout )
                pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
        }
    
        return bExecuted;
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
            if( dot3(v[0], (v[1]-v[0]).Cross(v[2]-v[0])) < 0 )
                swap(indices[i], indices[i+1]);
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

#endif
