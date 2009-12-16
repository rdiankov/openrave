#ifndef COMMON_MANIPULATION_H
#define COMMON_MANIPULATION_H

#include "plugindefs.h"

class CM
{
 public:
    static bool JitterActiveDOF(RobotBasePtr robot)
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
            if( iter > 5000 ) {
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

    static bool JitterTransform(KinBodyPtr pbody, float fJitter)
    {
        RAVELOG_VERBOSEA("starting jitter transform...\n");

        // randomly add small offset to the body until it stops being in collision
        Transform transorig = pbody->GetTransform();
        Transform transnew = transorig;
        int iter = 0;
        while(pbody->GetEnv()->CheckCollision(KinBodyConstPtr(pbody)) ) {
            if( iter > 1000 )
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

        int _numsamples = numsamples;

        vector< vector<dReal> > viksolutions;
        vector<dReal> vfree(pmanip->GetNumFreeParameters());
        for(int iter = 0; iter < 50*numsamples; ++iter) {
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

    class MoveUnsync
    {
    public:
        virtual void SetRobot(RobotBasePtr robot) { _robot = robot; thresh = 0; }
        virtual float GetGoalThresh() { return thresh; }
        
        virtual float Eval(const std::vector<dReal>& pConfiguration)
        {    
            // check if there's a collision when hand moves to final config
            RobotBase::RobotStateSaver saver(_robot);            
            _robot->SetActiveDOFValues(pConfiguration);
            
            _robot->SetActiveDOFs(vhandjoints);
            _robot->GetActiveDOFValues(vhandvalues);
            
            int numiter = 10;
            vhanddelta.resize(vhandjoints.size());
            for(size_t i = 0; i < vhandjoints.size(); ++i)
                vhanddelta[i] = (vhandgoal[i]-vhandvalues[i])/(dReal)numiter;
            
            while(numiter-- > 0) {
                
                for(size_t i = 0; i < vhandjoints.size(); ++i)
                    vhandvalues[i] += vhanddelta[i];
                
                _robot->SetActiveDOFValues(vhandvalues);
                
                // don't check self collisions!!!!
                if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)))
                    return 1000;
            }

            // check self collision with the final config
            if( _robot->CheckSelfCollision() )
                return 1000;

            // check that final goal is away from obstacles
            newvalues.resize(vhandgoal.size());
            numiter = 10;
            while(numiter > 0) {
                for(size_t i = 0; i < newvalues.size(); ++i)
                    newvalues[i] = vhandgoal[i] + 0.2f*(RaveRandomFloat()-0.5f);
                _robot->SetActiveDOFValues(newvalues, true);
                if( _robot->CheckSelfCollision() )
                    continue;

                if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) )
                    return 1000;
                --numiter;
            }

            // not in collision so returnt true
            return 0;
        }
    
        vector<dReal> vhandgoal;
        vector<int> vhandjoints;
        float thresh;

        static bool _MoveUnsyncJoints(EnvironmentBasePtr penv, RobotBasePtr robot, TrajectoryBasePtr ptraj, const vector<int>& vhandjoints, const vector<dReal>& vhandgoal, const std::string& pplannername="BasicRRT")
        {
            if( vhandjoints.size() == 0 || vhandjoints.size() != vhandgoal.size() || !ptraj )
                return false;
    
            boost::shared_ptr<MoveUnsync> pgoalfn(new MoveUnsync());
            pgoalfn->thresh = 0;
            pgoalfn->vhandjoints = vhandjoints;
            pgoalfn->vhandgoal = vhandgoal;
            pgoalfn->SetRobot(robot);
            
            PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
            params->SetRobotActiveJoints(robot);
            params->_goalfn = boost::bind(&MoveUnsync::Eval,pgoalfn,_1);
            params->_nMaxIterations = 20000;
            robot->GetActiveDOFValues(params->vinitialconfig);
            params->_fStepLength = 0.04f;

            if( pgoalfn->Eval(params->vinitialconfig) <= pgoalfn->GetGoalThresh() ) {
                // already done
                Trajectory::TPOINT pt; pt.q = params->vinitialconfig;
                ptraj->AddPoint(pt);
                return true;
            }

            boost::shared_ptr<PlannerBase> planner(penv->CreatePlanner(pplannername));
            if( !planner ) {
                RAVELOG_WARNA(str(boost::format("failed to find planner %s\n")%pplannername));
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
            if( strsavetraj.size() > 0 )
                pfulltraj->Write(strsavetraj, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);

            if( !!pout )
                pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
        }
    
        return bExecuted;
    }

    inline static dReal TransformDistance(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
    {
        dReal e1 = (t1.rot-t2.rot).lengthsqr4();
        dReal e2 = (t1.rot+t2.rot).lengthsqr4();
        return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*min(e1,e2));
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

class RAStarParameters : public PlannerBase::PlannerParameters
{
 public:
 RAStarParameters() : fRadius(0.1f), fDistThresh(0.03f), fGoalCoeff(1), nMaxChildren(5), nMaxSampleTries(10) {}
        
    dReal fRadius;      ///< _pDistMetric thresh is the radius that children must be within parents
    dReal fDistThresh;  ///< gamma * _pDistMetric->thresh is the sampling radius
    dReal fGoalCoeff;   ///< balancees exploratino vs cost
    int nMaxChildren;   ///< limit on number of children
    int nMaxSampleTries; ///< max sample tries before giving up on creating a child
 protected:
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;

        O << "<radius>" << fRadius << "</radius>" << endl;
        O << "<distthresh>" << fDistThresh << "</distthresh>" << endl;
        O << "<goalcoeff>" << fGoalCoeff << "</goalcoeff>" << endl;
        O << "<maxchildren>" << nMaxChildren << "</maxchildren>" << endl;
        O << "<maxsampletries>" << nMaxSampleTries << "</maxsampletries>" << endl;
    
        return !!O;
    }
        
    virtual bool endElement(const string& name)
    {
        if( name == "radius")
            _ss >> fRadius;
        else if( name == "distthresh")
            _ss >> fDistThresh;
        else if( name == "goalcoeff")
            _ss >> fGoalCoeff;
        else if( name == "maxchildren")
            _ss >> nMaxChildren;
        else if( name == "maxsampletries")
            _ss >> nMaxSampleTries;
        else
            return PlannerParameters::endElement(name);
        return false;
    }
};

class ExplorationParameters : public PlannerBase::PlannerParameters
{
 public:
 ExplorationParameters() : _fExploreProb(0), _nExpectedDataSize(100) {}
        
    dReal _fExploreProb;
    int _nExpectedDataSize;
        
 protected:
    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<exploreprob>" << _fExploreProb << "</exploreprob>" << endl;
        O << "<expectedsize>" << _nExpectedDataSize << "</expectedsize>" << endl;
        return !!O;
    }
 
    // called at the end of every XML tag, _ss contains the data 
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( name == "exploreprob")
            _ss >> _fExploreProb;
        else if( name == "expectedsize" )
            _ss >> _nExpectedDataSize;
        else // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        return false;
    }
};

class GraspParameters : public PlannerBase::PlannerParameters
{
 public:
 GraspParameters() : stand_off(0), roll_hand(0), face_target(false), bReturnTrajectory(false) {}
        
    dReal stand_off; /// start closing fingers when at this distance
    dReal roll_hand; /// rotate the hand about the palm normal by this many radians
    Vector direction,palmnormal;
    bool face_target; ///point the hand at the target or not (1 = yes, else no)
    bool bReturnTrajectory;
        
 protected:
    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<stand_off>" << stand_off << "</stand_off>" << endl;
        O << "<face_target>" << face_target << "</face_target>" << endl;
        O << "<roll_hand>" << roll_hand << "</roll_hand>" << endl;
        O << "<direction>" << direction << "</direction>" << endl;
        O << "<palmnormal>" << palmnormal << "</palmnormal>" << endl;
        O << "<returntrajectory>" << bReturnTrajectory << "</returntrajectory>" << endl;
        return !!O;
    }
 
    // called at the end of every XML tag, _ss contains the data 
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( name == "stand_off")
            _ss >> stand_off;
        else if( name == "face_target")
            _ss >> face_target;
        else if( name == "roll_hand")
            _ss >> roll_hand;
        else if( name == "direction")
            _ss >> direction;
        else if( name == "palmnormal")
            _ss >> palmnormal;
        else if( name == "returntrajectory")
            _ss >> bReturnTrajectory;
        else // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        return false;
    }
};

class GraspSetParameters : public PlannerBase::PlannerParameters
{
public:
 GraspSetParameters(EnvironmentBasePtr penv) : _nGradientSamples(5), _fVisibiltyGraspThresh(0), _fGraspDistThresh(1.4f), _penv(penv) {}
    
    vector<Transform> _vgrasps; ///< grasps with respect to the target object
    KinBodyPtr _ptarget;
    int _nGradientSamples;
    dReal _fVisibiltyGraspThresh; ///< if current grasp is less than this threshold, then visibilty is not checked
    dReal _fGraspDistThresh; ///< target grasps beyond this distance are ignored

 protected:
    EnvironmentBasePtr _penv;

    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<grasps>" << _vgrasps.size() << " ";
        FOREACHC(it, _vgrasps)
            O << *it << " ";
        O << "</grasps>" << endl;
        O << "<target>" << (!!_ptarget?_ptarget->GetNetworkId():0) << "</target>" << endl;
        O << "<numgradsamples>" << _nGradientSamples << "</numgradsamples>" << endl;
        O << "<visgraspthresh>" << _fVisibiltyGraspThresh << "</visgraspthresh>" << endl;
        O << "<graspdistthresh>" << _fGraspDistThresh << "</graspdistthresh>" << endl;
        return !!O;
    }
        
    virtual bool endElement(const string& name)
    {
        if( name == "grasps" ) {
            int ngrasps=0;
            _ss >> ngrasps;
            _vgrasps.resize(ngrasps);
            FOREACH(it, _vgrasps)
                _ss >> *it;
        }
        else if( name == "target" ) {
            int id = 0;
            _ss >> id;
            _ptarget = _penv->GetBodyFromNetworkId(id);
        }
        else if( name == "numgradsamples" )
            _ss >> _nGradientSamples;
        else if( name == "visgraspthresh" )
            _ss >> _fVisibiltyGraspThresh;
        else if( name == "graspdistthresh")
            _ss >> _fGraspDistThresh;
        else
            return PlannerParameters::endElement(name);
        return false;
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

enum IntervalType {
    IT_OPEN = 0,
    IT_OPEN_START,
    IT_OPEN_END,
    IT_CLOSED
};

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

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RandomPermuationExecutor)
#endif

#endif
