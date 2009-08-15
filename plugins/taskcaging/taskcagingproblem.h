// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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

// Functions to plan with caging grasps. See
// Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
// Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.
#ifndef OPENRAVE_TASKCONSTRAINT_H
#define OPENRAVE_TASKCONSTRAINT_H

class TaskCagingProblem : public CmdProblemInstance
{
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
        
        virtual bool endElement(void *ctx, const char *name)
        {
            if( stricmp(name,"radius") == 0 )
                _ss >> fRadius;
            else if( stricmp(name,"distthresh") == 0 )
                _ss >> fDistThresh;
            else if( stricmp(name, "goalcoeff") == 0 )
                _ss >> fGoalCoeff;
            else if( stricmp(name, "maxchildren") == 0 )
                _ss >> nMaxChildren;
            else if( stricmp(name, "maxsampletries") == 0 )
                _ss >> nMaxSampleTries;
            else
                return PlannerParameters::endElement(ctx, name);
            return false;
        }
    };

public:

    struct BODYTRAJ
    {
        BODYTRAJ() : time(0), ptarget(NULL) {}
        dReal time;
        KinBody* ptarget;
        boost::shared_ptr<Trajectory> ptraj;
    };

    // discrete algorithm
    class ConstrainedTaskData : public PlannerBase::SampleFunction, public PlannerBase::GoalFunction, public PlannerBase::ConfigurationState, public PlannerBase::DistanceMetric
    {
    public:
        enum IntervalType {
            OPEN = 0,
            OPEN_START,
            OPEN_END,
            CLOSED
        };

        struct FEATURES
        {
            FEATURES() : bSuccess(false) {}
            dReal features[3];
            dReal ftotal; // final weighted result
            bool bSuccess;
        };

        typedef pair< vector<dReal>, FEATURES > IKSOL;
        
        struct IkSolutionCompare
        {
            bool operator()( const IKSOL& a, const IKSOL& b ) const {
                return a.second.ftotal < b.second.ftotal;
            }  
        };

        struct GraspCompare
        {
            template <class T>
            bool operator()( const pair<T,dReal>& a, const pair<T,dReal>& b ) const {
                // always put the grasps with computed iksolutions first
                if( (a.first->iksolutions.size() > 0) == (b.first->iksolutions.size() > 0) )
                    return  a.second > b.second; // minimum on top of stack
                else
                    return a.first->iksolutions.size() == 0;
            }  
        };

        struct GRASP
        {
            GRASP(Transform t) { tgrasp = t; }
            
            Transform tgrasp;
            list<IKSOL> iksolutions; // first is the iksolution, second is its goal heuristic
        };

        ConstrainedTaskData(EnvironmentBase* penv);
        virtual ~ConstrainedTaskData();

        virtual void SetRobot(RobotBase* robot);
        virtual void SetState(const dReal* pstate);
        virtual void GetState(dReal* pstate);
        virtual void GetLimits(dReal* plower, dReal* pupper);
        virtual void GetState(std::vector<dReal>& state) { state.resize(GetDOF()); GetState(&state[0]); }
        virtual int GetDOF() const { return _robot->GetActiveDOF()+_vtargetjoints.size(); }

        virtual void GenerateFeatures(const dReal* q, dReal* pfeatures);
        virtual FEATURES EvalWithFeatures(const void* pConfiguration);

        virtual float GetGoalThresh() { return fGoalThresh; }

        /// robot config + target body config
        virtual float Eval(const void* pConfiguration);
        virtual float Eval(const void* c0, const void* c1);
        
        virtual dReal GraspDist(const Transform& tprev, const dReal* preshapeprev, const Transform& tnew);
        virtual bool AcceptConfig(const dReal* qprev, const vector<dReal>& qnew);

        // robot config + target body config
        virtual void Sample(dReal* pNewSample);
        virtual bool Sample(dReal* pNewSample, const dReal* pCurSample, dReal fRadius);

        bool SampleIkSolution(const Transform& tgrasp, const dReal* pCurSolution, dReal* psolution);
        
        void FillIkSolutions(GRASP& g, const vector<vector<dReal> >& solutions);
        void SetGenerateFeatures(const string& filename);
        void Log(IKSOL& iksol);
        
        bool _CheckCollision(const dReal *pQ0, const dReal *pQ1, IntervalType interval);

        inline bool IsGenerateFeatures() { return bGenerateFeatures; }

        // grasping
        vector< Transform > vGraspSet, vGraspContactSet, vGraspStartSet;
        vector< list<GRASP> > vlistGraspSet;
        dReal fGraspThresh, fConfigThresh;

        // target object
        KinBody* ptarget;
        KinBody::Link* ptargetlink;
        vector<vector<dReal> > vtargettraj;
        vector<int> _vtargetjoints; ///< active joints of the target
        int nMaxIkIterations; ///< for sampling free parameters
        int nMaxSamples; ///< for trying out heuristics
        float fGoalThresh;

        // heuristics
        dReal fWeights[3]; // joint limit, manipulability, goal config
        bool bSortSolutions;
        bool bCheckFullCollision;

    protected:

        struct PERMUTATIONDATA
        {
            ConstrainedTaskData* ptaskdata;
            vector<Transform>* pgrasps;
            dReal* psolution;
            dReal* pcursolution;
        };

        struct FINDGRASPDATA
        {
            ConstrainedTaskData* ptaskdata;
            Transform tcurgrasp;
            Transform tlink;
            vector<Transform>* pgrasps;
            dReal fThresh2;
            int status; // 0 not init, 1 init, 2 dead
        };

        static bool SampleIkPermutation(void* userdata, unsigned int index);
        static bool FindGraspPermutation(void* userdata, unsigned int index);

        bool bGenerateFeatures;
        ofstream flog; ///< logs features

        vector<dReal> _lower, _upper, _vsample, _vbestsample;
        vector<dReal> _J, _JJt;// _J is 3xdof
        vector<dReal> vtargvalues, _vfreeparams, _vcurfreeparams;
        vector<dReal> _vfeatures;
        vector<dReal> _vRobotWeights;
        map<int, RandomPermuationExecutor*> mapgrasps;
        PERMUTATIONDATA _permdata;

        EnvironmentBase* GetEnv() const { return _penv; }
        EnvironmentBase* _penv;
    };

public:
    TaskCagingProblem(EnvironmentBase* penv);
    virtual ~TaskCagingProblem();
    virtual void Destroy();
    virtual int main(const char* cmd);
    virtual void SetActiveRobots(const std::vector<RobotBase*>& robots);

    /// returns true when problem is solved
    virtual bool SimulationStep(dReal fElapsedTime);
    virtual bool SendCommand(const char* cmd, string& response);
 
private:

    bool CreateGraspSet(ostream& sout, istream& sinput); ///< creates a grasp set given a robot end-effector floating in space
    bool TaskConstrainedPlanner(ostream& sout, istream& sinput);
    bool SimpleConstrainedPlanner(ostream& sout, istream& sinput);
    bool BodyTrajectory(ostream& sout, istream& sinput);
    bool Help(ostream& sout, istream& sinput);

    // relaxed task constraints
    bool FindAllRelaxedForward(const dReal* qprev, int j, Trajectory* ptraj, ConstrainedTaskData& data);
    
    // simple task constraints
    bool FindAllSimple(const dReal* qprev, int j, list<vector<dReal> >& vtrajectory,
                       dReal fConfigThresh2, vector<list<vector<dReal> > >& vsolutions,
                       ConstrainedTaskData& data);

    // simple function
    bool JitterTransform(KinBody* pbody, dReal fJitter);

    list<BODYTRAJ> _listBodyTrajs;
    wstring _strRobotName; // name to init robot with
    RobotBase* robot;
};

///// General function to maintain a task constraint with respect to a link on the robot
//class TaskCagingFn : public PlannerBase::ConstraintFunction
//{
//public:
//    TaskCagingFn();
//
//    /// sets a door constraint
//    void SetHingeConstraint(RobotBase* probot, int linkindex, const Vector& vanchor, const Vector& vaxis);
//
//    /// passed to planner
//    bool Constraint(const dReal* pSrcConf, dReal* pDestConf, Transform* ptrans, int settings);
//
//protected:
//    int _nConstrainedLink; ///< the constrained link
//    RobotBase* _probot;
//    Transform _tTaskFrame, _tTaskFrameInv, _tLinkInitialInv; ///< task frame matrices, such that constraints*(_tTaskLeft*RobotLink*_tTaskRight) == 0
//    Vector _vConstraintTrans, _vConstraintRot; ///< constraints in the task frame
//
//    vector<dReal> J, Jinv;
//    vector<dReal> vconfig;
//    ap::real_2d_array JJt; // inverse calculation
//};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(TaskCagingProblem::BODYTRAJ)
BOOST_TYPEOF_REGISTER_TYPE(TaskCagingProblem::ConstrainedTaskData)
BOOST_TYPEOF_REGISTER_TYPE(TaskCagingProblem::ConstrainedTaskData::GRASP)
BOOST_TYPEOF_REGISTER_TYPE(TaskCagingProblem::ConstrainedTaskData::FEATURES)
BOOST_TYPEOF_REGISTER_TYPE(TaskCagingProblem::ConstrainedTaskData::PERMUTATIONDATA)
BOOST_TYPEOF_REGISTER_TYPE(TaskCagingProblem::ConstrainedTaskData::FINDGRASPDATA)

#endif

#endif
