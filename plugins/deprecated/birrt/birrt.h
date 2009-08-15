// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef  BIRRT_PLANNER_H
#define  BIRRT_PLANNER_H

#include <sstream>

class RrtNode {
public:
    //RrtNode() { Reset(); }
    RrtNode(int size,bool bFromGoal,int id) { Reset(); SetSize(size);_bFromGoal = bFromGoal;_iID = id;}
    ~RrtNode() {}

    // Methods
    inline void SetSize(int num) { _data.resize(num); }
    inline void Reset()         { _iParent = -1; _bFromGoal = false;
    _bBiasFlag = false; }
    
    inline void SetConfig(const dReal *pConfig) { memcpy(&_data[0], pConfig, _data.size()*sizeof(_data[0])); }
    inline void SetParent(int parent){_iParent = parent;}
    inline int GetID(){return _iID;}
    inline dReal* GetData()     { return &_data[0]; }
    inline vector<dReal>* GetDataVector()     { return &_data; }
    inline int GetParent() { return _iParent; }

    inline bool GetBiasFlag()   { return _bBiasFlag; }
    inline void SetBiasFlag(bool newVal)   { _bBiasFlag = newVal; }

    inline bool GetFromGoal()   { return _bFromGoal; }
    inline void SetFromGoal(bool newVal)   { _bFromGoal = newVal; }

    inline void Print() {
        if( RaveGetDebugLevel() ) {
            wstringstream s;
            s << L"ID: " << _iID << L" Parent: " << _iParent;
            for(unsigned int i = 0; i < _data.size();i++) {
                s << _data[i];
            }
            s << L"\n";
            RAVELOG(s.str().c_str());
        }
    }


private:
    // Data
    int        _iParent;
    vector<dReal> _data;
    bool       _bBiasFlag;
    bool       _bFromGoal;
    int        _iID;
};

//---------------------------------------------------------------------
//                            ColCheckRecord
//
//! Keep track of collision checks performed during optimization
//
//---------------------------------------------------------------------
class ColCheckRecord {
public:
    ColCheckRecord() {}
    ~ColCheckRecord() {}
    void Set(RrtNode *pA, RrtNode *pB)   { _pStart = pA; _pEnd = pB; }
    bool Equal(const RrtNode *pA, const RrtNode *pB)
    { return ((pA == _pStart) && (pB == _pEnd)); }
private:
    // Data
    RrtNode* _pStart;
    RrtNode* _pEnd;
};


class BirrtPlanner : public PlannerBase
{
public:

    class BirrtGoalMetric : public PlannerBase::GoalFunction
    {
    public:
        BirrtGoalMetric(RobotBase* robot_in,  const dReal* pGoalConfig_in, dReal thresh_in){_robot=robot_in; pGoalConfig = pGoalConfig_in; thresh = thresh_in;}
        ~BirrtGoalMetric(){}

        virtual float Eval(const void* pConfiguration);
        virtual float GetGoalThresh() { return thresh; }
        virtual void SetRobot(RobotBase* robot) { _robot = robot; }
        const dReal* pGoalConfig;
    protected:
        RobotBase* _robot;
        float thresh;
    };

    class BirrtDistanceMetric : public PlannerBase::DistanceMetric
    {
    public:
        BirrtDistanceMetric(RobotBase* robot_in, dReal thresh_in){_robot = robot_in; thresh = thresh_in;}
        ~BirrtDistanceMetric(){}

        /// evaluates two configurations of the robot (usually joint angles)
        virtual float Eval(const void* c0, const void* c1);

        virtual void SetRobot(RobotBase* robot) { _probot = robot; }

        dReal thresh; ///< if Eval() < thresh, then the two configurations can be considered the same

    protected:
        RobotBase* _probot;
    };

    class MakeNext
    {
    public:
        MakeNext(bool bFromGoal, RobotBase* robot);

        ~MakeNext(){}

        bool MakeNodes(int TreeSize, RrtNode* StartNode, std::vector<dReal>* targetConfig,std::vector<RrtNode>* voutput,BirrtPlanner* planner);

    private:
        bool _bFromGoal;
        RobotBase* _probot;
        std::vector<dReal> _lowerLimit;
        std::vector<dReal> _upperLimit;

        //variables for tree extension
        dReal* startConfig;
        bool bExtendConnect;
        unsigned int numDOF;
        bool success;
        dReal normDiff;
        dReal configDiff;
        dReal normDist;
        bool targetReached;

        dReal* diff;
        dReal* newConfig; 
        dReal* oldConfig; 
        dReal* dist;

    };


    class NodeTree
    {

    public:

        NodeTree(bool bFromGoal){_bFromGoal = bFromGoal; _pMakeNext = NULL; _vnodes.reserve(1000);}
        ~NodeTree(){};


        RrtNode* GetNode(const int index) {assert((index >= 0) && ((unsigned int)index < _vnodes.size()));return &_vnodes[index];}
        RrtNode* GetRandomNode() {return &_vnodes[RANDOM_INT((int)_vnodes.size())];}
        void AddNode(RrtNode node){_vnodes.push_back(node);}
        void AddNodes(std::vector<RrtNode>* nodes){for(unsigned int i=0;i < nodes->size();i++) _vnodes.push_back(nodes->at(i));}
        int GetSize() const {return (int)_vnodes.size();}
        bool GetFromGoal() const {return _bFromGoal;}
        void DeleteNodes() {_vnodes.clear(); _vnodes.reserve(1000);}

        int _iConnected;
        MakeNext* _pMakeNext;
        

    private:
        bool _bFromGoal;
        std::vector<RrtNode> _vnodes;

    };

    
    BirrtPlanner(EnvironmentBase* penv);
    ~BirrtPlanner();

    // Planning Methods
    bool InitPlan(RobotBase* pbase,const PlannerParameters* pparams);//, ColChecker* pCC);
    bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream);

    bool CleanUpReturn(bool retval);

    virtual RobotBase* GetRobot() const { return _pRobot; }

    // interval types   ( , )      ( , ]       [ , )      [ , ]
    enum IntervalType { OPEN=0,  OPEN_START,  OPEN_END,  CLOSED };

    // path memory (size = MAX_DOFS * MAX_FRAMES)
    vector<RrtNode> *GetPath() { return &vecpath; }

    virtual const wchar_t* GetDescription() const { return L"Dmitry's BiRRT Planner"; }

private:

    // Methods
    inline bool _InitChainInfo();
    inline RrtNode* _NewNode();
    inline bool _CheckCollision(dReal *pQ0, dReal *pQ1, IntervalType interval);
    inline bool _CheckCollision(const dReal *pConfig);
    inline void _PickRandomConfig();
    inline int _FindClosest(NodeTree* pNodeTree);
    inline bool _OptimizePath();
    inline bool _OptimizePathStep();
    inline bool _CreateTraj(Trajectory* traj);
    inline bool _JoinPathHalves();
    //inline int  _InterpolateNodes(const dReal* pQ0, const dReal* pQ1, dReal *frames);
    inline void _InterpolateNodes(const dReal* pQ0, const dReal* pQ1, Trajectory* traj);

    bool _OptimizePathPerDimension(int iterations);

    PlannerParameters _parameters;

    // Data
    RobotBase*         _pRobot;
    RrtNode*       _pActiveNode;
    RrtNode*       _pConnectNode;
    int            _numNodes;

    std::vector<RrtNode> _vnodes;

    std::vector<dReal>         _pInitConfig;  //!< initial and goal configs
    std::vector<dReal>         _pGoalConfig;


    bool           _bTowardBias;  //!< planning flags and parameters
    bool           _bFromGoal;
    bool           _bGoalReached;
    dReal          _towardBiasProb;  
    dReal*         _pTargetConfig;

    vector<RrtNode> vecpath;
    int            _numFrames;
    std::vector<int> _ActiveDOFIndices;
    int            _pathLength;
    ColCheckRecord* _pOptCheckRecords;
    int            _iNumOptRec;

    std::vector<dReal>         _randomConfig;  //!< chain configuration

    std::vector<dReal>          _jointResolution;  //!< joint discretization
    std::vector<dReal>          _jointResolutionInv;
    std::vector<dReal>          _jointIncrement;

    std::vector<dReal>          _lowerLimit;  //!< joint limits
    std::vector<dReal>          _upperLimit;
    std::vector<dReal>          _validRange;
    std::vector<dReal>          checkConfig;

    std::vector<Transform> _vectrans; ///< cached transformations

    bool bInit;

    bool bdelete_distmetric;

    NodeTree* _pForwardTree;
    NodeTree* _pBackwardTree;

};


#endif   // BIRRT_PLANNER_H

