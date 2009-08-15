#ifndef  OPENRAVE_RSENSORPROBLEM
#define  OPENRAVE_RSENSORPROBLEM

// Misc player communication functions
class PlayerProblem : public ProblemInstance
{
    

public:
    PlayerProblem(EnvironmentBase* penv);
    virtual ~PlayerProblem();
    virtual void Destroy();
    virtual int main(const char* cmd, EnvironmentBase*);
    virtual void SetActiveRobots(const std::vector<RobotBase*>& robots);

    /// returns true when problem is solved
    virtual bool SimulationStep(dReal fElapsedTime);
    virtual void SendCommand(const char* cmd, string& response);
    virtual void Query(const char* query, string& response);
    
private:

    void GetRealLaserMap(string& response);
    void GetRealLaserMap2D(string& response);
    void GetPosition3dData(string& response);
    
    wstring _strRobotName; // name to init robot with

    RobotBase* robot;

    void StartPlayerThread();
    void RunPlayerThread();
    void StopPlayerThread();
    boost::thread* _pThread;
    bool _bStopPlayerThread; // notification from main thread to reader thread
    bool _bDestroyPlayerThread; // notification from reader thread to main thread
    PlayerClient* _pclient;
    LaserProxy* _plaserclient;
    ActArrayProxy* _plasermotorclient;
    Position3dProxy* _pposclient;
    pthread_mutex_t _mutex_player;
};

#endif
