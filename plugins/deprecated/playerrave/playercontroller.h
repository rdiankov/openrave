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
#ifndef RAVE_PLAYER_CONTROLLER
#define RAVE_PLAYER_CONTROLLER

// controller for the SSC-32 board
class PlayerController : public ControllerBase
{
    enum StallAction {
        SA_Nothing=0,
        SA_StopAll=1, ///< if true, any new trajectories created will clear the trajectory queue on stalls
        SA_CancelCurrent=2,
        SA_ContinueCurrent=3,
        SA_CancelAndExecCommand=4,
    };

    struct PLAYERPROXY
    {
        PLAYERPROXY() : _pclient(NULL), pproxy(NULL), bIgnore(false), offset(0), _fCurTrajTime(0), _nCurExecutingTrajId(0) {}
        
        PlayerClient* _pclient;
        ActArrayProxy* pproxy;
        int offset;
        vector<player_actarray_actuatorgeom> vgeom; // cached actarray geometry

        // run time variables
        float _fCurTrajTime;
        int _nCurExecutingTrajId;
        bool bIgnore; ///< if true commands shouldn't be sent 
    };

    struct TRAJECTORY
    {
        TRAJECTORY() : _id(0), _stallaction(SA_Nothing) {}
        TRAJECTORY(int id, StallAction act, const string& stallcmd, const vector<PLAYERPROXY>& vproxies);

        int _id;
        StallAction _stallaction;
        string _strStallCommand;
        vector<bool> _vignore; // ignored state of the proxies in the current traj
        // specify other reactive behaviors for a trajectory
    };
    
public:
    PlayerController(EnvironmentBase* penv);
    virtual ~PlayerController();

    /// args format: host port [proxytype index]
    /// where proxytype is actarray, pos2d, or ...
    /// the order specified is the order the degrees of freedom will be arranged
    virtual bool Init(RobotBase* robot, const char* args);
    virtual void Destroy();
    
    virtual void Reset(int options);
    
    virtual bool SetDesired(const dReal* pValues);
    virtual bool SetPath(const Trajectory* ptraj);
    virtual bool SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime);
    
    virtual void SetVelocities(const std::vector<dReal>& vels);
    
    virtual bool SimulationStep(dReal fTimeElapsed);

    virtual bool SendCmd(const char* pcmd);
    virtual bool SupportsCmd(const char* pcmd);
    virtual int GetDOF() { return _probot != NULL ? _probot->GetDOF() : 0; }

    virtual float GetTime() const;
    virtual void GetVelocity(std::vector<dReal>& vel) const;
    virtual void GetTorque(std::vector<dReal>& torque) const;
    virtual ActuatorState GetActuatorState(int index) const;

    virtual bool IsDone() { return _bIsDone; }

    virtual float GetTime() const { return fTime; }
    virtual RobotBase* GetRobot() const { return _probot; }

private:

    bool MoveTo(const dReal* pdesired);
    bool MoveAtSpeed(const dReal* pdesired);

    void StartThread();
    void RunThread();
    void StopThread();

    RobotBase* _probot;           ///< controlled body
    u32 nFeedbackTime, nLastRecvTime;           ;

    std::vector<dReal> _vReadValues, _vReadVelocities, _vReadTorques;

    ActuatorState _asCurrentState;

    std::vector<ActuatorState> _vActuatorStates; ///< stores the current state of each actuator

    // player specific
    vector<PlayerClient*> _vclients;
    vector<PLAYERPROXY> _vecproxies; ///< each robot can consist of multiple proxies

    ofstream flog;
    int logid;

    // thread for controlling reads 
    uint8_t _mode; ///< internal player delivery mode
    boost::thread* _pThread;
    mutable pthread_mutex_t _mutexReadVals; ///< mutable since it will be locked in const members
    list<TRAJECTORY> _listQueuedTrajectories; ///< ids of the trajectories to wait for, if 0, don't sync the ids
    TRAJECTORY _trajCurrent; ///< current trajectory
    
    StallAction _saNextStallAction;
    string _strStallCommand;

    bool _bStopThread; ///< notification from main thread to reader thread
    bool _bDestroyThread; ///< notification from reader thread to main thread

    bool _bPause, _bIsDone;
    bool _bSendTimestamps; ///< if true, will send timestamps along with traj
    bool _bHoldOnStall;
};

#endif

