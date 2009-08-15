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
#ifndef RAVE_SIMPLE_PLAYER_CONTROLLER
#define RAVE_SIMPLE_PLAYER_CONTROLLER

// controller for the SSC-32 board
class SimplePlayerController : public ControllerBase
{
    enum ControllerState{
        None = 0,
        Servo, // done when servoed to position and the position is held
        Traj, // done when reaches last point
        Velocity // done when joints stop moving
    };

    struct PLAYERPROXY
    {
        PLAYERPROXY() : pproxy(NULL), bIgnore(false), offset(0) {}
        ActArrayProxy* pproxy;
        int offset;
        vector<player_actarray_actuatorgeom> vgeom; // joint limits
        bool bIgnore; // if true commands shouldn't be sent 
        bool bStopNextTrajOnStall;
    };


    Position3dProxy* ppos3dproxy; //this is used for setting masses of objects grabbed by the hand
    PlannerProxy* pplannerproxy; //this is an interface to the Segway planner

public:
    SimplePlayerController(EnvironmentBase* penv);
    virtual ~SimplePlayerController();

    /// args format: host port [proxytype index]
    /// where proxytype is actarray, pos2d, or ...
    /// the order specified is the order the degrees of freedom will be arranged
    virtual bool Init(RobotBase* robot, const char* args);
    virtual void Destroy();

    virtual bool SetDesired(const dReal* pValues);
    virtual bool SetPath(const Trajectory* ptraj);
    void SetVelocities(const std::vector<dReal>& vels);
    virtual int GetDOF() { return _probot != NULL ? _probot->GetDOF() : 0; }

    virtual bool SimulationStep(float fTimeElapsed);

    virtual bool SendCmd(const char* pcmd);
    virtual bool SupportsCmd(const char* pcmd);

    virtual bool IsDone() { return _bIsDone; }

    virtual ActuatorState GetActuatorState(int index);

    virtual float GetTime() const { return fTime; }
    virtual RobotBase* GetRobot() const { return _probot; }

private:
    
    static int _pos3dIndex;

    bool MoveTo(const dReal* pdesired);
    bool MoveAtSpeed(const dReal* pdesired);

    void StartThread();
    void RunThread();
    void StopThread();

    string host;
    int _port;

    RobotBase* _probot;           ///< controlled body
    u32 nFeedbackTime, nLastRecvTime, nCheckTime;

    std::vector<dReal> _vPrevValues, _vReadValues; //desired velocities, only used in "Velocity" state
    ControllerState _state; //stores the controller's current state
    ControllerState _segwaystate; //stores the controller's current state        
    float segwayplannergoal[3];
    float segwaycommand[3];
    float prevsegwaycommand[3];
    static float smalloffset;
    bool bplusminusoffset;


    ActuatorState _segwayDoneState;
    ActuatorState _prevsegwayDoneState;


    std::vector<ActuatorState> _vActuatorStates; //stores the current state of each actuator
    std::vector<ActuatorState> _vPrevActuatorStates; //stores the current state of each actuator

    // player specific
    PlayerClient* _pclient; //arm client
    PlayerClient* _psegwayclient; //segway client
    vector<PLAYERPROXY> _vecproxies; // each robot can consist of multiple proxies

    ofstream flog;
    int logid;
    vector<dReal> _vecdesired;
    float fTime;
    const Trajectory* _ptraj;

    // thread for controlling reads 
    uint8_t _mode; ///< internal player delivery mode
    boost::thread* _pThread;
    pthread_mutex_t _mutexReadVals;
    bool _bStopThread; // notification from main thread to reader thread
    bool _bDestroyThread; // notification from reader thread to main thread
    bool _bMonitorActuatorStates; // if true, monitors the actuator states for figuring out if 
                                  // the controller finished

    bool _bPause, _bIsDone;
    bool _bSendTimestamps; // if true, will send timestamps along with traj
    bool _bMonitorTraj;
    bool _bStopNextTrajOnStall;
};

#endif

