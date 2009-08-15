// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_CONTROLLERS_H
#define RAVE_CONTROLLERS_H

// controller for the SSC-32 board
class IdealController : public ControllerBase
{
public:
    IdealController(EnvironmentBase* penv);
    virtual ~IdealController();

    virtual bool Init(RobotBase* robot, const char* args = NULL);

    virtual bool SetDesired(const dReal* pValues);
    virtual bool SetPath(const Trajectory* ptraj);
    virtual bool SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime);
    
    virtual bool SimulationStep(dReal fTimeElapsed);
    virtual int GetDOF() { return _probot != NULL ? _probot->GetDOF() : 0; }

    virtual bool IsDone() { return _ptraj == NULL || _bIsDone; }
    virtual void Reset(int options);

    virtual bool SendCmd(const char* pcmd);
    virtual bool SupportsCmd(const char* pcmd);
    
    virtual float GetTime() const { return fTime; }
    virtual RobotBase* GetRobot() const { return _probot; }

private:
    RobotBase* _probot;           ///< controlled body
    float _fSpeed;                ///< how fast the robot should go
    const Trajectory* _ptraj;     ///< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()

    float fTime;

    std::vector<dReal> _vecdesired;     ///< desired values of the joints    

    ofstream flog;
    int cmdid;
    bool _bPause;
    bool _bIsDone;
};

#endif
