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
#ifndef OPENRAVE_MANIPULATION_H
#define OPENRAVE_MANIPULATION_H

class BaseManipulationProblem : public CmdProblemInstance
{
public:
    BaseManipulationProblem(EnvironmentBase* penv);
    virtual ~BaseManipulationProblem();
    virtual void Destroy();
    virtual int main(const char* cmd);
    virtual void SetActiveRobots(const std::vector<RobotBase*>& robots);
    
    /// returns true when problem is solved
    virtual bool SimulationStep(dReal fElapsedTime);
    virtual bool SendCommand(const char* cmd, string& response);
    virtual void Query(const char* query, string& response);
    
private:

    bool SetActiveManip(ostream& sout, istream& sinput);
    bool Traj(ostream& sout, istream& sinput);
    bool GrabBody(ostream& sout, istream& sinput);
    bool ReleaseAll(ostream& sout, istream& sinput);
    bool MoveHandStraight(ostream& sout, istream& sinput);
    bool MoveManipulator(ostream& sout, istream& sinput);
    bool MoveActiveJoints(ostream& sout, istream& sinput);
    bool MoveToHandPosition(ostream& sout, istream& sinput);
    bool MoveUnsyncJoints(ostream& sout, istream& sinput);
    bool CloseFingers(ostream& sout, istream& sinput);
    bool ReleaseFingers(ostream& sout, istream& sinput);
    bool IKtest(ostream& sout, istream& sinput);
    bool DebugIK(ostream& sout, istream& sinput);
    bool SmoothTrajectory(ostream& sout, istream& sinput);
    bool Help(ostream& sout, istream& sinput);

    bool SetTrajectory(Trajectory* pActiveTraj, bool bExecute, const string& strsavetraj, ostream* psout);
    
    bool _MoveUnsyncJoints(Trajectory* ptraj, const vector<int>& vhandjoints, const vector<dReal>& vhandgoal, const wchar_t* pplannername = NULL);

    RobotBase* robot;
    wstring _strRRTPlannerName;

    wstring _strRobotName; ///< name of the active robot
    ProblemInstance* pGrasperProblem;
};

#endif
