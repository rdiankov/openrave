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
#ifndef OPENRAVE_LOGGING_H
#define OPENRAVE_LOGGING_H

/// used to log scene elements
class LoggingProblem : public CmdProblemInstance
{
public:
    LoggingProblem(EnvironmentBase* penv);
    virtual ~LoggingProblem();
    virtual void Destroy();
    virtual int main(const char* cmd);
    
    /// returns true when problem is solved
    virtual bool SimulationStep(dReal fElapsedTime);
    
private:
    virtual bool SaveScene(ostream& sout, istream& sinput);
    virtual bool StartReplay(ostream& sout, istream& sinput);
    virtual bool StartRecording(ostream& sout, istream& sinput);
    virtual bool StopRecording(ostream& sout, istream& sinput);
    virtual bool Help(ostream& sout, istream& sinput);

    static void* log_thread(void* param);
    void* _log_thread();

    pthread_t _threadlog;
    bool bDoLog, bDestroyThread, bAbsoluteTiem;
};

#endif
