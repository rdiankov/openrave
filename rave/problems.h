// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#ifndef OPENRAVE_COMMAND_PROBLEM_INSTANCE_H
#define OPENRAVE_COMMAND_PROBLEM_INSTANCE_H

namespace OpenRAVE {

/// Base class for problem instances the user might want to instantiate. A problem
/// instance registers itself with OpenRAVE's SimulateStep calls and can accept
/// commands from the server or other plugins via SendCommand. A problem instance
/// stops receiving commands when it is destroyed.
class RAVE_API ProblemInstance : public InterfaceBase
{
protected:
    /// The function to be executed for every command.
    /// \param sinput - input of the command
    /// \param sout - output of the command
    /// \return If false, there was an error with the command, true if successful
    typedef boost::function<bool(std::ostream&, std::istream&)> CommandFn;
    struct COMMAND
    {
        COMMAND() {}
        COMMAND(CommandFn newfn, const std::string& newhelp) : fn(newfn), help(newhelp) {}
        COMMAND(const COMMAND& r) { fn = r.fn; help = r.help; }
        
        CommandFn fn; ///< command function to run
        std::string help; ///< help string explaining command arguments
    };

    typedef std::map<std::string, COMMAND, CaseInsensitiveCompare> CMDMAP;
    CMDMAP __mapCommands; ///< all registered commands

public:
    ProblemInstance(EnvironmentBasePtr penv);
    virtual ~ProblemInstance() {}

    /// gets called every time a problem instance is loaded to initialize the problem.
    /// Robots might not necessarily be set before this function call
    /// returns 0 on success
    virtual int main(const std::string& cmd) { return 0; }

    /// called when problem gets unloaded from environment
    virtual void Destroy() {}

    /// called when environment is reset
    virtual void Reset() {}

    virtual bool SimulationStep(dReal fElapsedTime) {return false;}

    /// Function for sending commands to the problem while it is executing. It can be used to control
    /// the problem remotely through the server
    /// Might be called in a different thread from the main thread, so make sure to lock the openrave environment when using environment calls.
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);
    
    /// Write the help commands to an output stream
    virtual bool GetCommandHelp(std::ostream& sout, std::istream& sinput) const;

protected:
    /// Registers a command and its help string
    /// \param cmdname - command name
    /// \param fncmd function to execute for the command
    /// \param strhelp - help string
    /// \return true if pcmdname was successfully added, false if there exists a registered command already
    virtual void RegisterCommand(const std::string& cmdname, CommandFn fncmd, const std::string& strhelp);
    
    virtual void DeleteCommand(const std::string& cmdname);

    /// get all the registered commands
    virtual const CMDMAP& GetCommands() const;

private:
    virtual const char* GetHash() const { return OPENRAVE_PROBLEM_HASH; }
};

} // end namespace OpenRAVE

#endif
