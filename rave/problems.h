// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
class ProblemInstance : public InterfaceBase
{
public:
    ProblemInstance(EnvironmentBase* penv) : InterfaceBase(PT_ProblemInstance, penv) {}
    virtual ~ProblemInstance() {}

    virtual void Destroy() = 0;

    /// gets called every time a problem instance is loaded to initialize the problem.
    /// Robots might not necessarily be set before this function call
    /// returns 0 on success
    virtual int main(const char* cmd) = 0;

    virtual bool SimulationStep(dReal fElapsedTime) {return false;}

    /// Function for sending commands to the problem while it is executing. It can be used to control
    /// the problem remotely through the server with orProblemSendCommand. The function will always
    /// be called in the main thread. Therefore planners and other processor heavy tasks can be called inside it
    /// \param cmd - the command to send
    /// \param response - the output
    virtual bool SendCommand(const char* cmd, std::string& response) = 0;

    /// Same as SendCommand except the function is meant to query simple data from the problem. Query
    /// might be called in a different thread from the main thread, so make sure to put the appropriate
    /// synchronization primitives in place. Because Query can execute in a different thread, it 
    /// can return information much faster than SendCommand.
    /// \param cmd - the command to send
    /// \param response - the output
    virtual void Query(const char* cmd, std::string& response) {}

private:
    virtual const char* GetHash() const { return OPENRAVE_PROBLEM_HASH; }
};

/// A wrapper class for registering commands for problem instances.
/// Derive from this class and call the RegisterCommand method to register a 
/// command name to a pointer to a member function.
class CmdProblemInstance : public ProblemInstance
{
public:
    
    /// The function to be executed for every command.
    /// \param sinput - input of the command
    /// \param sout - output of the command
    /// \return If false, there was an error with the command, true if successful
    typedef bool (ProblemInstance::*CommandFn)(std::ostream& sout, std::istream& sinput);
    struct COMMAND
    {
        COMMAND() : fn(NULL) {}
        COMMAND(CommandFn newfn, const std::string& newhelp) : fn(newfn), help(newhelp) {}
        COMMAND(const COMMAND& r) { fn = r.fn; help = r.help; }
        
        CommandFn fn; ///< command function to run
        std::string help; ///< help string explaining command arguments
    };

 private:
    class CaseInsentiveCompare
    {
    public:
        bool operator()(const std::string & s1, const std::string& s2) const
        {
            std::string::const_iterator it1=s1.begin();
            std::string::const_iterator it2=s2.begin();
        
            //has the end of at least one of the strings been reached?
            while ( (it1!=s1.end()) && (it2!=s2.end()) ) 
            { 
                if(::toupper(*it1) != ::toupper(*it2)) //letters differ?
                    // return -1 to indicate 'smaller than', 1 otherwise
                    return ::toupper(*it1) < ::toupper(*it2);
                //proceed to the next character in each string
                ++it1;
                ++it2;
            }
            std::size_t size1=s1.size(), size2=s2.size();// cache lengths
            //return -1,0 or 1 according to strings' lengths
            if (size1==size2) 
                return 0;
            return size1<size2;
        }
    };
    
    typedef std::map<std::string, COMMAND, CaseInsentiveCompare> CMDMAP;
        
    CMDMAP _mapCommands; ///< all registered commands
    
public:

    CmdProblemInstance(EnvironmentBase* penv) : ProblemInstance(penv) {}    
    virtual ~CmdProblemInstance();
    virtual void Destroy();

    /// parses one command
    virtual bool SendCommand(const char* pcmd, std::string& response);

    /// Registers a command and its help string
    /// \param cmdname - command name
    /// \param fncmd function to execute for the command
    /// \param strhelp - help string
    /// \return true if pcmdname was successfully added, false if there exists a registered command already
    virtual bool RegisterCommand(const std::string& cmdname, CommandFn fncmd, const std::string& strhelp);

    /// Registers a command and its help string
    /// \param pcmdname command name
    /// \param fncmd function to execute for the command
    /// \param strhelp - help string
    /// \return true if pcmdname was successfully added, false if there exists a registered command already
    virtual bool RegisterCommand(const char* pcmdname, CommandFn fncmd, const char* pstrhelp);

    /// Deletes a command from registration
    /// \param cmdname the command name
    /// \return true if pcmdname was successfully deleted
    virtual bool DeleteCommand(const std::string& cmdname);

    /// Gets the function to be called of a partiicular command
    /// \param cmdname the command name
    /// \return the command function to be called 
    virtual CommandFn GetCommand(const std::string& cmdname);

    /// get all the registered commands
    virtual const CMDMAP& GetCommands() const;

    /// Write the help commands to an output stream
    virtual void GetCommandHelp(std::ostream& o) const;
};

} // end namespace OpenRAVE

#endif
