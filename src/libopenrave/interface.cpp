// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
#include "libopenrave.h"

namespace OpenRAVE {
InterfaceBase::InterfaceBase(InterfaceType type, EnvironmentBasePtr penv) : __type(type), __penv(penv)
{
    RaveInitializeFromState(penv->GlobalState()); // make sure global state is set
    RegisterCommand("help",boost::bind(&InterfaceBase::_GetCommandHelp,this,_1,_2),
                    "display help commands.");
    __description = "Not documented yet.";
}

InterfaceBase::~InterfaceBase()
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    __mapCommands.clear();
    __pUserData.reset();
    __mapReadableInterfaces.clear();
    __penv.reset();
}

void InterfaceBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    if( !preference ) {
        throw openrave_exception("invalid cloning reference",ORE_InvalidArguments);
    }
    __pUserData = preference->__pUserData;
    __struri = preference->__struri;
    __mapReadableInterfaces = preference->__mapReadableInterfaces;
}

bool InterfaceBase::SendCommand(ostream& sout, istream& sinput)
{
    string cmd;
    sinput >> cmd;
    if( !sinput ) {
        throw openrave_exception("invalid command",ORE_InvalidArguments);
    }
    boost::shared_ptr<InterfaceCommand> interfacecmd;
    {
        boost::mutex::scoped_lock lock(_mutexInterface);
        CMDMAP::iterator it = __mapCommands.find(cmd);
        if( it == __mapCommands.end() ) {
            throw openrave_exception(str(boost::format("failed to find command '%s' in interface %s\n")%cmd.c_str()%GetXMLId()),ORE_CommandNotSupported);
        }
        interfacecmd = it->second;
    }
    if( !interfacecmd->fn(sout,sinput) ) {
        RAVELOG_VERBOSE(str(boost::format("command failed in interface %s: %s\n")%GetXMLId()%cmd));
        return false;
    }
    return true;
}

void InterfaceBase::RegisterCommand(const std::string& cmdname, InterfaceBase::InterfaceCommandFn fncmd, const std::string& strhelp)
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    if((cmdname.size() == 0)|| !IsValidName(cmdname) ||(stricmp(cmdname.c_str(),"commands") == 0)) {
        throw openrave_exception(str(boost::format("command '%s' invalid")%cmdname),ORE_InvalidArguments);
    }
    if( __mapCommands.find(cmdname) != __mapCommands.end() ) {
        throw openrave_exception(str(boost::format("command '%s' already registered")%cmdname),ORE_InvalidArguments);
    }
    __mapCommands[cmdname] = boost::shared_ptr<InterfaceCommand>(new InterfaceCommand(fncmd, strhelp));
}

void InterfaceBase::UnregisterCommand(const std::string& cmdname)
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    CMDMAP::iterator it = __mapCommands.find(cmdname);
    if( it != __mapCommands.end() ) {
        __mapCommands.erase(it);
    }
}

bool InterfaceBase::_GetCommandHelp(std::ostream& o, std::istream& sinput) const
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    string cmd, label;
    CMDMAP::const_iterator it;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput ) {
            break;
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "commands" ) {
            for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
                o << it->first << " ";
            }
            return true;
        }
        else if( cmd == "label" ) {
            sinput >> label;
        }
        else {
            it = __mapCommands.find(cmd);
            if( it != __mapCommands.end() ) {
                o << it->second->help;
                return true;
            }
        }

        if( !sinput ) {
            RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
            return false;
        }
    }

    // display full help string
    o << endl << GetXMLId() << " Commands" << endl;
    for(size_t i = 0; i < GetXMLId().size(); ++i) {
        o << "=";
    }
    o << "=========" << endl << endl;
    for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
        if( label.size() > 0 ) {
            string strlower = it->first;
            std::transform(strlower.begin(), strlower.end(), strlower.begin(), ::tolower);
            o << endl << ".. _" << label << strlower << ":" << endl << endl;
        }
        o << endl << it->first << endl;
        for(size_t i = 0; i < it->first.size(); ++i) {
            o << "~";
        }
        o << endl << endl << it->second->help << endl << endl << "~~~~" << endl << endl;
    }
    return true;
}

}
