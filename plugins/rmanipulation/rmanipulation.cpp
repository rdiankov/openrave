// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu), Carnegie Mellon University
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
#include "plugindefs.h"

#include "basemanipulation.h"
#include "taskmanipulation.h"
#include "taskcaging.h"
#include "visualfeedback.h"

#include <rave/plugin.h>

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_ProblemInstance:
        if( interfacename == "basemanipulation")
            return InterfaceBasePtr(new BaseManipulation(penv));
        else if( interfacename == "taskmanipulation" )
            return InterfaceBasePtr(new TaskManipulation(penv));
        else if( interfacename == "taskcaging")
            return InterfaceBasePtr(new TaskCagingProblem(penv));
        else if( interfacename == "visualfeedback")
            return InterfaceBasePtr(new VisualFeedbackProblem(penv));
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("BaseManipulation");
    info.interfacenames[PT_ProblemInstance].push_back("TaskManipulation");
    info.interfacenames[PT_ProblemInstance].push_back("TaskCaging");
    info.interfacenames[PT_ProblemInstance].push_back("VisualFeedback");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
