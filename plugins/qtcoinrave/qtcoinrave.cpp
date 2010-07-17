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
#include "qtcoin.h"
#include <rave/plugin.h>

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    static int s_SoQtArgc = 0; // has to be static!!
    switch(type) {
    case PT_Viewer:
        if( interfacename == "qtcoin" ) {
            if( QtCoinViewer::s_InitRefCount == 0 ) {
                SoQt::init(s_SoQtArgc, NULL, NULL);
                ++QtCoinViewer::s_InitRefCount;
            }
            return InterfaceBasePtr(new QtCoinViewer(penv));
        }
        break;
    default:
        break;
    }    
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Viewer].push_back("qtcoin");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
