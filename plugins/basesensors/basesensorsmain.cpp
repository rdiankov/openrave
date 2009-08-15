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
#include "plugindefs.h"
#include "baselaser.h"
#include "baseflashlidar3d.h"
#include "basecamera.h"

// declaring variables with stdcall can be a little complex
#ifdef _MSC_VER

#define PROT_STDCALL(name, paramlist) __stdcall name paramlist
#define DECL_STDCALL(name, paramlist) __stdcall name paramlist

#else

#ifdef __x86_64__
#define DECL_STDCALL(name, paramlist) name paramlist
#else
#define DECL_STDCALL(name, paramlist) __attribute__((stdcall)) name paramlist
#endif

#endif // _MSC_VER

// need c linkage
extern "C" {

// for some reason windows complains when the prototypes are different
InterfaceBase* DECL_STDCALL(ORCreate, (PluginType type, wchar_t* name, EnvironmentBase* penv))
{
    if( name == NULL ) return NULL;
    
    switch(type) {
        case PT_Sensor:
            if( wcsicmp(name, L"BaseLaser2D") == 0 ) 
                return new BaseLaser2DSensor(penv);
            else if( wcsicmp(name, L"BaseSpinningLaser2D") == 0 ) 
                return new BaseSpinningLaser2DSensor(penv);
            else if( wcsicmp(name, L"BaseFlashLidar3D") == 0 )
                return new BaseFlashLidar3DSensor(penv);
            else if( wcsicmp(name, L"BaseCamera") == 0 )
                return new BaseCameraSensor(penv);
            break;
        default:
            break;
    }

    return NULL;
}

bool DECL_STDCALL(GetPluginAttributes, (PLUGININFO* pinfo, int size))
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        RAVELOG_ERRORA("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->sensors.push_back(L"BaseLaser2D");
    pinfo->sensors.push_back(L"BaseSpinningLaser2D");
    pinfo->sensors.push_back(L"BaseFlashLidar3D");
    pinfo->sensors.push_back(L"BaseCamera");
    return true;
}

void DECL_STDCALL(DestroyPlugin, ())
{
}

}
