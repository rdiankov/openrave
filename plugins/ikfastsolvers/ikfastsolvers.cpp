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
#include "plugindefs.h"
#include "ikbase.h"

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

#include "ikfastproblem.h"

namespace barrettwam {
#include "ik_barrettwam.h"
}
namespace pa10 {
#include "ik_pa10.h"
}
namespace puma {
#include "ik_puma.h"
}
namespace manusleft {
#include "ik_manusleftarm.h"
}
namespace man1left {
#include "ik_man1left.h"
}
namespace man1right {
#include "ik_man1right.h"
}
namespace hrp3right {
#include "ik_hrp3right.h"
}
namespace katana {
#include "ik_katana.h"
}

// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(barrettwam::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(pa10::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(puma::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(manusleft::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(man1left::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(man1right::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(hrp3right::IKSolution)
BOOST_TYPEOF_REGISTER_TYPE(katana::IKSolution)
#endif

#include "ikfastproblem.h"

// need c linkage
extern "C" {

// for some reason windows complains when the prototypes are different
InterfaceBase* DECL_STDCALL(ORCreate, (PluginType type, wchar_t* name, EnvironmentBase* penv))
{
    if( name == NULL ) return NULL;
    
    switch(type) {
    case PT_InverseKinematicsSolver: {
        dReal freeinc = 0.04f;
        wstring name_ik;
        wstringstream ss(name);
        ss >> name_ik >> freeinc;

        if( wcsicmp(name_ik.c_str(),L"WAM7ikfast") == 0 ) {
            vector<int> vfree(barrettwam::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = barrettwam::getFreeParameters()[i];
            return new IkFastSolver<barrettwam::IKReal,barrettwam::IKSolution>(barrettwam::ik,vfree,freeinc,barrettwam::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"PA10ikfast") == 0 ) {
            vector<int> vfree(pa10::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = pa10::getFreeParameters()[i];
            return new IkFastSolver<pa10::IKReal,pa10::IKSolution>(pa10::ik,vfree,freeinc,pa10::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"PUMAikfast") == 0 ) {
            vector<int> vfree(puma::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = puma::getFreeParameters()[i];
            return new IkFastSolver<puma::IKReal,puma::IKSolution>(puma::ik,vfree,freeinc,puma::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"ManusLeftArmikfast") == 0 ) {
            vector<int> vfree(manusleft::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = manusleft::getFreeParameters()[i];
            return new IkFastSolver<manusleft::IKReal,manusleft::IKSolution>(manusleft::ik,vfree,freeinc,manusleft::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"Man1Leftikfast") == 0 ) {
            vector<int> vfree(man1left::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = man1left::getFreeParameters()[i];
            return new IkFastSolver<man1left::IKReal,man1left::IKSolution>(man1left::ik,vfree,freeinc,man1left::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"Man1Rightikfast") == 0 ) {
            vector<int> vfree(man1right::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = man1right::getFreeParameters()[i];
            return new IkFastSolver<man1right::IKReal,man1right::IKSolution>(man1right::ik,vfree,freeinc,man1right::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"HRP3Rightikfast") == 0 ) {
            vector<int> vfree(hrp3right::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = hrp3right::getFreeParameters()[i];
            return new IkFastSolver<hrp3right::IKReal,hrp3right::IKSolution>(hrp3right::ik,vfree,freeinc,hrp3right::getNumJoints(),penv);
        }
        else if( wcsicmp(name_ik.c_str(),L"Katanaikfast") == 0 ) {
            vector<int> vfree(katana::getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = katana::getFreeParameters()[i];
            return new IkFastSolver<katana::IKReal,katana::IKSolution>(katana::ik,vfree,freeinc,katana::getNumJoints(),penv);
        }

        // look at all the ikfast problem solvers
        FOREACHC(itprob, IKFastProblem::GetProblems()) {
            IkSolverBase* psolver = (*itprob)->CreateIkSolver(name_ik.c_str(), freeinc, penv);
            if( psolver != NULL )
                return psolver;
        }

        break;
    }
    case PT_ProblemInstance:
        if( wcsicmp(name, L"IKFast") == 0 )
            return new IKFastProblem(penv);
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

    pinfo->iksolvers.push_back(L"WAM7ikfast");
    pinfo->iksolvers.push_back(L"PA10ikfast");
    pinfo->iksolvers.push_back(L"PUMAikfast");
    pinfo->iksolvers.push_back(L"ManusLeftArmikfast");
    pinfo->iksolvers.push_back(L"Man1Leftikfast");
    pinfo->iksolvers.push_back(L"Man1Rightikfast");
    pinfo->iksolvers.push_back(L"HRP3Rightikfast");
    pinfo->iksolvers.push_back(L"Katanaikfast");
    pinfo->problems.push_back(L"IKFast");

    return true;
}

void DECL_STDCALL(DestroyPlugin, ())
{
}

}
