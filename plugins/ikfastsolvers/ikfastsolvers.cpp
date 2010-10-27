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
#include <rave/plugin.h>

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
namespace pr2_head {
#include "ik_pr2_head.h"
}
namespace pr2_head_torso {
#include "ik_pr2_head_torso.h"
}
namespace pr2_leftarm {
#include "ik_pr2_leftarm.h"
}
namespace pr2_rightarm {
#include "ik_pr2_rightarm.h"
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
BOOST_TYPEOF_REGISTER_TYPE(katana::IKSolution)
#endif

#include "ikfastproblem.h"

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    dReal freeinc = 0.04f;
    switch(type) {
    case PT_InverseKinematicsSolver: {
        if( interfacename == "ikfast" ) {
            string ikfastname;
            sinput >> ikfastname;
            if( !!sinput ) {
                sinput >> freeinc;
                // look at all the ikfast problem solvers
                IkSolverBasePtr psolver = IKFastProblem::CreateIkSolver(ikfastname, freeinc, penv);
                if( !!psolver ) {
                    return psolver;
                }
            }
        }
        else {
            sinput >> freeinc;
            if( interfacename == "wam7ikfast" ) {
                vector<int> vfree(barrettwam::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = barrettwam::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<barrettwam::IKReal,barrettwam::IKSolution>(barrettwam::ik,vfree,freeinc,barrettwam::getNumJoints(),(IkParameterization::Type)barrettwam::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "pa10ikfast" ) {
                vector<int> vfree(pa10::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pa10::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<pa10::IKReal,pa10::IKSolution>(pa10::ik,vfree,freeinc,pa10::getNumJoints(),(IkParameterization::Type)pa10::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "pumaikfast" ) {
                vector<int> vfree(puma::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = puma::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<puma::IKReal,puma::IKSolution>(puma::ik,vfree,freeinc,puma::getNumJoints(),(IkParameterization::Type)puma::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "manusleftarmikfast" ) {
                vector<int> vfree(manusleft::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = manusleft::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<manusleft::IKReal,manusleft::IKSolution>(manusleft::ik,vfree,freeinc,manusleft::getNumJoints(),(IkParameterization::Type)manusleft::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "katanaikfast" ) {
                vector<int> vfree(katana::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = katana::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<katana::IKReal,katana::IKSolution>(katana::ik,vfree,freeinc,katana::getNumJoints(),(IkParameterization::Type)katana::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "ikfast_pr2_head" ) {
                vector<int> vfree(pr2_head::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pr2_head::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<pr2_head::IKReal,pr2_head::IKSolution>(pr2_head::ik,vfree,freeinc,pr2_head::getNumJoints(),(IkParameterization::Type)pr2_head::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "ikfast_pr2_head_torso" ) {
                vector<int> vfree(pr2_head_torso::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pr2_head_torso::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<pr2_head_torso::IKReal,pr2_head_torso::IKSolution>(pr2_head_torso::ik,vfree,freeinc,pr2_head_torso::getNumJoints(),(IkParameterization::Type)pr2_head_torso::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "ikfast_pr2_rightarm" ) {
                vector<int> vfree(pr2_rightarm::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pr2_rightarm::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<pr2_rightarm::IKReal,pr2_rightarm::IKSolution>(pr2_rightarm::ik,vfree,freeinc,pr2_rightarm::getNumJoints(),(IkParameterization::Type)pr2_rightarm::getIKType(), boost::shared_ptr<void>(), penv));
            }
            else if( interfacename == "ikfast_pr2_leftarm" ) {
                vector<int> vfree(pr2_leftarm::getNumFreeParameters());
                for(size_t i = 0; i < vfree.size(); ++i)
                    vfree[i] = pr2_leftarm::getFreeParameters()[i];
                return InterfaceBasePtr(new IkFastSolver<pr2_leftarm::IKReal,pr2_leftarm::IKSolution>(pr2_leftarm::ik,vfree,freeinc,pr2_leftarm::getNumJoints(),(IkParameterization::Type)pr2_leftarm::getIKType(), boost::shared_ptr<void>(), penv));
            }
        }
        break;
    }        
    case PT_ProblemInstance:
        if( interfacename == "ikfast")
            return InterfaceBasePtr(new IKFastProblem(penv));
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("ikfast");
    info.interfacenames[PT_InverseKinematicsSolver].push_back("ikfast");
    info.interfacenames[PT_InverseKinematicsSolver].push_back("WAM7ikfast");
    info.interfacenames[PT_InverseKinematicsSolver].push_back("PA10ikfast");
    info.interfacenames[PT_InverseKinematicsSolver].push_back("PUMAikfast");
    info.interfacenames[PT_InverseKinematicsSolver].push_back("ManusLeftArmikfast");
    info.interfacenames[PT_InverseKinematicsSolver].push_back("Katanaikfast");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    delete IKFastProblem::GetLibraries();
    IKFastProblem::GetLibraries() = NULL;
}
