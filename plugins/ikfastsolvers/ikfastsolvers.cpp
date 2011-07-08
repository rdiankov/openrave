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
#include <openrave/plugin.h>

#include "ikfastproblem.h"

namespace ik_barrettwam { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pa10 { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_puma { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pr2_head { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pr2_head_torso { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pr2_leftarm { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pr2_leftarm_torso { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pr2_rightarm { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_pr2_rightarm_torso { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_schunk_lwa3 { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_katana5d { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }
namespace ik_katana5d_trans { IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc); }

#include "ikfastproblem.h"

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_IkSolver: {
        if( interfacename == "ikfast" ) {
            string ikfastname;
            sinput >> ikfastname;
            if( !!sinput ) {
                vector<dReal> vfreeinc((istream_iterator<dReal>(sinput)), istream_iterator<dReal>());
                // look at all the ikfast problem solvers
                IkSolverBasePtr psolver = IKFastProblem::CreateIkSolver(ikfastname, vfreeinc, penv);
                if( !!psolver ) {
                    return psolver;
                }
            }
        }
        else {
            vector<dReal> vfreeinc((istream_iterator<dReal>(sinput)), istream_iterator<dReal>());
            if( interfacename == "wam7ikfast" ) {
                return ik_barrettwam::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "pa10ikfast" ) {
                return ik_pa10::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "pumaikfast" ) {
                return ik_puma::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_pr2_head" ) {
                return ik_pr2_head::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_pr2_head_torso" ) {
                return ik_pr2_head_torso::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_pr2_rightarm" ) {
                return ik_pr2_rightarm::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_pr2_rightarm_torso" ) {
                return ik_pr2_rightarm_torso::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_pr2_leftarm" ) {
                return ik_pr2_leftarm::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_pr2_leftarm_torso" ) {
                return ik_pr2_leftarm_torso::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_schunk_lwa3" ) {
                return ik_schunk_lwa3::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_katana5d" ) {
                return ik_katana5d::CreateIkSolver(penv, vfreeinc);
            }
            else if( interfacename == "ikfast_katana5d_trans" ) {
                return ik_katana5d_trans::CreateIkSolver(penv, vfreeinc);
            }
        }
        break;
    }
    case PT_Module:
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
    info.interfacenames[PT_Module].push_back("ikfast");
    info.interfacenames[PT_IkSolver].push_back("ikfast");
    info.interfacenames[PT_IkSolver].push_back("wam7ikfast");
    info.interfacenames[PT_IkSolver].push_back("pa10ikfast");
    info.interfacenames[PT_IkSolver].push_back("pumaikfast");
    info.interfacenames[PT_IkSolver].push_back("ikfast_pr2_head");
    info.interfacenames[PT_IkSolver].push_back("ikfast_pr2_head_torso");
    info.interfacenames[PT_IkSolver].push_back("ikfast_pr2_rightarm");
    info.interfacenames[PT_IkSolver].push_back("ikfast_pr2_rightarm_torso");
    info.interfacenames[PT_IkSolver].push_back("ikfast_pr2_leftarm");
    info.interfacenames[PT_IkSolver].push_back("ikfast_pr2_leftarm_torso");
    info.interfacenames[PT_IkSolver].push_back("ikfast_schunk_lwa3");
    info.interfacenames[PT_IkSolver].push_back("ikfast_katana5d");
    info.interfacenames[PT_IkSolver].push_back("ikfast_katana5d_trans");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    delete IKFastProblem::GetLibraries();
    IKFastProblem::GetLibraries() = NULL;
}
