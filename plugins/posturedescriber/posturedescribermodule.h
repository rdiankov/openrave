// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui, Rosen Diankov <rosen.diankov@gmail.com>
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


#ifndef PLUGINS_FKCOMPUTERS_POSTUREDESCRIBERMODULE_H
#define PLUGINS_FKCOMPUTERS_POSTUREDESCRIBERMODULE_H

#include <string> // string
#include "plugindefs.h" // POSTUREDESCRIBER_CLASS_NAME

namespace OpenRAVE {

class OPENRAVE_API PostureDescriberModule : public ModuleBase
{
public:
    PostureDescriberModule() = delete; // disable default constructor
    PostureDescriberModule(const EnvironmentBasePtr& penv);
    virtual ~PostureDescriberModule() = default;

    std::string interfacename = POSTUREDESCRIBER_CLASS_NAME;

private:
    /// \brief Python `SendCommand` API that loads a robot posture describer onto a (base link, end-effector link) pair, or onto a manipulator that prescribes the pair
    bool _LoadPostureDescriberCommand(std::ostream& ssout, std::istream& ssin);
};

} // namepspace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_POSTUREDESCRIBERMODULE_H