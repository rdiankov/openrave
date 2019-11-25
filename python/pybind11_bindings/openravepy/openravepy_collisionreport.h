// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVEPY_INTERNAL_COLLISIONREPORT_H
#define OPENRAVEPY_INTERNAL_COLLISIONREPORT_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyCollisionReport
{
public:
    PyCollisionReport();
    PyCollisionReport(CollisionReportPtr report);
    virtual ~PyCollisionReport();

    struct PYCONTACT
    {
        PYCONTACT();
        PYCONTACT(const CollisionReport::CONTACT& c);

        std::string __str__();
        object __unicode__();
        object pos = py::none_();
        object norm = py::none_();
        dReal depth;
    };

    void init(PyEnvironmentBasePtr pyenv);

    std::string __str__();
    object __unicode__();

    int options;
    object plink1 = py::none_();
    object plink2 = py::none_();
    py::list vLinkColliding;
    dReal minDistance;
    int numWithinTol;
    py::list contacts;
    uint32_t nKeepPrevious;
    CollisionReportPtr report;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_COLLISIONREPORT_H
