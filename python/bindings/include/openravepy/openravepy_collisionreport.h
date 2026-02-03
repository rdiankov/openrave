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

struct PYCONTACT
{
    PYCONTACT();
    PYCONTACT(const CONTACT& c);

    std::string __str__() const;
    object __unicode__() const;
    object pos = py::none_();
    object norm = py::none_();
    dReal depth;
};

class OPENRAVEPY_API PyCollisionPairInfo
{
public:
    PyCollisionPairInfo(const CollisionPairInfo& cpinfo);
    std::string __str__() const;
    object __unicode__() const;

    py::object ExtractFirstBodyLinkGeomNames();
    py::object ExtractSecondBodyLinkGeomNames();

    std::string bodyLinkGeom1Name;
    std::string bodyLinkGeom2Name;
    py::list contacts;
};

class OPENRAVEPY_API PyCollisionReport
{
public:
    PyCollisionReport();
    PyCollisionReport(const CollisionReport& report);

    void Init(const CollisionReport& report);

    std::string __str__() const;
    object __unicode__() const;
    void Reset(int coloptions=0);

    py::list collisionInfos; // list of PyCollisionPairInfo
    int options = 0;
    OpenRAVE::dReal minDistance = 1e20;
    int numWithinTol = 0;
    uint32_t nKeepPrevious = 0;
};

struct CollisionReportInitializer
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    CollisionReportInitializer(py::module& m_);
    void init_openravepy_collisionreport();
    py::module& m;
    py::class_<PyCollisionReport, OPENRAVE_SHARED_PTR<PyCollisionReport> > collisionreport;
#else
    CollisionReportInitializer();
    void init_openravepy_collisionreport();
    py::class_<PyCollisionReport, OPENRAVE_SHARED_PTR<PyCollisionReport> > collisionreport;
#endif
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_COLLISIONREPORT_H
