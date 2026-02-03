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
#ifndef OPENRAVEPY_INTERNAL_COLLISIONCHECKERBASE_H
#define OPENRAVEPY_INTERNAL_COLLISIONCHECKERBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class OPENRAVEPY_API PyCollisionCheckerBase : public PyInterfaceBase
{
protected:
    CollisionCheckerBasePtr _pCollisionChecker;
public:
    PyCollisionCheckerBase(CollisionCheckerBasePtr pCollisionChecker, PyEnvironmentBasePtr pyenv);
    virtual ~PyCollisionCheckerBase();

    CollisionCheckerBasePtr GetCollisionChecker();

    bool SetCollisionOptions(int options);
    int GetCollisionOptions() const;

    bool InitEnvironment();

    void DestroyEnvironment();

    bool InitKinBody(PyKinBodyPtr pbody);

    void SetGeometryGroup(const std::string& groupname);

    bool SetBodyGeometryGroup(PyKinBodyPtr pybody, const std::string& groupname);

    object GetGeometryGroup();

    object GetBodyGeometryGroup(PyKinBodyPtr pybody);

    void RemoveKinBody(PyKinBodyPtr pbody);

    bool CheckCollision(PyKinBodyPtr pbody1);
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport);

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2);

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1);

    bool CheckCollision(object o1, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1, object o2);
    bool CheckCollision(object o1, object o2, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1, PyKinBodyPtr pybody2);

    bool CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded);

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport);

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded);

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyLinkPtr plink);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyLinkPtr plink, PyCollisionReportPtr pReport);

    object CheckCollisionRays(object rays, PyKinBodyPtr pbody,bool bFrontFacingOnly=false, object oCheckPreemptFn=py::none_());

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyCollisionReportPtr pReport);

    bool CheckCollisionTriMesh(object otrimesh, PyKinBodyPtr pybody, PyCollisionReportPtr pReport);

    bool CheckCollisionTriMesh(object otrimesh, PyCollisionReportPtr pReport);

    bool CheckCollisionOBB(object oaabb, object otransform, PyCollisionReportPtr pReport);

    bool CheckCollisionOBB(object oaabb, object otransform, object bodiesincluded, PyCollisionReportPtr pReport);

    virtual bool CheckSelfCollision(object o1, PyCollisionReportPtr pReport);
};

struct CollisionCheckerBaseInitializer
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    CollisionCheckerBaseInitializer(py::module& m_);
    void init_openravepy_collisionchecker();
    py::module& m;
    py::class_<PyCollisionCheckerBase, OPENRAVE_SHARED_PTR<PyCollisionCheckerBase>, PyInterfaceBase> collisionchecker;
#else
    CollisionCheckerBaseInitializer();
    void init_openravepy_collisionchecker();
    py::class_<PyCollisionCheckerBase, OPENRAVE_SHARED_PTR<PyCollisionCheckerBase>, bases<PyInterfaceBase> > collisionchecker;
#endif
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_COLLISIONCHECKERBASE_H
