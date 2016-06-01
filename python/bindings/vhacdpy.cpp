// Copyright (C) 2010 Rosen Diankov
//
// vhacdpy is free software: you can redistribute it and/or modify
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
#ifndef OPENRAVE_DISABLE_ASSERT_HANDLER
#define BOOST_ENABLE_ASSERT_HANDLER
#endif
#include <iostream>
#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/stl_iterator.hpp>
#include <pyconfig.h>
#include <numpy/arrayobject.h>

#include <exception>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/assert.hpp>
#include <VHACD.h>

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"

#include <openrave/openrave.h>
#include "openravepy_int.h"

using namespace boost::python;
using namespace std;
using namespace openravepy;
using OpenRAVE::TriMesh;
using OpenRAVE::Vector;

void ExtractTriMeshFromConvexhull(const VHACD::IVHACD::ConvexHull& ch, TriMesh& openraveMesh) {
  openraveMesh.vertices.resize(ch.m_nPoints);
  double* vertices = ch.m_points;
  FOREACH(itvertex, openraveMesh.vertices) {
    *itvertex = Vector(vertices[0], vertices[1], vertices[2]);
    vertices += 3;
  }

  openraveMesh.indices.resize(3*ch.m_nTriangles);
  std::copy(ch.m_triangles, ch.m_triangles + 3*ch.m_nTriangles, openraveMesh.indices.begin());
}

class RaveDebugLogger : public VHACD::IVHACD::IUserLogger {
  void Log(const char* msg) {
    RAVELOG_DEBUG_FORMAT("Vhacdpy : %s", msg);
  }
};

boost::python::list ComputeVHACD(object oMesh, object oParameters = object()) {

  TriMesh openraveMesh;
  if( !ExtractTriMesh(oMesh, openraveMesh)) {
    return boost::python::list();
  }

  VHACD::IVHACD::Parameters params;
  if( !oParameters.is_none() ) {
    params = extract<VHACD::IVHACD::Parameters>(oParameters);
  }
  RaveDebugLogger logger;
  params.m_logger = &logger;

  VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
  float* vertices = new float[openraveMesh.vertices.size() * 3];
  float* it = vertices;
  FOREACH(itvertex, openraveMesh.vertices) {
    *it++ = itvertex->x;
    *it++ = itvertex->y;
    *it++ = itvertex->z;
  }

  int* indices = &openraveMesh.indices[0];

  vhacd->Compute(vertices, 3, openraveMesh.vertices.size(), indices, 3, openraveMesh.indices.size()/3, params);

  delete[] vertices;

  boost::python::list result;
  VHACD::IVHACD::ConvexHull ch;
  for(size_t i = 0; i < vhacd->GetNConvexHulls(); ++i) {
    vhacd->GetConvexHull(i, ch);
    ExtractTriMeshFromConvexhull(ch, openraveMesh);
    result.append(toPyTriMesh(openraveMesh));
  }
  return result;
}

BOOST_PYTHON_FUNCTION_OVERLOADS(ComputeVHACD_overloads, ComputeVHACD, 1, 2)

BOOST_PYTHON_MODULE(vhacdpy)
{
  // Don't know if these lines are really necessary...
  import_array();
  numeric::array::set_module_and_type("numpy", "ndarray");
  int_from_int();

  class_<VHACD::IVHACD::Parameters>("Parameters")
    .def_readwrite("concativity", &VHACD::IVHACD::Parameters::m_concavity)
    .def_readwrite("alpha", &VHACD::IVHACD::Parameters::m_alpha)
    .def_readwrite("beta", &VHACD::IVHACD::Parameters::m_beta)
    .def_readwrite("gamma", &VHACD::IVHACD::Parameters::m_gamma)
    .def_readwrite("minVolumePerConvexhull", &VHACD::IVHACD::Parameters::m_minVolumePerCH)
    .def_readwrite("resolution", &VHACD::IVHACD::Parameters::m_resolution)
    .def_readwrite("maxNumVerticesPerConvexhull", &VHACD::IVHACD::Parameters::m_maxNumVerticesPerCH)
    .def_readwrite("depth", &VHACD::IVHACD::Parameters::m_depth)
    .def_readwrite("planeDownsampling", &VHACD::IVHACD::Parameters::m_planeDownsampling)
    .def_readwrite("convexhullDownsampling", &VHACD::IVHACD::Parameters::m_convexhullDownsampling)
    .def_readwrite("pca", &VHACD::IVHACD::Parameters::m_pca)
    .def_readwrite("mode", &VHACD::IVHACD::Parameters::m_mode)
    .def_readwrite("convexhullApproximation", &VHACD::IVHACD::Parameters::m_convexhullApproximation)
    .def_readwrite("oclAcceleration", &VHACD::IVHACD::Parameters::m_oclAcceleration)
    ;

  // does this have any use ?
  typedef return_value_policy< copy_const_reference > return_copy_const_ref;

  def("ComputeVHACD", ComputeVHACD, ComputeVHACD_overloads(args("mesh", "parameters"), "V-HACD library binding"));

  scope().attr("__author__") = "Khaled Mamou";
  scope().attr("__license__") = "\
 Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)\
 All rights reserved.\
 \
 \
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:\
 \
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.\
 \
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.\
 \
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.\
 \
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\
";
}
