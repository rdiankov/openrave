// Copyright (C) 2010 Rosen Diankov
//
// convexdecompositionpy is free software: you can redistribute it and/or modify
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
#include <openrave/config.h>

#define OPENRAVE_BINDINGS_PYARRAY
#include <openravepy/bindings.h>

#include "NvConvexDecomposition.h"

namespace py = openravepy::py;
namespace numeric = py::numeric;
using py::object;
using py::extract;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope;
using py::args;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::optional;
using py::def;
using openravepy::int_from_number;
using openravepy::float_from_number;
using openravepy::OpenRAVEBoostPythonExceptionTranslator;
#endif // USE_PYBIND11_PYTHON_BINDINGS

struct OPENRAVE_API cdpy_exception : std::exception
{
    cdpy_exception() : std::exception(), _s("unknown exception") {
    }
    cdpy_exception(const std::string& s) : std::exception() {
        _s = "cdpy: " + s;
    }
    virtual ~cdpy_exception() throw() {
    }
    char const* what() const throw() {
        return _s.c_str();
    }
    const std::string& message() const {
        return _s;
    }
private:
    std::string _s;
};

#if !defined(OPENRAVE_DISABLE_ASSERT_HANDLER) && defined(BOOST_ENABLE_ASSERT_HANDLER)
namespace boost
{
inline void assertion_failed(char const * expr, char const * function, char const * file, long line)
{
    throw cdpy_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr));
}
#if BOOST_VERSION>104600
inline void assertion_failed_msg(char const * expr, char const * msg, char const * function, char const * file, long line)
{
    throw cdpy_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s, msg: %s")%file%line%function%expr%msg));
}
#endif

}
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
object computeConvexDecomposition(py::array_t<float>& vertices, py::array_t<int>& indices,
#else
object computeConvexDecomposition(const boost::multi_array<float, 2>& vertices, const boost::multi_array<int, 2>& indices,
#endif
                                  NxF32 skinWidth=0, NxU32 decompositionDepth=8, NxU32 maxHullVertices=64, NxF32 concavityThresholdPercent=0.1f, NxF32 mergeThresholdPercent=30.0f, NxF32 volumeSplitThresholdPercent=0.1f, bool useInitialIslandGeneration=true, bool useIslandGeneration=false)
{
    OPENRAVE_SHARED_PTR<CONVEX_DECOMPOSITION::iConvexDecomposition> ic(CONVEX_DECOMPOSITION::createConvexDecomposition(),CONVEX_DECOMPOSITION::releaseConvexDecomposition);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html#direct-access
    // https://stackoverflow.com/questions/49582252/pybind-numpy-access-2d-nd-arrays
    const py::buffer_info& vertices_info = vertices.request();
    const std::vector<ssize_t> &vertices_shape = vertices_info.shape;
    BOOST_ASSERT(vertices_shape.size() == 2);
    const size_t nvertices = vertices_shape[0];
    BOOST_ASSERT(vertices_shape[1] == 3);
    float const* const p_vertices = (float *) vertices_info.ptr;

    const py::buffer_info& indices_info = indices.request();
    const std::vector<ssize_t> &indices_shape = indices_info.shape;
    BOOST_ASSERT(indices_shape.size() == 2);
    const size_t nindices = indices_shape[0];
    BOOST_ASSERT(indices_shape[1] == 3);
    int const* const p_indices = (int *) indices_info.ptr;
#endif

    if( indices.size() > 0 ) {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        for(size_t iindex = 0; iindex < 3 * nindices; iindex += 3) {
            ic->addTriangle(p_vertices + 3*(*(p_indices+iindex+0)),
                            p_vertices + 3*(*(p_indices+iindex+1)),
                            p_vertices + 3*(*(p_indices+iindex+2))
            );
        }
#else
        FOREACHC(it,indices) {
            ic->addTriangle(&vertices[(*it)[0]][0], &vertices[(*it)[1]][0], &vertices[(*it)[2]][0]);
        }
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }
    else {
        BOOST_ASSERT((vertices.size()%3)==0);
        for(size_t i = 0; i < vertices.size(); i += 3) {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            ic->addTriangle(p_vertices + 3*(i+0),
                            p_vertices + 3*(i+1),
                            p_vertices + 3*(i+2));
#else
            ic->addTriangle(&vertices[i][0], &vertices[i+1][0], &vertices[i+2][0]);
#endif
        }
    }

    ic->computeConvexDecomposition(skinWidth, decompositionDepth, maxHullVertices, concavityThresholdPercent, mergeThresholdPercent, volumeSplitThresholdPercent, useInitialIslandGeneration, useIslandGeneration, false);
    NxU32 hullCount = ic->getHullCount();
    py::list hulls;
    CONVEX_DECOMPOSITION::ConvexHullResult result;
    for(NxU32 i = 0; i < hullCount; ++i) {
        ic->getConvexHullResult(i,result);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        py::array_t<NxF32> pyvertices(3 * result.mVcount, result.mVertices);
        pyvertices.resize({(int) result.mVcount, 3});
        py::array_t<NxU32> pyindices(3 * result.mTcount, result.mIndices);
        pyindices.resize({(int) result.mTcount, 3});
        hulls.append(py::make_tuple(pyvertices, pyindices));
#else // USE_PYBIND11_PYTHON_BINDINGS
        npy_intp dims[] = { result.mVcount,3};
        PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(result.mVertices[0])==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        std::copy(result.mVertices, result.mVertices + 3 * result.mVcount, (NxF32*)PyArray_DATA(pyvertices));

        dims[0] = result.mTcount;
        dims[1] = 3;
        PyObject *pyindices = PyArray_SimpleNew(2,dims, PyArray_INT);
        std::copy(result.mIndices, result.mIndices + 3 * result.mTcount, (int*)PyArray_DATA(pyindices));
        hulls.append(py::make_tuple(py::to_array_astype<NxF32>(pyvertices), py::to_array_astype<int>(pyindices)));
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }

    return hulls;
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_FUNCTION_OVERLOADS(computeConvexDecomposition_overloads, computeConvexDecomposition, 2, 10)
#endif

OPENRAVE_PYTHON_MODULE(convexdecompositionpy)
{
    import_array();
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_number<int>();
    float_from_number<float>();
    float_from_number<double>();
    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::register_exception<cdpy_exception>(m, "_cdpy_exception_");
#else
    class_< cdpy_exception >( "_cdpy_exception_" )
    .def( init<const std::string&>() )
    .def( init<const cdpy_exception&>() )
    .def( "message", &cdpy_exception::message, return_copy_const_ref() )
    .def( "__str__", &cdpy_exception::message, return_copy_const_ref() )
    ;
    OpenRAVEBoostPythonExceptionTranslator<cdpy_exception>();
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
    m.def("computeConvexDecomposition", computeConvexDecomposition,
        "vertices"_a,
        "indices"_a,
        "skinWidth"_a = 0,
        "decompositionDepth"_a = 8,
        "maxHullVertices"_a = 64,
        "concavityThresholdPercent"_a = 0.1,
        "mergeThresholdPercent"_a = 30.0,
        "volumeSplitThresholdPercent"_a = 0.1,
        "useInitialIslandGeneration"_a = true,
        "useIslandGeneration"_a = false,
        "John Ratcliff's Convex Decomposition")
#else
    def("computeConvexDecomposition", computeConvexDecomposition,
        computeConvexDecomposition_overloads(PY_ARGS("vertices", "indices", "skinWidth", "decompositionDepth", "maxHullVertices", "concavityThresholdPercent", "mergeThresholdPercent", "volumeSplitThresholdPercent", "useInitialIslandGeneration", "useIslandGeneration") "John Ratcliff's Convex Decomposition"))
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.attr("__author__") = "John Ratcliff";
    m.attr("__license__") = "MIT";
#else
    scope().attr("__author__") = "John Ratcliff";
    scope().attr("__license__") = "MIT";
#endif // USE_PYBIND11_PYTHON_BINDINGS
};
