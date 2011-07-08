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
#define BOOST_ENABLE_ASSERT_HANDLER

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

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"

#include "NvConvexDecomposition.h"

using namespace boost::python;
using namespace std;

struct cdpy_exception : std::exception
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

namespace boost
{
inline void assertion_failed(char const * expr, char const * function, char const * file, long line)
{
    throw cdpy_exception(str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr));
}
}

object computeConvexDecomposition(const boost::multi_array<float, 2>& vertices, const boost::multi_array<int, 2>& indices,
                                  NxF32 skinWidth=0, NxU32 decompositionDepth=8, NxU32 maxHullVertices=64, NxF32 concavityThresholdPercent=0.1f, NxF32 mergeThresholdPercent=30.0f, NxF32 volumeSplitThresholdPercent=0.1f, bool useInitialIslandGeneration=true, bool useIslandGeneration=false)
{
    boost::shared_ptr<CONVEX_DECOMPOSITION::iConvexDecomposition> ic(CONVEX_DECOMPOSITION::createConvexDecomposition(),CONVEX_DECOMPOSITION::releaseConvexDecomposition);

    if( indices.size() > 0 ) {
        FOREACHC(it,indices)
        ic->addTriangle(&vertices[(*it)[0]][0], &vertices[(*it)[1]][0], &vertices[(*it)[2]][0]);
    }
    else {
        BOOST_ASSERT((vertices.size()%3)==0);
        for(size_t i = 0; i < vertices.size(); i += 3)
            ic->addTriangle(&vertices[i][0], &vertices[i+1][0], &vertices[i+2][0]);
    }

    ic->computeConvexDecomposition(skinWidth, decompositionDepth, maxHullVertices, concavityThresholdPercent, mergeThresholdPercent, volumeSplitThresholdPercent, useInitialIslandGeneration, useIslandGeneration, false);
    NxU32 hullCount = ic->getHullCount();
    boost::python::list hulls;
    CONVEX_DECOMPOSITION::ConvexHullResult result;
    for(NxU32 i = 0; i < hullCount; ++i) {
        ic->getConvexHullResult(i,result);

        npy_intp dims[] = { result.mVcount,3};
        PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(result.mVertices[0])==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        std::copy(&result.mVertices[0],&result.mVertices[3*result.mVcount],(NxF32*)PyArray_DATA(pyvertices));

        dims[0] = result.mTcount;
        dims[1] = 3;
        PyObject *pyindices = PyArray_SimpleNew(2,dims, PyArray_INT);
        std::copy(&result.mIndices[0],&result.mIndices[3*result.mTcount],(int*)PyArray_DATA(pyindices));

        hulls.append(boost::python::make_tuple(static_cast<numeric::array>(handle<>(pyvertices)), static_cast<numeric::array>(handle<>(pyindices))));
    }

    return hulls;
}

BOOST_PYTHON_FUNCTION_OVERLOADS(computeConvexDecomposition_overloads, computeConvexDecomposition, 2, 10)

BOOST_PYTHON_MODULE(convexdecompositionpy)
{
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_int();
    T_from_number<float>();
    T_from_number<double>();

    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
    class_< cdpy_exception >( "_cdpy_exception_" )
    .def( init<const std::string&>() )
    .def( init<const cdpy_exception&>() )
    .def( "message", &cdpy_exception::message, return_copy_const_ref() )
    .def( "__str__", &cdpy_exception::message, return_copy_const_ref() )
    ;
    exception_translator<cdpy_exception>();

    def("computeConvexDecomposition", computeConvexDecomposition,
        computeConvexDecomposition_overloads(args("vertices", "indices", "skinWidth", "decompositionDepth", "maxHullVertices", "concavityThresholdPercent", "mergeThresholdPercent", "volumeSplitThresholdPercent", "useInitialIslandGeneration", "useIslandGeneration"), "John Ratcliff's Convex Decomposition"));

    scope().attr("__author__") = "John Ratcliff";
    scope().attr("__license__") = "MIT";
};
