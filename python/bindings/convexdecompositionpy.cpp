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
#ifndef OPENRAVE_DISABLE_ASSERT_HANDLER
#define BOOST_ENABLE_ASSERT_HANDLER
#endif
#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/numpy.hpp>
#include <pyconfig.h>

#include <exception>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/assert.hpp>

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"

#include "NvConvexDecomposition.h"

using namespace boost::python;
using namespace std;
using namespace openravepy;

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

        boost::python::tuple shape = boost::python::make_tuple(result.mVcount, 3);
        numpy::dtype dt = numpy::dtype::get_builtin<NxF32>();
        numpy::ndarray pyvertices = numpy::empty(shape, dt);
        std::memcpy(pyvertices.get_data(), &result.mVertices[0], result.mVcount * 3 * sizeof(NxF32));

        boost::python::tuple indices_shape = boost::python::make_tuple(result.mTcount, 3);
        numpy::ndarray pyindices = numpy::empty(indices_shape, numpy::dtype::get_builtin<int>());
        std::memcpy(pyindices.get_data(), &result.mIndices[0], result.mTcount * 3 * sizeof(int));

        hulls.append(boost::python::make_tuple(pyvertices, pyindices));
    }

    return std::move(hulls);
}

BOOST_PYTHON_FUNCTION_OVERLOADS(computeConvexDecomposition_overloads, computeConvexDecomposition, 2, 10)

BOOST_PYTHON_MODULE(convexdecompositionpy)
{
    Py_Initialize();
    numpy::initialize();
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
