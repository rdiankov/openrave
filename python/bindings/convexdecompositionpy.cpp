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
#include "bindings.h"

using namespace boost::python;
using namespace std;

struct cdpy_exception : std::exception
{
    cdpy_exception() : std::exception(), _s("unknown exception") {}
    cdpy_exception(const std::string& s) : std::exception() { _s = "cdpy: " + s; }
    virtual ~cdpy_exception() throw() {}
    char const* what() const throw() { return _s.c_str(); }
    const std::string& message() const { return _s; }
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

BOOST_PYTHON_MODULE(convexdecompositionpy)
{
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_int();
    T_from_number<float>();
    T_from_number<double>();
 
//    CONVEX_DECOMPOSITION::iConvexDecomposition *ic = CONVEX_DECOMPOSITION::createConvexDecomposition();
//    ic->addTriangle( tris[i].p0, tris[i].p1, tris[i].p2 );
//ic->computeConvexDecomposition(skinWidth,
//decompositionDepth,
//maxHullVertices,
//concavityThresholdPercent,
//mergeThresholdPercent,
//volumeSplitThresholdPercent,
//useInitialIslandGeneration,
//useIslandGeneration,
//useBackgroundThreads);
//
// CONVEX_DECOMPOSITION::releaseConvexDecomposition(ic);

    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
    class_< cdpy_exception >( "_cdpy_exception_" )
        .def( init<const std::string&>() )
        .def( init<const cdpy_exception&>() )
        .def( "message", &cdpy_exception::message, return_copy_const_ref() )
        .def( "__str__", &cdpy_exception::message, return_copy_const_ref() )
        ;
    exception_translator<cdpy_exception>();
};
