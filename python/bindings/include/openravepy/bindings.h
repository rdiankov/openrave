// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

// shouldn't include openrave.h!
#ifndef OPENRAVE_BOOST_PYTHON_BINDINGS
#define OPENRAVE_BOOST_PYTHON_BINDINGS

#include <openravepy/openravepy_config.h>

#include <stdint.h>
// numpy
#include <numpy/arrayobject.h>
#include <numpy/arrayscalars.h>

#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST

#else // _MSC_VER
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <stdexcept>

// apparently there's a problem with higher versions of C++
#if __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)
#include <typeinfo>
#define FOREACH(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); )
#else
#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )
#endif // __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif // _MSC_VER

#include <complex>
#include <algorithm>
// openrave
#include <openrave/config.h>
#include <openrave/logging.h>
#include <openrave/smart_ptr.h>

#ifndef _RAVE_DISPLAY
#define _RAVE_DISPLAY(RUNCODE)                                               \
{                                                                              \
    printf(                                                                    \
        "\n%s:%d, [ %s "                                                       \
        "]\n-----------------------------------------------------------------" \
        "--------------\n",                                                    \
        __FILE__, __LINE__, __func__ /*__PRETTY_FUNCTION__*/);                 \
    RUNCODE;                                                                   \
    printf("\n");                                                              \
}
#endif // _RAVE_DISPLAY

namespace openravepy {

// https://stackoverflow.com/questions/35041268/how-to-convert-a-vector-to-numpy-array-with-templates-and-boost
template <typename T>
struct select_dtype
{};

template <>
struct select_dtype<double>
{
    static constexpr char type[] = "f8";
};

template <>
struct select_dtype<float>
{
    static constexpr char type[] = "f4";
};

template <>
struct select_dtype<int>
{
    static constexpr char type[] = "i4";
};

template <>
struct select_dtype<uint8_t>
{
    static constexpr char type[] = "u1";
};

template <>
struct select_dtype<uint16_t>
{
    static constexpr char type[] = "u2";
};

template <>
struct select_dtype<uint32_t>
{
    static constexpr char type[] = "u4";
};

template <>
struct select_dtype<uint64_t>
{
    static constexpr char type[] = "u8";
};

template <>
struct select_dtype<bool>
{
    static constexpr char type[] = "?";
};

template <typename T>
struct select_npy_type
{};

template <>
struct select_npy_type<double>
{
    static constexpr NPY_TYPES type = NPY_DOUBLE;
};

template <>
struct select_npy_type<float>
{
    static constexpr NPY_TYPES type = NPY_FLOAT;
};

template <>
struct select_npy_type<int>
{
    static constexpr NPY_TYPES type = NPY_INT;
};

template <>
struct select_npy_type<uint8_t>
{
    static constexpr NPY_TYPES type = NPY_UINT8;
};

template <>
struct select_npy_type<uint16_t>
{
    static constexpr NPY_TYPES type = NPY_UINT16;
};

template <>
struct select_npy_type<uint32_t>
{
    static constexpr NPY_TYPES type = NPY_UINT32;
};

template <>
struct select_npy_type<uint64_t>
{
    static constexpr NPY_TYPES type = NPY_UINT64;
};

template <>
struct select_npy_type<bool>
{
    static constexpr NPY_TYPES type = NPY_BOOL;
};
} // namespace openravepy

#ifdef USE_PYBIND11_PYTHON_BINDINGS
#include "pybind11/pybind11_bindings.h"
#else
#include "boostpython/boostpython_bindings.h"
#endif

namespace openravepy {

class PyVoidHandle
{
public:
    PyVoidHandle() {
    }
    PyVoidHandle(OPENRAVE_SHARED_PTR<void> handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    OPENRAVE_SHARED_PTR<void> _handle;
};

class PyVoidHandleConst
{
public:
    PyVoidHandleConst() {
    }
    PyVoidHandleConst(OPENRAVE_SHARED_PTR<void const> handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    OPENRAVE_SHARED_PTR<void const> _handle;
};

template <typename T>
inline std::vector<T> ExtractArray(const py::object& o)
{
    if( IS_PYTHONOBJECT_NONE(o) ) {
        return {};
    }
    std::vector<T> v;
    try {
        const size_t n = len(o);
        v.resize(n);
        for(size_t i = 0; i < n; ++i) {
            v[i] = py::extract<T>(o[i]);
        }
    }
    catch(...) {
        RAVELOG_WARN("Cannot do ExtractArray for " + std::string(typeid(T).name()));
    }
    return v;
}

template <typename T>
inline std::set<T> ExtractSet(const py::object& o)
{
    std::set<T> v;
    size_t nlen = len(o);
    for(size_t i = 0; i < nlen; ++i) {
        v.insert(py::extract<T>(o[i]));
    }
    return v;
}

inline std::string GetPyErrorString()
{
    PyObject *error, *value, *traceback, *string;
    PyErr_Fetch(&error, &value, &traceback);
    PyErr_NormalizeException(&error, &value, &traceback);
    std::string s;
    if(error != nullptr) {
        string = PyObject_Str(value);
        if(string != nullptr) {
            s.assign(PyString_AsString(string));
            Py_DECREF(string);
        }
    }
    // Does nothing when the ptr is nullptr
    Py_DECREF(error);
    Py_DECREF(value);
    Py_DECREF(traceback);

    return s;
}

/// should call in the beginning of all BOOST_PYTHON_MODULE
void init_python_bindings();

} // namespace openravepy

#endif // OPENRAVE_BOOST_PYTHON_BINDINGS
