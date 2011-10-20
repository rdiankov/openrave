// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_BOOST_PYTHON_BINDINGS
#define OPENRAVE_BOOST_PYTHON_BINDINGS

#include <Python.h>
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/python.hpp>
#include <boost/assert.hpp>
#include <boost/cstdint.hpp>
#include <stdint.h>

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

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <stdexcept>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <complex>
#include <algorithm>

using namespace boost::python;

class PyVoidHandle
{
public:
    PyVoidHandle() {
    }
    PyVoidHandle(boost::shared_ptr<void> handle) : _handle(handle) {
    }
    void close() {
        _handle.reset();
    }
    boost::shared_ptr<void> _handle;
};

class PyVoidHandleConst
{
public:
    PyVoidHandleConst() {
    }
    PyVoidHandleConst(boost::shared_ptr<void const> handle) : _handle(handle) {
    }
    void close() {
        _handle.reset();
    }
    boost::shared_ptr<void const> _handle;
};

template <typename T>
inline std::vector<T> ExtractArray(const object& o)
{
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = extract<T>(o[i]);
    }
    return v;
}

template <typename T>
inline std::set<T> ExtractSet(const object& o)
{
    std::set<T> v;
    size_t nlen = len(o);
    for(size_t i = 0; i < nlen; ++i) {
        v.insert(extract<T>(o[i]));
    }
    return v;
}

/// class for truly registering a C++ exception to a python exception
/// add this definition to BOOST_PYTHON_MODULE:
//
//    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
//    class_< T >( "_custom_exception_" )
//        .def( init<const std::string&>() )
//        .def( init<const T&>() )
//        .def( "message", &T::message, return_copy_const_ref() )
//        .def( "__str__", &T::message, return_copy_const_ref() )
//        ;
// inside python do:
//
//class custom_exception(Exception):
//    """wrap up the C++ custom_exception"""
//    def __init__( self, app_error ):
//        Exception.__init__( self )
//        self._pimpl = app_error
//    def __str__( self ):
//        return self._pimpl.message()
//    def __getattribute__(self, attr):
//        my_pimpl = super(custom_exception, self).__getattribute__("_pimpl")
//        try:
//            return getattr(my_pimpl, attr)
//        except AttributeError:
//            return super(custom_exception,self).__getattribute__(attr)
//
// import custom_module
// custom_module._custom_exception_.py_err_class = custom_exception
template <typename T>
struct exception_translator
{
    exception_translator() {
        register_exception_translator<T>(&exception_translator::translate);

        //Register custom r-value converter
        //There are situations, where we have to pass the exception back to
        //C++ library. This will do the trick
        converter::registry::push_back( &exception_translator::convertible, &exception_translator::construct, type_id<T>() );
    }

    static void translate( const T& err )
    {
        object pimpl_err( err );
        object pyerr_class = pimpl_err.attr( "py_err_class" );
        object pyerr = pyerr_class( pimpl_err );
        PyErr_SetObject( pyerr_class.ptr(), incref( pyerr.ptr() ) );
    }

    //Sometimes, exceptions should be passed back to the library.
    static void* convertible(PyObject* py_obj){
        if( 1 != PyObject_IsInstance( py_obj, PyExc_Exception ) ) {
            return 0;
        }

        if( !PyObject_HasAttrString( py_obj, "_pimpl" ) ) {
            return 0;
        }

        object pyerr( handle<>( borrowed( py_obj ) ) );
        object pimpl = getattr( pyerr, "_pimpl" );
        extract<T> type_checker( pimpl );
        if( !type_checker.check() ) {
            return 0;
        }
        return py_obj;
    }

    static void construct( PyObject* py_obj, converter::rvalue_from_python_stage1_data* data)
    {
        typedef converter::rvalue_from_python_storage<T> storage_t;

        object pyerr( handle<>( borrowed( py_obj ) ) );
        object pimpl = getattr( pyerr, "_pimpl" );

        storage_t* the_storage = reinterpret_cast<storage_t*>( data );
        void* memory_chunk = the_storage->storage.bytes;
        new (memory_chunk) T( extract<T>(pimpl) );
        data->convertible = memory_chunk;
    }
};

// register const versions of the classes
//template <class T> inline T* get_pointer( boost::shared_ptr<const T>
//const& p){
//     return const_cast<T*>(p.get());
//}
//
//template <class T> struct pintee< boost::shared_ptr<const T> >{
//     typedef T type;
//};
//
//boost::python::register_ptr_to_python< boost::shared_ptr<const my_class> >();

struct int_from_int
{
    int_from_int()
    {
        converter::registry::push_back(&convertible, &construct, type_id<int>());
    }

    static void* convertible( PyObject* obj)
    {
        PyObject* newobj = PyNumber_Int(obj);
        if (!PyString_Check(obj) && newobj) {
            Py_DECREF(newobj);
            return obj;
        }
        else {
            if (newobj) {
                Py_DECREF(newobj);
            }
            PyErr_Clear();
            return 0;
        }
    }

    static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* newobj = PyNumber_Int(_obj);
        int* storage = (int*)((converter::rvalue_from_python_storage<int>*)data)->storage.bytes;
        *storage = extract<int>(newobj);
        Py_DECREF(newobj);
        data->convertible = storage;
    }
};

template<typename T>
struct T_from_number
{
    T_from_number()
    {
        converter::registry::push_back(&convertible, &construct, type_id<T>());
    }

    static void* convertible( PyObject* obj)
    {
        PyObject* newobj = PyNumber_Float(obj);
        if (!PyString_Check(obj) && newobj) {
            Py_DECREF(newobj);
            return obj;
        }
        else {
            if (newobj) {
                Py_DECREF(newobj);
            }
            PyErr_Clear();
            return 0;
        }
    }

    static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* newobj = PyNumber_Float(_obj);
        T* storage = (T*)((converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
        *storage = extract<T>(newobj);
        Py_DECREF(newobj);
        data->convertible = storage;
    }
};

/// should call in the beginning of all BOOST_PYTHON_MODULE
void init_python_bindings();

#ifdef OPENRAVE_BININGS_PYARRAY

inline numeric::array toPyArrayN(const float* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    npy_intp dims[] = { N};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_FLOAT);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(float));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const float* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    uint64_t totalsize = 1;
    FOREACH(it,dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_FLOAT);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(float));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const double* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f8"));
    }
    npy_intp dims[] = { N};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_DOUBLE);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(double));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const double* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    uint64_t totalsize = 1;
    FOREACH(it,dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_DOUBLE);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(double));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint8_t* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    uint64_t totalsize = 1;
    for(size_t i = 0; i < dims.size(); ++i) {
        totalsize *= dims[i];
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_UINT8);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(uint8_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint8_t* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u1"));
    }
    npy_intp dims[] = { N};
    PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_UINT8);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(uint8_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const int* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("i4"));
    }
    npy_intp dims[] = { N};
    PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_INT32);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(int));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint32_t* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u4"));
    }
    npy_intp dims[] = { N};
    PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_UINT32);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(uint32_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

template <typename T>
inline object toPyList(const std::vector<T>& v)
{
    boost::python::list lvalues;
    FOREACHC(it,v)
    lvalues.append(object(*it));
    return lvalues;
}

template <typename T>
inline numeric::array toPyArray(const std::vector<T>& v)
{
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL,0);
    }
    return toPyArrayN(&v[0],v.size());
}

template <typename T>
inline numeric::array toPyArray(const std::vector<T>& v, std::vector<npy_intp>& dims)
{
    if( v.size() == 0 )
        return toPyArrayN((T*)NULL,0);
    uint64_t totalsize = 1;
    FOREACH(it,dims)
    totalsize *= *it;
    BOOST_ASSERT(totalsize == v.size());
    return toPyArrayN(&v[0],dims);
}

#endif

#endif
