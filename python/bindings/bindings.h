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

#include <stdint.h>
// numpy
#include <numpy/arrayobject.h>
#include <numpy/arrayscalars.h>
// boost
#include <boost/multi_array.hpp>
#include <boost/python.hpp>

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
#endif

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif // _MSC_VER

#include <complex>
#include <algorithm>

// is_none is not supported by older versions of python
#if BOOST_VERSION >= 104300
#define IS_PYTHONOBJECT_NONE(o) (o).is_none()
#else
#define IS_PYTHONOBJECT_NONE(o) (!!(o))
#endif

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

namespace py = boost::python;

inline py::object ConvertStringToUnicode(const std::string& s)
{
    return py::object(py::handle<>(PyUnicode_Decode(s.c_str(),s.size(), "utf-8", nullptr)));
}

class PyVoidHandle
{
public:
    PyVoidHandle() {
    }
    PyVoidHandle(boost::shared_ptr<void> handle) : _handle(handle) {
    }
    void Close() {
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
    void Close() {
        _handle.reset();
    }
    boost::shared_ptr<void const> _handle;
};

template <typename T>
inline std::vector<T> ExtractArray(const py::object& o)
{
    if( IS_PYTHONOBJECT_NONE(o) ) {
        return std::vector<T>();
    }
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = py::extract<T>(o[i]);
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

//template <typename ExceptionType>
//struct ExceptionTranslator
//{
//   typedef ExceptionTranslator<ExceptionType> type;
//
//   static PyObject *pyException;
//
//   static void RegisterExceptionTranslator(scope & scope, const char*
//moduleName, const char* name)
//   {
//     // Add the exception to the module scope
//     std::strstream exName;
//     exName << moduleName << "." << name << '\0';
//     pyException = PyErr_NewException(exName.str(), nullptr, nullptr);
//     handle<> instanceException(pyException);
//     scope.attr(name) = object(instanceException);
//
//     // Register a translator for the type
//     register_exception_translator< ExceptionType >
//       (
//        &ExceptionTranslator<ExceptionType>::translateException
//        );
//   }
//
//   static void translateException(const ExceptionType& ex)
//   {
//     PyErr_SetString(pyException, ex.getMessage().ptr());
//   }
//};
//
//template<typename ExceptionType>
//PyObject* ExceptionTranslator<ExceptionType>::pyException;
//
//// Convenience macro
//#define REGISTER_EXCEPTION(scopeRef, moduleName, className) ExceptionTranslator<className>::RegisterExceptionTranslator(scopeRef, moduleName, #className)
//
//
//// Module
//======================================================================
//BOOST_PYTHON_MODULE(my_module)
//{
//
//   scope moduleScope;
//
//   REGISTER_EXCEPTION(moduleScope, "my_module", InstanceException);
//
//.....
//}

template <typename T>
struct exception_translator
{
    exception_translator() {
        py::register_exception_translator<T>(&exception_translator::translate);

        //Register custom r-value converter
        //There are situations, where we have to pass the exception back to
        //C++ library. This will do the trick
        py::converter::registry::push_back( &exception_translator::convertible, &exception_translator::construct, py::type_id<T>() );
    }

    static void translate( const T& err )
    {
        py::object pimpl_err( err );
        py::object pyerr_class = pimpl_err.attr( "py_err_class" );
        py::object pyerr = pyerr_class( pimpl_err );
        PyErr_SetObject( pyerr_class.ptr(), py::incref( pyerr.ptr() ) );
    }

    //Sometimes, exceptions should be passed back to the library.
    static void* convertible(PyObject* py_obj){
        if( 1 != PyObject_IsInstance( py_obj, PyExc_Exception ) ) {
            return 0;
        }

        if( !PyObject_HasAttrString( py_obj, "_pimpl" ) ) {
            return 0;
        }

        py::object pyerr( py::handle<>( py::borrowed( py_obj ) ) );
        py::object pimpl = getattr( pyerr, "_pimpl" );
        py::extract<T> type_checker( pimpl );
        if( !type_checker.check() ) {
            return 0;
        }
        return py_obj;
    }

    static void construct( PyObject* py_obj, py::converter::rvalue_from_python_stage1_data* data)
    {
        typedef py::converter::rvalue_from_python_storage<T> storage_t;

        py::object pyerr( py::handle<>( py::borrowed( py_obj ) ) );
        py::object pimpl = getattr( pyerr, "_pimpl" );

        storage_t* the_storage = reinterpret_cast<storage_t*>( data );
        void* memory_chunk = the_storage->storage.bytes;
        new (memory_chunk) T( py::extract<T>(pimpl) );
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
//py::register_ptr_to_python< boost::shared_ptr<const my_class> >();

template<typename T>
struct float_from_number
{
    float_from_number()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<T>());
    }

    static void* convertible( PyObject* obj)
    {
        return PyNumber_Check(obj) ? obj : nullptr;
    }

    static void construct(PyObject* _obj, py::converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* tmp = PyNumber_Float(_obj);
        T* storage = (T*)((py::converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
        *storage = py::extract<T>(tmp);
        Py_DECREF(tmp);
        data->convertible = storage;
    }
};

template<typename T>
struct int_from_number
{
    int_from_number()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<T>());
    }

    static void* convertible( PyObject* obj)
    {
        return PyNumber_Check(obj) ? obj : nullptr;
    }

    static void construct(PyObject* _obj, py::converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* tmp = PyNumber_Long(_obj);
        T* storage = (T*)((py::converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
        *storage = py::extract<T>(tmp);
        Py_DECREF(tmp);
        data->convertible = storage;
    }
};

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

#ifdef OPENRAVE_BININGS_PYARRAY

template <typename T>
inline py::numeric::array toPyArrayN(const T* pvalues, const size_t N)
{
    if( N == 0 ) {
        return static_cast<py::numeric::array>(py::numeric::array(py::list()).astype(select_dtype<T>::type));
    }
    npy_intp dims[] = { npy_intp(N) };
    PyObject *pyvalues = PyArray_SimpleNew(1, dims, select_npy_type<T>::type);
    if( pvalues != nullptr ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, N * sizeof(T));
    }
    return static_cast<py::numeric::array>(py::handle<>(pyvalues));
}

template <typename T>
inline py::numeric::array toPyArrayN(const T* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.empty() ) {
        return static_cast<py::numeric::array>(py::numeric::array(py::list()).astype(select_dtype<T>::type));
    }
    size_t numel = 1;
    for(npy_intp dim : dims) {
        numel *= dim;
    }
    if( numel == 0 ) {
        return static_cast<py::numeric::array>(py::numeric::array(py::list()).astype(select_dtype<T>::type));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(), dims.data(), select_npy_type<T>::type);
    if( pvalues != nullptr ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, numel * sizeof(T));
    }
    return static_cast<py::numeric::array>(py::handle<>(pyvalues));
}

template <typename T>
inline py::object toPyList(const std::vector<T>& v)
{
    py::list lvalues;
    FOREACHC(it,v) {
        lvalues.append(object(*it));
    }
    return std::move(lvalues);
}

template <typename T>
inline py::numeric::array toPyArray(const std::vector<T>& v)
{
    if( v.empty() ) {
        return toPyArrayN((T*)nullptr, 0);
    }
    return toPyArrayN(v.data(), v.size());
}

template <typename T>
inline py::numeric::array toPyArray(const std::vector<T>& v, std::vector<npy_intp>& dims)
{
    if( v.empty() ) {
        return toPyArrayN((T*)nullptr, dims);
    }
    size_t numel = 1;
    for(npy_intp dim : dims) {
        numel *= dim;
    }
    BOOST_ASSERT(numel == v.size());
    return toPyArrayN(v.data(), dims);
}

template <typename T, int N>
inline py::numeric::array toPyArray(const boost::array<T,N>& v)
{
    if( v.empty() ) {
        return toPyArrayN((T*)nullptr, 0);
    }
    return toPyArrayN(v.data(), v.size());
}

#endif // OPENRAVE_BININGS_PYARRAY

} // namespace openravepy

#endif // OPENRAVE_BOOST_PYTHON_BINDINGS
