/// BoostPython specific bindings
#ifndef OPENRAVE_BOOSTPYTHON_BINDINGS_H
#define OPENRAVE_BOOSTPYTHON_BINDINGS_H

#include <boost/multi_array.hpp>

#define OPENRAVEPY_API
#include <boost/python.hpp> // already has #include <boost/shared_ptr.hpp>
#define OPENRAVE_PYTHON_MODULE(X) BOOST_PYTHON_MODULE(X)
// might need a space before "::"?
#define PY_ARGS(...) py::args(__VA_ARGS__),

namespace boost {
namespace python {
template <typename T>
inline object to_object(const T& t) {
    return object(t);
}
inline object handle_to_object(PyObject* pyo) {
    return object(handle<>(pyo));
}
template<typename T>
inline boost::python::numeric::array to_array_astype(PyObject* pyo) {
    return static_cast<boost::python::numeric::array>(static_cast<boost::python::numeric::array>(handle<>(pyo)).astype(openravepy::select_dtype<T>::type));
}
template<typename T>
inline object empty_array_astype() {
    return boost::python::numeric::array(boost::python::list()).astype(openravepy::select_dtype<T>::type);
}
template <typename T>
using extract_ = extract<T>;
// https://www.boost.org/doc/libs/1_62_0/libs/python/doc/html/reference/high_level_components/boost_python_scope_hpp.html
using scope_ = scope;
inline object none_() {
    return object();
}
using array_int = object; // py::array_int
} // namespace boost::python
} // namespace boost

// is_none is not supported by older versions of python
#if BOOST_VERSION >= 104300
#define IS_PYTHONOBJECT_NONE(o) (o).is_none()
#else
#define IS_PYTHONOBJECT_NONE(o) (!!(o))
#endif


#define IS_PYTHONOBJECT_STRING(o) (!IS_PYTHONOBJECT_NONE(o) && (PyString_Check((o).ptr()) || PyUnicode_Check((o).ptr())))

namespace openravepy {

namespace py = boost::python;

inline py::object ConvertStringToUnicode(const std::string& s)
{
    return py::handle_to_object(PyUnicode_Decode(s.c_str(), s.size(), "utf-8", nullptr));
}

#ifdef OPENRAVE_BINDINGS_PYARRAY

template <typename T>
inline py::numeric::array toPyArrayN(const T* pvalues, const size_t N)
{
    if( N == 0 ) {
        return static_cast<py::numeric::array>(py::empty_array_astype<T>());
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
        return static_cast<py::numeric::array>(py::empty_array_astype<T>());
    }
    size_t numel = 1;
    for(npy_intp dim : dims) {
        numel *= dim;
    }
    if( numel == 0 ) {
        return static_cast<py::numeric::array>(py::empty_array_astype<T>());
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(), dims.data(), select_npy_type<T>::type);
    if( pvalues != nullptr ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, numel * sizeof(T));
    }
    return static_cast<py::numeric::array>(py::handle<>(pyvalues));
}

template <typename T>
inline py::numeric::array toPyArray(const std::vector<T>& v)
{
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

template <typename T, long unsigned int N>
inline py::numeric::array toPyArray(const std::array<T, N>& v)
{
    return toPyArrayN(v.data(), N);
}

template <typename T, int N>
inline py::numeric::array toPyArray(const boost::array<T, N>& v)
{
    return toPyArrayN(v.data(), N);
}

#endif // OPENRAVE_BINDINGS_PYARRAY

template <typename T>
struct OpenRAVEBoostPythonExceptionTranslator
{
    OpenRAVEBoostPythonExceptionTranslator() {
        py::register_exception_translator<T>(&OpenRAVEBoostPythonExceptionTranslator::translate);

        //Register custom r-value converter
        //There are situations, where we have to pass the exception back to
        //C++ library. This will do the trick
        py::converter::registry::push_back( &OpenRAVEBoostPythonExceptionTranslator::convertible, &OpenRAVEBoostPythonExceptionTranslator::construct, py::type_id<T>() );
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
}; // struct OpenRAVEBoostPythonExceptionTranslator

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

}

#endif
