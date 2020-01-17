/// pybind11 specific bindings
#ifndef OPENRAVE_PYBIND11_BINDINGS_H
#define OPENRAVE_PYBIND11_BINDINGS_H

#define OPENRAVEPY_API __attribute__ ((visibility ("default")))
#include <iostream>
// use std::cout temporarily
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <boost/shared_ptr.hpp>
PYBIND11_DECLARE_HOLDER_TYPE(T, OPENRAVE_SHARED_PTR<T>);
namespace pybind11 {
namespace numeric {
// so py::numeric::array = py::array_t<double>
using array = ::pybind11::array_t<double>;
} // namespace pybind11::numeric
template <typename T>
inline T extract(object o) {
    return o.cast<T>();
}
template <typename T>
inline T extract(handle h) {
    return h.cast<T>();
}
template <typename T>
inline object to_object(const T& t) {
    // https://github.com/pybind/pybind11/issues/1201
    // to_object can either
    // (1) cast a shared pointer to py::object, or
    // (2) cast a Cpp type to py::object,
    // but (x) cannot cast *PyObject to py::object
    return cast(t);
}
inline object handle_to_object(PyObject* pyo) {
    return cast<object>(pyo);
}
template <typename T>
inline object to_array_astype(PyObject* pyo) {
    // do we need to implement the conversion?
    return handle(pyo).cast<array_t<T> >();
}
template <typename T>
inline object empty_array_astype() {
    return array_t<T>({}, nullptr);
}
template <typename T>
struct extract_ {
    extract_() = delete; // disable default constructor
    explicit extract_(const object& o) {
        try {
            _data = extract<T>(o);
        }
        catch(...) {
            _bcheck = false;
            RAVELOG_WARN("Cannot extract type " + std::string(typeid(T).name()) + " from a pybind11::object");
        }
    }
    // user-defined conversion:
    // https://en.cppreference.com/w/cpp/language/cast_operator
    // implicit conversion
    operator T() const { return _data; }
    // explicit conversion
    explicit operator T*() const {
        return _data;
    }
    bool check() const {
        return _bcheck;
    }
    bool _bcheck = true;
    T _data;
}; // struct extract_
using scope_ = object;
inline object none_() {
    return none();
}
using array_int = array_t<int>; // py::array_int
} // namespace pybind11
#define OPENRAVE_PYTHON_MODULE(X) PYBIND11_MODULE(X, m)
#include <openravepy/map.h>
#define PY_ARG_(x) py ::arg(x),
#define PY_ARGS(...) MAP(PY_ARG_, __VA_ARGS__)

// is_none is not supported by older versions of python
#define IS_PYTHONOBJECT_NONE(o) (o).is_none()

namespace openravepy
{
namespace py = pybind11;

inline py::object ConvertStringToUnicode(const std::string& s)
{
    /*
       TGN: Is the following an alternative?
       ```
       PyObject *pyo = PyUnicode_Decode(s.c_str(), s.size(), "utf-8", nullptr);
       return py::cast<py::object>(pyo); // py::handle_to_object(pyo);
       ```
     */
    return py::cast(s);
}

#ifdef OPENRAVE_BINDINGS_PYARRAY

template <typename T>
inline py::array_t<T> toPyArrayN(const T* pvalues, const size_t N)
{
    // one-dimension numpy array
    std::vector<npy_intp> dims {(long int)N};
    return py::array_t<T>(dims, pvalues); 
}

template <typename T>
inline py::array_t<T> toPyArrayN(const T* pvalues, std::vector<npy_intp>& dims)
{
    // n-dimension numpy array
    return py::array_t<T>(dims, pvalues); 
}

template <typename T>
inline py::array_t<T> toPyArray(const std::vector<T>& v)
{
    return toPyArrayN(v.data(), v.size());
}

template <typename T>
inline py::array_t<T> toPyArray(const std::vector<T>& v, std::vector<npy_intp>& dims)
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
inline py::array_t<T> toPyArray(const boost::array<T, N>& v)
{
    return toPyArrayN(v.data(), N);
}

#endif // OPENRAVE_BINDINGS_PYARRAY

} // end namespace openravepy

#endif
