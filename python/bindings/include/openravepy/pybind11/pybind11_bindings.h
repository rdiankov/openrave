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

template <typename type>
class PyOpenRAVEException : public py::object {
public:
    PyOpenRAVEException() = default;
    PyOpenRAVEException(handle scope, const char *name, PyObject *base = PyExc_Exception, PyObject* dict = NULL) {
        std::string full_name = scope.attr("__name__").cast<std::string>() +
                                std::string(".") + name;
        m_ptr = PyErr_NewException(const_cast<char *>(full_name.c_str()), base, dict);
        if (hasattr(scope, name))
            RAVELOG_WARN("Error during initialization: multiple incompatible "
                         "definitions with name \"" + std::string(name) + "\"");
        scope.attr(name) = *this;
    }

    template <typename Func, typename ... Extra>
    PyOpenRAVEException &def(const char *name_, Func&& f, const Extra& ... extra) {
        py::cpp_function cf(py::method_adaptor<type>(std::forward<Func>(f)), py::name(name_), py::is_method(*this),
                            py::sibling(py::getattr(*this, name_, py::none())), extra ...);
        attr(cf.name()) = cf;
        return *this;
    }

    /// Uses return_value_policy::reference_internal by default
    template <typename Getter, typename ... Extra>
    PyOpenRAVEException &def_property_readonly(const char *name, const Getter &fget, const Extra& ... extra) {
        return def_property_readonly(name, py::cpp_function(py::method_adaptor<type>(fget)),
                                     py::return_value_policy::reference_internal, extra ...);
    }

    /// Uses cpp_function's return_value_policy by default
    template <typename ... Extra>
    PyOpenRAVEException &def_property_readonly(const char *name, const py::cpp_function &fget, const Extra& ... extra) {
        return def_property(name, fget, py::cpp_function(), extra ...);
    }

    /// Uses return_value_policy::reference_internal by default
    template <typename Getter, typename Setter, typename ... Extra>
    PyOpenRAVEException &def_property(const char *name, const Getter &fget, const Setter &fset, const Extra& ... extra) {
        return def_property(name, fget, py::cpp_function(py::method_adaptor<type>(fset)), extra ...);
    }
    template <typename Getter, typename ... Extra>
    PyOpenRAVEException &def_property(const char *name, const Getter &fget, const py::cpp_function &fset, const Extra& ... extra) {
        return def_property(name, py::cpp_function(py::method_adaptor<type>(fget)), fset,
                            py::return_value_policy::reference_internal, extra ...);
    }

    /// Uses cpp_function's return_value_policy by default
    template <typename ... Extra>
    PyOpenRAVEException &def_property(const char *name, const py::cpp_function &fget, const py::cpp_function &fset, const Extra& ... extra) {
        return def_property_static(name, fget, fset, py::is_method(*this), extra ...);
    }

    /// Uses return_value_policy::reference by default
    template <typename Getter, typename ... Extra>
    PyOpenRAVEException &def_property_static(const char *name, const Getter &fget, const py::cpp_function &fset, const Extra& ... extra) {
        return def_property_static(name, py::cpp_function(fget), fset, py::return_value_policy::reference, extra ...);
    }

    /// Uses cpp_function's return_value_policy by default
    template <typename ... Extra>
    PyOpenRAVEException &def_property_static(const char *name, const py::cpp_function &fget, const py::cpp_function &fset, const Extra& ... extra) {
        auto rec_fget = get_function_record(fget), rec_fset = get_function_record(fset);
        char *doc_prev = rec_fget->doc; /* 'extra' field may include a property-specific documentation string */
        py::detail::process_attributes<Extra ...>::init(extra ..., rec_fget);
        if (rec_fget->doc && rec_fget->doc != doc_prev) {
            free(doc_prev);
            rec_fget->doc = strdup(rec_fget->doc);
        }
        if (rec_fset) {
            doc_prev = rec_fset->doc;
            py::detail::process_attributes<Extra ...>::init(extra ..., rec_fset);
            if (rec_fset->doc && rec_fset->doc != doc_prev) {
                free(doc_prev);
                rec_fset->doc = strdup(rec_fset->doc);
            }
        }
        def_property_static_impl(name, fget, fset, rec_fget);
        return *this;
    }

private:
    static py::detail::function_record *get_function_record(handle h) {
        h = py::detail::get_function(h);
        return h ? (py::detail::function_record *) py::reinterpret_borrow<py::capsule>(PyCFunction_GET_SELF(h.ptr()))
               : nullptr;
    }

    void def_property_static_impl(const char *name,
                                  py::handle fget, py::handle fset,
                                  py::detail::function_record *rec_fget) {
        const auto is_static = !(rec_fget->is_method && rec_fget->scope);
        const auto has_doc = rec_fget->doc && pybind11::options::show_user_defined_docstrings();

        auto property = handle((PyObject *) (is_static ? py::detail::get_internals().static_property_type
                                             : &PyProperty_Type));
        attr(name) = property(fget.ptr() ? fget : py::none(),
                              fset.ptr() ? fset : py::none(),
                              /*deleter*/ py::none(),
                              pybind11::str(has_doc ? rec_fget->doc : ""));
    }
};

} // end namespace openravepy

#endif
