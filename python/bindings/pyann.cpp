// Copyright (C) 2010 Dmitriy Morozov, modifications by Rosen Diankov
//
// pyANN is free software: you can redistribute it and/or modify
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

#include <ANN/ANN.h>

// to-do: put all backward compatibility related stuff into the same header
#ifdef USE_PYBIND11_PYTHON_BINDINGS
namespace py = pybind11;
#else
namespace py = boost::python;
#endif // USE_PYBIND11_PYTHON_BINDINGS
using py::object;
using py::extract;
using py::extract_;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope;
using py::args;
using py::len;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
#endif // USE_PYBIND11_PYTHON_BINDINGS

namespace numeric = py::numeric;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using openravepy::int_from_number;
using openravepy::float_from_number;
using openravepy::OpenRAVEBoostPythonExceptionTranslator;
#endif // USE_PYBIND11_PYTHON_BINDINGS

struct OPENRAVE_API pyann_exception : std::exception
{
    pyann_exception() : std::exception(), _s("unknown exception") {
    }
    pyann_exception(const std::string& s) : std::exception() {
        _s = "pyANN: " + s;
    }
    virtual ~pyann_exception() throw() {
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
    throw pyann_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr));
}
#if BOOST_VERSION>104600
inline void assertion_failed_msg(char const * expr, char const * msg, char const * function, char const * file, long line)
{
    throw pyann_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s, msg: %s")%file%line%function%expr%msg));
}
#endif
}
#endif

class ANNpointManaged
{
public:
    ANNpointManaged(int n) {
        pt = annAllocPt(n);
    }
    virtual ~ANNpointManaged() {
        annDeallocPt(pt);
    }
    ANNpoint pt;
};

// Constructor from list        TODO: change to iterator
OPENRAVE_SHARED_PTR<ANNkd_tree>       init_from_list(object lst)
{
    BOOST_ASSERT(sizeof(ANNdist)==8 || sizeof(ANNdist)==4);
    BOOST_ASSERT(sizeof(ANNidx)==4);

    const int dimension   = len(lst[0]);
    const int npts        = len(lst);
    ANNpointArray dataPts = annAllocPts(npts, dimension);

    // Convert points from Python list to ANNpointArray
    for (int p = 0; p < npts; ++p) {
        ANNpoint& pt = dataPts[p];
        for (int c = 0; c < dimension; ++c) {
            pt[c] = extract<ANNcoord>(lst[p][c]);
        }
    }

    OPENRAVE_SHARED_PTR<ANNkd_tree>   p(new ANNkd_tree(dataPts, npts, dimension));
    return p;
}

void destroy_points(ANNkd_tree& kdtree)
{
    ANNpointArray dataPts     = kdtree.thePoints();
    annDeallocPts(dataPts);
}

object search(ANNkd_tree& kdtree, object q, int k, double eps, bool priority = false)
{
    BOOST_ASSERT(k <= kdtree.nPoints() && kdtree.theDim() == len(q));
    ANNpointManaged annq(kdtree.theDim());
    for (int c = 0; c < kdtree.theDim(); ++c) {
        annq.pt[c] = extract<ANNcoord>(q[c]);
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // distance
    py::array_t<ANNdist> pydists({k});
    py::buffer_info bufdists = pydists.request();
    ANNdist* pdists = (ANNdist*) bufdists.ptr;

    // index
    py::array_t<ANNidx> pyidx({k});
    py::buffer_info bufidx = pyidx.request();
    ANNidx* pidx = (ANNidx*) bufidx.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { k};
    PyObject *pydists = PyArray_SimpleNew(1,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(1,dims, PyArray_INT);
    if( !pyidx ) {
        Py_DECREF(pydists);
    }
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);
#endif // USE_PYBIND11_PYTHON_BINDINGS

    if (priority) {
        kdtree.annkPriSearch(annq.pt, k, pidx, pdists, eps);
    }
    else {
        kdtree.annkSearch(annq.pt, k, pidx, pdists, eps);
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(pyidx, pydists);
#else // USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(py::to_array_astype<int>(pyidx), py::to_array_astype<ANNdist>(pydists));
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object search_array(ANNkd_tree& kdtree, object qarray, int k, double eps, bool priority = false)
{
    BOOST_ASSERT(k <= kdtree.nPoints());
    const int N = len(qarray);
    if( N == 0 ) {
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<ANNdist>());
    }

    BOOST_ASSERT(len(qarray[0])==kdtree.theDim());
    ANNpointManaged annq(kdtree.theDim());

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // distance
    py::array_t<ANNdist> pydists({N, k});
    py::buffer_info bufdists = pydists.request();
    ANNdist* pdists = (ANNdist*) bufdists.ptr;

    // index
    py::array_t<ANNidx> pyidx({N, k});
    py::buffer_info bufidx = pyidx.request();
    ANNidx* pidx = (ANNidx*) bufidx.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { N,k};
    PyObject *pydists = PyArray_SimpleNew(2,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(2,dims, PyArray_INT);
    if( !pyidx ) {
        Py_DECREF(pydists);
    }
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);
#endif // USE_PYBIND11_PYTHON_BINDINGS

    std::vector<ANNdist> dists(k);
    std::vector<ANNidx> nn_idx(k);
    for(int i = 0; i < N; ++i) {
        object q = qarray[i];
        for (int c = 0; c < kdtree.theDim(); ++c) {
            annq.pt[c] = extract<ANNcoord>(q[c]);
        }
        if (priority) {
            kdtree.annkPriSearch(annq.pt, k, nn_idx.data(), dists.data(), eps);
        }
        else {
            kdtree.annkSearch(annq.pt, k, nn_idx.data(), dists.data(), eps);
        }

        std::copy(nn_idx.begin(),nn_idx.end(),pidx); pidx += k;
        std::copy(dists.begin(),dists.end(),pdists); pdists += k;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(pyidx, pydists);
#else // USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(py::to_array_astype<int>(pyidx), py::to_array_astype<ANNdist>(pydists));
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object k_fixed_radius_search(ANNkd_tree& kdtree, object q, double sqRad, int k, double eps)
{
    BOOST_ASSERT(k <= kdtree.nPoints() && kdtree.theDim() == len(q));
    ANNpointManaged annq(kdtree.theDim());
    for (int c = 0; c < kdtree.theDim(); ++c) {
        annq.pt[c] = extract<ANNcoord>(q[c]);
    }

    if( k <= 0 ) {
        const int kball = kdtree.annkFRSearch(annq.pt, sqRad, k, NULL, NULL, eps);
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<ANNdist>(), kball);
    }

    std::vector<ANNdist> dists(k);
    std::vector<ANNidx> nn_idx(k);
    const int kball = kdtree.annkFRSearch(annq.pt, sqRad, k, nn_idx.data(), dists.data(), eps);
    if( kball <= 0 ) {
        return py::make_tuple(py::empty_array_astype<int>(),py::empty_array_astype<ANNdist>(),kball);
    }
    
    const int numel = std::min(k, kball);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // distance
    py::array_t<ANNdist> pydists({numel});
    py::buffer_info bufdists = pydists.request();
    ANNdist* pdists = (ANNdist*) bufdists.ptr;

    // index
    py::array_t<ANNidx> pyidx({numel});
    py::buffer_info bufidx = pyidx.request();
    ANNidx* pidx = (ANNidx*) bufidx.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS    
    npy_intp dims[] = {numel};
    PyObject *pydists = PyArray_SimpleNew(1,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(1,dims, PyArray_INT);
    if( !pyidx ) {
        Py_DECREF(pydists);
    }
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    int addindex=0;
    for (int i = 0; i < k; ++i) {
        if (nn_idx[i] != ANN_NULL_IDX) {
            pdists[addindex] = dists[i];
            pidx[addindex] = nn_idx[i];
            addindex++;
        }
    }

    BOOST_ASSERT(kball > k || addindex==kball);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(pyidx, pydists, kball);
#else // USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(py::to_array_astype<int>(pyidx), py::to_array_astype<ANNdist>(pydists), kball);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object k_fixed_radius_search_array(ANNkd_tree& kdtree, object qarray, double sqRad, int k, double eps)
{
    BOOST_ASSERT(k <= kdtree.nPoints());
    const int N = len(qarray);
    if( N == 0 ) {
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<ANNdist>(), py::empty_array_astype<int>());
    }

    BOOST_ASSERT(len(qarray[0])==kdtree.theDim());
    ANNpointManaged annq(kdtree.theDim());
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<int> pykball({N});
    py::buffer_info bufkball = pykball.request();
    int* pkball = (int*) bufkball.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dimsball[] = { N};
    PyObject *pykball = PyArray_SimpleNew(1,dimsball, PyArray_INT);
    BOOST_ASSERT(!!pykball);
    int* pkball = (int*)PyArray_DATA(pykball);
#endif // USE_PYBIND11_PYTHON_BINDINGS

    if( k <= 0 ) {
        for(int i = 0; i < N; ++i) {
            object q = qarray[i];
            for (int c = 0; c < kdtree.theDim(); ++c) {
                annq.pt[c] = extract<ANNcoord>(q[c]);
            }
            pkball[i] = kdtree.annkFRSearch(annq.pt, sqRad, k, NULL, NULL, eps);
        }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<ANNdist>(), pykball);
#else
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<ANNdist>(), py::to_array_astype<int>(pykball));
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // distance
    py::array_t<ANNdist> pydists({N, k});
    py::buffer_info bufdists = pydists.request();
    ANNdist* pdists = (ANNdist*) bufdists.ptr;

    // index
    py::array_t<ANNidx> pyidx({N, k});
    py::buffer_info bufidx = pyidx.request();
    ANNidx* pidx = (ANNidx*) bufidx.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { N,k};
    PyObject *pydists = PyArray_SimpleNew(2,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    if( !pydists ) {
        Py_DECREF(pykball);
    }
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(2,dims, PyArray_INT);
    if( !pyidx ) {
        Py_DECREF(pykball);
        Py_DECREF(pydists);
    }
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);
#endif // USE_PYBIND11_PYTHON_BINDINGS

    std::vector<ANNdist> dists(k);
    std::vector<ANNidx> nn_idx(k);
    for(int i = 0; i < N; ++i) {
        object q = qarray[i];
        for (int c = 0; c < kdtree.theDim(); ++c) {
            annq.pt[c] = extract<ANNcoord>(q[c]);
        }
        pkball[i] = kdtree.annkFRSearch(annq.pt, sqRad, k, nn_idx.data(), dists.data(), eps);

        std::copy(nn_idx.begin(),nn_idx.end(),pidx); pidx += k;
        std::copy(dists.begin(),dists.end(),pdists); pdists += k;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(pyidx, pydists, pykball);
#else // USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(py::to_array_astype<int>(pyidx), py::to_array_astype<ANNdist>(pydists), py::to_array_astype<int>(pykball));
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object ksearch(ANNkd_tree& kdtree, object q, int k, double eps)
{
    return search(kdtree, q, k, eps, false);
}

object ksearch_array(ANNkd_tree& kdtree, object q, int k, double eps)
{
    return search_array(kdtree, q, k, eps, false);
}

object k_priority_search(ANNkd_tree& kdtree, object q, int k, double eps)
{
    return search(kdtree, q, k, eps, true);
}

object k_priority_search_array(ANNkd_tree& kdtree, object q, int k, double eps)
{
    return search_array(kdtree, q, k, eps, true);
}

OPENRAVE_PYTHON_MODULE(pyANN_int)
{
    import_array();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
#else
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_number<int>();
    float_from_number<float>();
    float_from_number<double>();
    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::register_exception<pyann_exception>(m, "_pyann_exception_");
#else
    class_< pyann_exception >( "_pyann_exception_" )
    .def( init<const std::string&>() )
    .def( init<const pyann_exception&>() )
    .def( "message", &pyann_exception::message, return_copy_const_ref() )
    .def( "__str__", &pyann_exception::message, return_copy_const_ref() )
    ;
    OpenRAVEBoostPythonExceptionTranslator<pyann_exception>();
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<ANNkd_tree, OPENRAVE_SHARED_PTR<ANNkd_tree> >(m, "KDTree")
    .def(init<int, int, int>(), "n"_a = 0, "dd"_a = 0, "bs"_a = 1)
    // ANNpointArray is double**; pybind11 does not know how to convert, unless it is smart_ptr
    // .def(init<ANNpointArray, int, int, int, ANNsplitRule>(), "pa"_a, "n"_a, "dd"_a, "bs"_a = 1, "split"_a = ANN_KD_SUGGEST)
    // https://pybind11.readthedocs.io/en/stable/advanced/classes.html#custom-constructors
    .def( init<>(&init_from_list))
#else
    class_<ANNkd_tree, OPENRAVE_SHARED_PTR<ANNkd_tree> >("KDTree")
    .def("__init__", make_constructor(&init_from_list))
#endif
    .def("__del__", &destroy_points)

    .def("kSearch", &ksearch, PY_ARGS("q", "k", "eps") "Doc of kSearch")
    .def("kSearchArray", &ksearch_array, PY_ARGS("q", "k", "eps") "Doc of kSearchArray")
    .def("kPriSearch", &k_priority_search, PY_ARGS("q","k","eps") "Doc of kPriSearch")
    .def("kPriSearchArray", &k_priority_search_array, PY_ARGS("q", "k", "eps") "Doc of kPriSearchArray")
    .def("kFRSearch", &k_fixed_radius_search, PY_ARGS("q", "sqrad", "k", "eps") "Doc of kFRSearch")
    .def("kFRSearchArray", &k_fixed_radius_search_array, PY_ARGS("qarray", "sqrad", "k", "eps") "Doc of kFRSearchArray")

    .def("__len__",             &ANNkd_tree::nPoints)
    .def("dim",                 &ANNkd_tree::theDim)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("max_pts_visit",        &annMaxPtsVisit);
#else
    def("max_pts_visit",        &annMaxPtsVisit);
#endif
}
