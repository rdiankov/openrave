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

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"

#include <ANN/ANN.h>

namespace py = boost::python;
namespace numeric = py::numeric;
using py::object;
using py::extract;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::no_init;
using py::bases;
using py::init;
using py::scope;
using py::args;
using py::return_value_policy;
using py::copy_const_reference;
using py::docstring_options;
using py::optional;
using py::def;
using openravepy::int_from_number;
using openravepy::float_from_number;
using openravepy::exception_translator;

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
boost::shared_ptr<ANNkd_tree>       init_from_list(object lst)
{
    BOOST_ASSERT(sizeof(ANNdist)==8 || sizeof(ANNdist)==4);
    BOOST_ASSERT(sizeof(ANNidx)==4);

    int dimension   = len(lst[0]);
    int npts        = len(lst);
    ANNpointArray dataPts     = annAllocPts(npts, dimension);

    // Convert points from Python list to ANNpointArray
    for (int p = 0; p < len(lst); ++p) {
        ANNpoint& pt = dataPts[p];
        for (int c = 0; c < dimension; ++c)
            pt[c] = extract<ANNcoord>(lst[p][c]);
    }

    boost::shared_ptr<ANNkd_tree>   p(new ANNkd_tree(dataPts, npts, dimension));
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
    for (int c = 0; c < kdtree.theDim(); ++c)
        annq.pt[c] = extract<ANNcoord>(q[c]);

    npy_intp dims[] = { k};
    PyObject *pydists = PyArray_SimpleNew(1,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(1,dims, PyArray_INT);
    if( !pyidx )
        Py_DECREF(pydists);
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);

    std::vector<ANNidx> nn_idx(k);
    std::vector<ANNdist> dists(k);

    if (priority)
        kdtree.annkPriSearch(annq.pt, k, pidx, pdists, eps);
    else
        kdtree.annkSearch(annq.pt, k, pidx, pdists, eps);
    return py::make_tuple(static_cast<numeric::array>(handle<>(pyidx)), static_cast<numeric::array>(handle<>(pydists)));
}

object search_array(ANNkd_tree& kdtree, object qarray, int k, double eps, bool priority = false)
{
    BOOST_ASSERT(k <= kdtree.nPoints());
    int N = len(qarray);
    if( N == 0 )
        return py::make_tuple(numeric::array(py::list()).astype("i4"),numeric::array(py::list()));

    BOOST_ASSERT(len(qarray[0])==kdtree.theDim());
    ANNpointManaged annq(kdtree.theDim());
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

    std::vector<ANNdist> dists(k);
    std::vector<ANNidx> nn_idx(k);
    for(int i = 0; i < N; ++i) {
        object q = qarray[i];
        for (int c = 0; c < kdtree.theDim(); ++c)
            annq.pt[c] = extract<ANNcoord>(q[c]);
        if (priority)
            kdtree.annkPriSearch(annq.pt, k, &nn_idx[0], &dists[0], eps);
        else
            kdtree.annkSearch(annq.pt, k, &nn_idx[0], &dists[0], eps);

        std::copy(nn_idx.begin(),nn_idx.end(),pidx); pidx += k;
        std::copy(dists.begin(),dists.end(),pdists); pdists += k;
    }

    return py::make_tuple(static_cast<numeric::array>(handle<>(pyidx)), static_cast<numeric::array>(handle<>(pydists)));
}

object k_fixed_radius_search(ANNkd_tree& kdtree, object q, double sqRad, int k, double eps)
{
    BOOST_ASSERT(k <= kdtree.nPoints() && kdtree.theDim() == len(q));
    ANNpointManaged annq(kdtree.theDim());
    for (int c = 0; c < kdtree.theDim(); ++c)
        annq.pt[c] = extract<ANNcoord>(q[c]);

    if( k <= 0 ) {
        int kball = kdtree.annkFRSearch(annq.pt, sqRad, k, NULL, NULL, eps);
        return py::make_tuple(numeric::array(py::list()).astype("i4"),numeric::array(py::list()),kball);
    }

    std::vector<ANNdist> dists(k);
    std::vector<ANNidx> nn_idx(k);
    int kball = kdtree.annkFRSearch(annq.pt, sqRad, k, &nn_idx[0], &dists[0], eps);
    if( kball <= 0 )
        return py::make_tuple(numeric::array(py::list()).astype("i4"),numeric::array(py::list()),kball);

    npy_intp dims[] = {std::min(k, kball)};
    PyObject *pydists = PyArray_SimpleNew(1,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(1,dims, PyArray_INT);
    if( !pyidx )
        Py_DECREF(pydists);
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);
    int addindex=0;
    for (int i = 0; i < k; ++i) {
        if (nn_idx[i] != ANN_NULL_IDX) {
            pdists[addindex] = dists[i];
            pidx[addindex] = nn_idx[i];
            addindex++;
        }
    }

    BOOST_ASSERT(kball > k || addindex==kball);
    return py::make_tuple(static_cast<numeric::array>(handle<>(pyidx)), static_cast<numeric::array>(handle<>(pydists)),kball);
}

object k_fixed_radius_search_array(ANNkd_tree& kdtree, object qarray, double sqRad, int k, double eps)
{
    BOOST_ASSERT(k <= kdtree.nPoints());
    int N = len(qarray);
    if( N == 0 )
        return py::make_tuple(numeric::array(py::list()).astype("i4"),numeric::array(py::list()),numeric::array(py::list()));

    BOOST_ASSERT(len(qarray[0])==kdtree.theDim());
    ANNpointManaged annq(kdtree.theDim());
    npy_intp dimsball[] = { N};
    PyObject *pykball = PyArray_SimpleNew(1,dimsball, PyArray_INT);
    BOOST_ASSERT(!!pykball);
    int* pkball = (int*)PyArray_DATA(pykball);

    if( k <= 0 ) {
        for(int i = 0; i < N; ++i) {
            object q = qarray[i];
            for (int c = 0; c < kdtree.theDim(); ++c)
                annq.pt[c] = extract<ANNcoord>(q[c]);
            pkball[i] = kdtree.annkFRSearch(annq.pt, sqRad, k, NULL, NULL, eps);
        }
        return py::make_tuple(numeric::array(py::list()).astype("i4"),numeric::array(py::list()),static_cast<numeric::array>(handle<>(pykball)));
    }

    npy_intp dims[] = { N,k};
    PyObject *pydists = PyArray_SimpleNew(2,dims, sizeof(ANNdist)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    if( !pydists )
        Py_DECREF(pykball);
    BOOST_ASSERT(!!pydists);
    PyObject *pyidx = PyArray_SimpleNew(2,dims, PyArray_INT);
    if( !pyidx ) {
        Py_DECREF(pykball);
        Py_DECREF(pydists);
    }
    BOOST_ASSERT(!!pyidx);
    ANNdist* pdists = (ANNdist*)PyArray_DATA(pydists);
    ANNidx* pidx = (ANNidx*)PyArray_DATA(pyidx);

    std::vector<ANNdist> dists(k);
    std::vector<ANNidx> nn_idx(k);
    for(int i = 0; i < N; ++i) {
        object q = qarray[i];
        for (int c = 0; c < kdtree.theDim(); ++c)
            annq.pt[c] = extract<ANNcoord>(q[c]);
        pkball[i] = kdtree.annkFRSearch(annq.pt, sqRad, k, &nn_idx[0], &dists[0], eps);

        std::copy(nn_idx.begin(),nn_idx.end(),pidx); pidx += k;
        std::copy(dists.begin(),dists.end(),pdists); pdists += k;
    }

    return py::make_tuple(static_cast<numeric::array>(handle<>(pyidx)), static_cast<numeric::array>(handle<>(pydists)),static_cast<numeric::array>(handle<>(pykball)));
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

BOOST_PYTHON_MODULE(pyANN_int)
{
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_number<int>();
    float_from_number<float>();
    float_from_number<double>();

    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
    class_< pyann_exception >( "_pyann_exception_" )
    .def( init<const std::string&>() )
    .def( init<const pyann_exception&>() )
    .def( "message", &pyann_exception::message, return_copy_const_ref() )
    .def( "__str__", &pyann_exception::message, return_copy_const_ref() )
    ;
    exception_translator<pyann_exception>();

    class_<ANNkd_tree, boost::shared_ptr<ANNkd_tree> >("KDTree")
    .def("__init__", make_constructor(&init_from_list))
    .def("__del__", &destroy_points)

    .def("kSearch", &ksearch,args("q","k","eps"))
    .def("kSearchArray", &ksearch_array,args("q","k","eps"))
    .def("kPriSearch", &k_priority_search,args("q","k","eps"))
    .def("kPriSearchArray", &k_priority_search_array,args("q","k","eps"))
    .def("kFRSearch", &k_fixed_radius_search,args("q","sqrad","k","eps"))
    .def("kFRSearchArray", &k_fixed_radius_search_array,args("qarray","sqrad","k","eps"))

    .def("__len__",             &ANNkd_tree::nPoints)
    .def("dim",                 &ANNkd_tree::theDim)
    ;

    def("max_pts_visit",        &annMaxPtsVisit);
}
