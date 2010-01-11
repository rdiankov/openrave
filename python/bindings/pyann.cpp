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

#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
//#include <boost/functional/hash.hpp>

#include <ANN/ANN.h>

using namespace boost::python;
using namespace std;

// Constructor from list        TODO: change to iterator
boost::shared_ptr<ANNkd_tree>       init_from_list(boost::python::list lst)
{ 
    int             dimension   = len(lst[0]);
    int             npts        = len(lst);
    ANNpointArray   dataPts     = annAllocPts(npts, dimension);

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
    ANNpointArray   dataPts     = kdtree.thePoints();
    annDeallocPts(dataPts);
}

object search(ANNkd_tree& kdtree, boost::python::list q, int k, double eps, bool priority = false)
{
    ANNpoint annq = annAllocPt(len(q));
    for (int c = 0; c < len(q); ++c)
        annq[c] = extract<ANNcoord>(q[c]);

    ANNidxArray     nn_idx  = new ANNidx[k];
    ANNdistArray    dists   = new ANNdist[k];

    if (priority)
        kdtree.annkPriSearch(annq, k, nn_idx, dists, eps);
    else
        kdtree.annkSearch(annq, k, nn_idx, dists, eps);

    boost::python::list indices, distances;
    for (int i = 0; i < k; ++i) {
        indices.append(nn_idx[i]);
        distances.append(dists[i]);
    }

    delete nn_idx; delete dists;
    return boost::python::make_tuple(indices, distances);
}

object k_fixed_radius_search(ANNkd_tree& kdtree, boost::python::list q, double sqRad, int k, double eps)
{
    ANNpoint annq = annAllocPt(len(q));
    for (int c = 0; c < len(q); ++c)
        annq[c] = extract<ANNcoord>(q[c]);

    ANNidxArray     nn_idx  = new ANNidx[k];
    ANNdistArray    dists   = new ANNdist[k];

    int kball = kdtree.annkFRSearch(annq, sqRad, k, nn_idx, dists, eps);

    boost::python::list indices, distances;
    for (int i = 0; i < k; ++i) {
        if (nn_idx[i] != ANN_NULL_IDX)
            indices.append(nn_idx[i]);
        if (dists[i] != ANN_DIST_INF)
            distances.append(dists[i]);
    }

    delete nn_idx; delete dists;
    return boost::python::make_tuple(indices, distances, kball);
}

object ksearch(ANNkd_tree& kdtree, boost::python::list q, int k, double eps)
{
    return search(kdtree, q, k, eps, false);
}

object k_priority_search(ANNkd_tree& kdtree, boost::python::list q, int k, double eps)
{
    return search(kdtree, q, k, eps, true);
}

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

BOOST_PYTHON_MODULE(pyANN)
{
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_int();
    T_from_number<float>();
    T_from_number<double>();

    class_<ANNkd_tree>("KDTree")
        .def("__init__",            make_constructor(&init_from_list))
        .def("__del__",             &destroy_points)

        .def("kSearch",             &ksearch)
        .def("kPriSearch",          &k_priority_search)
        .def("kFRSearch",           &k_fixed_radius_search)

        .def("__len__",             &ANNkd_tree::nPoints)
        .def("dim",                 &ANNkd_tree::theDim)
    ;

    def("max_pts_visit",        &annMaxPtsVisit);
};
