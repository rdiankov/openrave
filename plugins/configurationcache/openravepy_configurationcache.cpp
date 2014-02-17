// -*- coding: utf-8 --*
// Copyright (C) 2011-2013 MUJIN Inc. <rosen.diankov@mujin.co.jp>
// Do not distribute this file outside of Mujin
// \author Rosen Diankov
#include "configurationcachetree.h"

#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/docstring_options.hpp>
#include <pyconfig.h>
#include <numpy/arrayobject.h>
#include <openrave/xmlreaders.h>

using namespace OpenRAVE;
using namespace boost::python;

// declared from openravepy_int
namespace openravepy {

Transform ExtractTransform(const object& oraw);
TransformMatrix ExtractTransformMatrix(const object& oraw);
object toPyArray(const TransformMatrix& t);
object toPyArray(const Transform& t);

XMLReadablePtr ExtractXMLReadable(object o);
object toPyXMLReadable(XMLReadablePtr p);
bool ExtractIkParameterization(object o, IkParameterization& ikparam);
object toPyIkParameterization(const IkParameterization& ikparam);
object toPyIkParameterization(const std::string& serializeddata);

object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params);

object toPyEnvironment(object);
object toPyKinBody(KinBodyPtr, object opyenv);
object toPyKinBodyLink(KinBody::LinkPtr plink, object opyenv);

//EnvironmentBasePtr GetEnvironment(boost::python::object);
TrajectoryBasePtr GetTrajectory(object);
KinBodyPtr GetKinBody(object);
KinBody::LinkPtr GetKinBodyLink(object);
RobotBasePtr GetRobot(object o);
RobotBase::ManipulatorPtr GetRobotManipulator(object);
PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(object);

EnvironmentBasePtr GetEnvironment(object o);
}

namespace configurationcachepy {

inline std::string _ExtractStringSafe(boost::python::object ostring)
{
    if( ostring == object() ) {
        return std::string();
    }
    else {
        return extract<std::string>(ostring);
    }
}

inline boost::python::object ConvertStringToUnicode(const std::string& s)
{
    return boost::python::object(boost::python::handle<>(PyUnicode_Decode(s.c_str(),s.size(), "utf-8", NULL)));
}

template <typename T>
inline RaveVector<T> ExtractVector3Type(const object& o)
{
    return RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]));
}

template <typename T>
inline std::vector<T> ExtractArray(const object& o)
{
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = extract<T>(o[i]);
    }
    return v;
}

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
    size_t totalsize = 1;
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
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f8"));
    }
    size_t totalsize = 1;
    FOREACH(it,dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f8"));
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
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u1"));
    }
    size_t totalsize = 1;
    for(size_t i = 0; i < dims.size(); ++i) {
        totalsize *= dims[i];
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u1"));
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
inline numeric::array toPyArray(const std::vector<T>& v)
{
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL,0);
    }
    return toPyArrayN(&v[0],v.size());
}

class PyConfigurationCache
{
public:
    PyConfigurationCache(object pyrobot)
    {
        _cache.reset(new configurationcache::ConfigurationCache(openravepy::GetRobot(pyrobot)));
    }
    virtual ~PyConfigurationCache(){
    }

//    void SetManipulatorBias(object pymanip, object biasdir, dReal nullsampleprob = 0.50, dReal nullbiassampleprob = 0.50, dReal deltasampleprob = 0.30)
//    {
//        _jitterer->SetManipulatorBias(openravepy::GetRobotManipulator(pymanip), ExtractVector3Type<dReal>(biasdir), nullsampleprob, nullbiassampleprob, deltasampleprob);
//    }
//
//    int Jitter(int maxiterations = 5000, dReal maxjitter=0.3, dReal perturbation=1e-5, dReal linkdistthresh=0.05)
//    {
//        return _jitterer->Jitter(maxiterations, maxjitter, perturbation, linkdistthresh);
//    }
//
//    void SetUseNeighborFunction(bool useneighfn)
//    {
//        _jitterer->SetUseNeighborFunction(useneighfn);
//    }
//
//    void SetUseCache(bool usecache)
//    {
//        _jitterer->SetUseCache(usecache);
//    }
//
//    void SetCollisionThresh(dReal colthresh)
//    {
//        _jitterer->SetCollisionThresh(colthresh);
//    }
//
//    dReal GetResultJitterDist()
//    {
//        return _jitterer->GetResultJitterDist();
//    }
//
//    int GetCacheSize()
//    {
//        return _jitterer->GetCacheSize();
//    }

    configurationcache::ConfigurationCachePtr _cache;
};

typedef boost::shared_ptr<PyConfigurationCache> PyConfigurationCachePtr;

} // end namespace configurationcachepy_int

//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetManipulatorBias_overloads, configurationcachepy::PyConfigurationCache::SetManipulatorBias, 2, 5)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetUseNeighborFunction_overloads, configurationcachepy::PyConfigurationCache::SetUseNeighborFunction, 1, 1)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetCollisionThresh_overloads, configurationcachepy::PyConfigurationCache::SetCollisionThresh, 1, 1)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Jitter_overloads, configurationcachepy::PyConfigurationCache::Jitter, 0, 4)

BOOST_PYTHON_MODULE(openravepy_configurationcache)
{
    using namespace configurationcachepy;
    import_array();
    scope().attr("__doc__") = "The module contains configuration cache bindings for openravepy\n";

    class_<PyConfigurationCache, PyConfigurationCachePtr >("ConfigurationCache", no_init)
    .def(init<object>(args("robot")))
    ;
//    .def("SetManipulatorBias",&configurationcachepy::PyConfigurationCache::SetManipulatorBias, SetManipulatorBias_overloads(args("manip","biasdirection", "nullsampleprob", "nulldeltasampleprob", "deltasampleprob")))
//    .def("SetUseNeighborFunction",&configurationcachepy::PyConfigurationCache::SetUseNeighborFunction, SetUseNeighborFunction_overloads(args("useneighfn")))
//    .def("SetUseCache",&configurationcachepy::PyConfigurationCache::SetUseCache)
//    .def("Jitter",&configurationcachepy::PyConfigurationCache::Jitter, Jitter_overloads(args("maxiterations","maxjitter","perturbation","linkdistthresh")))
//    .def("GetResultJitterDist",&configurationcachepy::PyConfigurationCache::GetResultJitterDist)
//    .def("GetCacheSize",&configurationcachepy::PyConfigurationCache::GetCacheSize)
//    .def("SetCollisionThresh",&configurationcachepy::PyConfigurationCache::SetCollisionThresh, SetCollisionThresh_overloads(args("colthresh")));
}
