// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openravepy/openravepy_int.h>

#include <openrave/utils.h>
#include <boost/thread/once.hpp>
#include <boost/scoped_ptr.hpp>

#include <openravepy/openravepy_kinbody.h>
#include <openravepy/openravepy_collisioncheckerbase.h>
#include <openravepy/openravepy_collisionreport.h>
#include <openravepy/openravepy_robotbase.h>
#include <openravepy/openravepy_sensorbase.h>
#include <openravepy/openravepy_module.h>
#include <openravepy/openravepy_physicalenginebase.h>
#include <openravepy/openravepy_environmentbase.h>

#define OPENRAVE_EXCEPTION_CLASS_NAME "_OpenRAVEException"

namespace openravepy
{
using py::object;
using py::extract;
using py::extract_;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope_; // py::object if USE_PYBIND11_PYTHON_BINDINGS
using py::args;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::optional;
using py::def;
using py::scope;
#endif // USE_PYBIND11_PYTHON_BINDINGS

namespace numeric = py::numeric;

// convert from rapidjson to python object
object toPyObject(const rapidjson::Value& value)
{
    switch (value.GetType()) {
    case rapidjson::kObjectType:
    {
        py::dict d;
        for (rapidjson::Value::ConstMemberIterator it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
            d[it->name.GetString()] = toPyObject(it->value);
        }
        return d;
    }
    case rapidjson::kArrayType:
    {
        py::list l;
        for (rapidjson::Value::ConstValueIterator it = value.Begin(); it != value.End(); ++it) {
            l.append(toPyObject(*it));
        }
        return l;
    }
    case rapidjson::kTrueType: {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        return py::bool_(true);
#else
        return py::to_object(py::handle<>(PyBool_FromLong(1)));
#endif
    }
    case rapidjson::kFalseType: {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        return py::bool_(false);
#else
        return py::to_object(py::handle<>(PyBool_FromLong(0)));
#endif
    }
    case rapidjson::kStringType: {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        return py::str(value.GetString());
#else
        return ConvertStringToUnicode(value.GetString());
#endif
    }
    case rapidjson::kNumberType: {
        if (value.IsDouble()) {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            return py::float_(value.GetDouble());
#else
            return py::to_object(py::handle<>(PyFloat_FromDouble(value.GetDouble())));
#endif
        }
        else {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            return py::int_(value.GetInt64());
#else
            return py::to_object(py::handle<>(PyInt_FromLong(value.GetInt64())));
#endif
        }
    }
    case rapidjson::kNullType: {
        return py::none_();
    }
    default: {
        PyErr_SetString(PyExc_RuntimeError, "unsupported type");
        return py::none_();
    }
    }
}

#define FILL_RAPIDJSON_FROMARRAY_1D(pyarrayvalues, T, rapidjsonsetfn) {  \
        const T *vdata = reinterpret_cast<T*>(PyArray_DATA(pyarrayvalues)); \
        for (int i = 0; i < dims[0]; i++) { \
            rapidjson::Value elementValue; \
            elementValue.rapidjsonsetfn(vdata[i]); \
            value.PushBack(elementValue, allocator); \
        } \
} \

#define FILL_RAPIDJSON_FROMARRAY_2D(pyarrayvalues, T, rapidjsonsetfn) {                            \
        const T *vdata = reinterpret_cast<T*>(PyArray_DATA(pyarrayvalues)); \
        for (int i = 0; i < dims[0]; i++) { \
            rapidjson::Value colvalues(rapidjson::kArrayType); \
            for (int j = 0; j < dims[1]; j++) { \
                rapidjson::Value elementValue; \
                elementValue.rapidjsonsetfn(vdata[i*dims[0]+j]); \
                colvalues.PushBack(elementValue, allocator); \
            } \
            value.PushBack(colvalues, allocator); \
        } \
} \

#define FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, T, rapidjsonsetfn, ndims) { \
        if( ndims == 1 ) { \
            FILL_RAPIDJSON_FROMARRAY_1D(pyarrayvalues, T, rapidjsonsetfn); \
        } \
        else if( ndims == 2 ) { \
            FILL_RAPIDJSON_FROMARRAY_2D(pyarrayvalues, T, rapidjsonsetfn); \
        } \
        else { \
            throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object with %d dims"), ndims, ORE_InvalidArguments); \
        } \
}

// convert from python object to rapidjson
void toRapidJSONValue(const object &obj, rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    if (obj.is_none())
#else
    if (obj.ptr() == Py_None)
#endif
    {
        value.SetNull();
    }
    else if (PyBool_Check(obj.ptr()))
    {
        value.SetBool(obj.ptr() == Py_True);
    }
    else if (PyFloat_Check(obj.ptr()))
    {
        value.SetDouble(PyFloat_AsDouble(obj.ptr()));
    }
    else if (PyInt_Check(obj.ptr()))
    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        value.SetInt64(PyInt_AsLong(obj.ptr()));
#else
        value.SetInt64(PyLong_AsLong(obj.ptr()));
#endif
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    else if (PyLong_Check(obj.ptr()))
    {
        value.SetInt64(PyLong_AsLong(obj.ptr()));
    }
#endif
    else if (PyString_Check(obj.ptr()))
    {
        value.SetString(PyString_AsString(obj.ptr()), PyString_GET_SIZE(obj.ptr()));
    }
    else if (PyUnicode_Check(obj.ptr()))
    {
        value.SetString(PyBytes_AsString(obj.ptr()), PyBytes_GET_SIZE(obj.ptr()));
    }
    else if (PyTuple_Check(obj.ptr()))
    {
        py::tuple t = py::extract<py::tuple>(obj);
        value.SetArray();
        for (size_t i = 0; i < (size_t)len(t); i++)
        {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            rapidjson::Value elementValue;
            toRapidJSONValue(t[i], elementValue, allocator);
            value.PushBack(elementValue, allocator);
#else
            object o = py::extract<object>(t[i]);
            rapidjson::Value elementValue;
            toRapidJSONValue(o, elementValue, allocator);
            value.PushBack(elementValue, allocator);
#endif // USE_PYBIND11_PYTHON_BINDINGS
        }
    }
    else if (PyList_Check(obj.ptr()))
    {
        py::list l = py::extract<py::list>(obj);
        value.SetArray();
        const int numitems = len(l);
        for (int i = 0; i < numitems; i++) {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            rapidjson::Value elementValue;
            toRapidJSONValue(l[i], elementValue, allocator);
            value.PushBack(elementValue, allocator);
#else
            object o = py::extract<object>(l[i]);
            rapidjson::Value elementValue;
            toRapidJSONValue(o, elementValue, allocator);
            value.PushBack(elementValue, allocator);
#endif // USE_PYBIND11_PYTHON_BINDINGS
        }
    }
    else if (PyDict_Check(obj.ptr()))
    {
        py::dict d = py::extract<py::dict>(obj);
        value.SetObject();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        for (const std::pair<py::handle, py::handle>& item : d) {
            rapidjson::Value k, v;
            toRapidJSONValue(py::reinterpret_borrow<py::object>(item.first), k, allocator);
            toRapidJSONValue(py::reinterpret_borrow<py::object>(item.second), v, allocator);
            value.AddMember(k, v, allocator);
        }
#else
        object iterator = d.iteritems();
        const int numitems = len(d);
        for (int i = 0; i < numitems; i++)
        {
            py::tuple kv = py::extract<py::tuple>(iterator.attr("next")());
            rapidjson::Value keyValue, valueValue;
            {
                object k = py::extract<object>(kv[0]);
                toRapidJSONValue(k, keyValue, allocator);
            }
            {
                object v = py::extract<object>(kv[1]);
                toRapidJSONValue(v, valueValue, allocator);
            }
            value.AddMember(keyValue, valueValue, allocator);
        }
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }
    else if (PyArray_Check(obj.ptr()) ) {
        PyArrayObject* pyarrayvalues = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(obj.ptr()));
        AutoPyArrayObjectDereferencer pydecref(pyarrayvalues);

        int ndims = PyArray_NDIM(pyarrayvalues);
        npy_intp* dims = PyArray_DIMS(pyarrayvalues);
        const size_t typeSize = PyArray_ITEMSIZE(pyarrayvalues);
        value.SetArray();
        if( ndims > 0 ) {
            if (PyArray_ISFLOAT(pyarrayvalues) ) {
                if( typeSize == sizeof(float)) {
                    FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, float, SetFloat, ndims);
                }
                else if( typeSize == sizeof(double)) {
                    FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, double, SetDouble, ndims);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object float with %d type size"), typeSize, ORE_InvalidArguments);
                }
            }
            else if (PyArray_ISINTEGER(pyarrayvalues) ) {
                if( typeSize == sizeof(int) ) {
                    if( PyArray_ISSIGNED(pyarrayvalues) ) {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, int, SetInt, ndims);
                    }
                    else {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, uint32_t, SetUint, ndims);
                    }
                }
                else if( typeSize == sizeof(int64_t) ) {
                    if( PyArray_ISSIGNED(pyarrayvalues) ) {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, uint64_t, SetInt64, ndims);
                    }
                    else {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, uint64_t, SetUint64, ndims);
                    }
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object integer with %d type size"), typeSize, ORE_InvalidArguments);
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object with %d type size"), typeSize, ORE_InvalidArguments);
            }
        }
    }
    else
    {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("unsupported python type"), ORE_InvalidArguments);
    }
}

/// if set, will return all transforms are 1x7 vectors where first 4 compoonents are quaternion
static bool s_bReturnTransformQuaternions = false;
bool GetReturnTransformQuaternions() {
    return s_bReturnTransformQuaternions;
}
void SetReturnTransformQuaternions(bool bset) {
    s_bReturnTransformQuaternions = bset;
}

Transform ExtractTransform(const object& oraw)
{
    return ExtractTransformType<dReal>(oraw);
}

TransformMatrix ExtractTransformMatrix(const object& oraw)
{
    return ExtractTransformMatrixType<dReal>(oraw);
}

object toPyArray(const TransformMatrix& t)
{
    npy_intp dims[] = { 4,4};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2]; pdata[3] = t.trans.x;
    pdata[4] = t.m[4]; pdata[5] = t.m[5]; pdata[6] = t.m[6]; pdata[7] = t.trans.y;
    pdata[8] = t.m[8]; pdata[9] = t.m[9]; pdata[10] = t.m[10]; pdata[11] = t.trans.z;
    pdata[12] = 0; pdata[13] = 0; pdata[14] = 0; pdata[15] = 1;
    return py::to_array_astype<dReal>(pyvalues);
}


object toPyArray(const Transform& t)
{
    npy_intp dims[] = { 7};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.rot.x; pdata[1] = t.rot.y; pdata[2] = t.rot.z; pdata[3] = t.rot.w;
    pdata[4] = t.trans.x; pdata[5] = t.trans.y; pdata[6] = t.trans.z;
    return py::to_array_astype<dReal>(pyvalues);
}

object toPyArray(const std::vector<KinBody::GeometryInfoPtr>& infos)
{
    py::list pyvalues;
    for(size_t i = 0; i < infos.size(); ++i) {
        pyvalues.append(toPyGeometryInfo(*infos[i]));
    }
    return pyvalues;
}


AttributesList toAttributesList(py::dict odict)
{
    AttributesList atts;
    if( !IS_PYTHONOBJECT_NONE(odict) ) {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        for (const std::pair<py::handle, py::handle>& item : odict)
#else
        py::list iterkeys = (py::list)odict.iterkeys();
        const size_t num = py::len(iterkeys);
        for (size_t i = 0; i < num; i++)
#endif
        {
            // Because we know they're strings, we can do this
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            std::string key = py::extract<std::string>(item.first);
            std::string value = py::extract<std::string>(item.second);
#else
            std::string key = py::extract<std::string>(iterkeys[i]);
            std::string value = py::extract<std::string>(odict[iterkeys[i]]);
#endif
            atts.emplace_back(key, value);
        }
    }
    return atts;
}

AttributesList toAttributesList(py::list oattributes)
{
    AttributesList atts;
    if( !IS_PYTHONOBJECT_NONE(oattributes) ) {
        size_t num=len(oattributes);
        for (size_t i = 0; i < num; i++) {
            // Because we know they're strings, we can do this
            std::string key = py::extract<std::string>(oattributes[i][0]);
            std::string value = py::extract<std::string>(oattributes[i][1]);
            atts.emplace_back(key, value);
        }
    }
    return atts;
}

AttributesList toAttributesList(object oattributes)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    if( PyDict_Check(oattributes.ptr()) ) {
        // assume dict
        py::dict d = py::extract<py::dict>(oattributes);
        return toAttributesList(d);
    }
    else if (PyList_Check(oattributes.ptr())) {
        // assume list
        py::list l = py::extract<py::list>(oattributes);
        return toAttributesList(l);
    }
#else
    if( !IS_PYTHONOBJECT_NONE(oattributes) ) {
        // assume dict
        py::extract_<py::dict> odictextractor(oattributes);
        if( odictextractor.check() ) {
            return toAttributesList((py::dict)odictextractor());
        }
        // assume list
        py::extract_<py::list> olistextractor(oattributes);
        return toAttributesList((py::list)olistextractor());
    }
#endif // USE_PYBIND11_PYTHON_BINDINGS
    return AttributesList();
}

/// \brief manages all the viewers created through SetViewer into a single thread
class ViewerManager
{
    /// \brief info about the viewer to create or that is created
    struct ViewerInfo
    {
        EnvironmentBasePtr _penv;
        std::string _viewername;
        ViewerBasePtr _pviewer; /// the created viewer
        boost::condition _cond;  ///< notify when viewer thread is done processing and has initialized _pviewer
        bool _bShowViewer; ///< true if should show the viewer when initially created
    };
    typedef OPENRAVE_SHARED_PTR<ViewerInfo> ViewerInfoPtr;
public:
    ViewerManager() {
        _bShutdown = false;
        _bInMain = false;
        _threadviewer.reset(new boost::thread(boost::bind(&ViewerManager::_RunViewerThread, this)));
    }

    virtual ~ViewerManager() {
        Destroy();
    }

    static ViewerManager& GetInstance() {
        boost::call_once(_InitializeSingleton, _onceInitialize);
        // Return reference to object.
        return *_singleton;
    }

    /// \brief adds a viewer to the environment whose GUI thread will be managed by _RunViewerThread
    ///
    /// \param bDoNotAddIfExists if true, will not add a viewer if one already exists and is added to the manager
    ViewerBasePtr AddViewer(EnvironmentBasePtr penv, const string &strviewer, bool bShowViewer, bool bDoNotAddIfExists=true)
    {
        ViewerBasePtr pviewer;
        if( strviewer.size() > 0 ) {

            if( bDoNotAddIfExists ) {
                // check all existing viewers
                boost::mutex::scoped_lock lock(_mutexViewer);
                std::list<ViewerInfoPtr>::iterator itviewer = _listviewerinfos.begin();
                while(itviewer != _listviewerinfos.end() ) {
                    if( (*itviewer)->_penv == penv ) {
                        if( (*itviewer)->_viewername == strviewer ) {
                            if( !!(*itviewer)->_pviewer ) {
                                (*itviewer)->_pviewer->Show(bShowViewer);
                            }
                            return (*itviewer)->_pviewer;
                        }

                        // should remove the viewer so can re-add a new one
                        if( !!(*itviewer)->_pviewer ) {
                            (*itviewer)->_penv->Remove((*itviewer)->_pviewer);
                        }
                        itviewer = _listviewerinfos.erase(itviewer);
                    }
                    else {
                        ++itviewer;
                    }
                }
            }

            ViewerInfoPtr pinfo(new ViewerInfo());
            pinfo->_penv = penv;
            pinfo->_viewername = strviewer;
            pinfo->_bShowViewer = bShowViewer;
            if( _bInMain ) {
                // create in this thread since viewer thread is already waiting on another viewer
                pviewer = RaveCreateViewer(penv, strviewer);
                if( !!pviewer ) {
                    penv->AddViewer(pviewer);
                    // TODO uncomment once Show posts to queue
                    if( bShowViewer ) {
                        pviewer->Show(1);
                    }
                    pinfo->_pviewer = pviewer;
                    boost::mutex::scoped_lock lock(_mutexViewer);
                    _listviewerinfos.push_back(pinfo);
                    _conditionViewer.notify_all();
                }
            }
            else {
                // no viewer has been created yet, so let the viewer thread create it (if using Qt, this initializes the QApplication in the right thread
                boost::mutex::scoped_lock lock(_mutexViewer);
                _listviewerinfos.push_back(pinfo);
                _conditionViewer.notify_all();

                /// wait until viewer thread process it
                pinfo->_cond.wait(_mutexViewer);
                pviewer = pinfo->_pviewer;
            }
        }
        return pviewer;
    }

    /// \brief if removed, returns true
    bool RemoveViewer(ViewerBasePtr pviewer)
    {
        if( !pviewer ) {
            return false;
        }
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            FOREACH(itviewer, _listviewerinfos) {
                ViewerBasePtr ptestviewer = (*itviewer)->_pviewer;
                if(ptestviewer == pviewer ) {
                    pviewer->quitmainloop();
                    _listviewerinfos.erase(itviewer);
                    return true;
                }
            }
        }
        return false;
    }

    /// \brief if anything removed, returns true
    bool RemoveViewersOfEnvironment(EnvironmentBasePtr penv)
    {
        if( !penv ) {
            return false;
        }
        bool bremoved = false;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            std::list<ViewerInfoPtr>::iterator itinfo = _listviewerinfos.begin();
            while(itinfo != _listviewerinfos.end() ) {
                if( (*itinfo)->_penv == penv ) {
                    itinfo = _listviewerinfos.erase(itinfo);
                    bremoved = true;
                }
                else {
                    ++itinfo;
                }
            }
        }
        return bremoved;
    }

    void Destroy() {
        _bShutdown = true;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            // have to notify everyone
            FOREACH(itinfo, _listviewerinfos) {
                (*itinfo)->_cond.notify_all();
            }
            _listviewerinfos.clear();
            _conditionViewer.notify_all();
        }
        if( !!_threadviewer ) {
            _threadviewer->join();
        }
        _threadviewer.reset();
    }

protected:
    void _RunViewerThread()
    {
        while(!_bShutdown) {
            std::list<ViewerBasePtr> listviewers, listtempviewers;
            bool bShowViewer = true;
            {
                boost::mutex::scoped_lock lock(_mutexViewer);
                if( _listviewerinfos.size() == 0 ) {
                    _conditionViewer.wait(lock);
                    if( _listviewerinfos.size() == 0 ) {
                        continue;
                    }
                }

                listtempviewers.clear(); // viewers to add to env once lock is released
                listviewers.clear();
                std::list<ViewerInfoPtr>::iterator itinfo = _listviewerinfos.begin();
                while(itinfo != _listviewerinfos.end() ) {
                    ViewerInfoPtr pinfo = *itinfo;
                    if( !pinfo->_pviewer ) {
                        pinfo->_pviewer = RaveCreateViewer(pinfo->_penv, pinfo->_viewername);
                        // have to notify other thread that viewer is present before the environment lock happens! otherwise we can get into deadlock between c++ and python
                        pinfo->_cond.notify_all();
                        if( !!pinfo->_pviewer ) {
                            listtempviewers.push_back(pinfo->_pviewer);
                            ++itinfo;
                        }
                        else {
                            // erase from _listviewerinfos
                            itinfo = _listviewerinfos.erase(itinfo);
                        }
                    }
                    else {
                        ++itinfo;
                    }

                    if( !!pinfo->_pviewer ) {
                        if( listviewers.size() == 0 ) {
                            bShowViewer = pinfo->_bShowViewer;
                        }
                        listviewers.push_back(pinfo->_pviewer);
                    }
                }
            }

            FOREACH(itaddviewer, listtempviewers) {
                (*itaddviewer)->GetEnv()->AddViewer(*itaddviewer);
            }

            ViewerBasePtr puseviewer;
            FOREACH(itviewer, listviewers) {
                // double check if viewer is added to env
                bool bfound = false;
                listtempviewers.clear();
                (*itviewer)->GetEnv()->GetViewers(listtempviewers);
                FOREACH(itviewer2, listtempviewers) {
                    if( *itviewer == *itviewer2 ) {
                        bfound = true;
                        break;
                    }
                }
                if( bfound ) {
                    puseviewer = *itviewer;
                    break;
                }
                else {
                    // viewer is not in environment any more, so erase from list
                    listviewers.erase(itviewer);
                    break; // break since modifying list
                }
            }

            listtempviewers.clear();

            if( !!puseviewer ) {
                _bInMain = true;
                try {
                    puseviewer->main(bShowViewer);
                }
                catch(const std::exception& ex) {
                    RAVELOG_ERROR_FORMAT("got exception in viewer main thread %s", ex.what());
                }
                catch(...) {
                    RAVELOG_ERROR("got unknown exception in viewer main thread\n");
                }

                _bInMain = false;
                // remove from _listviewerinfos in order to avoid running the main loop again
                {
                    boost::mutex::scoped_lock lock(_mutexViewer);
                    FOREACH(itinfo, _listviewerinfos) {
                        if( (*itinfo)->_pviewer == puseviewer ) {
                            _listviewerinfos.erase(itinfo);
                            break;
                        }
                    }
                }
                puseviewer.reset();
            }
            // just go and run the next viewer's loop, don't exit here!
        }
        RAVELOG_DEBUG("shutting down viewer manager thread\n");
    }

    static void _InitializeSingleton()
    {
        _singleton.reset(new ViewerManager());

    }

    OPENRAVE_SHARED_PTR<boost::thread> _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    std::list<ViewerInfoPtr> _listviewerinfos;

    bool _bShutdown; ///< if true, shutdown everything
    bool _bInMain; ///< if true, viewer thread is running a main function

    static boost::scoped_ptr<ViewerManager> _singleton; ///< singleton
    static boost::once_flag _onceInitialize; ///< makes sure initialization is atomic

};

boost::scoped_ptr<ViewerManager> ViewerManager::_singleton(0);
boost::once_flag ViewerManager::_onceInitialize = BOOST_ONCE_INIT;

PyInterfaceBase::PyInterfaceBase(InterfaceBasePtr pbase, PyEnvironmentBasePtr pyenv) : _pbase(pbase), _pyenv(pyenv)
{
    CHECK_POINTER(_pbase);
    CHECK_POINTER(_pyenv);
}

object PyInterfaceBase::GetUserData(const std::string& key) const {
    return openravepy::GetUserData(_pbase->GetUserData(key));
}

bool PyInterfaceBase::SupportsCommand(const string& cmd)
{
    return _pbase->SupportsCommand(cmd);
}

bool PyInterfaceBase::SupportsJSONCommand(const string& cmd)
{
    return _pbase->SupportsJSONCommand(cmd);
}

object PyInterfaceBase::SendCommand(const string& in, bool releasegil, bool lockenv)
{
    std::stringstream sin(in), sout;
    {
        openravepy::PythonThreadSaverPtr statesaver;
        openravepy::PyEnvironmentLockSaverPtr envsaver;
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
            if( lockenv ) {
                // GIL is already released, so use a regular environment lock
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, true));
            }
        }
        else {
            if( lockenv ) {
                // try to safely lock the environment first
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, false));
            }
        }
        sout << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        if( !_pbase->SendCommand(sout,sin) ) {
            return py::none_();
        }
    }
    return py::to_object(sout.str());
}

object PyInterfaceBase::SendJSONCommand(const string& cmd, object input, bool releasegil, bool lockenv)
{
    rapidjson::Document in, out;
    toRapidJSONValue(input, in, in.GetAllocator());

    {
        openravepy::PythonThreadSaverPtr statesaver;
        openravepy::PyEnvironmentLockSaverPtr envsaver;
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
            if( lockenv ) {
                // GIL is already released, so use a regular environment lock
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, true));
            }
        }
        else {
            if( lockenv ) {
                // try to safely lock the environment first
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, false));
            }
        }

        _pbase->SendJSONCommand(cmd, in, out);
    }

    return toPyObject(out);
}

object PyInterfaceBase::GetReadableInterfaces()
{
    py::dict ointerfaces;
    FOREACHC(it,_pbase->GetReadableInterfaces()) {
        ointerfaces[it->first] = toPyReadable(it->second);
    }
    return ointerfaces;
}

object PyInterfaceBase::GetReadableInterface(const std::string& id)
{
    return toPyReadable(_pbase->GetReadableInterface(id));
}

void PyInterfaceBase::SetReadableInterface(const std::string& id, object oreadable)
{
    _pbase->SetReadableInterface(id,ExtractReadable(oreadable));
}

PyInterfaceBasePtr PyEnvironmentBase::_toPyInterface(InterfaceBasePtr pinterface)
{
    if( !pinterface ) {
        return PyInterfaceBasePtr();
    }
    switch(pinterface->GetInterfaceType()) {
    case PT_Planner: return openravepy::toPyPlanner(OPENRAVE_STATIC_POINTER_CAST<PlannerBase>(pinterface),shared_from_this());
    case PT_Robot: return openravepy::toPyRobot(OPENRAVE_STATIC_POINTER_CAST<RobotBase>(pinterface),shared_from_this());
    case PT_SensorSystem: return openravepy::toPySensorSystem(OPENRAVE_STATIC_POINTER_CAST<SensorSystemBase>(pinterface),shared_from_this());
    case PT_Controller: return openravepy::toPyController(OPENRAVE_STATIC_POINTER_CAST<ControllerBase>(pinterface),shared_from_this());
    case PT_Module: return openravepy::toPyModule(OPENRAVE_STATIC_POINTER_CAST<ModuleBase>(pinterface),shared_from_this());
    case PT_IkSolver: return openravepy::toPyIkSolver(OPENRAVE_STATIC_POINTER_CAST<IkSolverBase>(pinterface),shared_from_this());
    case PT_KinBody: return openravepy::toPyKinBody(OPENRAVE_STATIC_POINTER_CAST<KinBody>(pinterface),shared_from_this());
    case PT_PhysicsEngine: return openravepy::toPyPhysicsEngine(OPENRAVE_STATIC_POINTER_CAST<PhysicsEngineBase>(pinterface),shared_from_this());
    case PT_Sensor: return openravepy::toPySensor(OPENRAVE_STATIC_POINTER_CAST<SensorBase>(pinterface),shared_from_this());
    case PT_CollisionChecker: return openravepy::toPyCollisionChecker(OPENRAVE_STATIC_POINTER_CAST<CollisionCheckerBase>(pinterface),shared_from_this());
    case PT_Trajectory: return openravepy::toPyTrajectory(OPENRAVE_STATIC_POINTER_CAST<TrajectoryBase>(pinterface),shared_from_this());
    case PT_Viewer: return openravepy::toPyViewer(OPENRAVE_STATIC_POINTER_CAST<ViewerBase>(pinterface),shared_from_this());
    case PT_SpaceSampler: return openravepy::toPySpaceSampler(OPENRAVE_STATIC_POINTER_CAST<SpaceSamplerBase>(pinterface),shared_from_this());
    }
    return PyInterfaceBasePtr();
}

void PyEnvironmentBase::_BodyCallback(object fncallback, KinBodyPtr pbody, int action)
{
    object res;
    PyGILState_STATE gstate = PyGILState_Ensure();
    try {
        fncallback(openravepy::toPyKinBody(pbody, shared_from_this()), action);
    }
    catch(...) {
        RAVELOG_ERROR("exception occured in python body callback:\n");
        PyErr_Print();
    }
    PyGILState_Release(gstate);
}

CollisionAction PyEnvironmentBase::_CollisionCallback(object fncallback, CollisionReportPtr preport, bool bFromPhysics)
{
    object res;
    PyGILState_STATE gstate = PyGILState_Ensure();
    try {
        res = fncallback(openravepy::toPyCollisionReport(preport,shared_from_this()),bFromPhysics);
    }
    catch(...) {
        RAVELOG_ERROR("exception occured in python collision callback:\n");
        PyErr_Print();
    }
    CollisionAction ret = CA_DefaultAction;
    if( IS_PYTHONOBJECT_NONE(res) || !res ) {
        ret = CA_DefaultAction;
        RAVELOG_WARN("collision callback nothing returning, so executing default action\n");
    }
    else {
        extract_<int> xi(res);
        if( xi.check() ) {
            ret = (CollisionAction)(int) xi;
        }
        else {
            RAVELOG_WARN("collision callback nothing returning, so executing default action\n");
        }
    }
    PyGILState_Release(gstate);
    return ret;
}

PyEnvironmentBase::PyEnvironmentBase(int options)
{
    if( !RaveGlobalState() ) {
        RaveInitialize(true);
    }
    _penv = RaveCreateEnvironment(options);
}
PyEnvironmentBase::PyEnvironmentBase(EnvironmentBasePtr penv) : _penv(penv) {
}

PyEnvironmentBase::PyEnvironmentBase(const PyEnvironmentBase &pyenv)
{
    _penv = pyenv._penv;
}

PyEnvironmentBase::~PyEnvironmentBase()
{
}

void PyEnvironmentBase::Reset() {
    _penv->Reset();
}
void PyEnvironmentBase::Destroy() {
    ViewerManager::GetInstance().RemoveViewersOfEnvironment(_penv);
    _penv->Destroy();
}

PyEnvironmentBasePtr PyEnvironmentBase::CloneSelf(int options)
{
//        string strviewer;
//        if( options & Clone_Viewer ) {
//            boost::mutex::scoped_lock lockcreate(_mutexViewer);
//            if( !!_penv->GetViewer() ) {
//                strviewer = _penv->GetViewer()->GetXMLId();
//            }
//        }
    PyEnvironmentBasePtr pnewenv(new PyEnvironmentBase(_penv->CloneSelf(options)));
//        if( strviewer.size() > 0 ) {
//            pnewenv->SetViewer(strviewer);
//        }
    return pnewenv;
}

void PyEnvironmentBase::Clone(PyEnvironmentBasePtr pyreference, int options)
{
    if( options & Clone_Viewer ) {
        if( !!_penv->GetViewer() && !!pyreference->GetEnv()->GetViewer() ) {
            if( _penv->GetViewer()->GetXMLId() != pyreference->GetEnv()->GetViewer()->GetXMLId() ) {
                RAVELOG_VERBOSE("reset the viewer since it has to be cloned\n");
                //boost::mutex::scoped_lock lockcreate(pyreference->_mutexViewer);
                SetViewer("");
            }
        }
    }
    _penv->Clone(pyreference->GetEnv(),options);
}

bool PyEnvironmentBase::SetCollisionChecker(PyCollisionCheckerBasePtr pchecker)
{
    return _penv->SetCollisionChecker(openravepy::GetCollisionChecker(pchecker));
}
object PyEnvironmentBase::GetCollisionChecker()
{
    return py::to_object(openravepy::toPyCollisionChecker(_penv->GetCollisionChecker(), shared_from_this()));
}
bool PyEnvironmentBase::CheckCollision(PyKinBodyPtr pbody1)
{
    CHECK_POINTER(pbody1);
    return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)));
}
bool PyEnvironmentBase::CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(pbody1);
    bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
{
    CHECK_POINTER(pbody1);
    CHECK_POINTER(pbody2);
    return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)));
}

bool PyEnvironmentBase::CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(pbody1);
    CHECK_POINTER(pbody2);
    bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)), openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(object o1)
{
    CHECK_POINTER(o1);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        return _penv->CheckCollision(plink);
    }
    KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
    if( !!pbody ) {
        return _penv->CheckCollision(pbody);
    }
    throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
}

bool PyEnvironmentBase::CheckCollision(object o1, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(o1);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    bool bCollision;
    if( !!plink ) {
        bCollision = _penv->CheckCollision(plink,openravepy::GetCollisionReport(pReport));
    }
    else {
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            bCollision = _penv->CheckCollision(pbody,openravepy::GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument"),ORE_InvalidArguments);
        }
    }
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(object o1, object o2)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(o2);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
        if( !!plink2 ) {
            return _penv->CheckCollision(plink,plink2);
        }
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
        if( !!pbody2 ) {
            return _penv->CheckCollision(plink,pbody2);
        }
        CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
        if( !!preport2 ) {
            bool bCollision = _penv->CheckCollision(plink,preport2);
            openravepy::UpdateCollisionReport(o2,shared_from_this());
            return bCollision;
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
    }
    KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
    if( !!pbody ) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
        if( !!plink2 ) {
            return _penv->CheckCollision(plink2,pbody);
        }
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
        if( !!pbody2 ) {
            return _penv->CheckCollision(pbody,pbody2);
        }
        CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
        if( !!preport2 ) {
            bool bCollision = _penv->CheckCollision(pbody,preport2);
            openravepy::UpdateCollisionReport(o2,shared_from_this());
            return bCollision;
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
    }
    throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
}
bool PyEnvironmentBase::CheckCollision(object o1, object o2, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(o2);
    bool bCollision = false;
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
        if( !!plink2 ) {
            bCollision = _penv->CheckCollision(plink,plink2, openravepy::GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
            if( !!pbody2 ) {
                bCollision = _penv->CheckCollision(plink,pbody2, openravepy::GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
            }
        }
    }
    {
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                bCollision = _penv->CheckCollision(plink2,pbody, openravepy::GetCollisionReport(pReport));
            }
            else {
                KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
                if( !!pbody2 ) {
                    bCollision = _penv->CheckCollision(pbody,pbody2, openravepy::GetCollisionReport(pReport));
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
                }
            }
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
        }
    }
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(object o1, PyKinBodyPtr pybody2)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(pybody2);
    KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        return _penv->CheckCollision(plink,pbody2);
    }
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
    if( !!pbody1 ) {
        return _penv->CheckCollision(pbody1,pbody2);
    }
    throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
}

bool PyEnvironmentBase::CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(pybody2);
    KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    bool bCollision = false;
    if( !!plink ) {
        bCollision = _penv->CheckCollision(plink,pbody2,openravepy::GetCollisionReport(pReport));
    }
    else {
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
        if( !!pbody1 ) {
            bCollision = _penv->CheckCollision(pbody1,pbody2,openravepy::GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
        }
    }
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(object o1, object bodyexcluded, object linkexcluded)
{
    CollisionReportPtr preport = openravepy::GetCollisionReport(linkexcluded);
    if( !!preport ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("3rd argument should be linkexcluded, rather than CollisionReport! Try report="),ORE_InvalidArguments);
    }

    KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

    std::vector<KinBodyConstPtr> vbodyexcluded;
    for(size_t i = 0; i < (size_t)len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < (size_t)len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }
    if( !!plink1 ) {
        return _penv->CheckCollision(plink1,vbodyexcluded,vlinkexcluded);
    }
    else if( !!pbody1 ) {
        return _penv->CheckCollision(pbody1,vbodyexcluded,vlinkexcluded);
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
    }
}

bool PyEnvironmentBase::CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
{
    std::vector<KinBodyConstPtr> vbodyexcluded;
    KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

    for(size_t i = 0; i < (size_t)len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < (size_t)len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }

    bool bCollision=false;
    if( !!plink1 ) {
        bCollision = _penv->CheckCollision(plink1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
    }
    else if( !!pbody1 ) {
        bCollision = _penv->CheckCollision(pbody1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
    }

    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded)
{
    std::vector<KinBodyConstPtr> vbodyexcluded;
    for(size_t i = 0; i < (size_t)len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < (size_t)len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }
    return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)),vbodyexcluded,vlinkexcluded);
}

bool PyEnvironmentBase::CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
{
    std::vector<KinBodyConstPtr> vbodyexcluded;
    for(size_t i = 0; i < (size_t)len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < (size_t)len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }

    bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)), vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody)
{
    return _penv->CheckCollision(pyray->r,KinBodyConstPtr(openravepy::GetKinBody(pbody)));
}

bool PyEnvironmentBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
{
    bool bCollision = _penv->CheckCollision(pyray->r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

object PyEnvironmentBase::CheckCollisionRays(py::numeric::array rays, PyKinBodyPtr pbody, bool bFrontFacingOnly)
{
    object shape = rays.attr("shape");
    int nRays = extract<int>(shape[0]);
    if( nRays == 0 ) {
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<dReal>());
    }
    if( extract<int>(shape[1]) != 6 ) {
        throw OpenRAVEException(_("rays object needs to be a Nx6 vector\n"));
    }
    CollisionReport report;
    CollisionReportPtr preport(&report,null_deleter());

    PyArrayObject *pPyRays = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(rays.ptr()));
    AutoPyArrayObjectDereferencer pyderef(pPyRays);

    if( !PyArray_ISFLOAT(pPyRays) ) {
        throw OpenRAVEException(_("rays has to be a float array\n"));
    }

    bool isFloat = PyArray_ITEMSIZE(pPyRays) == sizeof(float); // or double
    const float *pRaysFloat = isFloat ? reinterpret_cast<const float*>(PyArray_DATA(pPyRays)) : NULL;
    const double *pRaysDouble = isFloat ? NULL : reinterpret_cast<const double*>(PyArray_DATA(pPyRays));

    RAY r;
    npy_intp dims[] = { nRays,6};
    PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal) == sizeof(double) ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* ppos = (dReal*)PyArray_DATA(pypos);
    std::memset(ppos, 0, nRays * sizeof(dReal));
    PyObject* pycollision = PyArray_SimpleNew(1,&dims[0], PyArray_BOOL);
    // numpy bool = uint8_t
    uint8_t* pcollision = (uint8_t*)PyArray_DATA(pycollision);
    std::memset(pcollision, 0, nRays * sizeof(uint8_t));
    {
        openravepy::PythonThreadSaver threadsaver;

        for(int i = 0; i < nRays; ++i, ppos += 6) {
            if (isFloat) {
                r.pos.x = pRaysFloat[0];
                r.pos.y = pRaysFloat[1];
                r.pos.z = pRaysFloat[2];
                r.dir.x = pRaysFloat[3];
                r.dir.y = pRaysFloat[4];
                r.dir.z = pRaysFloat[5];
                pRaysFloat += 6;
            } else {
                r.pos.x = pRaysDouble[0];
                r.pos.y = pRaysDouble[1];
                r.pos.z = pRaysDouble[2];
                r.dir.x = pRaysDouble[3];
                r.dir.y = pRaysDouble[4];
                r.dir.z = pRaysDouble[5];
                pRaysDouble += 6;
            }

            bool bCollision;
            if( !pbody ) {
                bCollision = _penv->CheckCollision(r, preport);
            }
            else {
                bCollision = _penv->CheckCollision(r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), preport);
            }

            if( bCollision &&( report.contacts.size() > 0) ) {
                if( !bFrontFacingOnly ||( report.contacts[0].norm.dot3(r.dir)<0) ) {
                    pcollision[i] = true;
                    ppos[0] = report.contacts[0].pos.x;
                    ppos[1] = report.contacts[0].pos.y;
                    ppos[2] = report.contacts[0].pos.z;
                    ppos[3] = report.contacts[0].norm.x;
                    ppos[4] = report.contacts[0].norm.y;
                    ppos[5] = report.contacts[0].norm.z;
                }
            }
        }
    }

    return py::make_tuple(py::to_array_astype<bool>(pycollision), py::to_array_astype<dReal>(pypos));
}

bool PyEnvironmentBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray)
{
    return _penv->CheckCollision(pyray->r);
}

bool PyEnvironmentBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyCollisionReportPtr pReport)
{
    bool bCollision = _penv->CheckCollision(pyray->r, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,shared_from_this());
    return bCollision;
}

bool PyEnvironmentBase::Load(const std::string &filename) {
    openravepy::PythonThreadSaver threadsaver;
    return _penv->Load(filename);
}
bool PyEnvironmentBase::Load(const std::string &filename, object odictatts) {
    AttributesList dictatts = toAttributesList(odictatts);
    openravepy::PythonThreadSaver threadsaver;
    return _penv->Load(filename, dictatts);
}
bool PyEnvironmentBase::LoadURI(const std::string &filename, object odictatts) {
    AttributesList dictatts = toAttributesList(odictatts);
    openravepy::PythonThreadSaver threadsaver;
    return _penv->LoadURI(filename, dictatts);
}
bool PyEnvironmentBase::LoadData(const std::string &data) {
    openravepy::PythonThreadSaver threadsaver;
    return _penv->LoadData(data);
}
bool PyEnvironmentBase::LoadData(const std::string &data, object odictatts) {
    AttributesList dictatts = toAttributesList(odictatts);
    openravepy::PythonThreadSaver threadsaver;
    return _penv->LoadData(data, dictatts);
}

void PyEnvironmentBase::Save(const std::string &filename, const int options, object odictatts) {
    bool bSuccess = false;
    // avoid destined extract failure
    if(!IS_PYTHONOBJECT_NONE(odictatts)) {
        extract_<std::string> otarget(odictatts);
        if( otarget.check() ) {
            // old versions
            AttributesList atts;
            atts.emplace_back("target", (std::string)otarget);
            openravepy::PythonThreadSaver threadsaver;
            _penv->Save(filename, (EnvironmentBase::SelectionOptions) options, atts);
            bSuccess = true;
        }
    }
    if(!bSuccess) {
        AttributesList dictatts = toAttributesList(odictatts);
        openravepy::PythonThreadSaver threadsaver;
        _penv->Save(filename, (EnvironmentBase::SelectionOptions) options, dictatts);
    }
}

object PyEnvironmentBase::WriteToMemory(const std::string &filetype, const int options, object odictatts) {
    std::vector<char> output;
    bool bSuccess = false;
    // avoid destined extract failure
    if(!IS_PYTHONOBJECT_NONE(odictatts)) {
        extract_<std::string> otarget(odictatts);
        if( otarget.check() ) {
            // old versions
            AttributesList atts;
            atts.emplace_back("target", (std::string)otarget);
            _penv->WriteToMemory(filetype, output, (EnvironmentBase::SelectionOptions) options, atts);
            bSuccess = true;
        }
    }
    if(!bSuccess) {
        _penv->WriteToMemory(filetype, output, (EnvironmentBase::SelectionOptions) options, toAttributesList(odictatts));
    }

    if( output.empty() ) {
        return py::none_();
    }
    else {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // https://github.com/pybind/pybind11/issues/1201
        return py::cast<py::object>(PyString_FromStringAndSize(&output[0], output.size()));
#else
        return py::to_object(py::handle<>(PyString_FromStringAndSize(&output[0], output.size())));
#endif
    }
}

object PyEnvironmentBase::ReadRobotURI(const string &filename)
{
    return py::to_object(openravepy::toPyRobot(_penv->ReadRobotURI(filename),shared_from_this()));
}
object PyEnvironmentBase::ReadRobotURI(const string &filename, object odictatts)
{
    return py::to_object(openravepy::toPyRobot(_penv->ReadRobotURI(RobotBasePtr(), filename,toAttributesList(odictatts)),shared_from_this()));
}
object PyEnvironmentBase::ReadRobotData(const string &data)
{
    return py::to_object(openravepy::toPyRobot(_penv->ReadRobotData(RobotBasePtr(), data, AttributesList()), shared_from_this()));
}
object PyEnvironmentBase::ReadRobotData(const string &data, object odictatts)
{
    return py::to_object(openravepy::toPyRobot(_penv->ReadRobotData(RobotBasePtr(), data, toAttributesList(odictatts)),shared_from_this()));
}
object PyEnvironmentBase::ReadKinBodyURI(const string &filename)
{
    return py::to_object(openravepy::toPyKinBody(_penv->ReadKinBodyURI(filename), shared_from_this()));
}
object PyEnvironmentBase::ReadKinBodyURI(const string &filename, object odictatts)
{
    return py::to_object(openravepy::toPyKinBody(_penv->ReadKinBodyURI(KinBodyPtr(), filename, toAttributesList(odictatts)),shared_from_this()));
}
object PyEnvironmentBase::ReadKinBodyData(const string &data)
{
    return py::to_object(openravepy::toPyKinBody(_penv->ReadKinBodyData(KinBodyPtr(), data, AttributesList()),shared_from_this()));
}
object PyEnvironmentBase::ReadKinBodyData(const string &data, object odictatts)
{
    return py::to_object(openravepy::toPyKinBody(_penv->ReadKinBodyData(KinBodyPtr(), data, toAttributesList(odictatts)),shared_from_this()));
}
PyInterfaceBasePtr PyEnvironmentBase::ReadInterfaceURI(const std::string& filename)
{
    return _toPyInterface(_penv->ReadInterfaceURI(filename));
}
PyInterfaceBasePtr PyEnvironmentBase::ReadInterfaceURI(const std::string& filename, object odictatts)
{
    return _toPyInterface(_penv->ReadInterfaceURI(filename, toAttributesList(odictatts)));
}
object PyEnvironmentBase::ReadTrimeshURI(const std::string& filename)
{
    OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshURI(OPENRAVE_SHARED_PTR<TriMesh>(),filename);
    if( !ptrimesh ) {
        return py::none_();
    }
    return toPyTriMesh(*ptrimesh);
}
object PyEnvironmentBase::ReadTrimeshURI(const std::string& filename, object odictatts)
{
    OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshURI(OPENRAVE_SHARED_PTR<TriMesh>(),filename,toAttributesList(odictatts));
    if( !ptrimesh ) {
        return py::none_();
    }
    return toPyTriMesh(*ptrimesh);
}

object PyEnvironmentBase::ReadTrimeshData(const std::string& data, const std::string& formathint)
{
    OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshData(OPENRAVE_SHARED_PTR<TriMesh>(),data,formathint);
    if( !ptrimesh ) {
        return py::none_();
    }
    return toPyTriMesh(*ptrimesh);
}
object PyEnvironmentBase::ReadTrimeshData(const std::string& data, const std::string& formathint, object odictatts)
{
    OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshData(OPENRAVE_SHARED_PTR<TriMesh>(),data,formathint,toAttributesList(odictatts));
    if( !ptrimesh ) {
        return py::none_();
    }
    return toPyTriMesh(*ptrimesh);
}

void PyEnvironmentBase::Add(PyInterfaceBasePtr pinterface, bool bAnonymous, const std::string& cmdargs) {
    _penv->Add(pinterface->GetInterfaceBase(), bAnonymous, cmdargs);
}

void PyEnvironmentBase::AddKinBody(PyKinBodyPtr pbody) {
    CHECK_POINTER(pbody); _penv->Add(openravepy::GetKinBody(pbody));
}
void PyEnvironmentBase::AddKinBody(PyKinBodyPtr pbody, bool bAnonymous) {
    CHECK_POINTER(pbody); _penv->Add(openravepy::GetKinBody(pbody),bAnonymous);
}
void PyEnvironmentBase::AddRobot(PyRobotBasePtr robot) {
    CHECK_POINTER(robot);
    _penv->Add(openravepy::GetRobot(robot));
}
void PyEnvironmentBase::AddRobot(PyRobotBasePtr robot, bool bAnonymous) {
    CHECK_POINTER(robot);
    _penv->Add(openravepy::GetRobot(robot),bAnonymous);
}
void PyEnvironmentBase::AddSensor(PySensorBasePtr sensor) {
    CHECK_POINTER(sensor);
    _penv->Add(openravepy::GetSensor(sensor));
}
void PyEnvironmentBase::AddSensor(PySensorBasePtr sensor, bool bAnonymous) {
    CHECK_POINTER(sensor);
    _penv->Add(openravepy::GetSensor(sensor),bAnonymous);
}
void PyEnvironmentBase::AddViewer(PyViewerBasePtr viewer) {
    CHECK_POINTER(viewer);
    _penv->Add(openravepy::GetViewer(viewer));
}

bool PyEnvironmentBase::RemoveKinBody(PyKinBodyPtr pbody) {
    CHECK_POINTER(pbody);
    RAVELOG_WARN("openravepy RemoveKinBody deprecated, use Remove\n");
    return _penv->Remove(openravepy::GetKinBody(pbody));
}

bool PyEnvironmentBase::RemoveKinBodyByName(const std::string& name) {
    return _penv->RemoveKinBodyByName(name);
}

object PyEnvironmentBase::GetKinBody(const string &name)
{
    KinBodyPtr pbody = _penv->GetKinBody(name);
    if( !pbody ) {
        return py::none_();
    }
    if( pbody->IsRobot() ) {
        return py::to_object(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(pbody),shared_from_this()));
    }
    else {
        return py::to_object(openravepy::toPyKinBody(pbody,shared_from_this()));
    }
}
object PyEnvironmentBase::GetRobot(const string &name)
{
    return py::to_object(openravepy::toPyRobot(_penv->GetRobot(name), shared_from_this()));
}
object PyEnvironmentBase::GetSensor(const string &name)
{
    return py::to_object(openravepy::toPySensor(_penv->GetSensor(name),shared_from_this()));
}

object PyEnvironmentBase::GetBodyFromEnvironmentId(int id)
{
    return py::to_object(openravepy::toPyKinBody(_penv->GetBodyFromEnvironmentId(id),shared_from_this()));
}

int PyEnvironmentBase::AddModule(PyModuleBasePtr prob, const string &PY_ARGS) {
    CHECK_POINTER(prob);
    return _penv->AddModule(openravepy::GetModule(prob),PY_ARGS);
}
bool PyEnvironmentBase::RemoveProblem(PyModuleBasePtr prob) {
    CHECK_POINTER(prob);
    RAVELOG_WARN("openravepy RemoveProblem deprecated, use Remove\n");
    return _penv->Remove(openravepy::GetModule(prob));
}
bool PyEnvironmentBase::Remove(PyInterfaceBasePtr obj) {
    CHECK_POINTER(obj);

    // have to check if viewer in order to notify viewer manager
    ViewerBasePtr pviewer = RaveInterfaceCast<ViewerBase>(obj->GetInterfaceBase());
    if( !!pviewer ) {
        ViewerManager::GetInstance().RemoveViewer(pviewer);
    }
    return _penv->Remove(obj->GetInterfaceBase());
}

object PyEnvironmentBase::GetModules()
{
    std::list<ModuleBasePtr> listModules;
    _penv->GetModules(listModules);
    py::list modules;
    FOREACHC(itprob, listModules) {
        modules.append(openravepy::toPyModule(*itprob,shared_from_this()));
    }
    return modules;
}

bool PyEnvironmentBase::SetPhysicsEngine(PyPhysicsEngineBasePtr pengine)
{
    return _penv->SetPhysicsEngine(openravepy::GetPhysicsEngine(pengine));
}
object PyEnvironmentBase::GetPhysicsEngine() {
    return py::to_object(openravepy::toPyPhysicsEngine(_penv->GetPhysicsEngine(),shared_from_this()));
}

object PyEnvironmentBase::RegisterBodyCallback(object fncallback)
{
    if( !fncallback ) {
        throw OpenRAVEException(_("callback not specified"));
    }
    UserDataPtr p = _penv->RegisterBodyCallback(boost::bind(&PyEnvironmentBase::_BodyCallback,shared_from_this(),fncallback,_1,_2));
    if( !p ) {
        throw OpenRAVEException(_("registration handle is NULL"));
    }
    return openravepy::GetUserData(p);
}

object PyEnvironmentBase::RegisterCollisionCallback(object fncallback)
{
    if( !fncallback ) {
        throw OpenRAVEException(_("callback not specified"));
    }
    UserDataPtr p = _penv->RegisterCollisionCallback(boost::bind(&PyEnvironmentBase::_CollisionCallback,shared_from_this(),fncallback,_1,_2));
    if( !p ) {
        throw OpenRAVEException(_("registration handle is NULL"));
    }
    return openravepy::GetUserData(p);
}

bool PyEnvironmentBase::HasRegisteredCollisionCallbacks()
{
    return _penv->HasRegisteredCollisionCallbacks();
}

void PyEnvironmentBase::StepSimulation(dReal timeStep) {
    _penv->StepSimulation(timeStep);
}
void PyEnvironmentBase::StartSimulation(dReal fDeltaTime, bool bRealTime) {
    _penv->StartSimulation(fDeltaTime,bRealTime);
}
void PyEnvironmentBase::StopSimulation(int shutdownthread) {
    _penv->StopSimulation(shutdownthread);
}
uint64_t PyEnvironmentBase::GetSimulationTime() {
    return _penv->GetSimulationTime();
}
bool PyEnvironmentBase::IsSimulationRunning() {
    return _penv->IsSimulationRunning();
}

void PyEnvironmentBase::Lock()
{
    // first try to lock without releasing the GIL since it is faster
    uint64_t nTimeoutMicroseconds = 2000; // 2ms
    uint64_t basetime = OpenRAVE::utils::GetMicroTime();
    while(OpenRAVE::utils::GetMicroTime()-basetime<nTimeoutMicroseconds ) {
        if( TryLock() ) {
            return;
        }
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
    }

    // failed, so must be a python thread blocking it...
    LockReleaseGil();
}

/// \brief raw locking without any python overhead
void PyEnvironmentBase::LockRaw()
{
#if BOOST_VERSION < 103500
    boost::mutex::scoped_lock envlock(_envmutex);
    if( _listfreelocks.size() > 0 ) {
        _listfreelocks.back()->lock();
        _listenvlocks.splice(_listenvlocks.end(),_listfreelocks,--_listfreelocks.end());
    }
    else {
        _listenvlocks.push_back(OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
    }
#else
    _penv->GetMutex().lock();
#endif
}

void PyEnvironmentBase::LockReleaseGil()
{
    PythonThreadSaver saver;
    LockRaw();
}

void PyEnvironmentBase::Unlock()
{
#if BOOST_VERSION < 103500
    boost::mutex::scoped_lock envlock(_envmutex);
    BOOST_ASSERT(_listenvlocks.size()>0);
    _listenvlocks.back()->unlock();
    _listfreelocks.splice(_listfreelocks.end(),_listenvlocks,--_listenvlocks.end());
#else
    _penv->GetMutex().unlock();
#endif
}

/// try locking the environment while releasing the GIL. This can get into a deadlock after env lock is acquired and before gil is re-acquired
bool PyEnvironmentBase::TryLockReleaseGil()
{
    bool bSuccess = false;
    PythonThreadSaver saver;
#if BOOST_VERSION < 103500
    OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
    if( !!lockenv->try_lock() ) {
        bSuccess = true;
        _listenvlocks.push_back(OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
    }
#else
    if( _penv->GetMutex().try_lock() ) {
        bSuccess = true;
    }
#endif
    return bSuccess;
}

bool PyEnvironmentBase::TryLock()
{
    bool bSuccess = false;
#if BOOST_VERSION < 103500
    OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
    if( !!lockenv->try_lock() ) {
        bSuccess = true;
        _listenvlocks.push_back(OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
    }
#else
    if( _penv->GetMutex().try_lock() ) {
        bSuccess = true;
    }
#endif
    return bSuccess;
}


bool PyEnvironmentBase::Lock(float timeout)
{
    uint64_t nTimeoutMicroseconds = timeout*1000000;
    uint64_t basetime = OpenRAVE::utils::GetMicroTime();
    while(OpenRAVE::utils::GetMicroTime()-basetime<nTimeoutMicroseconds ) {
        if( TryLock() ) {
            return true;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    return false;
}

void PyEnvironmentBase::__enter__()
{
    Lock();
}

void PyEnvironmentBase::__exit__(object type, object value, object traceback)
{
    Unlock();
}

bool PyEnvironmentBase::SetViewer(const string &viewername, bool showviewer)
{
    ViewerBasePtr pviewer = ViewerManager::GetInstance().AddViewer(_penv, viewername, showviewer, true);
    return !(pviewer == NULL);
}

/// \brief sets the default viewer
bool PyEnvironmentBase::SetDefaultViewer(bool showviewer)
{
    std::string viewername = RaveGetDefaultViewerType();
    if( viewername.size() > 0 ) {
        ViewerBasePtr pviewer = ViewerManager::GetInstance().AddViewer(_penv, viewername, showviewer, true);
        return !!pviewer;
    }

    return false;
}

object PyEnvironmentBase::GetViewer()
{
    return py::to_object(openravepy::toPyViewer(_penv->GetViewer(),shared_from_this()));
}

/// returns the number of points
size_t PyEnvironmentBase::_getGraphPoints(object opoints, std::vector<float>&vpoints)
{
    if( PyObject_HasAttrString(opoints.ptr(),"shape") ) {
        object pointshape = opoints.attr("shape");
        switch(len(pointshape)) {
        case 1:
            vpoints = ExtractArray<float>(opoints);
            if( vpoints.size()%3 ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("points have bad size %d"), vpoints.size(),ORE_InvalidArguments);
            }
            return vpoints.size()/3;
        case 2: {
            int num = extract<int>(pointshape[0]);
            int dim = extract<int>(pointshape[1]);
            vpoints = ExtractArray<float>(opoints.attr("flat"));
            if(dim % 3) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("points have bad size %dx%d"), num%dim,ORE_InvalidArguments);
            }
            return num*(dim/3);
        }
        default:
            throw OpenRAVEException(_("points have bad dimension"));
        }
    }
    // assume it is a regular 1D list
    vpoints = ExtractArray<float>(opoints);
    if( vpoints.size()% 3 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("points have bad size %d"), vpoints.size(),ORE_InvalidArguments);
    }
    return vpoints.size()/3;
}

/// returns the number of colors
size_t PyEnvironmentBase::_getGraphColors(object ocolors, std::vector<float>&vcolors)
{
    if( !IS_PYTHONOBJECT_NONE(ocolors) ) {
        if( PyObject_HasAttrString(ocolors.ptr(),"shape") ) {
            object colorshape = ocolors.attr("shape");
            switch( len(colorshape) ) {
            case 1:
                break;
            case 2: {
                int numcolors = extract<int>(colorshape[0]);
                int colordim = extract<int>(colorshape[1]);
                if(( colordim != 3) &&( colordim != 4) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("colors dim %d needs to be 3 or 4"),colordim, ORE_InvalidArguments);
                }
                vcolors = ExtractArray<float>(ocolors.attr("flat"));
                return numcolors;
            }
            default:
                throw OPENRAVE_EXCEPTION_FORMAT(_("colors has %d dimensions"),len(colorshape), ORE_InvalidArguments);
            }
        }
        vcolors = ExtractArray<float>(ocolors);
        if( vcolors.size() == 3 ) {
            vcolors.push_back(1.0f);
        }
        else if( vcolors.size() != 4 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("colors has incorrect number of values %d"),vcolors.size(), ORE_InvalidArguments);
        }
        return 1;
    }
    // default
    RaveVector<float> vcolor(1,0.5,0.5,1.0);
    vcolors.resize(4);
    vcolors[0] = 1; vcolors[1] = 0.5f; vcolors[2] = 0.5f; vcolors[3] = 1.0f;
    return 1;
}

std::pair<size_t,size_t> PyEnvironmentBase::_getGraphPointsColors(object opoints, object ocolors, std::vector<float>&vpoints, std::vector<float>&vcolors)
{
    size_t numpoints = _getGraphPoints(opoints,vpoints);
    size_t numcolors = _getGraphColors(ocolors,vcolors);
    if( numpoints <= 0 ) {
        throw OpenRAVEException(_("points cannot be empty"),ORE_InvalidArguments);
    }
    if(( numcolors > 1) &&( numpoints != numcolors) ) {
        throw OpenRAVEException(boost::str(boost::format(_("number of points (%d) need to match number of colors (%d)"))%numpoints%numcolors));
    }
    return make_pair(numpoints,numcolors);
}

object PyEnvironmentBase::plot3(object opoints,float pointsize,object ocolors, int drawstyle)
{
    std::vector<float> vpoints, vcolors;
    pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
    bool bhasalpha = vcolors.size() == 4*sizes.second;
    if( sizes.first == sizes.second ) {
        return toPyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,&vcolors[0],drawstyle,bhasalpha));
    }
    BOOST_ASSERT(vcolors.size()<=4);
    RaveVector<float> vcolor;
    for(int i = 0; i < (int)vcolors.size(); ++i) {
        vcolor[i] = vcolors[i];
    }
    return toPyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,vcolor,drawstyle));
}

object PyEnvironmentBase::drawlinestrip(object opoints,float linewidth,object ocolors, int drawstyle)
{
    std::vector<float> vpoints, vcolors;
    pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
    //bool bhasalpha = vcolors.size() == 4*sizes.second;
    if( sizes.first == sizes.second ) {
        return toPyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0]));
    }
    BOOST_ASSERT(vcolors.size()<=4);
    RaveVector<float> vcolor;
    for(int i = 0; i < (int)vcolors.size(); ++i) {
        vcolor[i] = vcolors[i];
    }
    return toPyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor));
}

object PyEnvironmentBase::drawlinelist(object opoints,float linewidth,object ocolors, int drawstyle)
{
    std::vector<float> vpoints, vcolors;
    pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
    //bool bhasalpha = vcolors.size() == 4*sizes.second;
    if( sizes.first == sizes.second ) {
        return toPyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0]));
    }
    BOOST_ASSERT(vcolors.size()<=4);
    RaveVector<float> vcolor;
    for(int i = 0; i < (int)vcolors.size(); ++i) {
        vcolor[i] = vcolors[i];
    }
    return toPyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor));
}

object PyEnvironmentBase::drawarrow(object op1, object op2, float linewidth, object ocolor)
{
    RaveVector<float> vcolor(1,0.5,0.5,1);
    if( !IS_PYTHONOBJECT_NONE(ocolor) ) {
        vcolor = ExtractVector34(ocolor,1.0f);
    }
    return toPyGraphHandle(_penv->drawarrow(ExtractVector3(op1),ExtractVector3(op2),linewidth,vcolor));
}

object PyEnvironmentBase::drawbox(object opos, object oextents, object ocolor)
{
    RaveVector<float> vcolor(1,0.5,0.5,1);
    if( !IS_PYTHONOBJECT_NONE(ocolor) ) {
        vcolor = ExtractVector34(ocolor,1.0f);
    }
    return toPyGraphHandle(_penv->drawbox(ExtractVector3(opos),ExtractVector3(oextents)));
}

object PyEnvironmentBase::drawplane(object otransform, object oextents, const boost::multi_array<float,2>&_vtexture)
{
    boost::multi_array<float,3> vtexture(boost::extents[1][_vtexture.shape()[0]][_vtexture.shape()[1]]);
    vtexture[0] = _vtexture;
    boost::array<size_t,3> dims = { { _vtexture.shape()[0],_vtexture.shape()[1],1}};
    vtexture.reshape(dims);
    return toPyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), RaveVector<float>(extract<float>(oextents[0]),extract<float>(oextents[1]),0), vtexture));
}
object PyEnvironmentBase::drawplane(object otransform, object oextents, const boost::multi_array<float,3>&vtexture)
{
    return toPyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), RaveVector<float>(extract<float>(oextents[0]),extract<float>(oextents[1]),0), vtexture));
}

object PyEnvironmentBase::drawtrimesh(object opoints, object oindices, object ocolors)
{
    std::vector<float> vpoints;
    _getGraphPoints(opoints,vpoints);
    std::vector<int> vindices;
    int* pindices = NULL;
    int numTriangles = vpoints.size()/9;
    if( !IS_PYTHONOBJECT_NONE(oindices) ) {
        vindices = ExtractArray<int>(oindices.attr("flat"));
        if( vindices.size() > 0 ) {
            numTriangles = vindices.size()/3;
            pindices = &vindices[0];
        }
    }
    RaveVector<float> vcolor(1,0.5,0.5,1);
    if( !IS_PYTHONOBJECT_NONE(ocolors) ) {
        object shape = ocolors.attr("shape");
        if( len(shape) == 1 ) {
            return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,ExtractVector34(ocolors,1.0f)));
        }
        else {
            BOOST_ASSERT(extract<size_t>(shape[0])==vpoints.size()/3);
            return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,extract<boost::multi_array<float,2> >(ocolors)));
        }
    }
    return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,RaveVector<float>(1,0.5,0.5,1)));
}

object PyEnvironmentBase::GetBodies()
{
    std::vector<KinBodyPtr> vbodies;
    _penv->GetBodies(vbodies);
    py::list bodies;
    FOREACHC(itbody, vbodies) {
        if( (*itbody)->IsRobot() ) {
            bodies.append(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(*itbody),shared_from_this()));
        }
        else {
            bodies.append(openravepy::toPyKinBody(*itbody,shared_from_this()));
        }
    }
    return bodies;
}

object PyEnvironmentBase::GetRobots()
{
    std::vector<RobotBasePtr> vrobots;
    _penv->GetRobots(vrobots);
    py::list robots;
    FOREACHC(itrobot, vrobots) {
        robots.append(openravepy::toPyRobot(*itrobot,shared_from_this()));
    }
    return robots;
}

object PyEnvironmentBase::GetSensors()
{
    std::vector<SensorBasePtr> vsensors;
    _penv->GetSensors(vsensors);
    py::list sensors;
    FOREACHC(itsensor, vsensors) {
        sensors.append(openravepy::toPySensor(*itsensor,shared_from_this()));
    }
    return sensors;
}

void PyEnvironmentBase::UpdatePublishedBodies()
{
    _penv->UpdatePublishedBodies();
}

object PyEnvironmentBase::GetPublishedBodies(uint64_t timeout)
{
    std::vector<KinBody::BodyState> vbodystates;
    _penv->GetPublishedBodies(vbodystates, timeout);
    py::list ostates;
    FOREACH(itstate, vbodystates) {
        py::dict ostate;
        ostate["body"] = toPyKinBody(itstate->pbody, shared_from_this());
        py::list olinktransforms;
        FOREACH(ittransform, itstate->vectrans) {
            olinktransforms.append(ReturnTransform(*ittransform));
        }
        ostate["linktransforms"] = olinktransforms;
        ostate["jointvalues"] = toPyArray(itstate->jointvalues);
        ostate["linkEnableStates"] = toPyArray(itstate->vLinkEnableStates);
        ostate["connectedBodyActiveStates"] = toPyArray(itstate->vConnectedBodyActiveStates);
        ostate["name"] = ConvertStringToUnicode(itstate->strname);
        ostate["uri"] = ConvertStringToUnicode(itstate->uri);
        ostate["updatestamp"] = itstate->updatestamp;
        ostate["environmentid"] = itstate->environmentid;
        ostate["activeManipulatorName"] = itstate->activeManipulatorName;
        ostate["activeManipulatorTransform"] = ReturnTransform(itstate->activeManipulatorTransform);
        ostates.append(ostate);
    }
    return ostates;
}

object PyEnvironmentBase::GetPublishedBody(const std::string &name, uint64_t timeout)
{
    KinBody::BodyState bodystate;
    if( !_penv->GetPublishedBody(name, bodystate, timeout) ) {
        return py::none_();
    }

    py::dict ostate;
    ostate["body"] = toPyKinBody(bodystate.pbody, shared_from_this());
    py::list olinktransforms;
    FOREACH(ittransform, bodystate.vectrans) {
        olinktransforms.append(ReturnTransform(*ittransform));
    }
    ostate["linktransforms"] = olinktransforms;
    ostate["jointvalues"] = toPyArray(bodystate.jointvalues);
    ostate["linkEnableStates"] = toPyArray(bodystate.vLinkEnableStates);
    ostate["connectedBodyActiveStates"] = toPyArray(bodystate.vConnectedBodyActiveStates);
    ostate["name"] = ConvertStringToUnicode(bodystate.strname);
    ostate["uri"] = ConvertStringToUnicode(bodystate.uri);
    ostate["updatestamp"] = bodystate.updatestamp;
    ostate["environmentid"] = bodystate.environmentid;
    ostate["activeManipulatorName"] = bodystate.activeManipulatorName;
    ostate["activeManipulatorTransform"] = ReturnTransform(bodystate.activeManipulatorTransform);
    return ostate;
}

object PyEnvironmentBase::GetPublishedBodyJointValues(const std::string &name, uint64_t timeout)
{
    std::vector<dReal> jointValues;
    if( !_penv->GetPublishedBodyJointValues(name, jointValues, timeout) ) {
        return py::none_();
    }
    return toPyArray(jointValues);
}

object PyEnvironmentBase::GetPublishedBodyTransformsMatchingPrefix(const string &prefix, uint64_t timeout) {
    std::vector< std::pair<std::string, Transform> > nameTransfPairs;
    _penv->GetPublishedBodyTransformsMatchingPrefix(prefix, nameTransfPairs, timeout);

    py::dict otransforms;
    FOREACH(itpair, nameTransfPairs) {
        otransforms[itpair->first] = ReturnTransform(itpair->second);
    }

    return otransforms;
}

object PyEnvironmentBase::Triangulate(PyKinBodyPtr pbody)
{
    CHECK_POINTER(pbody);
    TriMesh mesh;
    _penv->Triangulate(mesh, *openravepy::GetKinBody(pbody));
    return toPyTriMesh(mesh);
}

object PyEnvironmentBase::TriangulateScene(const int options, const std::string &name)
{
    TriMesh mesh;
    _penv->TriangulateScene(mesh, (EnvironmentBase::SelectionOptions) options, name);
    return toPyTriMesh(mesh);
}

void PyEnvironmentBase::SetDebugLevel(object olevel) {
    _penv->SetDebugLevel(pyGetIntFromPy(olevel,Level_Info));
}
int PyEnvironmentBase::GetDebugLevel() const {
    return _penv->GetDebugLevel();
}

std::string PyEnvironmentBase::GetHomeDirectory() {
    RAVELOG_WARN("Environment.GetHomeDirectory is deprecated, use RaveGetHomeDirectory\n"); return RaveGetHomeDirectory();
}

void PyEnvironmentBase::SetUserData(PyUserData pdata) {
    _penv->SetUserData(pdata._handle);
}
void PyEnvironmentBase::SetUserData(object o) {
    _penv->SetUserData(OPENRAVE_SHARED_PTR<UserData>(new PyUserObject(o)));
}
object PyEnvironmentBase::GetUserData() const {
    return openravepy::GetUserData(_penv->GetUserData());
}

void PyEnvironmentBase::SetUnit(std::string unitname, dReal unitmult){
    _penv->SetUnit(std::make_pair(unitname, unitmult));
}

object PyEnvironmentBase::GetUnit() const {
    std::pair<std::string, dReal> unit = _penv->GetUnit();
    return py::make_tuple(unit.first, unit.second);

}

bool PyEnvironmentBase::__eq__(PyEnvironmentBasePtr p) {
    return !!p && _penv==p->_penv;
}
bool PyEnvironmentBase::__ne__(PyEnvironmentBasePtr p) {
    return !p || _penv!=p->_penv;
}
std::string PyEnvironmentBase::__repr__() {
    return boost::str(boost::format("RaveGetEnvironment(%d)")%RaveGetEnvironmentId(_penv));
}
std::string PyEnvironmentBase::__str__() {
    return boost::str(boost::format("<env %d>")%RaveGetEnvironmentId(_penv));
}
object PyEnvironmentBase::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

EnvironmentBasePtr PyEnvironmentBase::GetEnv() const {
    return _penv;
}

PyEnvironmentBasePtr PyInterfaceBase::GetEnv() const
{
#if BOOST_VERSION >= 103500
    return _pyenv;
#else
    // if raw shared_ptr is returned, then python will throw RuntimeError: tr1::bad_weak_ptr when env is used
    return PyEnvironmentBasePtr(new PyEnvironmentBase(_pyenv->GetEnv()));
#endif
}

object GetUserData(UserDataPtr pdata)
{
    OPENRAVE_SHARED_PTR<PyUserObject> po = OPENRAVE_DYNAMIC_POINTER_CAST<PyUserObject>(pdata);
    if( !!po ) {
        return po->_o;
    }
    else {
        SerializableDataPtr pserializable = OPENRAVE_DYNAMIC_POINTER_CAST<SerializableData>(pdata);
        if( !!pserializable ) {
            return py::to_object(PySerializableData(pserializable));
        }
        else if( !!pdata ) {
            return py::to_object(PyUserData(pdata));
        }
        else {
            return py::none_();
        }
    }
}

EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr pyenv)
{
    return !pyenv ? EnvironmentBasePtr() : pyenv->GetEnv();
}

EnvironmentBasePtr GetEnvironment(object o)
{
    if( !IS_PYTHONOBJECT_NONE(o)) {
        extract_<PyEnvironmentBasePtr> pyenv(o);
        if( pyenv.check() ) {
            return ((PyEnvironmentBasePtr)pyenv)->GetEnv();
        }
    }
    return EnvironmentBasePtr();
}

object toPyEnvironment(object o)
{
    extract_<PyInterfaceBasePtr> pyinterface(o);
    if( pyinterface.check() ) {
        return py::to_object(((PyInterfaceBasePtr)pyinterface)->GetEnv());
    }
    return py::none_();
}

void LockEnvironment(PyEnvironmentBasePtr pyenv)
{
    pyenv->Lock();
}

void UnlockEnvironment(PyEnvironmentBasePtr pyenv)
{
    pyenv->Unlock();
}

PyEnvironmentLockSaver::PyEnvironmentLockSaver(PyEnvironmentBasePtr pyenv, bool braw) : _pyenv(pyenv)
{
    if( braw ) {
        _pyenv->LockRaw();
    }
    else {
        _pyenv->Lock();
    }
}
PyEnvironmentLockSaver::~PyEnvironmentLockSaver()
{
    _pyenv->Unlock();
}

object RaveGetEnvironments()
{
    std::list<EnvironmentBasePtr> listenvironments;
    OpenRAVE::RaveGetEnvironments(listenvironments);
    py::list oenvironments;
    FOREACH(it,listenvironments) {
        oenvironments.append(PyEnvironmentBasePtr(new PyEnvironmentBase(*it)));
    }
    return oenvironments;
}
int RaveGetEnvironmentId(PyEnvironmentBasePtr pyenv)
{
    return OpenRAVE::RaveGetEnvironmentId(pyenv->GetEnv());
}

PyEnvironmentBasePtr RaveGetEnvironment(int id)
{
    EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(id);
    if( !penv ) {
        return PyEnvironmentBasePtr();
    }
    return PyEnvironmentBasePtr(new PyEnvironmentBase(penv));
}

PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name)
{
    InterfaceBasePtr p = OpenRAVE::RaveCreateInterface(pyenv->GetEnv(), type, name);
    if( !p ) {
        return PyInterfaceBasePtr();
    }
    return PyInterfaceBasePtr(new PyInterfaceBase(p,pyenv));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LoadURI_overloads, LoadURI, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetCamera_overloads, SetCamera, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(StartSimulation_overloads, StartSimulation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(StopSimulation_overloads, StopSimulation, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetViewer_overloads, SetViewer, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetDefaultViewer_overloads, SetDefaultViewer, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckCollisionRays_overloads, CheckCollisionRays, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(plot3_overloads, plot3, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinestrip_overloads, drawlinestrip, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinelist_overloads, drawlinelist, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawarrow_overloads, drawarrow, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawbox_overloads, drawbox, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawtrimesh_overloads, drawtrimesh, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SendCommand_overloads, SendCommand, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SendJSONCommand_overloads, SendJSONCommand, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Add_overloads, Add, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Save_overloads, Save, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(WriteToMemory_overloads, WriteToMemory, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetUserData_overloads, GetUserData, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBody_overloads, GetPublishedBody, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBodies_overloads, GetPublishedBodies, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBodyJointValues_overloads, GetPublishedBodyJointValues, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBodyTransformsMatchingPrefix_overloads, GetPublishedBodyTransformsMatchingPrefix, 1, 2)

object get_openrave_exception_unicode(OpenRAVEException* p)
{
    std::string s = p->message();
    return ConvertStringToUnicode(s);
}

std::string get_openrave_exception_repr(OpenRAVEException* p)
{
    return boost::str(boost::format("<OpenRAVEException('%s','%s')>")%p->message()%RaveGetErrorCodeString(p->GetCode()));
}

object get_std_runtime_error_unicode(std::runtime_error* p)
{
    std::string s(p->what());
    return ConvertStringToUnicode(s);
}

std::string get_std_runtime_error_repr(std::runtime_error* p)
{
    return boost::str(boost::format("<std_exception('%s')>")%p->what());
}

py::object GetCodeStringOpenRAVEException(OpenRAVEException* p)
{
    return ConvertStringToUnicode(RaveGetErrorCodeString(p->GetCode()));
}

#endif // USE_PYBIND11_PYTHON_BINDINGS

} // namespace openravepy

OPENRAVE_PYTHON_MODULE(openravepy_int)
{
    using namespace openravepy;
    import_array(); // not sure if this is necessary for pybind11
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
#else // USE_PYBIND11_PYTHON_BINDINGS
#if BOOST_VERSION >= 103500
    docstring_options doc_options;
    doc_options.disable_cpp_signatures();
    doc_options.enable_py_signatures();
    doc_options.enable_user_defined();
#endif
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_number<int>();
    int_from_number<uint8_t>();
    float_from_number<float>();
    float_from_number<double>();
    init_python_bindings();
    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS

    static PyOpenRAVEException<OpenRAVEException> pyOpenRAVEException(m, OPENRAVE_EXCEPTION_CLASS_NAME, PyExc_Exception);
    pyOpenRAVEException.def_property_readonly("message",[](py::object o) {
        return o.attr("args")[0];
    });
    pyOpenRAVEException.def_property_readonly("errortype",[](py::object o) {
        py::object oargs = o.attr("args");
        py::object oret;
        if( len(oargs) > 1 ) {
            oret = oargs[1];
        }
        else {
            oret = py::none_();
        }
        return oret;
    });
    pyOpenRAVEException.def("GetCode", [](py::object o) {
        py::object oargs = o.attr("args");
        py::object oret;
        if( len(oargs) > 1 ) {
            oret = oargs[1];
        }
        else {
            oret = py::none_();
        }
        return oret;
    });

    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) {
                std::rethrow_exception(p);
            }
        }
        catch( const boost::bad_function_call& e ) {
            py::object pyerrdata = ConvertStringToUnicode(e.what());
            pyerrdata.inc_ref(); // since passing to PyErr_SetObject
            PyErr_SetObject(PyExc_TypeError, pyerrdata.ptr() );
        }
        catch( const std::runtime_error& e ) {
            py::object pyerrdata = ConvertStringToUnicode(e.what());
            pyerrdata.inc_ref(); // since passing to PyErr_SetObject
            PyErr_SetObject(PyExc_RuntimeError, pyerrdata.ptr() );
        }
        catch( const OpenRAVEException &e ) {
            py::object pyerrdata = py::make_tuple(ConvertStringToUnicode(e.message()), ConvertStringToUnicode(RaveGetErrorCodeString(e.GetCode())));
            pyerrdata.inc_ref(); // since passing to PyErr_SetObject
            PyErr_SetObject(pyOpenRAVEException.ptr(), pyerrdata.ptr() );
        }
    });
#else
    class_< OpenRAVEException >( OPENRAVE_EXCEPTION_CLASS_NAME, DOXY_CLASS(OpenRAVEException) )
    .def( init<const std::string&>() )
    .def( init<const OpenRAVEException&>() )
    .def( "message", &OpenRAVEException::message, return_copy_const_ref() )
    .def("GetCode", GetCodeStringOpenRAVEException)
    .def( "__str__", &OpenRAVEException::message, return_copy_const_ref() )
    .def( "__unicode__", get_openrave_exception_unicode)
    .def( "__repr__", get_openrave_exception_repr)
    ;
    OpenRAVEBoostPythonExceptionTranslator<OpenRAVEException>();

    class_< std::runtime_error >( "_std_runtime_error_", no_init)
    .def( init<const std::string&>() )
    .def( init<const std::runtime_error&>() )
    .def( "message", &std::runtime_error::what)
    .def( "__str__", &std::runtime_error::what)
    .def( "__unicode__", get_std_runtime_error_unicode)
    .def( "__repr__", get_std_runtime_error_repr)
    ;
    OpenRAVEBoostPythonExceptionTranslator<std::runtime_error>();

    //OpenRAVEBoostPythonExceptionTranslator<std::exception>();
    class_< boost::bad_function_call, bases<std::runtime_error> >( "_boost_bad_function_call_");
    OpenRAVEBoostPythonExceptionTranslator<boost::bad_function_call>();
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
    class_<PyEnvironmentBase, PyEnvironmentBasePtr > classenv(m, "Environment", DOXY_CLASS(EnvironmentBase));
#else
    class_<PyEnvironmentBase, PyEnvironmentBasePtr > classenv("Environment", DOXY_CLASS(EnvironmentBase));
#endif
    {
        void (PyInterfaceBase::*setuserdata1)(PyUserData) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata2)(object) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata3)(const std::string&, PyUserData) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata4)(const std::string&, object) = &PyInterfaceBase::SetUserData;
        std::string sSendCommandDoc = std::string(DOXY_FN(InterfaceBase,SendCommand)) + std::string("The calling conventions between C++ and Python differ a little.\n\n\
In C++ the syntax is::\n\n  success = SendCommand(OUT, IN)\n\n\
In python, the syntax is::\n\n\
  OUT = SendCommand(IN,releasegil)\n\
  success = OUT is not None\n\n\n\
The **releasegil** parameter controls whether the python Global Interpreter Lock should be released when executing this code. For calls that take a long time and if there are many threads running called from different python threads, releasing the GIL could speed up things a lot. Please keep in mind that releasing and re-acquiring the GIL also takes computation time.\n\
Because race conditions can pop up when trying to lock the openrave environment without releasing the GIL, if lockenv=True is specified, the system can try to safely lock the openrave environment without causing a deadlock with the python GIL and other threads.\n");

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyInterfaceBase, OPENRAVE_SHARED_PTR<PyInterfaceBase> >(m, "Interface", DOXY_CLASS(InterfaceBase))
#else
        class_<PyInterfaceBase, OPENRAVE_SHARED_PTR<PyInterfaceBase> >("Interface", DOXY_CLASS(InterfaceBase), no_init)
#endif
        .def("GetInterfaceType",&PyInterfaceBase::GetInterfaceType, DOXY_FN(InterfaceBase,GetInterfaceType))
        .def("GetXMLId",&PyInterfaceBase::GetXMLId, DOXY_FN(InterfaceBase,GetXMLId))
        .def("GetPluginName",&PyInterfaceBase::GetPluginName, DOXY_FN(InterfaceBase,GetPluginName))
        .def("GetDescription",&PyInterfaceBase::GetDescription, DOXY_FN(InterfaceBase,GetDescription))
        .def("SetDescription",&PyInterfaceBase::SetDescription, DOXY_FN(InterfaceBase,SetDescription))
        .def("GetEnv",&PyInterfaceBase::GetEnv, DOXY_FN(InterfaceBase,GetEnv))
        .def("Clone",&PyInterfaceBase::Clone, PY_ARGS("ref","cloningoptions") DOXY_FN(InterfaceBase,Clone))
        .def("SetUserData",setuserdata1, PY_ARGS("data") DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata2, PY_ARGS("data") DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata3, PY_ARGS("key","data") DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata4, PY_ARGS("key", "data") DOXY_FN(InterfaceBase,SetUserData))
        .def("RemoveUserData", &PyInterfaceBase::RemoveUserData, DOXY_FN(InterfaceBase, RemoveUserData))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("GetUserData",&PyInterfaceBase::GetUserData,
             "key"_a = "",
             DOXY_FN(InterfaceBase,GetUserData)
             )
#else
        .def("GetUserData",&PyInterfaceBase::GetUserData, GetUserData_overloads(PY_ARGS("key") DOXY_FN(InterfaceBase,GetUserData)))
#endif
        .def("SupportsCommand",&PyInterfaceBase::SupportsCommand, PY_ARGS("cmd") DOXY_FN(InterfaceBase,SupportsCommand))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("SendCommand",&PyInterfaceBase::SendCommand,
             "cmd"_a,
             "releasegil"_a = false,
             "lockenv"_a = false,
             sSendCommandDoc.c_str()
             )
#else
        .def("SendCommand",&PyInterfaceBase::SendCommand, SendCommand_overloads(PY_ARGS("cmd","releasegil","lockenv") sSendCommandDoc.c_str()))
#endif
        .def("SupportsJSONCommand",&PyInterfaceBase::SupportsJSONCommand, PY_ARGS("cmd") DOXY_FN(InterfaceBase,SupportsJSONCommand))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("SendJSONCommand",&PyInterfaceBase::SendJSONCommand,
             "cmd"_a,
             "input"_a,
             "releasegil"_a = false,
             "lockenv"_a = false,
             DOXY_FN(InterfaceBase,SendJSONCommand)
             )
#else
        .def("SendJSONCommand",&PyInterfaceBase::SendJSONCommand, SendJSONCommand_overloads(PY_ARGS("cmd","input","releasegil","lockenv") DOXY_FN(InterfaceBase,SendJSONCommand)))
#endif
        .def("GetReadableInterfaces",&PyInterfaceBase::GetReadableInterfaces, DOXY_FN(InterfaceBase,GetReadableInterfaces))
        .def("GetReadableInterface",&PyInterfaceBase::GetReadableInterface, DOXY_FN(InterfaceBase,GetReadableInterface))
        .def("SetReadableInterface",&PyInterfaceBase::SetReadableInterface, PY_ARGS("id","readable") DOXY_FN(InterfaceBase,SetReadableInterface))
        .def("__repr__", &PyInterfaceBase::__repr__)
        .def("__str__", &PyInterfaceBase::__str__)
        .def("__unicode__", &PyInterfaceBase::__unicode__)
        .def("__hash__",&PyInterfaceBase::__hash__)
        .def("__eq__",&PyInterfaceBase::__eq__)
        .def("__ne__",&PyInterfaceBase::__ne__)
        ;
    }

    {
        bool (PyEnvironmentBase::*pcolb)(PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbr)(PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbb)(PyKinBodyPtr,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbbr)(PyKinBodyPtr, PyKinBodyPtr,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoll)(object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollr)(object, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolll)(object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolllr)(object,object, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollb)(object, PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollbr)(object, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolle)(object,object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoller)(object, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbe)(PyKinBodyPtr,object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolber)(PyKinBodyPtr, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolyb)(OPENRAVE_SHARED_PTR<PyRay>,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolybr)(OPENRAVE_SHARED_PTR<PyRay>, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoly)(OPENRAVE_SHARED_PTR<PyRay>) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolyr)(OPENRAVE_SHARED_PTR<PyRay>, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;

        void (PyEnvironmentBase::*Lock1)() = &PyEnvironmentBase::Lock;
        bool (PyEnvironmentBase::*Lock2)(float) = &PyEnvironmentBase::Lock;

        object (PyEnvironmentBase::*drawplane1)(object, object, const boost::multi_array<float,2>&) = &PyEnvironmentBase::drawplane;
        object (PyEnvironmentBase::*drawplane2)(object, object, const boost::multi_array<float,3>&) = &PyEnvironmentBase::drawplane;

        void (PyEnvironmentBase::*addkinbody1)(PyKinBodyPtr) = &PyEnvironmentBase::AddKinBody;
        void (PyEnvironmentBase::*addkinbody2)(PyKinBodyPtr,bool) = &PyEnvironmentBase::AddKinBody;
        void (PyEnvironmentBase::*addrobot1)(PyRobotBasePtr) = &PyEnvironmentBase::AddRobot;
        void (PyEnvironmentBase::*addrobot2)(PyRobotBasePtr,bool) = &PyEnvironmentBase::AddRobot;
        void (PyEnvironmentBase::*addsensor1)(PySensorBasePtr) = &PyEnvironmentBase::AddSensor;
        void (PyEnvironmentBase::*addsensor2)(PySensorBasePtr,bool) = &PyEnvironmentBase::AddSensor;
        void (PyEnvironmentBase::*setuserdata1)(PyUserData) = &PyEnvironmentBase::SetUserData;
        void (PyEnvironmentBase::*setuserdata2)(object) = &PyEnvironmentBase::SetUserData;
        bool (PyEnvironmentBase::*load1)(const std::string &) = &PyEnvironmentBase::Load;
        bool (PyEnvironmentBase::*load2)(const std::string &, object) = &PyEnvironmentBase::Load;
        bool (PyEnvironmentBase::*loaddata1)(const std::string &) = &PyEnvironmentBase::LoadData;
        bool (PyEnvironmentBase::*loaddata2)(const std::string &, object) = &PyEnvironmentBase::LoadData;
        object (PyEnvironmentBase::*readrobotxmlfile1)(const std::string &) = &PyEnvironmentBase::ReadRobotURI;
        object (PyEnvironmentBase::*readrobotxmlfile2)(const std::string &,object) = &PyEnvironmentBase::ReadRobotURI;
        object (PyEnvironmentBase::*readrobotxmldata1)(const std::string &) = &PyEnvironmentBase::ReadRobotData;
        object (PyEnvironmentBase::*readrobotxmldata2)(const std::string &,object) = &PyEnvironmentBase::ReadRobotData;
        object (PyEnvironmentBase::*readkinbodyxmlfile1)(const std::string &) = &PyEnvironmentBase::ReadKinBodyURI;
        object (PyEnvironmentBase::*readkinbodyxmlfile2)(const std::string &,object) = &PyEnvironmentBase::ReadKinBodyURI;
        object (PyEnvironmentBase::*readkinbodyxmldata1)(const std::string &) = &PyEnvironmentBase::ReadKinBodyData;
        object (PyEnvironmentBase::*readkinbodyxmldata2)(const std::string &,object) = &PyEnvironmentBase::ReadKinBodyData;
        PyInterfaceBasePtr (PyEnvironmentBase::*readinterfacexmlfile1)(const std::string &) = &PyEnvironmentBase::ReadInterfaceURI;
        PyInterfaceBasePtr (PyEnvironmentBase::*readinterfacexmlfile2)(const std::string &,object) = &PyEnvironmentBase::ReadInterfaceURI;
        object (PyEnvironmentBase::*readtrimeshfile1)(const std::string&) = &PyEnvironmentBase::ReadTrimeshURI;
        object (PyEnvironmentBase::*readtrimeshfile2)(const std::string&,object) = &PyEnvironmentBase::ReadTrimeshURI;
        object (PyEnvironmentBase::*readtrimeshdata1)(const std::string&,const std::string&) = &PyEnvironmentBase::ReadTrimeshData;
        object (PyEnvironmentBase::*readtrimeshdata2)(const std::string&,const std::string&,object) = &PyEnvironmentBase::ReadTrimeshData;

        scope_ env = classenv
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def(init<int>(), "options"_a = (int) ECO_StartSimulationThread)
                     .def(init<EnvironmentBasePtr>(), "penv"_a)
                     .def(init<const PyEnvironmentBase&>(), "penv"_a)
#else
                     .def(init<optional<int> >(py::args("options")))
#endif
                     .def("Reset",&PyEnvironmentBase::Reset, DOXY_FN(EnvironmentBase,Reset))
                     .def("Destroy",&PyEnvironmentBase::Destroy, DOXY_FN(EnvironmentBase,Destroy))
                     .def("CloneSelf",&PyEnvironmentBase::CloneSelf, PY_ARGS("options") DOXY_FN(EnvironmentBase,CloneSelf))
                     .def("Clone",&PyEnvironmentBase::Clone, PY_ARGS("reference","options") DOXY_FN(EnvironmentBase,Clone))
                     .def("SetCollisionChecker",&PyEnvironmentBase::SetCollisionChecker, PY_ARGS("collisionchecker") DOXY_FN(EnvironmentBase,SetCollisionChecker))
                     .def("GetCollisionChecker",&PyEnvironmentBase::GetCollisionChecker, DOXY_FN(EnvironmentBase,GetCollisionChecker))

                     .def("CheckCollision",pcolb, PY_ARGS("body") DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolbr, PY_ARGS("body","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolbb, PY_ARGS("body1","body2") DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolbbr, PY_ARGS("body1","body2","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcoll, PY_ARGS("link") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcollr, PY_ARGS("link","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolll, PY_ARGS("link1","link2") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolllr, PY_ARGS("link1","link2","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcollb, PY_ARGS("link","body") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcollbr, PY_ARGS("link","body","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolle, PY_ARGS("link","bodyexcluded","linkexcluded") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                     .def("CheckCollision",pcoller, PY_ARGS("link","bodyexcluded","linkexcluded","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                     .def("CheckCollision",pcolbe, PY_ARGS("body","bodyexcluded","linkexcluded") DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                     .def("CheckCollision",pcolber, PY_ARGS("body","bodyexcluded","linkexcluded","report") DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                     .def("CheckCollision",pcolyb, PY_ARGS("ray","body") DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcolybr, PY_ARGS("ray","body","report") DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
                     .def("CheckCollision",pcoly, PY_ARGS("ray") DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
                     .def("CheckCollision",pcolyr, PY_ARGS("ray", "report") DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,
                          "rays"_a,
                          "body"_a,
                          "front_facing_only"_a = false,
                          "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction*range."
                          )
#else
                     .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,
                          CheckCollisionRays_overloads(PY_ARGS("rays","body","front_facing_only")
                                                       "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction*range."))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("LoadURI", &PyEnvironmentBase::LoadURI,
                          "filename"_a,
                          "atts"_a = py::none_(),
                          DOXY_FN(EnvironmentBase, LoadURI)
                          )
#else
                     .def("LoadURI",&PyEnvironmentBase::LoadURI,LoadURI_overloads(PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,LoadURI)))
#endif
                     .def("Load",load1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,Load))
                     .def("Load",load2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,Load))
                     .def("LoadData",loaddata1, PY_ARGS("data") DOXY_FN(EnvironmentBase,LoadData))
                     .def("LoadData",loaddata2, PY_ARGS("data","atts") DOXY_FN(EnvironmentBase,LoadData))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("Save",&PyEnvironmentBase::Save,
                          "filename"_a,
                          "options"_a = (int) EnvironmentBase::SelectionOptions::SO_Everything,
                          "atts"_a = py::none_(),
                          DOXY_FN(EnvironmentBase,Save)
                          )
#else
                     .def("Save",&PyEnvironmentBase::Save,Save_overloads(PY_ARGS("filename","options","atts") DOXY_FN(EnvironmentBase,Save)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("WriteToMemory",&PyEnvironmentBase::WriteToMemory,
                          "filetype"_a,
                          "options"_a = (int) EnvironmentBase::SelectionOptions::SO_Everything,
                          "atts"_a = py::none_(),
                          DOXY_FN(EnvironmentBase,WriteToMemory)
                          )
#else
                     .def("WriteToMemory",&PyEnvironmentBase::WriteToMemory,WriteToMemory_overloads(PY_ARGS("filetype","options","atts") DOXY_FN(EnvironmentBase,WriteToMemory)))
#endif
                     .def("ReadRobotURI",readrobotxmlfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadRobotURI "const std::string"))
                     .def("ReadRobotXMLFile",readrobotxmlfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadRobotURI "const std::string"))
                     .def("ReadRobotURI",readrobotxmlfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadRobotURI "RobotBasePtr; const std::string; const AttributesList"))
                     .def("ReadRobotXMLFile",readrobotxmlfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadRobotURI "RobotBasePtr; const std::string; const AttributesList"))
                     .def("ReadRobotData",readrobotxmldata1, PY_ARGS("data") DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                     .def("ReadRobotXMLData",readrobotxmldata1, PY_ARGS("data") DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                     .def("ReadRobotData",readrobotxmldata2, PY_ARGS("data","atts") DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                     .def("ReadRobotXMLData",readrobotxmldata2, PY_ARGS("data","atts") DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                     .def("ReadKinBodyURI",readkinbodyxmlfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadKinBodyURI "const std::string"))
                     .def("ReadKinBodyXMLFile",readkinbodyxmlfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadKinBodyURI "const std::string"))
                     .def("ReadKinBodyURI",readkinbodyxmlfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadKinBodyURI "KinBody; const std::string; const AttributesList"))
                     .def("ReadKinBodyXMLFile",readkinbodyxmlfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadKinBodyURI "KinBody; const std::string; const AttributesList"))
                     .def("ReadKinBodyData",readkinbodyxmldata1, PY_ARGS("data") DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                     .def("ReadKinBodyXMLData",readkinbodyxmldata1, PY_ARGS("data") DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                     .def("ReadKinBodyData",readkinbodyxmldata2, PY_ARGS("data","atts") DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                     .def("ReadKinBodyXMLData",readkinbodyxmldata2, PY_ARGS("data","atts") DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                     .def("ReadInterfaceURI",readinterfacexmlfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                     .def("ReadInterfaceXMLFile",readinterfacexmlfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                     .def("ReadInterfaceURI",readinterfacexmlfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                     .def("ReadInterfaceXMLFile",readinterfacexmlfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                     .def("ReadTrimeshURI",readtrimeshfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                     .def("ReadTrimeshURI",readtrimeshfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                     .def("ReadTrimeshFile",readtrimeshfile1, PY_ARGS("filename") DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                     .def("ReadTrimeshFile",readtrimeshfile2, PY_ARGS("filename","atts") DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                     .def("ReadTrimeshData",readtrimeshdata1, PY_ARGS("data", "formathint") DOXY_FN(EnvironmentBase,ReadTrimeshData))
                     .def("ReadTrimeshData",readtrimeshdata2, PY_ARGS("data","formathint","atts") DOXY_FN(EnvironmentBase,ReadTrimeshData))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("Add", &PyEnvironmentBase::Add,
                          "interface"_a,
                          "anonymous"_a = false,
                          "cmdargs"_a = "",
                          DOXY_FN(EnvironmentBase, Add)
                          )
#else
                     .def("Add", &PyEnvironmentBase::Add, Add_overloads(PY_ARGS("interface","anonymous","cmdargs") DOXY_FN(EnvironmentBase,Add)))
#endif
                     .def("AddKinBody",addkinbody1, PY_ARGS("body") DOXY_FN(EnvironmentBase,AddKinBody))
                     .def("AddKinBody",addkinbody2, PY_ARGS("body","anonymous") DOXY_FN(EnvironmentBase,AddKinBody))
                     .def("AddRobot",addrobot1, PY_ARGS("robot") DOXY_FN(EnvironmentBase,AddRobot))
                     .def("AddRobot",addrobot2, PY_ARGS("robot","anonymous") DOXY_FN(EnvironmentBase,AddRobot))
                     .def("AddSensor",addsensor1, PY_ARGS("sensor") DOXY_FN(EnvironmentBase,AddSensor))
                     .def("AddSensor",addsensor2, PY_ARGS("sensor","anonymous") DOXY_FN(EnvironmentBase,AddSensor))
                     .def("AddViewer",addsensor2, PY_ARGS("sensor","anonymous") DOXY_FN(EnvironmentBase,AddViewer))
                     .def("RemoveKinBody",&PyEnvironmentBase::RemoveKinBody, PY_ARGS("body") DOXY_FN(EnvironmentBase,RemoveKinBody))
                     .def("RemoveKinBodyByName",&PyEnvironmentBase::RemoveKinBodyByName, PY_ARGS("name") DOXY_FN(EnvironmentBase,RemoveKinBodyByName))
                     .def("Remove",&PyEnvironmentBase::Remove, PY_ARGS("interface") DOXY_FN(EnvironmentBase,Remove))
                     .def("GetKinBody",&PyEnvironmentBase::GetKinBody, PY_ARGS("name") DOXY_FN(EnvironmentBase,GetKinBody))
                     .def("GetRobot",&PyEnvironmentBase::GetRobot, PY_ARGS("name") DOXY_FN(EnvironmentBase,GetRobot))
                     .def("GetSensor",&PyEnvironmentBase::GetSensor, PY_ARGS("name") DOXY_FN(EnvironmentBase,GetSensor))
                     .def("GetBodyFromEnvironmentId",&PyEnvironmentBase::GetBodyFromEnvironmentId, DOXY_FN(EnvironmentBase,GetBodyFromEnvironmentId))
                     .def("AddModule",&PyEnvironmentBase::AddModule,PY_ARGS("module","args") DOXY_FN(EnvironmentBase,AddModule))
                     .def("LoadProblem",&PyEnvironmentBase::AddModule,PY_ARGS("module","args") DOXY_FN(EnvironmentBase,AddModule))
                     .def("RemoveProblem",&PyEnvironmentBase::RemoveProblem, PY_ARGS("prob") DOXY_FN(EnvironmentBase,RemoveProblem))
                     .def("GetModules",&PyEnvironmentBase::GetModules, DOXY_FN(EnvironmentBase,GetModules))
                     .def("GetLoadedProblems",&PyEnvironmentBase::GetModules, DOXY_FN(EnvironmentBase,GetModules))
                     .def("SetPhysicsEngine",&PyEnvironmentBase::SetPhysicsEngine, PY_ARGS("physics") DOXY_FN(EnvironmentBase,SetPhysicsEngine))
                     .def("GetPhysicsEngine",&PyEnvironmentBase::GetPhysicsEngine, DOXY_FN(EnvironmentBase,GetPhysicsEngine))
                     .def("RegisterBodyCallback",&PyEnvironmentBase::RegisterBodyCallback, PY_ARGS("callback") DOXY_FN(EnvironmentBase,RegisterBodyCallback))
                     .def("RegisterCollisionCallback",&PyEnvironmentBase::RegisterCollisionCallback, PY_ARGS("callback") DOXY_FN(EnvironmentBase,RegisterCollisionCallback))
                     .def("HasRegisteredCollisionCallbacks",&PyEnvironmentBase::HasRegisteredCollisionCallbacks,DOXY_FN(EnvironmentBase,HasRegisteredCollisionCallbacks))
                     .def("StepSimulation",&PyEnvironmentBase::StepSimulation, PY_ARGS("timestep") DOXY_FN(EnvironmentBase,StepSimulation))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("StartSimulation", &PyEnvironmentBase::StartSimulation,
                          "timestep"_a,
                          "realtime"_a = true,
                          DOXY_FN(EnvironmentBase,StartSimulation)
                          )
#else
                     .def("StartSimulation",&PyEnvironmentBase::StartSimulation,StartSimulation_overloads(PY_ARGS("timestep","realtime") DOXY_FN(EnvironmentBase,StartSimulation)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("StopSimulation", &PyEnvironmentBase::StopSimulation,
                          "shutdownthread"_a = 1,
                          DOXY_FN(EnvironmentBase,StopSimulation)
                          )
#else
                     .def("StopSimulation",&PyEnvironmentBase::StopSimulation, StopSimulation_overloads(PY_ARGS("shutdownthread") DOXY_FN(EnvironmentBase,StopSimulation)))
#endif
                     .def("GetSimulationTime",&PyEnvironmentBase::GetSimulationTime, DOXY_FN(EnvironmentBase,GetSimulationTime))
                     .def("IsSimulationRunning",&PyEnvironmentBase::IsSimulationRunning, DOXY_FN(EnvironmentBase,IsSimulationRunning))
                     .def("Lock",Lock1,"Locks the environment mutex.")
                     .def("Lock",Lock2,PY_ARGS("timeout") "Locks the environment mutex with a timeout.")
                     .def("Unlock",&PyEnvironmentBase::Unlock,"Unlocks the environment mutex.")
                     .def("TryLock",&PyEnvironmentBase::TryLock,"Tries to locks the environment mutex, returns false if it failed.")
                     .def("LockPhysics", Lock1, "Locks the environment mutex.")
                     .def("LockPhysics", Lock2, PY_ARGS("timeout") "Locks the environment mutex with a timeout.")
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("SetViewer", &PyEnvironmentBase::SetViewer,
                          "viewername"_a,
                          "showviewer"_a = true,
                          "Attaches the viewer and starts its thread"
                          )
#else
                     .def("SetViewer",&PyEnvironmentBase::SetViewer,SetViewer_overloads(PY_ARGS("viewername","showviewer") "Attaches the viewer and starts its thread"))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("SetDefaultViewer", &PyEnvironmentBase::SetDefaultViewer,
                          "showviewer"_a = true,
                          "Attaches the default viewer (controlled by environment variables and internal settings) and starts its thread"
                          )
#else
                     .def("SetDefaultViewer",&PyEnvironmentBase::SetDefaultViewer,SetDefaultViewer_overloads(PY_ARGS("showviewer") "Attaches the default viewer (controlled by environment variables and internal settings) and starts its thread"))
#endif
                     .def("GetViewer",&PyEnvironmentBase::GetViewer, DOXY_FN(EnvironmentBase,GetViewer))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("plot3", &PyEnvironmentBase::plot3,
                          "points"_a,
                          "pointsize"_a,
                          "colors"_a = py::none_(),
                          "drawstyle"_a = 0,
                          DOXY_FN(EnvironmentBase,plot3 "const float; int; int; float; const float; int, bool")
                          )
#else
                     .def("plot3",&PyEnvironmentBase::plot3,plot3_overloads(PY_ARGS("points","pointsize","colors","drawstyle") DOXY_FN(EnvironmentBase,plot3 "const float; int; int; float; const float; int, bool")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("drawlinestrip",&PyEnvironmentBase::drawlinestrip,
                          "points"_a,
                          "linewidth"_a,
                          "colors"_a = py::none_(),
                          "drawstyle"_a = 0,
                          DOXY_FN(EnvironmentBase,drawlinestrip "const float; int; int; float; const float")
                          )
#else
                     .def("drawlinestrip",&PyEnvironmentBase::drawlinestrip,drawlinestrip_overloads(PY_ARGS("points","linewidth","colors","drawstyle") DOXY_FN(EnvironmentBase,drawlinestrip "const float; int; int; float; const float")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("drawlinelist", &PyEnvironmentBase::drawlinelist,
                          "points"_a,
                          "linewidth"_a,
                          "colors"_a = py::none_(),
                          "drawstyle"_a = 0,
                          DOXY_FN(EnvironmentBase,drawlinelist "const float; int; int; float; const float")
                          )
#else
                     .def("drawlinelist",&PyEnvironmentBase::drawlinelist,drawlinelist_overloads(PY_ARGS("points","linewidth","colors","drawstyle") DOXY_FN(EnvironmentBase,drawlinelist "const float; int; int; float; const float")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("drawarrow", &PyEnvironmentBase::drawarrow,
                          "p1"_a,
                          "p2"_a,
                          "linewidth"_a = 2e-3,
                          "color"_a = py::none_(),
                          DOXY_FN(EnvironmentBase,drawarrow)
                          )
#else
                     .def("drawarrow",&PyEnvironmentBase::drawarrow,drawarrow_overloads(PY_ARGS("p1","p2","linewidth","color") DOXY_FN(EnvironmentBase,drawarrow)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("drawbox", &PyEnvironmentBase::drawbox,
                          "pos"_a,
                          "extents"_a,
                          "color"_a = py::none_(),
                          DOXY_FN(EnvironmentBase,drawbox)
                          )
#else
                     .def("drawbox",&PyEnvironmentBase::drawbox,drawbox_overloads(PY_ARGS("pos","extents","color") DOXY_FN(EnvironmentBase,drawbox)))
#endif
                     .def("drawplane",drawplane1, PY_ARGS("transform","extents","texture") DOXY_FN(EnvironmentBase,drawplane))
                     .def("drawplane",drawplane2, PY_ARGS("transform","extents","texture") DOXY_FN(EnvironmentBase,drawplane))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("drawtrimesh", &PyEnvironmentBase::drawtrimesh,
                          "points"_a,
                          "indices"_a = py::none_(),
                          "colors"_a = py::none_(),
                          DOXY_FN(EnvironmentBase, drawtrimesh "const float; int; const int; int; const boost::multi_array")
                          )
#else
                     .def("drawtrimesh",&PyEnvironmentBase::drawtrimesh,drawtrimesh_overloads(PY_ARGS("points","indices","colors") DOXY_FN(EnvironmentBase,drawtrimesh "const float; int; const int; int; const boost::multi_array")))
#endif
                     .def("GetRobots",&PyEnvironmentBase::GetRobots, DOXY_FN(EnvironmentBase,GetRobots))
                     .def("GetBodies",&PyEnvironmentBase::GetBodies, DOXY_FN(EnvironmentBase,GetBodies))
                     .def("GetSensors",&PyEnvironmentBase::GetSensors, DOXY_FN(EnvironmentBase,GetSensors))
                     .def("UpdatePublishedBodies",&PyEnvironmentBase::UpdatePublishedBodies, DOXY_FN(EnvironmentBase,UpdatePublishedBodies))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                     .def("GetPublishedBody",&PyEnvironmentBase::GetPublishedBody,
                          "name"_a,
                          "timeout"_a = 0,
                          DOXY_FN(EnvironmentBase,GetPublishedBody))
                     .def("GetPublishedBodies", &PyEnvironmentBase::GetPublishedBodies,
                          "timeout"_a = 0,
                          DOXY_FN(EnvironmentBase,GetPublishedBodies))
                     .def("GetPublishedBodyJointValues", &PyEnvironmentBase::GetPublishedBodyJointValues,
                          "name"_a,
                          "timeout"_a=0,
                          DOXY_FN(EnvironmentBase,GetPublishedBodyJointValues))
                     .def("GetPublishedBodyTransformsMatchingPrefix", &PyEnvironmentBase::GetPublishedBodyTransformsMatchingPrefix,
                          "prefix"_a,
                          "timeout"_a = 0,
                          DOXY_FN(EnvironmentBase,GetPublishedBodyTransformsMatchingPrefix))
#else
                     .def("GetPublishedBody",&PyEnvironmentBase::GetPublishedBody, GetPublishedBody_overloads(PY_ARGS("name", "timeout") DOXY_FN(EnvironmentBase,GetPublishedBody)))
                     .def("GetPublishedBodies",&PyEnvironmentBase::GetPublishedBodies, GetPublishedBodies_overloads(PY_ARGS("timeout") DOXY_FN(EnvironmentBase,GetPublishedBodies)))

                     .def("GetPublishedBodyJointValues",&PyEnvironmentBase::GetPublishedBodyJointValues, GetPublishedBodyJointValues_overloads(PY_ARGS("name", "timeout") DOXY_FN(EnvironmentBase,GetPublishedBodyJointValues)))

                     .def("GetPublishedBodyTransformsMatchingPrefix",&PyEnvironmentBase::GetPublishedBodyTransformsMatchingPrefix, GetPublishedBodyTransformsMatchingPrefix_overloads(PY_ARGS("prefix", "timeout") DOXY_FN(EnvironmentBase,GetPublishedBodyTransformsMatchingPrefix)))
#endif
                     .def("Triangulate",&PyEnvironmentBase::Triangulate, PY_ARGS("body") DOXY_FN(EnvironmentBase,Triangulate))
                     .def("TriangulateScene",&PyEnvironmentBase::TriangulateScene, PY_ARGS("options","name") DOXY_FN(EnvironmentBase,TriangulateScene))
                     .def("SetDebugLevel",&PyEnvironmentBase::SetDebugLevel, PY_ARGS("level") DOXY_FN(EnvironmentBase,SetDebugLevel))
                     .def("GetDebugLevel",&PyEnvironmentBase::GetDebugLevel, DOXY_FN(EnvironmentBase,GetDebugLevel))
                     .def("GetHomeDirectory",&PyEnvironmentBase::GetHomeDirectory, DOXY_FN(EnvironmentBase,GetHomeDirectory))
                     .def("SetUserData",setuserdata1, PY_ARGS("data") DOXY_FN(InterfaceBase,SetUserData))
                     .def("SetUserData",setuserdata2, PY_ARGS("data") DOXY_FN(InterfaceBase,SetUserData))
                     .def("GetUserData",&PyEnvironmentBase::GetUserData, DOXY_FN(InterfaceBase,GetUserData))
                     .def("GetUnit",&PyEnvironmentBase::GetUnit, DOXY_FN(EnvironmentBase,GetUnit))
                     .def("SetUnit",&PyEnvironmentBase::SetUnit, PY_ARGS("unitname","unitmult") DOXY_FN(EnvironmentBase,SetUnit))
                     .def("__enter__",&PyEnvironmentBase::__enter__)
                     .def("__exit__",&PyEnvironmentBase::__exit__)
                     .def("__eq__",&PyEnvironmentBase::__eq__)
                     .def("__ne__",&PyEnvironmentBase::__ne__)
                     .def("__repr__",&PyEnvironmentBase::__repr__)
                     .def("__str__",&PyEnvironmentBase::__str__)
                     .def("__unicode__",&PyEnvironmentBase::__unicode__)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        object selectionoptions = enum_<EnvironmentBase::SelectionOptions>(env, "SelectionOptions" DOXY_ENUM(SelectionOptions))
#else
        object selectionoptions = enum_<EnvironmentBase::SelectionOptions>("SelectionOptions" DOXY_ENUM(SelectionOptions))
#endif
                                  .value("NoRobots",EnvironmentBase::SelectionOptions::SO_NoRobots)
                                  .value("Robots",EnvironmentBase::SelectionOptions::SO_Robots)
                                  .value("Everything",EnvironmentBase::SelectionOptions::SO_Everything)
                                  .value("Body",EnvironmentBase::SelectionOptions::SO_Body)
                                  .value("AllExceptBody",EnvironmentBase::SelectionOptions::SO_AllExceptBody)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                  .export_values()
#endif
        ;
        env.attr("TriangulateOptions") = selectionoptions;
    }

    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ options = class_<DummyStruct>(m, "options").def_readwrite_static
                             ("returnTransformQuaternion", &s_bReturnTransformQuaternions);
#else
        scope_ options = class_<DummyStruct>("options").add_static_property
                             ("returnTransformQuaternion",GetReturnTransformQuaternions,SetReturnTransformQuaternions);
#endif

    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.attr("__version__") = OPENRAVE_VERSION_STRING;
    m.attr("__author__") = "Rosen Diankov, Guangning Tan";
    m.attr("__copyright__") = "2009-2019 Rosen Diankov (rosen.diankov@gmail.com), Guangning Tan (tgntanguangning@gmail.com)";
    m.attr("__license__") = "Lesser GPL";
    m.attr("__docformat__") = "restructuredtext";
    m.attr("__pythonbinding__") = "pybind11";
#else
    scope().attr("__version__") = OPENRAVE_VERSION_STRING;
    scope().attr("__author__") = "Rosen Diankov, Guangning Tan";
    scope().attr("__copyright__") = "2009-2019 Rosen Diankov (rosen.diankov@gmail.com), Guangning Tan (tgntanguangning@gmail.com)";
    scope().attr("__license__") = "Lesser GPL";
    scope().attr("__docformat__") = "restructuredtext";
    scope().attr("__pythonbinding__") = "Boost.Python." + std::to_string(BOOST_VERSION);
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    openravepy::init_openravepy_global(m);
    openravepy::InitPlanningUtils(m);

    openravepy::init_openravepy_collisionchecker(m);
    openravepy::init_openravepy_controller(m);
    openravepy::init_openravepy_ikparameterization(m);
    openravepy::init_openravepy_iksolver(m);
    openravepy::init_openravepy_kinbody(m);
    openravepy::init_openravepy_robot(m);
    openravepy::init_openravepy_module(m);
    openravepy::init_openravepy_physicsengine(m);
    openravepy::init_openravepy_planner(m);
    openravepy::init_openravepy_trajectory(m);
    openravepy::init_openravepy_sensor(m);
    openravepy::init_openravepy_sensorsystem(m);
    openravepy::init_openravepy_spacesampler(m);
    openravepy::init_openravepy_viewer(m);
#else
    openravepy::init_openravepy_global();
    openravepy::InitPlanningUtils();

    openravepy::init_openravepy_collisionchecker();
    openravepy::init_openravepy_controller();
    openravepy::init_openravepy_ikparameterization();
    openravepy::init_openravepy_iksolver();
    openravepy::init_openravepy_kinbody();
    openravepy::init_openravepy_robot();
    openravepy::init_openravepy_module();
    openravepy::init_openravepy_physicsengine();
    openravepy::init_openravepy_planner();
    openravepy::init_openravepy_trajectory();
    openravepy::init_openravepy_sensor();
    openravepy::init_openravepy_sensorsystem();
    openravepy::init_openravepy_spacesampler();
    openravepy::init_openravepy_viewer();
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetEnvironmentId", openravepy::RaveGetEnvironmentId, DOXY_FN1(RaveGetEnvironmentId));
    m.def("RaveGetEnvironment", openravepy::RaveGetEnvironment, DOXY_FN1(RaveGetEnvironment));
    m.def("RaveGetEnvironments", openravepy::RaveGetEnvironments, DOXY_FN1(RaveGetEnvironments));
    m.def("RaveCreateInterface", openravepy::RaveCreateInterface, PY_ARGS("env","type","name") DOXY_FN1(RaveCreateInterface));
#else
    def("RaveGetEnvironmentId",openravepy::RaveGetEnvironmentId,DOXY_FN1(RaveGetEnvironmentId));
    def("RaveGetEnvironment",openravepy::RaveGetEnvironment,DOXY_FN1(RaveGetEnvironment));
    def("RaveGetEnvironments",openravepy::RaveGetEnvironments,DOXY_FN1(RaveGetEnvironments));
    def("RaveCreateInterface",openravepy::RaveCreateInterface, PY_ARGS("env","type","name") DOXY_FN1(RaveCreateInterface));
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
// bind enums
    py::enum_<OpenRAVE::EnvironmentCreateOptions>(m, "EnvironmentCreateOptions")
    .value("ECO_StartSimulationThread", OpenRAVE::EnvironmentCreateOptions::ECO_StartSimulationThread)
    .export_values();
#endif // USE_PYBIND11_PYTHON_BINDINGS

}
