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
#ifndef OPENRAVEPY_INTERNAL_ENVIRONMENTBASE_H
#define OPENRAVEPY_INTERNAL_ENVIRONMENTBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>
#include <boost/scoped_ptr.hpp>

#include <condition_variable>

namespace openravepy {
using py::object;

/// \brief manages all the viewers created through SetViewer into a single thread
class ViewerManager
{
    /// \brief info about the viewer to create or that is created
    struct ViewerInfo
    {
        EnvironmentBasePtr _penv;
        std::string _viewername;
        ViewerBasePtr _pviewer; /// the created viewer, managed by _RunViewerThread
        std::condition_variable _cond;  ///< notify when viewer thread is done processing and has initialized _pviewer
        bool _bShowViewer = false; ///< true if should show the viewer when initially created
        bool _bFailed = false; ///< if true, failed adding the viewer once, so do not try again
        bool _bDestroyed = false; ///< if true, then viewer is already destroyed
    };
    typedef OPENRAVE_SHARED_PTR<ViewerInfo> ViewerInfoPtr;
public:
    ViewerManager();

    virtual ~ViewerManager();

    static ViewerManager& GetInstance();

    /// \brief adds a viewer to the environment whose GUI thread will be managed by _RunViewerThread.
    ///
    /// Try to block until viewer is created, unless main thread is already taken up.
    /// \param bDoNotAddIfExists if true, will not add a viewer if one already exists and is added to the manager
    void AddViewer(EnvironmentBasePtr penv, const string &strviewer, bool bShowViewer, bool bDoNotAddIfExists);

    /// \brief if removed, returns true
    bool RemoveViewer(ViewerBasePtr pviewer);

    /// \brief if anything removed, returns true
    bool RemoveViewersOfEnvironment(EnvironmentBasePtr penv);

    void Initialize();
    void Destroy();

protected:
    void _RunViewerThread();

    static void _InitializeSingleton();

    OPENRAVE_SHARED_PTR<std::thread> _threadviewer;
    std::mutex _mutexViewer;
    std::condition_variable _conditionViewer;
    std::list<ViewerInfoPtr> _listviewerinfos;

    bool _bShutdown = false; ///< if true, shutdown everything
    bool _bInMain = false; ///< if true, viewer thread is running a main function

    static boost::scoped_ptr<ViewerManager> _singleton; ///< singleton
    static std::once_flag _onceInitialize; ///< makes sure initialization is atomic
};

class OPENRAVEPY_API PyEnvironmentBase : public OPENRAVE_ENABLE_SHARED_FROM_THIS<PyEnvironmentBase>
{
    std::mutex _envmutex;
    std::list<OPENRAVE_SHARED_PTR<EnvironmentLock> > _listenvlocks, _listfreelocks;

public:
    class PyEnvironmentBaseInfo
    {
public:
        PyEnvironmentBaseInfo();
        PyEnvironmentBaseInfo(const EnvironmentBase::EnvironmentBaseInfo& info);
        py::object SerializeJSON(dReal fUnitScale=1.0, py::object options=py::none_());
        void DeserializeJSON(py::object obj, dReal fUnitScale=1.0, py::object options=py::none_());
        EnvironmentBase::EnvironmentBaseInfoPtr GetEnvironmentBaseInfo() const;
        int _revision = 0;

        object _gravity = toPyVector3(Vector(0,0,-9.797930195020351));
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        std::vector<std::string> _keywords;
        py::object _vBodyInfos = py::none_();
        std::string _name;
        std::string _description;
#else
        py::list _keywords;
        py::object _vBodyInfos = py::none_();
        py::object _name = py::none_();
        py::object _description = py::none_();
#endif
        virtual std::string __str__();
        virtual py::object __unicode__();

protected:
        void _Update(const EnvironmentBase::EnvironmentBaseInfo& info);
    }; // class PyEnvironmentBaseInfo
    typedef OPENRAVE_SHARED_PTR<PyEnvironmentBaseInfo> PyEnvironmentBaseInfoPtr;

protected:
    EnvironmentBasePtr _penv;

    PyInterfaceBasePtr _toPyInterface(InterfaceBasePtr pinterface);

    void _BodyCallback(object fncallback, KinBodyPtr pbody, int action);

    CollisionAction _CollisionCallback(object fncallback, CollisionReportPtr preport, bool bFromPhysics);

public:
    PyEnvironmentBase(int options=ECO_StartSimulationThread);
    PyEnvironmentBase(const std::string& name, int options=ECO_StartSimulationThread);
    PyEnvironmentBase(EnvironmentBasePtr penv);

    PyEnvironmentBase(const PyEnvironmentBase &pyenv);

    virtual ~PyEnvironmentBase();

    void Reset();
    void Destroy();

    PyEnvironmentBasePtr CloneSelf(int options);
    PyEnvironmentBasePtr CloneSelf(const std::string& clonedEnvName, int options);

    void Clone(PyEnvironmentBasePtr pyreference, int options);
    void Clone(PyEnvironmentBasePtr pyreference, const std::string& clonedEnvName, int options);

    bool SetCollisionChecker(PyCollisionCheckerBasePtr pchecker);
    object GetCollisionChecker();
    bool CheckCollision(PyKinBodyPtr pbody1);
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport);

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2);

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1);

    bool CheckCollision(object o1, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1, object o2);
    bool CheckCollision(object o1, object o2, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1, PyKinBodyPtr pybody2);

    bool CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport);

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded);

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport);

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded);

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport);

    object CheckCollisionRays(py::numeric::array rays, PyKinBodyPtr pbody,bool bFrontFacingOnly=false);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray);

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyCollisionReportPtr pReport);

    bool Load(const std::string &filename);
    bool Load(const std::string &filename, object odictatts);
    bool LoadURI(const std::string &filename, object odictatts=py::none_());
    py::object LoadJSON(py::object oEnvInfo, UpdateFromInfoMode updateMode, object odictatts=py::none_(), const std::string &uri = "");
    bool LoadData(const std::string &data);
    bool LoadData(const std::string &data, object odictatts);

    void Save(const std::string &filename, const int options = EnvironmentBase::SelectionOptions::SO_Everything, object odictatts = py::none_());

    object WriteToMemory(const std::string &filetype, const int options = EnvironmentBase::SelectionOptions::SO_Everything, object odictatts = py::none_());

    /// will be unlocking GIL since doing FS or memory-intensive operations
    //@{
    object ReadRobotURI(const std::string &filename);
    object ReadRobotURI(const std::string &filename, object odictatts);
    object ReadRobotData(const std::string &data, object odictatts=py::none_(), const std::string&uri=std::string());
    object ReadRobotJSON(py::object oEnvInfo, object odictatts=py::none_(), const std::string &uri = std::string());
    object ReadKinBodyURI(const std::string &filename);
    object ReadKinBodyURI(const std::string &filename, object odictatts);
    object ReadKinBodyData(const std::string &data);
    object ReadKinBodyData(const std::string &data, object odictatts);
    object ReadKinBodyJSON(py::object oEnvInfo, object odictatts=py::none_(), const std::string &uri = std::string());
    PyInterfaceBasePtr ReadInterfaceURI(const std::string& filename);
    PyInterfaceBasePtr ReadInterfaceURI(const std::string& filename, object odictatts);
    //@}

    object ReadTrimeshURI(const std::string& filename);
    object ReadTrimeshURI(const std::string& filename, object odictatts);

    object ReadTrimeshData(const std::string& data, const std::string& formathint);
    object ReadTrimeshData(const std::string& data, const std::string& formathint, object odictatts);

    void Add(PyInterfaceBasePtr pinterface, py::object addMode=py::none_(), const std::string& cmdargs=std::string());

    void AddKinBody(PyKinBodyPtr pbody);
    void AddKinBody(PyKinBodyPtr pbody, bool bAnonymous);
    void AddRobot(PyRobotBasePtr robot);
    void AddRobot(PyRobotBasePtr robot, bool bAnonymous);
    void AddSensor(PySensorBasePtr sensor);
    void AddSensor(PySensorBasePtr sensor, bool bAnonymous);
    void AddViewer(PyViewerBasePtr viewer);

    bool RemoveKinBody(PyKinBodyPtr pbody);

    bool RemoveKinBodyByName(const std::string& name);

    object GetKinBody(const std::string &name);
    object GetRobot(const std::string &name);
    object GetSensor(const std::string &name);

    object GetBodyFromEnvironmentId(int id);
    object GetBodyFromEnvironmentBodyIndex(int id);
    object GetBodiesFromEnvironmentBodyIndices(object bodyIndices);

    int GetMaxEnvironmentBodyIndex();

    int AddModule(PyModuleBasePtr prob, const std::string &args);
    bool RemoveProblem(PyModuleBasePtr prob);
    bool Remove(PyInterfaceBasePtr obj);

    object GetModules();

    bool SetPhysicsEngine(PyPhysicsEngineBasePtr pengine);
    object GetPhysicsEngine();

    object RegisterBodyCallback(object fncallback);

    object RegisterCollisionCallback(object fncallback);

    bool HasRegisteredCollisionCallbacks();

    void StepSimulation(dReal timeStep);
    void StartSimulation(dReal fDeltaTime, bool bRealTime=true);
    void StopSimulation(int shutdownthread=1);
    uint64_t GetSimulationTime();
    bool IsSimulationRunning();

    void Lock();

    /// \brief raw locking without any python overhead
    void LockRaw();

    void LockReleaseGil();

    void Unlock();

    /// try locking the environment while releasing the GIL. This can get into a deadlock after env lock is acquired and before gil is re-acquired
    bool TryLockReleaseGil();

    bool TryLock();

    bool Lock(float timeout);

    void __enter__();

    void __exit__(object type, object value, object traceback);

    bool SetViewer(const std::string &viewername, bool showviewer=true);

    /// \brief sets the default viewer
    bool SetDefaultViewer(bool showviewer=true);

    object GetViewer();

    /// returns the number of points
    static size_t _getGraphPoints(object opoints, std::vector<float>&vpoints);

    /// returns the number of colors
    static size_t _getGraphColors(object ocolors, std::vector<float>&vcolors);

    /// returns the number of vectors
    static size_t _getListVector(object odata, std::vector<RaveVector<float> >& vvectors, size_t numcol);

    static std::pair<size_t,size_t> _getGraphPointsColors(object opoints, object ocolors, std::vector<float>&vpoints, std::vector<float>&vcolors);

    object plot3(object opoints,float pointsize,object ocolors=py::none_(),int drawstyle=0);

    object drawlinestrip(object opoints,float linewidth,object ocolors=py::none_(),int drawstyle=0);

    object drawlinelist(object opoints,float linewidth,object ocolors=py::none_(),int drawstyle=0);

    object drawarrow(object op1, object op2, float linewidth=0.002, object ocolor=py::none_());

    object drawlabel(const std::string &label, object worldPosition, object ocolor=py::none_(), float height=0.05);

    object drawbox(object opos, object oextents, object ocolor=py::none_());
    object drawboxarray(object opos, object oextents, object ocolors=py::none_());
    object drawaabb(object oaabb, object otransform, object ocolor=py::none_(), float transparency=0.0f);
    object drawobb(object oobb, object ocolor=py::none_(), float transparency=0.0f);

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object drawplane(object otransform, object oextents, const std::vector<std::vector<dReal> >&_vtexture);
    object drawplane(object otransform, object oextents, const std::vector<std::vector<std::vector<dReal> > >&vtexture);
#else
    object drawplane(object otransform, object oextents, const boost::multi_array<float,2>&_vtexture);
    object drawplane(object otransform, object oextents, const boost::multi_array<float,3>&vtexture);
#endif

    object drawtrimesh(object opoints, object oindices=py::none_(), object ocolors=py::none_());

    object GetBodies();
    int GetNumBodies();

    object GetRobots();

    object GetSensors();

    void UpdatePublishedBodies();

    object GetPublishedBodies(uint64_t timeout=0);

    object GetPublishedBody(const std::string &name, uint64_t timeout = 0);

    object GetPublishedBodyJointValues(const std::string &name, uint64_t timeout=0);

    object GetPublishedBodyTransformsMatchingPrefix(const std::string &prefix, uint64_t timeout=0);

    object Triangulate(PyKinBodyPtr pbody);

    object TriangulateScene(const int options, const std::string &name);

    void SetDebugLevel(object olevel);
    int GetDebugLevel() const;

    std::string GetHomeDirectory();

    void SetUserData(PyUserData pdata);
    void SetUserData(object o);
    object GetUserData() const;

    void SetUnit(std::string unitname, dReal unitmult);
    void SetUnitInfo(const UnitInfo& unitInfo);

    object GetUnit() const;
    UnitInfo GetUnitInfo() const;
    int GetId() const;

    object ExtractInfo() const;
    object UpdateFromInfo(PyEnvironmentBaseInfoPtr info, UpdateFromInfoMode updateMode);

    int GetRevision() const;

    py::object GetName() const;

    py::object GetNameId() const;

    void SetDescription(const std::string& sceneDescription);

    py::object GetDescription() const;

    void SetKeywords(object oSceneKeywords);

    py::list GetKeywords() const;

    void SetUInt64Parameter(const std::string& parameterName, uint64_t value);

    bool RemoveUInt64Parameter(const std::string& parameterName);

    uint64_t GetUInt64Parameter(const std::string& parameterName, uint64_t defaultValue) const;

    int _revision = 0;
    py::list _keywords;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    std::string _name;
    std::string _description;
#else
    object _name = py::none_();
    object _description = py::none_();
#endif

    bool __eq__(PyEnvironmentBasePtr p);
    bool __ne__(PyEnvironmentBasePtr p);
    long __hash__();
    std::string __repr__();
    std::string __str__();
    object __unicode__();

    EnvironmentBasePtr GetEnv() const;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_ENVIRONMENTBASE_H
