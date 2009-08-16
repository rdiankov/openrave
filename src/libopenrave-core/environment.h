// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_ENVIRONMENT_H
#define RAVE_ENVIRONMENT_H

/// database of planners, obstacles, sensors, and problem from plugins
class RaveDatabase
{
public:
    /// create the interfaces
    typedef InterfaceBase* DECLPTR_STDCALL(CreateFn, (PluginType type, const wchar_t* name, EnvironmentBase* penv));

    /// called to get information about what the plugin provides
    typedef bool DECLPTR_STDCALL(GetPluginAttributesFn, (PLUGININFO* pinfo, int size));

    /// called when plugin is about to be destroyed, all the interfaces created by this plugin should have been destroyed
    /// before this call
    typedef void DECLPTR_STDCALL(DestroyPluginFn, ());

    struct PLUGIN
    {
        PLUGIN() : plibrary(NULL), pfnCreate(NULL), pfnGetPluginAttributes(NULL), pfnDestroyPlugin(NULL) {}
        virtual ~PLUGIN();

        string ppluginname;
        PLUGININFO info;

        void* plibrary; // loaded library (NULL if not loaded)
        CreateFn pfnCreate;
        GetPluginAttributesFn pfnGetPluginAttributes;
        DestroyPluginFn pfnDestroyPlugin;
    };

    RaveDatabase();
    virtual ~RaveDatabase() { Destroy(); }

    RobotBase* CreateRobot(EnvironmentBase* penv, const wchar_t* pname);
    RobotBase* CreateRobot(EnvironmentBase* penv, const char* pname);

    PlannerBase* CreatePlanner(EnvironmentBase* penv, const wchar_t* pname) { return (PlannerBase*)Create(penv, PT_Planner, pname, OPENRAVE_PLANNER_HASH); }
    PlannerBase* CreatePlanner(EnvironmentBase* penv, const char* pname) { return (PlannerBase*)Create(penv, PT_Planner, pname, OPENRAVE_PLANNER_HASH); }
    SensorSystemBase* CreateSensorSystem(EnvironmentBase* penv, const wchar_t* pname) { return (SensorSystemBase*)Create(penv, PT_SensorSystem, pname, OPENRAVE_SENSORSYSTEM_HASH); }
    SensorSystemBase* CreateSensorSystem(EnvironmentBase* penv, const char* pname) { return (SensorSystemBase*)Create(penv, PT_SensorSystem, pname, OPENRAVE_SENSORSYSTEM_HASH); }
    ControllerBase* CreateController(EnvironmentBase* penv, const wchar_t* pname) { return (ControllerBase*)Create(penv, PT_Controller, pname, OPENRAVE_CONTROLLER_HASH); }
    ControllerBase* CreateController(EnvironmentBase* penv, const char* pname) { return (ControllerBase*)Create(penv, PT_Controller, pname, OPENRAVE_CONTROLLER_HASH); }
    ProblemInstance* CreateProblem(EnvironmentBase* penv, const wchar_t* pname) { return (ProblemInstance*)Create(penv, PT_ProblemInstance, pname, OPENRAVE_PROBLEM_HASH); }
    ProblemInstance* CreateProblem(EnvironmentBase* penv, const char* pname) { return (ProblemInstance*)Create(penv, PT_ProblemInstance, pname, OPENRAVE_PROBLEM_HASH); }
    IkSolverBase* CreateIkSolver(EnvironmentBase* penv, const wchar_t* pname) { return (IkSolverBase*)Create(penv, PT_InverseKinematicsSolver, pname, OPENRAVE_IKSOLVER_HASH); }
    IkSolverBase* CreateIkSolver(EnvironmentBase* penv, const char* pname) { return (IkSolverBase*)Create(penv, PT_InverseKinematicsSolver, pname, OPENRAVE_IKSOLVER_HASH); }
    PhysicsEngineBase* CreatePhysicsEngine(EnvironmentBase* penv, const wchar_t* pname) { return (PhysicsEngineBase*)Create(penv, PT_PhysicsEngine, pname, OPENRAVE_PHYSICSENGINE_HASH); }
    PhysicsEngineBase* CreatePhysicsEngine(EnvironmentBase* penv, const char* pname) { return (PhysicsEngineBase*)Create(penv, PT_PhysicsEngine, pname, OPENRAVE_PHYSICSENGINE_HASH); }
    SensorBase* CreateSensor(EnvironmentBase* penv, const wchar_t* pname) { return (SensorBase*)Create(penv, PT_Sensor, pname, OPENRAVE_SENSOR_HASH); }
    SensorBase* CreateSensor(EnvironmentBase* penv, const char* pname) { return (SensorBase*)Create(penv, PT_Sensor, pname, OPENRAVE_SENSOR_HASH); }
    CollisionCheckerBase* CreateCollisionChecker(EnvironmentBase* penv, const wchar_t* pname) { return (CollisionCheckerBase*)Create(penv, PT_CollisionChecker, pname, OPENRAVE_COLLISIONCHECKER_HASH); }
    CollisionCheckerBase* CreateCollisionChecker(EnvironmentBase* penv, const char* pname) { return (CollisionCheckerBase*)Create(penv, PT_CollisionChecker, pname, OPENRAVE_COLLISIONCHECKER_HASH); }
    RaveViewerBase* CreateViewer(EnvironmentBase* penv, const wchar_t* pname) { return (RaveViewerBase*)Create(penv, PT_Viewer, pname, OPENRAVE_VIEWER_HASH); }
    RaveViewerBase* CreateViewer(EnvironmentBase* penv, const char* pname) { return (RaveViewerBase*)Create(penv, PT_Viewer, pname, OPENRAVE_VIEWER_HASH); }
    RaveServerBase* CreateServer(EnvironmentBase* penv, const wchar_t* pname) { return (RaveServerBase*)Create(penv, PT_Server, pname, OPENRAVE_SERVER_HASH); }
    RaveServerBase* CreateServer(EnvironmentBase* penv, const char* pname) { return (RaveServerBase*)Create(penv, PT_Server, pname, OPENRAVE_SERVER_HASH); }

    /// Destroy all plugins and directories
    virtual void Destroy();

    /// loads all the plugins that are currently loaded
    /// \param pnewlocaldir If NULL, uses old local dir
    void ReloadPlugins(const wchar_t* pnewlocaldir);

    const std::list<PLUGIN>& GetPlugins() { return _listplugins; }
    
    void* Create(EnvironmentBase* penv, PluginType type, const char* pname);
    void* Create(EnvironmentBase* penv, PluginType type, const wchar_t* pname, const char* hash);
    void* Create(EnvironmentBase* penv, PluginType type, const char* pname, const char* hash);

    /// loads all the plugins in this dir
    /// If pdir is already specified, reloads all 
    bool AddDirectory(const char* pdir);
    bool AddPlugin(const char* pname);

    /// deletes the plugin from memory. Note that the function doesn't check if there's any object
    /// pointers that were instantiated from the specific plugin. Those objects should be removed
    /// before calling this function in order to avoid segmentation faults later on.
    bool DeletePlugin(const char* pname);

    static const char* GetInterfaceHash(InterfaceBase* pint) { return pint->GetHash(); }

protected:

    bool _LoadPlugin(PLUGIN* p);

    bool GetInfo(PLUGIN* p);

    static void* SysLoadLibrary(const char* lib);
    static void* SysLoadSym(void* lib, const char* sym);
    static void SysCloseLibrary(void* lib);

    list<PLUGIN> _listplugins;
    
    vector<string> vplugindirs; ///< local directory for all plugins
    map<PluginType,string> _mapinterfacenames;
};

class Environment : public EnvironmentBase
{
public:
    class EnvMutexLock : public EnvLock
    {
    public:
        EnvMutexLock(pthread_mutex_t* pmutex) : _pmutex(pmutex) { pthread_mutex_lock(_pmutex); }
        ~EnvMutexLock() { pthread_mutex_unlock(_pmutex); }
        pthread_mutex_t* _pmutex;
    };

    Environment(bool bLoadAllPlugins=true);
    virtual ~Environment();

    void Destroy();     ///< Destroys everything
    virtual void Reset();

    void Init();

    RaveDatabase& GetDatabase() { return *_pdatabase; }

    virtual void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins);
    virtual void GetLoadedInterfaces(PLUGININFO& info);
    virtual bool LoadPlugin(const char* pname) { WaitForPlugins(); return GetDatabase().AddPlugin(pname); }

    virtual InterfaceBase* CreateInterface(PluginType type,const char* pinterfacename);
    virtual RobotBase* CreateRobot(const wchar_t* pname);
    virtual RobotBase* CreateRobot(const char* pname);
    virtual PlannerBase* CreatePlanner(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreatePlanner(this,pname); }
    virtual PlannerBase* CreatePlanner(const char* pname) { WaitForPlugins(); return _pdatabase->CreatePlanner(this,pname); }
    virtual SensorSystemBase* CreateSensorSystem(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateSensorSystem(this,pname); }
    virtual SensorSystemBase* CreateSensorSystem(const char* pname) { WaitForPlugins(); return _pdatabase->CreateSensorSystem(this,pname); }
    virtual ControllerBase* CreateController(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateController(this,pname); }
    virtual ControllerBase* CreateController(const char* pname) { WaitForPlugins(); return _pdatabase->CreateController(this,pname); }
    virtual ProblemInstance* CreateProblem(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateProblem(this,pname); }
    virtual ProblemInstance* CreateProblem(const char* pname) { WaitForPlugins(); return _pdatabase->CreateProblem(this,pname); }
    virtual IkSolverBase* CreateIkSolver(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateIkSolver(this,pname); }
    virtual IkSolverBase* CreateIkSolver(const char* pname) { WaitForPlugins(); return _pdatabase->CreateIkSolver(this,pname); }
    virtual PhysicsEngineBase* CreatePhysicsEngine(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreatePhysicsEngine(this,pname); }
    virtual PhysicsEngineBase* CreatePhysicsEngine(const char* pname) { WaitForPlugins(); return _pdatabase->CreatePhysicsEngine(this,pname); }
    virtual SensorBase* CreateSensor(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateSensor(this,pname); }
    virtual SensorBase* CreateSensor(const char* pname) { WaitForPlugins(); return _pdatabase->CreateSensor(this,pname); }
    virtual CollisionCheckerBase* CreateCollisionChecker(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateCollisionChecker(this,pname); }
    virtual CollisionCheckerBase* CreateCollisionChecker(const char* pname) { WaitForPlugins(); return _pdatabase->CreateCollisionChecker(this,pname); }
    virtual RaveViewerBase* CreateViewer(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateViewer(this,pname); }
    virtual RaveViewerBase* CreateViewer(const char* pname) { WaitForPlugins(); return _pdatabase->CreateViewer(this,pname); }
    virtual RaveServerBase* CreateServer(const wchar_t* pname) { WaitForPlugins(); return _pdatabase->CreateServer(this,pname); }
    virtual RaveServerBase* CreateServer(const char* pname) { WaitForPlugins(); return _pdatabase->CreateServer(this,pname); }

    virtual EnvironmentBase* CloneSelf(int options);
    
    int LoadProblem(ProblemInstance* prob, const char* cmdargs);
    bool RemoveProblem(ProblemInstance* prob);
    virtual const std::list<ProblemInstance*>& GetProblems() const { return listProblems; }

    virtual bool Load(const wchar_t *filename);
    virtual bool Load(const char *filename);

    virtual bool Save(const char* filename);

    virtual bool AddKinBody(KinBody* pbody);
    virtual bool AddRobot(RobotBase* robot);

    virtual bool RemoveKinBody(KinBody* pbody, bool bDestroy);
    virtual KinBody* GetKinBody(const wchar_t* pname);

    virtual bool SetPhysicsEngine(PhysicsEngineBase* pengine);
    virtual PhysicsEngineBase* GetPhysicsEngine() const;

    virtual KinBody* CreateKinBody();
    virtual Trajectory* CreateTrajectory(int nDOF) { return new Trajectory(this,nDOF); }

    /// Collision specific functions
    //@{
    virtual bool SetCollisionChecker(CollisionCheckerBase* pchecker);
    virtual CollisionCheckerBase* GetCollisionChecker() const;

    virtual bool SetCollisionOptions(int options);
    virtual int GetCollisionOptions() const;
    
    virtual bool CheckCollision(const KinBody* pbody1, COLLISIONREPORT* pReport = NULL);
    virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* pReport = NULL);
    virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* pReport = NULL);
    virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* pReport= NULL);
    virtual bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* pReport = NULL);
    
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport = NULL);
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport = NULL);

    virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport = NULL);
    virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport = NULL);
    virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport=NULL);

    //tolerance check
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance);
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance);
    //@}

    virtual void StepSimulation(dReal timeStep);

    virtual void SetCamera(const RaveTransform<float>& trans);
    virtual void SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat);
    virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup);
    virtual RaveTransform<float> GetCameraTransform();

    virtual bool LockPhysics(bool bLock, float timeout=0);
    virtual bool IsPhysicsLocked();

    virtual const std::vector<RobotBase*>& GetRobots() const { return _vecrobots; }
    virtual void GetBodies(std::vector<KinBody*>& bodies) const;

    virtual EnvLock* GetLockedBodies(std::vector<KinBody*>& bodies) const;
    virtual EnvLock* GetLockedRobots(std::vector<RobotBase*>& robots) const;

    /// triangulation of the body including its current transformation. trimesh will be appended the new data.
    virtual bool Triangulate(KinBody::Link::TRIMESH& trimesh, const KinBody* pbody);

    /// general triangulation of the whole scene. trimesh will be appended the new data.
    virtual bool TriangulateScene(KinBody::Link::TRIMESH& trimesh, TriangulateOptions opts, const wchar_t* pName);

    virtual const char* GetHomeDirectory() const { return _homedirectory.c_str(); }

    /// public XML readers.
    //@{
    virtual RobotBase* ReadRobotXML(RobotBase* robot, const char* filename, const char** atts);
    virtual KinBody* ReadKinBodyXML(KinBody* robot, const char* filename, const char** atts);

    virtual void RegisterXMLReader(PluginType type, const char* xmltag, CreateXMLReaderFn pfn);
    virtual void UnregisterXMLReader(PluginType type, const char* xmltag);
    
    virtual bool ParseXMLFile(BaseXMLReader* preader, const char* filename);
    virtual bool ParseXMLData(BaseXMLReader* preader, const char* pdata, int len);
    //@}    

    virtual bool AttachViewer(RaveViewerBase* pnewviewer);
    virtual RaveViewerBase* GetViewer() const;

    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0);
    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0);

    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color);
    
    virtual void* drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents);
    
    virtual void* drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color);

    virtual void closegraph(void* handle);

    virtual bool GetFractionOccluded(KinBody* pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded);
    virtual bool GetCameraImage(void* pMemory, int width, int height, const RaveTransform<float>& t, const float* pKK);
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const float* pKK, const char* fileName, const char* extension);

    /// Random generator functions using Mersene Twister
    virtual unsigned int RandomInt();
    virtual void RandomInt(unsigned int n, std::vector<int>& v);
    virtual float RandomFloat();
    virtual void RandomFloat(unsigned int n, std::vector<float>& v);
    virtual double RandomDouble();
    virtual void RandomDouble(unsigned int n, std::vector<double>& v);

    virtual void SetUniqueNetworkId(KinBody* pbody, int* pOutNetworkId);
    virtual void RemoveUniqueNetworkId(int id);
    virtual KinBody* GetBodyFromNetworkId(int id);

    void AdvanceFrame(bool bForward);
        
    virtual void StartSimulation(dReal fDeltaTime);
    virtual void StopSimulation();
    virtual uint64_t GetSimulationTime() { return _nCurSimTime; }

    virtual const std::vector<KinBody*>& GetBodies() const { return _vecbodies; }

    virtual void SetDebugLevel(DebugLevel level) { RaveSetDebugLevel(level); }
    virtual DebugLevel GetDebugLevel() const { return RaveGetDebugLevel(); }

    virtual void GetPublishedBodies(std::vector<BODYSTATE>& vbodies);
    virtual void SetPublishBodiesAnytime(bool bAnytime) { _bPublishBodiesAnytime = bAnytime; }
    virtual bool GetPublishBodiesAnytime() const { return _bPublishBodiesAnytime; }

    virtual bool IsPluginsLoaded() { return _bPluginsLoaded; }

    const vector<string>& GetDataDirs() const { return _vdatadirs; }

    virtual bool AttachServer(RaveServerBase* pserver);
    virtual RaveServerBase* GetServer() const { return _pserver; }

protected:

    Environment(const Environment& r, int options);

    class DummyPhysicsEngine : public PhysicsEngineBase
    {
    public:
        DummyPhysicsEngine(EnvironmentBase* penv) : PhysicsEngineBase(penv) {}
        virtual bool SetPhysicsOptions(int physicsoptions) { return true; }
        virtual int GetPhysicsOptions() const { return 0; }
        
        virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) { return true; }

        virtual bool InitEnvironment() { return true; }
        virtual void DestroyEnvironment() {}
        
        virtual bool InitKinBody(KinBody* pbody) { return true; } 
        virtual bool DestroyKinBody(KinBody* pbody) { return true; }

        virtual bool SetBodyVelocity(KinBody* pbody, const Vector& linearvel, const Vector& angularvel, const dReal* pJointVelocity) { return true; }
        virtual bool SetBodyVelocity(KinBody* pbody, const Vector* pLinearVelocities, const Vector* pAngularVelocities) { return true; }
        virtual bool GetBodyVelocity(const KinBody* pbody, Vector& linearvel, Vector& angularvel, dReal* pJointVelocity);
        virtual bool GetBodyVelocity(KinBody* pbody, Vector* pLinearVelocities, Vector* pAngularVelocities);

        virtual bool SetJointVelocity(KinBody::Joint* pjoint, const dReal* pJointVelocity) { return true; }
        virtual bool GetJointVelocity(const KinBody::Joint* pjoint, dReal* pJointVelocity);

        virtual bool SetBodyForce(KinBody::Link* plink, const Vector& force, const Vector& position, bool bAdd) { return true; }
        virtual bool SetBodyTorque(KinBody::Link* plink, const Vector& torque, bool bAdd) { return true; }
        virtual bool AddJointTorque(KinBody::Joint* pjoint, const dReal* pTorques) { return true; }

        virtual void SetGravity(const Vector& gravity) {}
        virtual Vector GetGravity() { return Vector(0,0,0); }

        virtual void SimulateStep(dReal fTimeElapsed) { }
    };

    class DummyCollisionChecker : public CollisionCheckerBase
    {
    public:
        DummyCollisionChecker(EnvironmentBase* penv) : CollisionCheckerBase(penv) {}
        virtual ~DummyCollisionChecker() {}

        virtual bool InitEnvironment() { return true; }
        virtual void DestroyEnvironment() {}
        virtual bool InitKinBody(KinBody* pbody) { return true; }
        virtual bool DestroyKinBody(KinBody* pbody) { return true; }

        virtual bool Enable(const KinBody* pbody, bool bEnable) { return true; }
        virtual bool EnableLink(const KinBody::Link* pbody, bool bEnable) { return true; }

        virtual bool SetCollisionOptions(int collisionoptions) { return true; }
        virtual int GetCollisionOptions() const { return 0; }

        virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { return true; }

        virtual bool CheckCollision(const KinBody* pbody1, COLLISIONREPORT*) { return false; }
        virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT*) { return false; }
        virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT*) { return false; }
        virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT*) { return false; }
        virtual bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT*) { return false; }
    
        virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT*) { return false; }
        virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT*) { return false; }

        virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport) { return false; }
        virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport) { return false; }
        virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport) { return false; }
        virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, dReal tolerance) { return false; }
        virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, dReal tolerance) { return false; }

        virtual bool CheckSelfCollision(const KinBody* pbody, COLLISIONREPORT* pReport = NULL) { return false; }
    };

    /// Base class for the graphics and gui engine. Derive a class called
    /// RaveViewer for anything more specific
    class DummyRaveViewer : public RaveViewerBase
    {
    public:
        DummyRaveViewer(Environment* penv) : RaveViewerBase(penv) {
            assert( penv != NULL );
            _penv = penv;
            _bQuitMainLoop = true;
        }
        virtual ~DummyRaveViewer() {}

        //void   AddItem(Item *pItem, bool bVisibility = false);
        /// reset the camera depending on its mode
        virtual void UpdateCameraTransform() {}

        /// goes into the main loop
        virtual int main(bool bShow) {
            _bQuitMainLoop = false;
            _penv->LockPhysics(true);
            _penv->StartSimulation(0.01f);
            _penv->LockPhysics(false);

            while(!_bQuitMainLoop) {
                AdvanceFrame(true);
            }

            return 0;
        }
        
        virtual void quitmainloop() {
            if( !_bQuitMainLoop ) {
                _penv->StopSimulation();
                _bQuitMainLoop = true;
            }
        }

        /// methods relating to playback
        virtual void AdvanceFrame(bool bForward) {
            // update the camera
            UpdateCameraTransform();

            if( _bSaveVideo )
                _RecordVideo();

            // process sensors (camera images)
            Sleep(1);
        }
    
        virtual bool GetFractionOccluded(KinBody* pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded) { return false; }

        /// Retries a 24bit RGB image of dimensions width and height from the current scene
        /// extrinsic is the rotation and translation of the camera
        /// pKK is 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
        virtual bool GetCameraImage(void* pMemory, int width, int height, const RaveTransform<float>& extrinsic, const float* pKK) { return false; }
        virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const float* pKK, const char* fileName, const char* extension) { return false; }
        virtual void SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat) {}
        virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup) {}
        virtual RaveTransform<float> GetCameraTransform() {return RaveTransform<float>();}

        virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle=0) { return NULL; }
        virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle=0) { return NULL; }

        virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) { return NULL; }
        virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) { return NULL; }

        virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) { return NULL; }
        virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) { return NULL; }

        virtual void* drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color) { return NULL; }

        virtual void* drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) { return NULL; }

        virtual void* drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) { return NULL; }

        virtual void closegraph(void* handle) {}
        virtual void deselect() {}

        virtual void Reset() {}
        virtual void SetBkgndColor(const RaveVector<float>& color) {}

        virtual bool _RecordVideo() { return false; }

        virtual void StartPlaybackTimer() {}
        virtual void StopPlaybackTimer() {}

        virtual void ViewerSetSize(int w, int h) {}
        virtual void ViewerMove(int x, int y) {}
        virtual void ViewerSetTitle(const char* ptitle) {};
        
        virtual bool LoadModel(const char* pfilename) { return true; }
    protected:

        bool _bTimeElapsed;
        bool _bSaveVideo;
    
        // environment thread related
        bool _bQuitMainLoop;
    
        Environment* _penv;                 ///< environment
    };

    static void* main(void*);
    void* _main();

    static void KinBodyDestroyCallback(EnvironmentBase* penv, KinBody* pbody);

    void WaitForPlugins() const;

    // grasping & manipulation
    bool _GraspObject();
    bool _ReleaseObject();
    bool _ReturnToRest();
    void _CleanRemovedBodies();

    uint64_t _nLastSimTime;
    boost::shared_ptr<RaveDatabase> _pdatabase;

    std::vector<RobotBase*> _vecrobots;  ///< robots (possibly controlled)
    std::vector<KinBody*> _vecbodies; ///< all objects that are collidable (includes robots)

    list<ProblemInstance*> listProblems; ///< problems loaded in the environment
    list<KinBody*> _listRemovedBodies;

    dReal _fDeltaSimTime;                ///< delta time for simulate step
    uint64_t _nCurSimTime;                    ///< simulation time since the start of the environment
    int _nBodiesModifiedStamp; ///< incremented every tiem bodies vector is modified

    PhysicsEngineBase* _pPhysicsEngine;
    
    RaveServerBase* _pserver;
    RaveViewerBase* _pCurrentViewer;
    
    int nNetworkIndex;               ///< next network index
    std::map<int, KinBody*> _mapBodies; ///< a map of all the bodies in the environment. Controlled through the KinBody constructor and destructors
    
    pthread_t _threadLoop;                  ///< main loop for the environment

    mutable pthread_mutex_t _mutexNetworkIds;  ///< protects _vecbodies/_vecrobots from multithreading issues
    mutable pthread_mutex_t _mutexBodies;  ///< protects _mapBodies from multithreading issues
    mutable pthread_mutex_t _mutexPhysics;      ///< protects internal data from multithreading issues
    mutable pthread_mutex_t _mutexDestroy;
    mutable pthread_mutex_t _mutexProblems;
    mutable pthread_mutex_t _mutexXML;

    bool _bPublishBodiesAnytime;            ///< if true, will publish the bodies at anypoint in time,
                                            ///< if false, will only publish the bodies after step simulation and the server are done
                                           
    vector<BODYSTATE> _vPublishedBodies;
    vector<string> _vplugindirs, _vdatadirs;
    string _homedirectory;

    //bool _bSelfCollision;           ///< check collision between KinBody links for contact forces
    bool _bDestroying;              ///< destroying envrionment, so exit from all processes
    bool _bDestroyed;               ///< environment has been destroyed
    bool _bEnableSimulation;        ///< enable simulation loop
    bool _bPluginsLoaded;

    CollisionCheckerBase* pCurrentChecker;
    DummyPhysicsEngine _dummyphysics;
    DummyCollisionChecker _dummychecker;
    DummyRaveViewer _dummyviewer;

    boost::shared_ptr<CollisionCheckerBase> _localchecker;
    boost::shared_ptr<PhysicsEngineBase> _localphysics;
    boost::shared_ptr<RaveViewerBase> _localviewer;
    friend class EnvironmentXMLReader;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RaveDatabase::PLUGIN)
#endif

#endif
