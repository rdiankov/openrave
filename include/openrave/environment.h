// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file environment.h
    \brief Definition of the EnvironmentBase interface used for managing all objects in an environment.

    Automatically included with \ref openrave.h
 */
#ifndef  OPENRAVE_ENVIRONMENTBASE_H
#define  OPENRAVE_ENVIRONMENTBASE_H

namespace OpenRAVE {

#if OPENRAVE_ENVIRONMENT_RECURSIVE_LOCK
#if __cplusplus >= 201703L
#include <mutex>
using EnvironmentMutex = ::std::recursive_mutex;
using EnvironmentLock  = ::std::unique_lock<std::recursive_mutex>;
using defer_lock_t     = ::std::defer_lock_t;
using try_to_lock_t    = ::std::try_to_lock_t;
#else
using EnvironmentMutex = ::boost::recursive_try_mutex;
using EnvironmentLock  = EnvironmentMutex::scoped_lock;
using defer_lock_t     = ::boost::defer_lock_t;
using try_to_lock_t    = ::boost::try_to_lock_t;
#endif // __cplusplus >= 201703L
#else
using EnvironmentMutex = ::std::mutex;
using EnvironmentLock  = ::std::unique_lock<std::mutex>;
using defer_lock_t     = ::std::defer_lock_t;
using try_to_lock_t    = ::std::try_to_lock_t;
#endif // OPENRAVE_ENVIRONMENT_RECURSIVE_LOCK

/// \brief used when adding interfaces to the environment
enum InterfaceAddMode
{
    IAM_AllowRenaming = 0, ///< rename the name and id. if true and there exists a body/robot with the same name, will make body's name and id unique
    IAM_StrictNameChecking = 1, ///< name is strict, will throw exception if it conflicts. id is not strict
    IAM_StrictIdChecking = 2, ///< id is strict, will throw exception if it conflicts. id is not strict
    IAM_StrictNameIdChecking = 3, ///< name and id are both strict, will throw exception if it conflicts.
};

/** \brief Maintains a world state, which serves as the gateway to all functions offered through %OpenRAVE. See \ref arch_environment.
 */
class OPENRAVE_API EnvironmentBase : public boost::enable_shared_from_this<EnvironmentBase>
{
public:
    EnvironmentBase();
    EnvironmentBase(const std::string& name);
    virtual ~EnvironmentBase();

    /// \brief Releases all environment resources, should be always called when environment stops being used.
    ///
    /// Removing all environment pointer might not be enough to destroy the environment resources.
    virtual void Destroy()=0;

    /// \brief initializes the environment
    virtual void Init(bool bStartSimulationThread) = 0;
            
    /// \brief Resets all objects of the scene (preserves all problems, planners). <b>[multi-thread safe]</b>
    ///
    /// Do not call inside a SimulationStep call
    virtual void Reset()=0;

    /// \brief set user data
    virtual void SetUserData(UserDataPtr data) {
        __pUserData = data;
    }
    /// \brief return the user custom data
    virtual UserDataPtr GetUserData() const {
        return __pUserData;
    }

    /// \brief Returns the OpenRAVE global state, used for initializing plugins
    virtual UserDataPtr GlobalState() = 0;

    /// \brief Environment will own the interface until EnvironmentBase::Destroy is called.
    virtual void OwnInterface(InterfaceBasePtr pinterface) = 0;

    /// \brief Remove ownership of the interface.
    virtual void DisownInterface(InterfaceBasePtr pinterface) = 0;

    /// \brief Create and return a clone of the current environment.
    ///
    /// Clones do not share any memory or resource between each other.
    /// or their parent making them ideal for performing separte planning experiments while keeping
    /// the parent environment unchanged.
    /// By default a clone only copies the collision checkers and physics engine.
    /// When bodies are cloned, the unique ids are preserved across environments (each body can be referenced with its id in both environments). The attached and grabbed bodies of each body/robot are also copied to the new environment.
    /// \param options A set of \ref CloningOptions describing what is actually cloned.
    /// \return An environment of the same type as this environment containing the copied information.
    virtual EnvironmentBasePtr CloneSelf(int options) = 0;

    /// \brief Create and return a clone of the current environment.
    ///
    /// Clones do not share any memory or resource between each other.
    /// or their parent making them ideal for performing separte planning experiments while keeping
    /// the parent environment unchanged.
    /// By default a clone only copies the collision checkers and physics engine.
    /// When bodies are cloned, the unique ids are preserved across environments (each body can be referenced with its id in both environments). The attached and grabbed bodies of each body/robot are also copied to the new environment.
    /// \param clonedEnvName The name of the cloned (and retuned) environment
    /// \param options A set of \ref CloningOptions describing what is actually cloned.
    /// \return An environment of the same type as this environment containing the copied information.
    virtual EnvironmentBasePtr CloneSelf(const std::string& clonedEnvName, int options) = 0;

    /// \brief Clones the reference environment into the current environment
    ///
    /// Tries to preserve computation by re-using bodies/interfaces that are already similar between the current and reference environments.
    /// \param[in] cloningoptions The parts of the environment to clone. Parts not specified are left as is.
    virtual void Clone(EnvironmentBaseConstPtr preference, int cloningoptions) = 0;

    /// \brief Clones the reference environment into the current environment
    ///
    /// Tries to preserve computation by re-using bodies/interfaces that are already similar between the current and reference environments.
    /// \param[in] clonedEnvName The name of the cloned environment
    /// \param[in] cloningoptions The parts of the environment to clone. Parts not specified are left as is.
    virtual void Clone(EnvironmentBaseConstPtr preference, const std::string& clonedEnvName, int cloningoptions) = 0;

    /// \brief Each function takes an optional pointer to a CollisionReport structure and returns true if collision occurs. <b>[multi-thread safe]</b>
    ///
    /// \name Collision specific functions.
    /// \anchor env_collision_checking
    //@{
    /// set the global environment collision checker
    virtual bool SetCollisionChecker(CollisionCheckerBasePtr pchecker)=0;
    virtual CollisionCheckerBasePtr GetCollisionChecker() const =0;

    /// \see CollisionCheckerBase::CheckCollision(KinBodyConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(KinBodyConstPtr,KinBodyConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(KinBody::LinkConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(KinBody::LinkConstPtr,KinBody::LinkConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(KinBody::LinkConstPtr,KinBodyConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(KinBody::LinkConstPtr,const std::vector<KinBodyConstPtr>&,const std::vector<KinBody::LinkConstPtr>&,CollisionReportPtr)
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(KinBodyConstPtr,const std::vector<KinBodyConstPtr>&,const std::vector<KinBody::LinkConstPtr>&,CollisionReportPtr)
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;

    /// \see CollisionCheckerBase::CheckCollision(const RAY&,KinBody::LinkConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \see CollisionCheckerBase::CheckCollision(const RAY&,KinBodyConstPtr,CollisionReportPtr)
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \see CollisionCheckerBase::CheckCollision(const RAY&,CollisionReportPtr)
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \see CollisionCheckerBase::CheckCollision(const TriMesh&,KinBodyConstPtr, CollisionReportPtr)
    virtual bool CheckCollision(const TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \see CollisionCheckerBase::CheckSelfCollision
    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// \deprecated (13/04/09)
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) RAVE_DEPRECATED
    {
        //RAVELOG_WARN("CollisionCheckerBase::CheckSelfCollision has been deprecated, please use CollisionCheckerBase::CheckStandaloneSelfCollision\n");
        return CheckStandaloneSelfCollision(pbody, report);
    }

    typedef boost::function<CollisionAction(CollisionReportPtr,bool)> CollisionCallbackFn;

    /// Register a collision callback.
    ///
    /// Whenever a collision is detected between between bodies during a CheckCollision call or physics simulation, the callback is called.
    /// The callback should return an action specifying how the collision should be handled:
    /// <b>action = callback(CollisionReport,bool IsCalledFromPhysicsEngine)</b>
    /// \return a handle to the registration, once the handle loses scope, the callback is unregistered
    virtual UserDataPtr RegisterCollisionCallback(const CollisionCallbackFn& callback) = 0;
    virtual bool HasRegisteredCollisionCallbacks() const = 0;

    /// \brief return all the collision callbacks, the environment must be locked!
    ///
    /// \param listcallbacks filled with the user callbacks. Once the environment is unlocked, the list becomes invalid.
    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>&) const = 0;

    //@}

    /// \name Physics and Simulation
    //@{
    /// set the physics engine, disabled by default
    /// \param physics the engine to set, if NULL, environment sets an dummy physics engine
    virtual bool SetPhysicsEngine(PhysicsEngineBasePtr physics) = 0;
    virtual PhysicsEngineBasePtr GetPhysicsEngine() const = 0;

    /// \brief Makes one simulation time step. <b>[multi-thread safe]</b>
    ///
    /// Can be called manually by the user inside planners. Keep in mind that the internal simulation thread also calls this function periodically. See \ref arch_simulation for more about the simulation thread.
    virtual void StepSimulation(dReal timeStep) = 0;

    /** \brief Start the internal simulation thread. <b>[multi-thread safe]</b>

        Resets simulation time to 0. See \ref arch_simulation for more about the simulation thread.

        \param fDeltaTime the delta step to take in simulation
        \param bRealTime if false will call SimulateStep as fast as possible, otherwise will time the simulate step calls so that simulation progresses with real system time.
     */
    virtual void StartSimulation(dReal fDeltaTime, bool bRealTime=true) = 0;

    /// \brief Stops the internal physics loop, stops calling SimulateStep for all modules. <b>[multi-thread safe]</b>
    ///
    /// See \ref arch_simulation for more about the simulation thread.
    /// \param shutdownthread
    virtual void StopSimulation(int shutdownthread=1) = 0;

    /// \brief Return true if inner simulation loop is executing. <b>[multi-thread safe]</b>
    ///
    /// See \ref arch_simulation for more about the simulation thread.
    virtual bool IsSimulationRunning() const = 0;

    /// \brief Return simulation time since the start of the environment (in microseconds). <b>[multi-thread safe]</b>
    ///
    /// See \ref arch_simulation for more about the simulation thread.
    virtual uint64_t GetSimulationTime() = 0;
    //@}

    /// \name File Loading and Parsing
    /// \anchor env_loading
    //@{

    /// \brief A set of options used to select particular parts of the scene
    enum SelectionOptions
    {
        SO_NoRobots = 1,               ///< everything but robots
        TO_Obstacles = 1,
        SO_Robots = 2,                  ///< all robots
        TO_Robots = 2,
        SO_Everything = 3,              ///< all bodies and robots everything
        TO_Everything = 3,
        SO_Body = 4,                    ///< robot/kinbody. atts 'target' key contains the body name. If multiple targets are specified, then will save all related bodies.
        TO_Body = 4,
        SO_AllExceptBody = 5,             ///< save everything but the robot/kinbody objects specified through the atts 'target' key. Multiple targets can be specified.
        TO_AllExceptBody = 5,
    };
    typedef SelectionOptions TriangulateOptions;

    /** \brief Loads a scene from a file and adds all objects in the environment. <b>[multi-thread safe]</b>

        For collada readers, the options are passed through to
        \code
        DAE::getIOPlugin()->setOption(key,value).
        \endcode
     */
    virtual bool Load(const std::string& filename, const AttributesList& atts = AttributesList()) = 0;

    /** \brief Loads a scene from a URI and adds all objects in the environment. <b>[multi-thread safe]</b>

        \param uri the URI of the file to load. Scheme can be 'file:' or 'openrave:' or 'X:' if openravescheme is overwritten by atts
        \param atts a string set of attributes to pass to each loader. For example: 'openravescheme' can be overwritten.
     */
    virtual bool LoadURI(const std::string& uri, const AttributesList& atts = AttributesList()) = 0;

    /// \brief Loads a scene from in-memory data and adds all objects in the environment. <b>[multi-thread safe]</b>
    virtual bool LoadData(const std::string& data, const AttributesList& atts = AttributesList(), const std::string& uri=std::string()) = 0;

    /// \brief loads a scene from rapidjson document
    ///
    /// \param updateMode specifies how to update the current environment from rEnvInfo
    /// \param vCreatedBodies the bodies created in this operation
    /// \param vModifiedBodies the bodies modified in this operation
    /// \param vRemovedBodies the bodies removed from the environment in this operation
    /// \param atts attributes that is passed to JSONReader for further options.
    /// \param uri the URI of the scene. Used to inject the URI into the environment.
    virtual bool LoadJSON(const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts = AttributesList(), const std::string &uri = std::string()) = 0;

    virtual bool LoadXMLData(const std::string& data, const AttributesList& atts = AttributesList()) {
        return LoadData(data,atts);
    }

    /** \brief Saves a scene depending on the filename extension. Default is in COLLADA format

        \param filename the filename to save the results at. Use the suffix extension of the filename to figure out the type to save. Supports: "dae", "json", "msgpack"
        \param options controls what to save
        \param atts attributes that refine further options. For collada-dom parsing, the options are passed through
        \code
        DAE::getIOPlugin()->setOption(key,value).
        \endcode
        Several default options are:
        - 'target' - the target body name of the options, if relevant
        - 'password' - the password/key to encrypt the data with, collada supports this through zae zip archives
        \throw openrave_exception Throw if failed to save anything
     */
    virtual void Save(const std::string& filename, SelectionOptions options=SO_Everything, const AttributesList& atts = AttributesList()) = 0;

    /// \deprecated (12/08/15)
    virtual void Save(const std::string& filename, SelectionOptions options, const std::string& selectname) RAVE_DEPRECATED {
        AttributesList atts;
        atts.emplace_back("target", selectname);
        Save(filename,options,atts);
    }

    /// \brief saves a scene to rapidjson document
    virtual void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, SelectionOptions options=SO_Everything, const AttributesList& atts = AttributesList()) = 0;

    /** \brief Saves a scene depending on the filename extension.

        \param filetype the type of file to save, can be: "collada", "json", "msgpack"
        \param output the output data if saving is successful
        \param options controls what to save
        \param atts attributes that refine further options. For collada parsing, the options are passed through
        \code
        DAE::getIOPlugin()->setOption(key,value).
        \endcode
        Several default options are:
        - 'target' - the target body name of the options, if relevant
        - 'password' - the password/key to encrypt the data with, collada supports this through zae zip archives
        \throw openrave_exception Throw if failed to save anything
     */
    virtual void WriteToMemory(const std::string& filetype, std::vector<char>& output, SelectionOptions options=SO_Everything, const AttributesList& atts = AttributesList()) = 0;

    /** \brief Initializes a robot from a resource file. The robot is not added to the environment when calling this function. <b>[multi-thread safe]</b>

        \param robot If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled
        \param filename the name of the resource file, its extension determines the format of the file. See \ref supported_formats.
        \param atts The attribute/value pair specifying loading options. Defined in \ref arch_robot.
     */
    virtual RobotBasePtr ReadRobotURI(RobotBasePtr robot, const std::string& filename, const AttributesList& atts = AttributesList()) = 0;
    virtual RobotBasePtr ReadRobotXMLFile(RobotBasePtr robot, const std::string& filename, const AttributesList& atts = AttributesList()) {
        return ReadRobotURI(robot,filename,atts);
    }

    /// \brief Creates a new robot from a file with no extra load options specified. <b>[multi-thread safe]</b>
    virtual RobotBasePtr ReadRobotURI(const std::string& filename) {
        return ReadRobotURI(RobotBasePtr(),filename,AttributesList());
    }
    virtual RobotBasePtr ReadRobotXMLFile(const std::string& filename) {
        return ReadRobotURI(filename);
    }

    /** \brief Initialize a robot from in-memory data. <b>[multi-thread safe]</b>

        The robot should not be added the environment when calling this function.
        \param robot If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled
        \param atts The attribute/value pair specifying loading options. If contains "uri", then will set the new body's uri string to it. If the file is COLLADA, can also specify articulatdSystemId for the then atts can have articulatdSystemId.  More info in \ref arch_robot.
\param uri the URI of the scene. Used to inject the URI into the environment.
     */
    virtual RobotBasePtr ReadRobotData(RobotBasePtr robot, const std::string& data, const AttributesList& atts = AttributesList(), const std::string& uri=std::string()) = 0;
    virtual RobotBasePtr ReadRobotXMLData(RobotBasePtr robot, const std::string& data, const AttributesList& atts = AttributesList()) {
        return ReadRobotData(robot,data,atts);
    }

    /** \brief Initialize a robot from rapidjson document.

        The robot should not be added to the environment when calling this function.
        \param robot If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled.
        \param rEnvInfo the rapidjson document that contains the robot information.
        \param atts attributes that is passed to JSONReader for further options.
        \param uri the URI of the scene. Used to inject the URI into the environment.
        \returns the robot that is created.
     */
    virtual RobotBasePtr ReadRobotJSON(RobotBasePtr robot, const rapidjson::Value& rEnvInfo, const AttributesList& atts = AttributesList(), const std::string &uri = std::string()) = 0;

    /** \brief Initializes a kinematic body from a resource file. The body is not added to the environment when calling this function. <b>[multi-thread safe]</b>

        \param filename the name of the resource file, its extension determines the format of the file. See \ref supported_formats.
        \param body If a null pointer is passed, a new body will be created, otherwise an existing robot will be filled
        \param atts The attribute/value pair specifying loading options. Defined in \ref arch_kinbody.
     */
    virtual KinBodyPtr ReadKinBodyURI(KinBodyPtr body, const std::string& filename, const AttributesList& atts = AttributesList()) = 0;
    virtual KinBodyPtr ReadKinBodyXMLFile(KinBodyPtr body, const std::string& filename, const AttributesList& atts = AttributesList()) {
        return ReadKinBodyURI(body,filename,atts);
    }

    /// \brief Creates a new kinbody from an XML file with no extra load options specified. <b>[multi-thread safe]</b>
    virtual KinBodyPtr ReadKinBodyURI(const std::string& filename) {
        return ReadKinBodyURI(KinBodyPtr(),filename,AttributesList());
    }
    virtual KinBodyPtr ReadKinBodyXMLFile(const std::string& filename) {
        return ReadKinBodyURI(filename);
    }

    /** \brief Initializes a kinematic body from in-memory data. <b>[multi-thread safe]</b>

        The body should not be added to the environment when calling this function.
        \param body If a null pointer is passed, a new body will be created, otherwise an existing robot will be filled
        \param atts The attribute/value pair specifying loading options. If contains "uri", then will set the new body's uri string to it. If the file is COLLADA, can also specify articulatdSystemId for the then atts can have articulatdSystemId. More info in \ref arch_kinbody.
        \param uri the URI of the scene. Used to inject the URI into the environment.
     */
    virtual KinBodyPtr ReadKinBodyData(KinBodyPtr body, const std::string& data, const AttributesList& atts = AttributesList(), const std::string& uri=std::string()) = 0;
    virtual KinBodyPtr ReadKinBodyXMLData(KinBodyPtr body, const std::string& data, const AttributesList& atts = AttributesList()) {
        return ReadKinBodyData(body,data,atts);
    }

    /** \brief Initializes a kinematic body from rapidjson document.

        The body should not be added to the environment when calling this function.
        \param body If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled.
        \param rEnvInfo the rapidjson document that contains the robot information.
        \param atts attributes that is passed to JSONReader for further options.
        \param uri the URI of the scene. Used to inject the URI into the environment.
        \returns the body that is created.
     */
    virtual KinBodyPtr ReadKinBodyJSON(KinBodyPtr body, const rapidjson::Value& rEnvInfo, const AttributesList& atts = AttributesList(), const std::string &uri = std::string()) = 0;

    /** \brief Initializes an interface from a resource file. <b>[multi-thread safe]</b>

        \param pinterface If a null pointer is passed, a new interface will be created, otherwise an existing interface will be filled
        \param filename the name of the resource file, its extension determines the format of the file. See \ref supported_formats.
        \param atts The attribute/value pair specifying loading options. See the individual interface descriptions at \ref interface_concepts.
     */
    virtual InterfaceBasePtr ReadInterfaceURI(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts = AttributesList()) = 0;
    virtual InterfaceBasePtr ReadInterfaceXMLFile(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts = AttributesList()) {
        return ReadInterfaceURI(pinterface,type,filename,atts);
    }

    virtual InterfaceBasePtr ReadInterfaceURI(const std::string& filename, const AttributesList& atts = AttributesList()) = 0;
    virtual InterfaceBasePtr ReadInterfaceXMLFile(const std::string& filename, const AttributesList& atts = AttributesList()) {
        return ReadInterfaceURI(filename,atts);
    }

    /** \brief Initializes an interface from in-memory data. <b>[multi-thread safe]</b>

        \param pinterface If a null pointer is passed, a new interface will be created, otherwise an existing interface will be filled
        \param data string containing data
        \param atts The attribute/value pair specifying loading options. See the individual interface descriptions at \ref interface_concepts.
        \param uri the URI of the scene. Used to inject the URI into the environment.
     */
    virtual InterfaceBasePtr ReadInterfaceData(InterfaceBasePtr pinterface, InterfaceType type, const std::string& data, const AttributesList& atts = AttributesList(), const std::string& uri=std::string()) = 0;
    virtual InterfaceBasePtr ReadInterfaceXMLData(InterfaceBasePtr pinterface, InterfaceType type, const std::string& data, const AttributesList& atts = AttributesList()) {
        return ReadInterfaceData(pinterface,type,data,atts);
    }

    /** \brief reads in the rigid geometry of a resource file into a TriMesh structure

        \param filename the name of the resource file, its extension determines the format of the file. Complex meshes and articulated meshes are all triangulated appropriately. See \ref supported_formats.
        \param atts Options to control the parsing process.
     */
    virtual boost::shared_ptr<TriMesh> ReadTrimeshURI(boost::shared_ptr<TriMesh> ptrimesh, const std::string& filename, const AttributesList& atts = AttributesList()) = 0;
    virtual boost::shared_ptr<TriMesh> ReadTrimeshFile(boost::shared_ptr<TriMesh> ptrimesh, const std::string& filename, const AttributesList& atts = AttributesList()) {
        return ReadTrimeshURI(ptrimesh,filename,atts);
    }

    /** \brief reads in the rigid geometry from in-memory data from an opened cad file into a TriMesh structure

        \param data the contents of a file that stores the cad data
        \param formathint is the hint to the underlying cad importer for the format of data
        \param atts Options to control the parsing process.
     */
    virtual boost::shared_ptr<TriMesh> ReadTrimeshData(boost::shared_ptr<TriMesh> ptrimesh, const std::string& data, const std::string& formathint, const AttributesList& atts = AttributesList()) = 0;

    //@}

    /// \name Object Setting and Querying
    /// \anchor env_objects
    //@{

    /** \brief Add an interface to the environment

        Depending on the type of interface, the addition behaves differently. For bodies/robots, will add them to visual/geometric environment. For modules, will call their main() method and add them officially. For viewers, will attach a viewer to the environment and start sending it data.
        For interfaces that don't make sense to add, will throw an exception.
        \param[in] pinterface the pointer to an initialized interface
        \param[in] addMode One of IAM_X
        \param[in] cmdargs The command-line arguments for the module.
        \throw openrave_exception Throw if interface is invalid or already added
     */
    virtual void Add(InterfaceBasePtr pinterface, InterfaceAddMode addMode, const std::string& cmdargs=std::string()) = 0;

    virtual void Add(InterfaceBasePtr pinterface, bool bAnonymous, const std::string& cmdargs=std::string()) RAVE_DEPRECATED;

    /// \brief bodycallback(body, action)
    ///
    /// \param body KinBodyPtr
    /// \param action if 0 body has been removed from the environment (environment id is already reset), if 1 body was just added to environment
    typedef boost::function<void (KinBodyPtr, int)> BodyCallbackFn;
    virtual UserDataPtr RegisterBodyCallback(const BodyCallbackFn& callback) = 0;

    /// \brief Fill an array with all sensors loaded in the environment. <b>[multi-thread safe]</b>
    ///
    /// The sensors come from the currently loaded robots and the explicitly added sensors
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void GetSensors(std::vector<SensorBasePtr>& sensors, uint64_t timeout=0) const = 0;

    /// \brief Removes a currently loaded interface from the environment. <b>[multi-thread safe]</b>
    ///
    /// The function removes currently loaded bodies, robots, sensors, problems from the actively used
    /// interfaces of the environment. This does not destroy the interface, but it does remove all
    /// references managed. Some interfaces like modules have destroy methods that are called to signal
    /// unloading. Note that the active interfaces are different from the owned interfaces.
    /// \param[in] obj interface to remove
    /// \return true if the interface was successfully removed from the environment.
    virtual bool Remove(InterfaceBasePtr obj) = 0;

    /// \brief Removes all kinbodies that match the name
    ///
    /// \param[in] name of the kinbody to remove
    /// \return true if the kinbody was successfully removed from the environment.
    virtual bool RemoveKinBodyByName(const std::string& name) = 0;

    /// \brief Query a body from its name. <b>[multi-thread safe]</b>
    ///
    /// \return first KinBody (including robots) that matches with name
    virtual KinBodyPtr GetKinBody(const std::string& name) const =0;

    /// \brief query a body from its name
    virtual KinBodyPtr GetKinBody(const string_view name) const =0;

    virtual KinBodyPtr GetKinBody(const char* pname) const =0;

    /// \brief Query a body from its id. <b>[multi-thread safe]</b>
    ///
    /// \return first KinBody (including robots) that matches with the id (ie KinBody::GetId). This is different from KinBody::GetEnvironmentBodyIndex!
    virtual KinBodyPtr GetKinBodyById(const std::string& id) const =0;

    virtual KinBodyPtr GetKinBodyById(const string_view id) const =0;

    /// \brief Query the largest environment body index in this environment. <b>[multi-thread safe]</b>
    ///
    /// \return largetst environment body index among the bodies in this environment
    virtual int GetMaxEnvironmentBodyIndex() const = 0;

    /// \brief Return the number of bodies currently in the environment. <b>[multi-thread safe]</b>
    virtual int GetNumBodies() const = 0;
    
    /// \brief Query a sensor from its name. <b>[multi-thread safe]</b>
    /// \return first sensor that matches with name, note that sensors attached to robots have the robot name as a prefix.
    virtual SensorBasePtr GetSensor(const std::string& name) const =0;

    /// \brief Query a robot from its name. <b>[multi-thread safe]</b>
    /// \return first Robot that matches the name
    virtual RobotBasePtr GetRobot(const std::string& name) const =0;

    /// \brief Get all bodies loaded in the environment (including robots). <b>[multi-thread safe]</b>
    ///
    /// A separate **interface mutex** is locked for reading the bodies.
    /// \param[out] bodies filled with all the bodies
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void GetBodies(std::vector<KinBodyPtr>& bodies, uint64_t timeout=0) const = 0;

    /// \brief Fill an array with all robots loaded in the environment. <b>[multi-thread safe]</b>
    ///
    /// A separate **interface mutex** is locked for reading the bodies.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void GetRobots(std::vector<RobotBasePtr>& robots, uint64_t timeout=0) const = 0;

    /// \brief Retrieve published bodies, completes even if environment is locked. <b>[multi-thread safe]</b>
    ///
    /// A separate **interface mutex** is locked for reading the modules.
    /// Note that the pbody pointer might become invalid as soon as GetPublishedBodies returns.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void GetPublishedBodies(std::vector<KinBody::BodyState>& vbodies, uint64_t timeout=0) = 0;

    /// \brief Retrieve published body of specified name, completes even if environment is locked. <b>[multi-thread safe]</b>
    ///
    /// A separate **interface mutex** is locked for reading the modules.
    /// Note that the pbody pointer might become invalid as soon as GetPublishedBody returns.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    /// \return true if name matches to a published body
    virtual bool GetPublishedBody(const std::string& name, KinBody::BodyState& bodystate, uint64_t timeout=0) = 0;

    /// \brief Retrieve joint values of published body of specified name, completes even if environment is locked. <b>[multi-thread safe]</b>
    ///
    /// A separate **interface mutex** is locked for reading the modules.
    /// Note that the pbody pointer might become invalid as soon as GetPublishedBodyJointValues returns.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    /// \return true if name matches to a published body
    virtual bool GetPublishedBodyJointValues(const std::string& name, std::vector<dReal> &jointValues, uint64_t timeout=0) = 0;

    /// \brief Retrieve body transform of all published bodies whose name matches prefix, completes even if environment is locked. <b>[multi-thread safe]</b>
    ///
    /// A separate **interface mutex** is locked for reading the modules.
    /// Note that the pbody pointer might become invalid as soon as GetPublishedBody returns.
    /// \param prefix the prefix to match to the target names.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void GetPublishedBodyTransformsMatchingPrefix(const std::string& prefix, std::vector<std::pair<std::string, Transform> >& nameTransfPairs, uint64_t timeout = 0) = 0;

    /// \brief Updates the published bodies that viewers and other programs listening in on the environment see.
    ///
    /// For example, calling this function inside a planning loop allows the viewer to update the environment
    /// reflecting the status of the planner.
    /// Assumes that the physics are locked.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void UpdatePublishedBodies(uint64_t timeout=0) = 0;

    /// Get the corresponding body from its unique network id
    virtual KinBodyPtr GetBodyFromEnvironmentBodyIndex(int bodyIndex) const = 0;

    /// Get the name of a kinbody based on its index in the environment
    /// Returns false if the index does not map to a valid body in the environment
    virtual bool GetBodyNameFromEnvironmentBodyIndex(int bodyIndex, std::string& out) const = 0;

    /// Fetch the index of a body in the environment by name
    /// Returns zero if no body with that name exists in the environment
    virtual int GetEnvironmentBodyIndexByName(string_view bodyName) const = 0;

    /// Get the corresponding bodies from its unique network id
    ///
    /// Calling GetBodyFromEnvironmentBodyIndex in loop should be replaced by this function to minimize scoped lock constrution and deconstruction
    /// \param[in] bodyIndices body indices
    /// \param[out] bodies vector of bodies in the same order as bodyIndices
    virtual void GetBodiesFromEnvironmentBodyIndices(const std::vector<int>& bodyIndices,
                                                     std::vector<KinBodyPtr>& bodies) const = 0;

    /// Get the corresponding body from its unique network id
    inline KinBodyPtr GetBodyFromEnvironmentId(int bodyIndex) RAVE_DEPRECATED {
        return GetBodyFromEnvironmentBodyIndex(bodyIndex);
    }

    /// \brief Triangulation of the body including its current transformation. trimesh will be appended the new data.  <b>[multi-thread safe]</b>
    ///
    /// \param[out] trimesh - The output triangle mesh
    /// \param[in] body body the triangulate
    /// \throw openrave_exception Throw if failed to add anything
    void Triangulate(TriMesh& trimesh, KinBodyConstPtr pbody) RAVE_DEPRECATED {
        return Triangulate(trimesh, *pbody);
    }
    virtual void Triangulate(TriMesh& trimesh, const KinBody &body) = 0;

    /// \brief General triangulation of the whole scene. <b>[multi-thread safe]</b>
    ///
    /// \param[out] trimesh - The output triangle mesh. The new triangles are appended to the existing triangles!
    /// \param[in] options - Controlls what to triangulate.
    /// \param[in] selectname - name of the body used in options
    /// \throw openrave_exception Throw if failed to add anything
    virtual void TriangulateScene(TriMesh& trimesh, SelectionOptions options, const std::string& selectname) = 0;
    //@}

    /// \brief Load a new module, need to Lock if calling outside simulation thread
    virtual int AddModule(ModuleBasePtr module, const std::string& cmdargs) = 0;

    /// \deprecated (12/01/30) see \ref EnvironmentBase::AddModule
    virtual int LoadProblem(ModuleBasePtr module, const std::string& cmdargs) RAVE_DEPRECATED {
        return AddModule(module,cmdargs);
    }

    /// \brief Fills a list with the loaded modules in the environment.
    ///
    /// A separate **interface mutex** is locked for reading the modules.
    /// If the environment is locked, the modules are guaranteed to stay loaded in the environment.
    /// \param timeout microseconds to wait before throwing an exception, if 0, will block indefinitely.
    /// \throw openrave_exception with ORE_Timeout error code
    virtual void GetModules(std::list<ModuleBasePtr>& listModules, uint64_t timeout=0) const = 0;

    /// \deprecated (12/01/30)
    virtual void GetLoadedProblems(std::list<ModuleBasePtr>& listModules) const {
        GetModules(listModules);
    }

    /// \brief Return the global environment mutex used to protect environment information access in multi-threaded environments.
    ///
    /// Accessing environment body information and adding/removing bodies
    /// or changing any type of scene property should have the environment lock acquired. Once the environment
    /// is locked, the user is guaranteed that nnothing will change in the environment.
    virtual EnvironmentMutex& GetMutex() const = 0;

    /// \name 3D plotting methods.
    /// \anchor env_plotting
    //@{

    /// \deprecated (10/11/05)
    typedef OpenRAVE::GraphHandlePtr GraphHandlePtr RAVE_DEPRECATED;

    /// \brief Return a viewer with a particular name.
    ///
    /// When no name is specified, the first loaded viewer is returned.
    virtual ViewerBasePtr GetViewer(const std::string& name="") const = 0;

    /// \brief Returns a list of loaded viewers with a pointer to a lock preventing the list from being modified.
    ///
    /// If the environment is locked, the viewers are guaranteed to stay loaded in the environment.
    virtual void GetViewers(std::list<ViewerBasePtr>& listViewers) const = 0;

    /// \brief Plot a point cloud with one color. <b>[multi-thread safe]</b>
    ///
    /// \param ppoints array of points
    /// \param numPoints number of points to plot
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param fPointSize size of a point in pixels
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending
    /// \param drawstyle if 0 will draw pixels. if 1, will draw 3D spheres
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1), int drawstyle = 0) = 0;

    /// \brief. Plots 3D points with individual colors. <b>[multi-thread safe]</b>
    ///
    /// Arguments same as plot3 with one color, except has an individual color for every point
    /// \param colors An array of rgb colors of size numPoints where each channel is in [0,1].
    ///               colors+(bhasalpha?4:3) points to the second color.
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param drawstyle if 0 will draw pixels. if 1, will draw 3D spherse
    /// \param bhasalpha if true, then each color consists of 4 values with the last value being the alpha of the point (1 means opaque). If false, then colors is 3 values.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha = false) = 0;

    /// \brief Draws a series of connected lines with one color. <b>[multi-thread safe]</b>
    ///
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;

    /// \brief Draws a series of connected lines with individual colors. <b>[multi-thread safe]</b>
    ///
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    /// \brief Draws a list of individual lines, each specified by a succeeding pair of points. <b>[multi-thread safe]</b>
    ///
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;

    /// \brief Draws a list of individual lines, each specified by a succeeding pair of points. <b>[multi-thread safe]</b>
    ///
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    /// \brief Draws an arrow p1 is start, p2 is finish. <b>[multi-thread safe]</b>
    ///
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;

    /// \brief Draws a label. <b>[multi-thread safe]</b>
    ///
    /// \param worldPosition is the position of the label in world space.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawlabel(const std::string& label, const RaveVector<float>& worldPosition, const RaveVector<float>& color = RaveVector<float>(0,0,0,1), float height = 0.05) = 0;

    /// \brief Draws a box. <b>[multi-thread safe]</b>
    ///
    /// extents are half the width, height, and depth of the box
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) = 0;

    /// \brief Draws an array of box. <b>[multi-thread safe]</b>
    ///
    /// extents are half the width, height, and depth of the box
    /// \return handle to plotted boxes, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawboxarray(const std::vector<RaveVector<float>>& vpos, const RaveVector<float>& vextents, const std::vector<RaveVector<float>>& colors = {}) = 0;

    /// \brief Draws a AABB. <b>[multi-thread safe]</b>
    ///
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawaabb(const AABB& aabb, const RaveTransform<float>& transform, const RaveVector<float>& vcolor, float transparency) = 0;

    /// \brief Draws a OBB. <b>[multi-thread safe]</b>
    ///
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawobb(const OrientedBox& obb, const RaveVector<float>& vcolor, float transparency) = 0;

    /// \brief Draws a textured plane. <b>[multi-thread safe]</b>
    ///
    /// \param tplane describes the center of the plane. the zaxis of this coordinate is the normal of the plane
    /// \param vextents the extents of the plane along the x and y directions (z is ignored)
    /// \param vtexture a 3D array specifying height x width x color (the color dimension can be 1, 3, or 4 (for alpha blending))
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture) = 0;

    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) = 0;

    /// \brief Draws a triangle mesh, each vertices of each triangle should be counter-clockwise. <b>[multi-thread safe]</b>
    ///
    /// \param ppoints - array of 3D points
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param pIndices If not NULL, zero-based indices into the points for every triangle. pIndices
    /// should be of size numTriangles. If pIndices is NULL, ppoints is assumed to contain numTriangles*3
    /// points and triangles will be rendered in list order.
    /// \param color The color of the triangle. The last component of the color is used for alpha blending
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors) = 0;
    //@}

    //@{ debug/global commands

    /// sets the debug level
    /// \param level 0 for no debug, 1 - to print all debug messeges. Default
    ///             value for release builds is 0, for debug builds it is 1
    /// declaring variables with stdcall can be a little complex
    virtual void SetDebugLevel(int level) = 0;
    virtual int GetDebugLevel() const = 0;

    /// \brief unit - (name, mult factor to meters)
    ///
    /// \return (name, mult factor) that describes the unit's name and how many meters there are in 1 unit. For example ("mm", 0.001)
    /// \deprecated (23/07/03), use GetUnitInfo instead.
    virtual std::pair<std::string, dReal> GetUnit() const RAVE_DEPRECATED = 0;

    /// \brief set units for the current environment.
    ///
    /// \param unit (name, mult factor) that describes the unit's name and how many meters there are in 1 unit. For example ("mm", 0.001)
    /// \deprecated (23/07/03), use SetUnitInfo instead.
    virtual void SetUnit(std::pair<std::string, dReal> unit) RAVE_DEPRECATED = 0;

    /// \brief unitInfo
    ///
    /// \return UnitInfo that describes length unit, mass unit, time unit and angle unit
    virtual UnitInfo GetUnitInfo() const = 0;

    /// \brief set unitInfo for the current environment.
    ///
    /// \param unitInfo that describes length unit, mass unit, time unit and angle unit
    virtual void SetUnitInfo(UnitInfo unitInfo) = 0;

    //@}

    /// \brief returns the unique id of the environment
    inline int GetId() const {
        return __nUniqueId;
    }

    /// \brief returns the scene name
    inline const std::string& GetName() const {
        return _name;
    }

    /// \brief returns the scene name and id as formated string
    inline const std::string& GetNameId() const {
        return _formatedNameId;
    }

    /// \brief sets a named parameter to be tracked by the environment
    ///
    /// internally locks the environment mutex
    virtual void SetUInt64Parameter(const std::string& parameterName, uint64_t value) = 0;

    /// \brief removes a named parameter tracked by the environment
    ///
    /// internally locks the environment mutex
    /// \return true if parameter was in the environment and now is removed
    virtual bool RemoveUInt64Parameter(const std::string& parameterName) = 0;

    /// \brief retries the named parameter to be tracked by the environment.
    ///
    /// internally locks the environment mutex
    /// if parameter is not present, will return defaultValue
    virtual uint64_t GetUInt64Parameter(const std::string& parameterName, uint64_t defaultValue) const = 0;

    /// \brief notifys name of kin body is changed.
    ///
    /// Should be called when name of body added to this env is modified. Should not be called when name of body in other env or not added to any env is modified.
    /// \param oldName name before change
    /// \param newName name after change
    /// \return true if can make the change, and the changes are notified. Otherwise false meaning there will be a conflict
    virtual bool NotifyKinBodyNameChanged(const std::string& oldName, const std::string& newName) = 0;

    /// \brief retries the named parameter to be tracked by the environment.
    ///
    /// Should be called when id of body added to this env is modified. Should not be called when name of body in other env or not added to any env is modified.
    /// \param oldId id before change
    /// \param newId id after change
    /// \return true if can make the change, and the changes are notified. Otherwise false meaning there will be a conflict
    virtual bool NotifyKinBodyIdChanged(const std::string& oldId, const std::string& newId) = 0;

    /// \brief info structure used to initialize environment
    class OPENRAVE_API EnvironmentBaseInfo : public InfoBase
    {
public:
        EnvironmentBaseInfo();
        EnvironmentBaseInfo(const EnvironmentBaseInfo& other);
        bool operator==(const EnvironmentBaseInfo& other) const;
        bool operator!=(const EnvironmentBaseInfo& other) const;

        void Reset() override;
        void SerializeJSON(rapidjson::Value& rEnvInfo, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options=0) const override;

        void DeserializeJSON(const rapidjson::Value& rEnvInfo, dReal fUnitScale, int options) override;

        /// \param vInputToBodyInfoMapping maps indices into rEnvInfo["bodies"] into indices of _vBodyInfos: rEnvInfo["bodies"][i] -> _vBodyInfos[vInputToBodyInfoMapping[i]]. This forces certain _vBodyInfos to get updated with specific input. Use -1 for no mapping
        void DeserializeJSONWithMapping(const rapidjson::Value& rEnvInfo, dReal fUnitScale, int options, const std::vector<int>& vInputToBodyInfoMapping);

        std::string _description;   ///< environment description
        std::vector<std::string> _keywords;  ///< some string values for describinging the environment
        Vector _gravity = Vector(0,0,-9.797930195020351);  ///< gravity and gravity direction of the environment
        std::string _uri; ///< optional, the URI this environment comes from
        std::string _referenceUri; ///< optional, if the environment was opened by referencing another environment file, then this is the URI for that file.
        std::vector<KinBody::KinBodyInfoPtr> _vBodyInfos; ///< list of pointers to KinBodyInfo
        std::map<std::string, uint64_t> _uInt64Parameters; ///< user parameters associated with the environment
        int _revision = 0;  ///< environment revision number
        UnitInfo _unitInfo; ///< environment unitInfo
        int64_t _lastModifiedAtUS = 0; ///< us, linux epoch, last modified time of the environment when it was originally loaded from the environment.
        int64_t _revisionId = 0; ///< the webstack revision for this loaded kinbody
    };
    typedef boost::shared_ptr<EnvironmentBaseInfo> EnvironmentBaseInfoPtr;
    typedef boost::shared_ptr<EnvironmentBaseInfo const> EnvironmentBaseInfoConstPtr;

    /// \brief returns environment revision number
    virtual int GetRevision() const = 0;

    /// \brief sets the scene description
    virtual void SetDescription(const std::string& sceneDescription) = 0;

    /// \brief returns the scene description
    virtual std::string GetDescription() const = 0;

    /// \brief sets the scene keywords
    virtual void SetKeywords(const std::vector<std::string>& sceneKeywords) = 0;

    /// \brief returns the scene keywords
    virtual std::vector<std::string> GetKeywords() const = 0;

    /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
    virtual void ExtractInfo(EnvironmentBaseInfo& info) = 0;

    /// \brief update EnvironmentBase according to new EnvironmentBaseInfo
    ///
    /// \param vCreatedBodies the bodies created in this operation
    /// \param vModifiedBodies the bodies modified in this operation
    /// \param vRemovedBodies the bodies removed from the environment in this operation
    /// \param updateMode one of UFIM_X
    virtual void UpdateFromInfo(const EnvironmentBaseInfo& info, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, UpdateFromInfoMode updateMode) = 0;

    int _revision = 0;  ///< environment current revision
    std::string _description;   ///< environment description
    std::vector<std::string> _keywords;  ///< some string values for describinging the environment

protected:
    virtual const char* GetHash() const {
        return OPENRAVE_ENVIRONMENT_HASH;
    }

    void _InitializeInternal();

    std::string _name;   ///< environment name. only set during construction and cloning.
    std::string _formatedNameId; ///< environment name and id. \see GetNameId

private:
    UserDataPtr __pUserData;         ///< \see GetUserData
    int __nUniqueId;         ///< \see RaveGetEnvironmentId
};

} // end namespace OpenRAVE

#endif
