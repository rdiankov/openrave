.. _changelog:

ChangeLog
#########

Version 0.5.0 Unstable
======================

Subversion Revision: **Unreleased**

Initial Release: **Unreleased**

Core
----

* fixed physics simulation loop freezing, added tests

* fixed "prefix" attribute when colldata models are used.

* added "scalegeometry" attribute to kinbody loading. can have different scales along XYZ.

* Geometry files imported with assimp now load multiple geometries per material in order to preserve colors. Added :meth:`.KinBody.InitFromGeometries`.

* KinBody::KinBodyStateSaver and RobotBase::RobotStateSaver now have **Restore** functions that allows users to get back to the original robot without having to destroy the handle.

* Now properly handling inter-grabbed-body collisions: if two grabbed bodies are initially colliding when grabbed, then their self-colision should be ignored. Also fixed a bug with :meth:`.Robot.Manipulator.CheckEndEffectorCollision`

* Added a new class :class:`.ConfigurationSpecification` to manage configuration spaces, it is shared by both planners and trajectories.

* Separated the affine DOF spece configuration from robot class into the global openrave space. See :class:`.DOFAffine`, :meth:`.RaveGetIndexFromAffineDOF`, :meth:`.RaveGetAffineDOFFromIndex`, :meth:`.RaveGetAffineDOF`, and :meth:`.RaveGetAffineDOFValuesFromTransform`

* Can now reset the local manipulator coordinate system with :meth:`.Robot.Manipulator.SetLocalToolTransform`

Inverse Kinematics
------------------

* added :meth:`.IkSolver.RegisterCustomFilter` that allows any number of filters to be registered with priority. :meth:`.IkSolver.SetCustomFilter` is deprecated.

* Fixed TranslationDirection5D IK bug, upgrade ikfast version

* ikfast IkSolvers only check collisions of links that can possible move due to new joint values.

* Added new :class:`.IkFilterOptions.IgnoreEndEffectorCollision` option, this disables the end effector links and their attached bodies from environment collision considerations.

* fixed ikfast bug when prismatic joints are present, ikfast version is now **48**.

Grasping
--------

* fixes in grasping with standoff

* added IK checking option to :ref:`module-grasper-graspthreaded`, showing usage in :mod:`.examples.fastgraspingthreaded` example.

* added new :mod:`.examples.fastgraspingthreaded` example to show how to use multithreaded functions to compute good grasps in real-time.

* added **--numthreads** option to **openrave.py --database grasping** to allow users to set number of threads.

Planning
--------

* Can register callback functions during planners to stop the planner via :meth:`.Planner.RegisterPlanCallback`. Planner developers should use :meth:`.Planner._CallCallbacks` to call the callbacks.

* :meth:`.Planner.PlanPath` now returns a :ref:`.PlannerStatus` enum showing how planner exited. It does not support pOutStream anymore.

* Added velocity and acceleration limits to :class:`.Planner.PlannerParameters`

* Each planner needs to initialize the trajectory with :meth:`.Trajectory.Init` (GetParameters()->_configurationspecification);

Trajectories
------------

* Completely redesigned the :class:`.Trajectory` class, see `Trajectory Concepts`_ for usage.
* Added :meth:`.Trajectory.Clone`

* Changed trajectory serialization format to XML, see :ref:`arch_trajectory_format`

* Added trajectory API to openravepy.

* Added many useful trajectory routines in the :class:`.planningutils` namespace. For example: :meth:`.planningutils.VerifyTrajectory`, :meth:`.planningutils.RetimeActiveDOFTrajectory`, :meth:`.planningutils.RetimeAffineTrajectory`, :meth:`.planningutils.ConvertTrajectorySpecification`, :meth:`.planningutils.ReverseTrajectory`.

Python
------

* Added **releasegil** parameter to :meth:`.Interface.SendCommand` that can temporarily release the Python GIL.

* added two python examples showing how to use PyQt + OpenRAVE together. :mod:`.examples.qtexampleselector` :mod:`.examples.qtserverprocess`

* split openravepy into smaller files for faster compilation

Misc
----

* "skipgeometry" now being acknowledged in :meth:`.Environment.Load`, fixes the **openrave.py inversekinematics database --getfilename** option.

* <render> tag for non-trimesh objects works now

* more reasonable default acceleration and velocity limits

* fixed octave graspplanning demo

* odephysics now uses dJointFeedback to compute forces/torques on links

* removed **KinBody.SetGuiData** and **KinBody.GetGuiData** and replaced with :meth:`.KinBody.GetViewerData` similar to how collision/physics are handled.

* added  :mod:`.examples.cubeassembly` to show a robot assembling a cube from randomly scattered blocks.

* updated :ref:`collisionchecker-bullet` collision checker to be up to par with ODE. Now the two engines should be interchangeable,

Version 0.4.2
=============

Subversion Revision: 2678

Initial Release: 2011/08/11

Core
----

* CMake OpenRAVE_CORE_LIBRARIES variable now returns both openrave and openrave-core.

* Now reading physics data from COLLADA files, also fixed bugs in collada readers/writers.

* Can compile without qt4 being present.

* Fixed collision caching bug with ODE/Bullet CheckSelfCollision.

Planning
--------

* MoveToHandPosition, MoveManipulator, MoveActiveJoints, and Manipulator::CheckIndependentCollision now only check the **active** links if the CO_ActiveDOFs option is set on the collision checker.

* added multiple goals to MoveManipulator and MoveActiveJoints commands

Release
-------

* Debian packages of different openrave versions will now install without conflicting with each other since they will share no common files. symlinks pointing to non-versioned programs are written in a version-independent 'openrave' package.

Python
------

* Redesigned the openravepy structure so that loading is faster and multiple openravepy versions can be selected at load time.

* Started a new :mod:`openravepy.misc` library that is not loaded by default. The OpenRAVEGlobalArguments and MultiManipIKSolver helper classes are now part of it.

Octave/Matlab
-------------

* fixed the grasping demo

* added orRobotSensorConfigure.m to power and sensors and display their data

* Octave stripping symbols

Inverse Kinematics
------------------

* Fixed major IK fast bug when intersecting axes of robot are not at the ends.

Tests
-----

* test_programs is now runnable by windows

* test_ikfast is now also included in the regular tests to determine release. The full IK tests are run separately to gather statistics on ikfast.

Grasping
--------

* grasping database now uses the producer, consumer, gatherer model, removed updateenv and disableallbodies from the generate method

* implemented the unfinished :meth:`.databases.grasping.GraspingModel.computeSphereApproachRays`

Misc
----

* fixed bug in ODE physics when contacts overflow and added check for 0 quaternions.

* ode physics is more stable, can now modify erp and cfm parameters through xml

* fixed bug grasperplanner that exits at coarse step without going to fine step phase

* fixed bug with non-adjacent link computation

* fixed bug with not checking joint limits when resetting robot pose in KinBody::_ComputeInternalInformation

* fixed bug in BaseLaser <resolution> tag

* Logging module: added exporting geometric primitives of :ref:`savescene <module-logging-savescene>`

* fixed ode bug with ray collisions now returning closest point

Version 0.4.1
=============

Subversion Revision: 2574

Initial Release: 2011/07/08

Core
----

* Fixed self-collision problem when grasping two objects (#31).

Grasping
--------

* Fixed major bug in force closure computation.

* The direction on the gripper is now a parameter of the grasp set.

* Added 5D IK support for grasp planning. Check out the :mod:`.examples.graspplanning` example. This required handling and passing goals as :class:`.IkParameterization` structures.

Version 0.4.0
=============

Subversion Revision: 2557

Initial Release: 2011/07/02

Core
----

* fixed collada loading of formulas

* fixed caching issue with ik files in ikfastsolvers

* added a new :class:`.SpaceSampler` interface for sophisticated discrete/deterministic/randomized samplers.

* deprecated the RaveRandomX functions in favor of the new samplers

* Added a Prop_RobotActiveDOFs change callback in order to catch SetActiveDOFs messages

* renamed ProblemInstance interface into Module. Users should use the ModuleBase class.

* Environment can now support multiple viewers attached to it and can query them with their name. Plotting methods through the environment send commands to all viewers at once.

* **Compatibility Break:** EnvironmentBase AddKinBody/AddRobot/AddSensor return void instead of bool.

* added a Level_VerifyPlans debug level that globally notifies planners/modules to double check their outputs. Used for testing.

* added :meth:`.KinBody.Joint.SetWrapOffset`, :meth:`.KinBody.Link.SetStatic`, :meth:`.KinBody.Link.GeomProperties.SetRenderFilename` functions

* added :meth:`.KinBody.SetZeroConfiguration` for calibration

* caching computation of hashes for faster kinbody/robot loading

* the Environment Load methods takes an attributes list, and Save method allows for selection of what gets saved.

 * renamed EnvironmentBase::TriangulateOptions to EnvironmentBase::SelectionOptions

* renamed EnvironmentBase *XMLFile and *XMLData methods to *URI and *Data.

Planning
--------

* added a new planner parameter _neighstatefn that adds two states together.

* added a RobotConfiguration sampler for sampling robot active DOFs used for planning

* added a Halton Sequence sampler

* removed the PlannerParameters::_constraintfn and replaced it with PlannerParameters::_checkpathconstraints. Combined with _neighstatefn, the behavior of the old PlannerParameters::_constraintfn can be achieved. Allows us to remove all collision calls and dependencies on robots from planners!!

* removed the PlannerParameters::_tWorkspaceGoal parameter since it is non-generic and not used in openrave.

* added PlannerParameters::_sampleinitialfn to sample initial goals for the planner

* added a _fromgoal parameter to PlannerParameters::_neighstatefn so users can know which direction the tree is growing in.

* added a new **openrave/planningutils.h** file that contains many functions/heuristics to help users build planning algorithms.

 * LineCollisionConstraint
 * SimpleDistanceMetric
 * SimpleNeighborhoodSampler
 * ManipulatorIKGoalSampler
 * VerifyTrajectory
 * JitterActiveDOF
 * JitterTransform

* added VerifyTrajectory command in BaseManipulation.

* fixed major bug in :ref:`WorkspaceTrajectoryTracker <planner-workspacetrajectorytracker>` (ie MoveHandStraight) due to obstacle checking

* many changes to the RRT extend function to prevent infinite loops

* Jittering uses perterbutation in order to reject border collisions easily

Inverse Kinematics
------------------

* implemented '--show' command for inversekinematics

* ikfast fix in solvePairVariablesHalfAngle, lookat3d works for simple mechanisms.

* added a validation step to the ikfast openrave iksolver so wrong solutions are **never** returned.

Sensors
-------

* camera intrinsics now include distortion model and focal length, viewer rendering respects the focal length

* removed transform from laser data, all sensors have a transform data type that is not part of the data state

Viewers
-------

* viewer showing scene normals

* added a new :ref:`module-viewerrecorder` interface that can attach to viewers and record their images. The recorder works on a separate thread, so it should have a minimal impact on performance.

* Removed ffmpeg/video recording from qtcoin viewer.

* added watermarking support through :ref:`SetWatermark command <module-viewerrecorder-setwatermark>`

* deprecated the ViewerBase::RegisterCallback function and added individdual functions for item selection and new viewer image: RegisterItemSelectionCallback and RegisterViewerImageCallback

* Added ViewerBase::GetCameraIntrinsics for the current camera location

Misc
----

* added more tests: openrave global runtime, API Sanity Autotest XML

* added :meth:`.IkSolver.SetCustomFilter` in openravepy

* fixed bug in velocity controller mimic joints

* added Kawada Hiro NX (robots/kawada-hironx.zae) industrial robot model

* fixed IV/VRML model loading scaling

* removed links without any geometry attached to them from the non-adjacent lists

* added examples :mod:`.examples.simplemanipulation` (thanks to Alan Tan), added :mod:`.examples.simplegrasping`

* added GraspThreaded command to grasper plugin to allow for multithreaded computation of grasps. Added the corresponding bindings to the openravepy grasping module.

* fixed assert in ODE when collision checking with contact points.

Version 0.3.2
=============

Subversion Revision: 2452

Initial Release: 2011/05/11

Core
----

* fixed major bug in synchronizing collision and openrave world

* added openrave-robot.py which allows introspection into robot files. This deprecates openrave-hash.py. added bash completion for it.

* added openrave-createplugin.py which allows new users to easily setup the plugin directories and get something running. also works on creating executables. added bash completion for it.

* changed way of searching for collada-dom to prepare for its 2.3.1 release.

* removed a dependency on mathextra.h from geometry.h

* ReadKinBody*, ReadRobot*, and Load can now process rigid body models like IV, VRML, STL, etc and convert them automatically to KinBody objects. For example::

  openrave windmill.iv
  openrave test1.iv
  Environment.Load('test1.iv')
  Environment.ReadKinBodyXMLFile('test1.iv')

* fixed collada bug in parsing robot sensors, added a barrett-wam-sensors.zae file to show a working example.

Windows
-------

* small changes to the way symlinks are handled on install/uninstall since windows does not handle symlinks.

* rearranged the windows pre-compiled DLLs and added official libcollada pre-compiled DLLs.

* All openrave DLLs are now suffixed with the msvc version and openrave soversion.

Testing
-------

* fixed bugs in multiprocess plugin

* added extensive basic math and kinematics tests

* added a 'testmode' in all python examples so unit testing can run the examples safely

Release
-------

* adding the soversion suffix to all libopenrave libraries: libopenrave -> libopenraveX.Y. There is no libopenrave or libopenrave-core anymore, so linking with "-lopenrave" or "-lopenrave-core" will fail.

* releases are now suffxed with floating-point precision mode

Version 0.3.1
=============

Subversion Revision: 2402

Initial Release: 2011/04/24

Core
----

* Fixed OpenRAVE freeze when closed with Ctrl-C

* Fixed problem with detecting system crlibm installs

Python
------

* openravepy now gets copied onto the python site-packages or dist-packages folder. For Linux users, this means it is not necessary to set the PYTHONPATH anymore when installing to /usr or /usr/local.

* ikfast fixes inclusion of math libraries and python-mpmath

Release
-------

* The openravepy python bindings now get installed into the python site-packages/dist-packages folder.

* Using cpack to componentize all the installs and create debian source packages. The debian source packages are created with DebSourcePPA.cmake and can handle multiple distributions.

Version 0.3.0
=============

Subversion Revision: r2371

Initial Release: 2011/04/18

Core
----

* Moved all the header files to the 'include/openrave-$MAJOR.$MINOR/openrave' folder. 'rave' folder is now deprecated.

* Include files will now be installed in openrave-$MAJOR.$MINOR folders

* Binaries will now be suffixed with $MAJOR.$MINOR. Ie openrave0.3-config, openrave0.3.py. Symlinks will be provied to openrave

* OpenRAVE installs version-specific cmake configuration files stored in lib/cmake/openrave-$MAJOR.$MINOR/. The FindOpenRAVE.cmake file just looks for these openrave installations.

* Removed linking with Coin3d due to GPL license issue. Now will attemp to load only if a ProblemInstance supports model loading.

IKFast
------

* Added TranslationLocalGlobal6D new IK type

* Fixed inversekinematics database generator loading/caching problems. Fixed a cloning problem.

* Made sure all python examples rely on pre-generated ik files.

Release
-------

* Windows Installer using Nullsoft Scriptable Install System. It automatically downloads necessary libraries and registers openrave to the windows registry.

* Added many scripts to automate upload to sourceforge

Windows
-------

* Updated all DLLs and libraries, cleaned a lot of old stuff, now relying on official installations of boost and qt4.

* Reduced the number of MSVC special cases in the build system

Misc
----

* Fixed ivcon loading bug

* Added a ivmodelloader interface to use coin3d functionality across plugin boundaries

Version 0.2.20
==============

Subversion Revision: r2241

Initial Release: 2011/03/28

Core
----

* Addding linking with assimp. If present, libopenrave-core will not use the buggy ivcon.

* Added EnvironmentBase::ReadTrimeshFile allowing ability to load kinbody files from the openrave command line.

* Refactored openrave-core and several plugins in order to make compilation more parallelizable.

* Clone now returns a void instead of bool since it relies on exceptions for error handling instead of return values.

* Fixed many bugs with prefixing names for robots/kinbodies in the XML/COLLADA readers.

* Better detection of sympy installation. Can now use sympy system installs if they pass a "compatibility test". If 0.6.7, can patch sympy dynamically.

* removed recursive lock from plugindatabase.h

* FindIKSolution forces environment lock since it is so common to use

* Support compilation with Visual Studio 2010

Python
------

* Can now lock environments in multiple python threads without deadlocking.

IKFast
------

* ikfastsolvers: ikfast c++ files are individually compiled instead of included as headers. speeds up compilation

* perf timing is more accurate using CLOCK_MONOTONIC

* Added automatic updating of the cached files in sandbox/updateikfiles.py

* Added a lot of documentation on ikfast.

* Added 5DOF inverse kinematics: position+direction.

* Added a TranslationXY2D primitive for 2D translation (see tutorial_iktranslation2d example)

Testing
-------

* Unit testing of ikfast using python nose, developed several custom plugins in test/nosetests

* Linked with jenkins test server now at http://www.openrave.org/testing

Documentation
-------------

* Documentation infrastructure rewritten. It now uses mostly reStructuredText and compiled with sphinx. the official openrave homepage is also outputted by sphinx. this allows us to combine interfaces, testing results, python docs, and C++ docs all in one. epydoc has been removed.

* The robot database is now compiled from the ikfast results with robot images and links to the testing server.

Sensors
-------

* Removed sensor Init/Reset methods and added a Configure method for controlling power and rendering properties since all sensors will share these configurations.

* Added an actuator sensor for modeling motors, etc.

* Added a camera viewer GUI that pops up whenever the SensorBase::Configure(CC_RenderDataOn) function is called.

* Added a showsensors tutorial

Version 0.2.19
==============

Subversion Revision: r2031

Initial Release: 2011/02/17

Core
----

* Now OpenRAVE explicitly controls what symbols are exported and imported into the dynamic table. This means much faster loading times of openrave and its plugins!

http://gcc.gnu.org/wiki/Visibility

* OpenRAVE exceptions are now caught across shared object boundaries.

* Added OPENRAVE_DLL and OPENRAVE_DLL_EXPORTS to control import vs export of symbols. This changed the FindOpenRAVE.cmake file changed.

* Added a "Release" cmake build type that disables all stl/boost asserts and security checks. This will produce the most optimized code possible, but should be used only for well-tested production code. (default build is still RelWithDebInfo).

* Removed "vanchor" parameter from KinBody::Joint since it could be autogenerated.

IKFast
------

* ikfast now supports solving IK for robots that do not have intersecting axes. This includes work from Raghavan, Roth, Osvatic, Kohli, Manocha, and Canny.

* Generation process itself became about 3-5x faster. For example, the puma ik can be generated in 9 seconds and wam ik in 27 seconds. Fixes freezes users have experienced before.

* Now uses infinite precision fractions for all its computations, therefore there is no more rounding and hunting for zeros.

* PR2 IK improved a lot after replacing conic section intersection with 4th degree polynomial root finding.

* solving pairwise variables is now handles much more cases.

* 5DOF IK works and can detect special geometry like intersecting axes (katana arm)

* added sanity checks to high degree polynomials to remove solutions due to precision error

Misc
----

* Added a orpythonbinding example showing how users can register their python classes/functions with the OpenRAVE C++ framework.

Version 0.2.18
==============

Subversion Revision: r1975

Initial Release: 2011/01/18

Core
----

* moved the check_libm_accuracy script in libopenrave folder

* Moved all configuration files to the build (BINARY) folder rather than have it in source. The build process for configuration files changed a little to accommodate simultaneous builds with different options better. This allows us tohave double/float precision + debug/release all at the same time without forcing a rebuild. In order to avoid any collision troubles, the following files were renamed::

  classhashes.h -> interfacehashes.h
  defines.h -> config.h

* updated zlib 1.2.5 and minizip

* Added more joint types involving all permutations of revolution and prismatic joints! For example Revolute, Revolute, Revolute or Revolute,Prismatic. or Prismatic,Prismatic,Revolute. In order to support joints with multiple axes better, many of the fields were changed from single values to vectors of values. Most of the Joint::Get* methods now take an axis index.

* Organized the joint hierarchy and added a Joint::_ComputeInternalInformation to do some of the preprocessing that was previously done in the individual readers.

* Added normalizeAxisRotation - Find the rotation theta around axis such that rot(axis,theta) * q is closest to the identity rotation. This is used in extracting joint angles and converting rotation to euler angles.

COLLADA
-------

* can now read and write compressed collada files (zae)

* fixed many bugs in colladareader units

* all collada robots are offered as zae. Many previous .robot.xml robots were removed to prefer the COLLADA counterparts. The models.tgz file size reduced greatly.

* There is now a folder dedicated to all possible COLLADA robots that is seaprate from the openrave trunk:

https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots

* For ROS users: There is now a collada_robots ROS package that will check out all these robots. You would need to add the following bashrc line to get them into your openrave path::

  export OPENRAVE_DATA=$OPENRAVE_DATA:`rospack find collada_robots`/data

* can open collada visual scenes without any kinematics scene definitions (collada 1.4).

* can write physics data (masses + inertias) and collision adjacency data

Version 0.2.17
==============

Subversion Revision: r1955

Initial Release: 2011/01/09

COLLADA Robot Specification
---------------------------

Released initial specification for robotics info in COLLADA.

http://openrave.programmingvision.com/index.php/Started:COLLADA

Core
----

Mimic Joints Support Arbitrary Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is now possible to define the value of a joint as a complex formula involving any number of joints from the robot. For example:

j2 = arctan(0.5*cos(j0)) - j1

Here's the C++ API modifications:

http://openrave.programmingvision.com/ordocs/en/html/classOpenRAVE_1_1KinBody_1_1Joint.html#a0e31c5be31c4145afa786e0c0d6a46ae

OpenRAVE XML tutorial:
http://openrave.programmingvision.com/index.php/Started:Formats#Closed-chains_and_Mimic_Joints

And of course COLLADA spec modifications necessary:
http://openrave.programmingvision.com/index.php/Started:COLLADA#formula.2Ftechnique

Complex kinematics support
~~~~~~~~~~~~~~~~~~~~~~~~~~

Kinematics hierarchy now supports closed-chains correctly. It uses graph theory to find places to find the loops and how to compute link transformations with the least dependencies. This information is pre-computed in KinBody::_ComputeInternalInformation() making calls to SetDOFValues/SetDOFVelocities much faster. Some of the added functions:

KinBody::GetClosedLoops - returns all the unique closed loops of the robot.
KinBody::GetChain - returns a chain of joints or a chain of links
KinBody::Link::GetParentLinks - returns all parent links
KinBody::Link::IsParentLink
KinBody::Joint::GetHierarchyParentLink - joint values computed in this coordinate system
KinBody::Joint::GetHierarchyChildLink - joint moves this link
KinBody::GetDependencyOrderedJoints - will return the joints in the correct topological order.

Thanks to Nick Hillier for giving us the Bobcat S185 skid-steer loader model to test closed-chains with! This robot has 11 joints with 3 closed-loops and only 2 degrees of freedom, which makes it an interesting challenge.

http://www.bobcat.com/publicadmin/viewArticle.html?id=3910

Started development on a new tool called 'fkfast'. It solves the analytic equations for closed loops. It turns out that the Bobcat fk requires a quadratic equation to be solved with coefficients involving powers up to 8. Combined with the new mimic joint features, openrave can solve and simulate the mechanism correctly! If anyone is interested in checking it out, here's the corresponding file (from ticket #94):

fkfast is still experimental, so is not as usable as ikfast. For anyone curious, the file can be found in

test/fkfast.py

Accurate/Robust Math
~~~~~~~~~~~~~~~~~~~~

Added high precision math functions using crlibm. On compilation time, OpenRAVE checks with functions from libm are inaccurate and replaces them.


Planning
--------

Introduced a new planner called "WorkspaceTrajectoryTracker" that can take arbitrary trajectories of the end effector and quickly produce smooth configuration space trajectories that can follow the workspace path. The planner can also follow constraints as specified in the PlannerParameters::_constrainfn. The "MoveHandStraight" function now defaults to this planner. There's an example that shows off this functionality here::

  openrave.py --example movehandstraight

Models
------

Added the DARPA RE2 model and updated Care-O-Bot3 and arm models.

ROS
---

Added 3 useful scripts connecting openrave planning to the ROS world. They are all in orrosplanning package:

* ik_openrave.py - uses the openrave ik offering the orrosplanning/IK service

* armplanning_openrave.py - offers arm planning using the orrosplanning/MoveToHandPosition service

* graspplanning_openrave.py - offers finding grasp sets for new objects using with object_manipulation_msgs/GraspPlanning service

Misc
----

* Added convex hull computation command inside grasper plugin so that openrave can convert point clouds into meshes for grasping.

* Added several new python examples: :mod:`.examples.checkconvexdecomposition`, :mod:`.examples.checkvisibility`, :mod:`.examples.fastgrasping`,

Version 0.2.15
==============

Subversion Revision: r1846

Initial Release: 2010/11/20

Core
----

* All user data is now derived from UserData class, this allows RTTI to be used making type casts safe.

* Added a global openrave state that manages environments, environments now have unique ids.

* Added OPENRAVE_DATABASE environment variable to allow users to specify multiple database directories different from ~/.openrave.

* Safer destruction of all openrave resources using RaveDestroy, no more segfauls on exit.

Velocities and Physics API
--------------------------

* Cleaned up velocity functions in the physics engine (interface is simpler). KinBody class now converts joint velocities to link velocities (and vice versa) internally. All joint velocity functions have been removed from the physics engine interface, ie only link velocity functions are offered. Link velocities always point to the link's coordinate system origin (not the center of mass).

* Setting velocity behaves similar to setting dof values. the default physics engine now stores velocities (it did not before).

* Some discussion for these changes can be found in `this ticket <http://sourceforge.net/apps/trac/openrave/ticket/69>`_.

Controller API
--------------

* Controller interface cleaning up, setting a controller on a robot now requires the degrees of freedom that the controller uses to be specified. The controller dof features allows multiple controllers to use the same robot without interfering with each other.

* Added a MultiController class to simplify setting multiple controllers per robot. A C++ example is shown in the ormulticontrol C++ demo:

http://openrave.programmingvision.com/ordocs/en/html/ormulticontrol_8cpp-example.html

ikfast
------

* ikfast can solve more complex kinematics requiring conic section intersections or 6th degree polynomial solutions. solving equations is now faster and more accurate (internal precision is to 30 decimal digits).

* ikfast supports a new lookat type.

* PR2 IK is pre-generated.

Sensors
-------

* Added many new sensor types (Odometry/Tactle) and exporting them through the python interface.

* One sensor can support multiple sensor data (ie Odometry+Tactile+IMU).

Other
-----

* Viewer graph handles allow changing transformation and showing/hiding.

* Major upgrades on collada reader extending robot-specific information (more on this in a future email once writer is done).

Version 0.2.13
==============

Subversion Revision: r1756

Initial Release: 2010/10/04

Core
----

Separates the global OpenRAVE state from the environment state. The main reason for this move was for better management of multiple environments and for a new upcoming ROSEnvironment class that will integrate better with the ROS package file system.

More specifically, the new global state

* manages plugins/interfaces

* allows users to better manage multiple environments

* manages debug levels

* fixes many race conditions by organizing destruction order of all global resources.

* allows destruction of entire OpenRAVE state and all resources using a single call: RaveDestroy. These changes fix all thrown exceptions when a program exits.

* OpenRAVE is initialized by first calling RaveInitialize, independent of the environment.

All the global functions are prefixed with Rave*.

Version 0.2.12
==============

Subversion Revision: r1736

Initial Release: 2010/09/16

Core
----

* Destruction order has been cleaned up. Before, openrave would freeze up when locking the environment in a Destroy method, now it doesn't.

* RemoveKinBody/RemoveProblem/RemoveSensor are all handled now by one :meth:`.Environment.Remove`

Sensors
-------

* A sensor can be added into the environment without a robot using :meth:`.Enviornment.AddSensor`

* All the sensors in the environment can be queried using Environment.GetSensors, this returns all sensors attached to all the robots and all the environment sensors. Individual sensors can be queried by name using :meth:`.Environment.GetSensor`

* Can now store sensor parameters in side *.sensor.xml files and include them from a parent xml file using the file="..." attribute. This applies to all interface types, not just sensors. `Here's a tutorial <http://openrave.programmingvision.com/wiki/index.php/Format:XML#Sensor>`_.

* Added IMU sensor definitions

* Cloning treats sensors separately now. In order to clone sensors (robot+environment), the Clone_Sensors option has to be specified. The definitions of the robot attached sensors are still cloned, but not the internal interfaces.

Version 0.2.11
==============

Subversion Revision: r1689

Initial Release: 2010/07/30

Core
----

Every interface now has a :meth:`.InterfaceBase.SendCommand` function

Robot
-----

* iksolver methods in manipulator class were cleaned up. It is now possible to get the pointer using ManipulatorBase::GetIkSolver. and then do ManipulatorBase::GetIkSolver()->SendCommand(...).

Version 0.2.9
=============

Subversion Revision: r1648

Initial Release: 2010/07/23

Core
----

* Cleans up a lot of the interfaces and puts in a consistent documentation system for plugin authors.

* There is now a rave/plugin.h file that helps plugin authors export interfaces much simpler.

* Plugin loading at start-up is now 2x+ faster. Users do not have to worry about having too many plugins in openrave.

* All interfaces (not just problems) now have a RegisterCommand function.

Planning
--------

* A lot of bug fixes on camera visibility planning (VisualFeedback problem)

Kinematics
----------

* Moved methods like GetJointXXX to GetDOFXXX. With some joints having multiple degrees of freedom, the joint indices are not necessarily equal to the DOF indices.

ikfast
------

* IKFast has been greatly improved, the ray inverse kinematics is also working nicely

* It is now possible to use the inversekinematics.py database generator through the ikfast problem instance using :ref:`module-ikfast-loadikfastsolver` command.
