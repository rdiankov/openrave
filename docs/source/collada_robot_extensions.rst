.. _collada_robot_extensions:

COLLADA Robot Extensions (Version 0.3.6)
----------------------------------------

OpenRAVE maintains a set of robot-specific extensions to the `COLLADA 1.5 specification <http://www.khronos.org/collada/>`_ in order to exchange data with robotics applications. By default, COLLADA 1.5 handles geometry, visual effects, physical properties, and complex kinematics while the robot-specific extensions include:

* manipulators
* sensors
* collision data - geometries for environment, self-collision, visual info, etc
* planning-specific parameters

COLLADA allows extensions of any of its tags using the **<extra>** tag. Each **<extra>** defines what type of information to provide and a format for that information, also called **technique**. All custom data defined here uses the **OpenRAVE** technique. 

There are one-to-one correspondences between the OpenRAVE interface types and COLLADA tags:

* Robot/KinBody <-> articulated_system
* Sensor <-> sensor
* Manipulator <-> manipulator

interface_type
==============

Introduction
~~~~~~~~~~~~

Specifies the type of kinematics body/robot type to instantiate inside the code.

Concepts
~~~~~~~~

All of the kinematics body methods can be overridden with new implementations. Because this requires loading user code, a user-provided instantiation has to be used. The interface type specifies what this type is and where to load it from.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 80
  
  Parent elements | <articulated_system>, <kinematics_model>
  Child elements | See the following subsection.
  Other | 

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances
  
  <interface> | Contains the string id of the interface | 1
  <plugin> | Optional. Contains the string of the location of the shared object object to load. Because plugin prefixes and suffixes depends on the OS, a prefix and suffix independent name can be specified. | 0 or 1

Attributes for <interface>
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  type | **xs:token** | Required. The type of interface to create: robot, kinbody, sensor, iksolver, etc

Example
~~~~~~~

.. code-block:: xml

  <extra type="interface_type">
    <technique profile="OpenRAVE">
      <interface type="robot">MyGenericRobot</interface>
      <plugin>myplugin</plugin>
    </technique>
  </extra>

manipulator
===========

Introduction
~~~~~~~~~~~~

Defines a subset of the robot that acts as an **arm** and a **gripper**.

Concepts
~~~~~~~~

The arm is a chain of joints whose **end effector** is treated as a gripper. The arm is extracted from the **origin** and **tip** links. The **tip** contains the manipulator frame of reference. The gripper axes have to be specified manually. The direction is used as a hint for grasping and inverse kinematics.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 80
  
  Parent elements | <articulated_system>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances
  
  <frame_origin> | The base frame that the arm starts at | 1
  <frame_tip> | The end effector frame the arm ends at | 1
  <gripper_joint> | Defines one joint of the gripper | 0 or more
  <iksolver> | Defines properties of inverse kinematics functions when used with the arm | 0 or more

Attributes for <frame_origin>/<frame_tip>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  link | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>.

Child Elements for <frame_tip>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :header: Element, Description, Occurances
  
  <translate> | Translation. See main entry in Core. | 0 or more
  <rotate> | Rotation axis. See main entry in Core. | 0 or more
  <direction> | Direction meta information. Sometimes IK and other modules require the manipulator to have a direction to measure angles from. This is defined inside the frame tip coordinate system. | 0 or 1

Attributes for <gripper_joint>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  joint | **xs:token** | Required. The reference of the joint in the instantiated kinematics model that is part of the gripper.

Child Elements for <gripper_joint>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances
  
  <chucking_direction> | **common_float_or_param_type** that contains the default chucking direction of an axis on the joint. If a chucking direction is not specified for an axis in the joint, it defaults to 0. | 0 or more

Attributes for <gripper_joint>/<chucking_direction>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  axis | **xs:token** | Required. The SID of the axis inside the referenced joint.

Attributes for <iksolver>
~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  type | **xs:token** | Required. The type of the inverse kinematics to set a property for. Possible types are: **Transform6D, Rotation3D, Translation3D, Direction3D, Ray4D, Lookat3D, TranslationDirection5D**.

Child Elements for <iksolver>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <free_joint> | Specifies one free joint to use for ik. | 0 or more
  <interface_type> | Optional. Specifies the interface of the inverse kinematics solver. | 0 or 1

Attributes for <iksolver>/<free_joint>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  joint | **xs:token** | Required. The reference of the joint in the instantiated kinematics model that is part of the manipulator chain.
  stepsize | **xs:float** | The discretization value of this joint when searching for solutions

Details
~~~~~~~

The current IK types are:

* Transform6D - end effector reaches desired 6D transformation
* Rotation3D - end effector reaches desired 3D rotation
* Translation3D - end effector origin reaches desired 3D translation
* Direction3D - direction on end effector coordinate system reaches desired direction
* Ray4D - ray on end effector coordinate system reaches desired global ray
* Lookat3D - direction on end effector coordinate system points to desired 3D position
* TranslationDirection5D - end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.

The IK types are meant to be hints as to how a manipulator can be used. Multiple IK types can be set for one manipulator and differing free joint values. It is possible for a post-processing stage to determine what IK types are best suited for a particular manipulator structure, and then add those into the COLLADA file.

* Why is a manipulator frame necessary?

  * Answer: Manipulator frames allow the user to define a coordinate system where it makes target tasks easier to complete. In this regard, the manipulator frame can be freely chosen by the user without worrying about destroying the link coordinate systems. For example, link frames are usually aligned with joint axes and center of masses and robot state is defined by their 6D transform in space. Having them also represent task-specific information could destroy consistency when the task changes. Also, the z-axis of the manipulator frame can define the "direction" of the manipulator. Direction can be used in many places like sensor line of sight and grasping approach, which makes it possible to quickly use the robot for planning.

* Question: For dual arm manipulation, would a leftright manipulator ever be used including all joints? In this case, will it might be necessary to define two frame tips (one for left arm and one for right arm)?

  * Answer: Having a leftright manipulator destroys the one-to-one correspondence between gripper joints and ik solver, and not much is gained. So better to have only have one frame tip and origin and treat two arms as separate. The constraint between the end effectors of the two arms is not always rigid, it very task dependent. Therefore, the user should take care of the dual relation.

* Question: What about chucking gripper direction for complex hands? Fingers with many DOF might need special grasping strategies.

  * Answer: The chucking direction just provide a hint as to the usage. The real gripper movement depends on the grasp strategy, which is beyond the definition of this scope. 

Example
~~~~~~~

The example defines an arm with an end effector at link wam7 with a local coordinate system. It also defines two gripper axes. For the 'transform6d' inverse kinematics type, it specifies that the free joint should be 'joint4'.

.. code-block:: xml

  <articulated_system>
    <extra type="manipulator" name="leftarm">
      <technique profile="OpenRAVE">
        <frame_origin link="wam0"/>
        <frame_tip link="wam7">
          <translate>0.0 0.0 0.22</translate>
          <rotate>0.0 1.0 0.0 90.0</rotate>
          <direction>0.0 0.0 1.0</direction>
        </frame_tip>
        <gripper_joint joint="jointname">
          <chucking_direction axis="axis0">
            <float>1</float>
          </chucking_direction>
        </gripper_joint>
        <gripper_joint joint="jointname2">
          <chucking_direction axis="axis0">
            <float>-1</float>
          </chucking_direction>
        </gripper_joint>
        <iksolver type="Transform6D">
          <free_joint joint="jointname3"/>
          <interface_type>
            <interface>WAM7ikfast</interface>
            <plugin>WAM7ikfast</plugin>
          </interface_type>
        </iksolver>
        <iksolver type="Translation3D">
          <free_joint joint="jointname4"/>
        </iksolver>
      </technique>
    </extra>
  </articulated_system>

.. _collada_dynamic_rigid_constraints:

dynamic_rigid_constraints
=========================

Introduction
~~~~~~~~~~~~

Defines a list of rigid constraints within pairs of bodies modeling dynamic inter-relationships of physics models like a robot hand grabbing an object.

Concepts
~~~~~~~~

Allows new rigid constraints to be specified that are not associated with physics models, but instead of with the instantiated physics and visual scenes. Common rigid constraints for robotics are:

- a robot hand grasping an object with its hand.
- robot grasping a tool, which then grasps the object
- robot hand grasps a cylindrical pole, so it is free to rotate the hand with respect to the cylinder axis.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 80
  
  Parent elements | <instance_physics_scene>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances
  
  <rigid_constraint> | A list of rigid constraints | 0 or more
  <rigid_constraint>/<technique>/<ignore_link> | The sid of a link whose collision is ignored with the attached body. The link belongs to the ref_attachment physics model. | 0 or more

Details
~~~~~~~

Example
~~~~~~~

Describes robot whose physics model SID is **pmodel1_inst** grabbing with its **rigid_hand** link an object whose physics model SID is **pmodel2_inst**. **rigid_hand** is the SID of the **<instance_rigid_body>** inside **pmodel1_inst**.

.. code-block:: xml

  <instance_physics_scene url="#pscene">
    <extra type="dynamic_rigid_constraints">
      <technique profile="OpenRAVE">
        <rigid_constraint sid="grab0">
          <ref_attachment rigid_body="pmodel1_inst/rigid_hand"/>
          <attachment rigid_body="pmodel2_inst/rigid0"/>
          <technique profile="OpenRAVE">
            <ignore_link link="pmodel1_inst/rigid_dummy"/>
            <ignore_link link="pmodel1_inst/rigid2"/>
          </technique>
        </rigid_constraint>
      </technique>
    </extra>
  </instance_physics_scene>


.. _collada_collision:

collision
=========

Introduction
~~~~~~~~~~~~

Links all possible collision meshes and properties for one kinematics body. The meshes depends on the usage.

Concepts
~~~~~~~~

A link can have three different collision meshes:

* for visual rendering
* for self-collisions
* for environment collisions

For each link, COLLADA will store three geometries in the **<library_geometries>**. The geometries will have an <extra> tag that specifies which usage they are meant to. The **self** and **env** will be referenced inside the visual geometry.

The tag also stores information about what pairs of links can be completely ignored from self-collision detection. These links are either adjacent to each other, or so far from each other that no configuration of the robot can get them into possible collision.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <kinematics_model>, <articulated_system>, <instance_articulated_system>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <bind_instance_geometry> | The geometry used for a particular link | 0 or more
  <ignore_link_pair> | Specifies two links pairs whose self-collision should not be checked | 0 or more
  <link_collision_state> | Contains a **common_bool_or_param_type** that specifies if a link should be used for collision or not. Can enable or disable it. | 0 or more
  <link_visible_state> | Contains a **common_bool_or_param_type** that specifies if a link should be used for visible or not. Can enable or disable it. | 0 or more

Attributes for <bind_instance_geometry>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  type | **xs:token** | Required. The usage type: **environment** or **self**
  link | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>. This link is where the geometries will be added.
  url | **xs:anyURI** | Required. The URL of the location of the <geometry> element to instantiate. Can refer to a local instance or external reference.

Attributes for <ignore_link_pair>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  link0 | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>. One of the links defining the pair to be ignored.
  link1 | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>. One of the links defining the pair to be ignored.

Attributes for <link_collision_state>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  link | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>. One of the links defining the pair to be ignored.

Details
~~~~~~~

<ignore_link_pair> tags help self-collision detection to help prune possibilities. The adjacency information is not just the neighboring links. It is also meant to prune any collisions between two links that *cannot* possibly happen if the robot maintains its joint limits. This information depends not only on the kinematics of the robot, but also on the geometry of every link. Also for triplets of joints j1, j2, j3 that intersect at a common axis, you would want to add (j1,j2),(j2,j3),(j1,j3).

By default, all links are colliding unless turned off via **<link_collision_state>**.

If **<link_collision_state>** is defined at the instance_articulated_system level, then it will do a AND operation with the **<link_collision_state>** defined at the articulated_system level to figure out the true collision of the objects.
If **<link_visible_state>** is defined at the instance_articulated_system level, then it will do a AND operation with the **<link_visible_state>** defined at the articulated_system level to figure out the true visible of the objects.

Example
~~~~~~~

.. code-block:: xml

  <library_visual_scenes>
    <node id="mynode">
      <instance_geometry url="#linka_vis0"/>
      <instance_geometry url="#linka_vis1"/>
    </node>
  </library_visual_scenes>
  <library_geometries>
    <geometry id="linka_vis0"/>
    <geometry id="linka_vis1"/>
    <geometry id="linka_env0"/>
    <geometry id="linka_env1"/>
    <geometry id="linka_self"/>
    <geometry id="linkb_env0"/>
  </library_geometries>
  <library_kinematics_models>
    <kinematics_model>
      <extra type="collision">
        <technique profile="OpenRAVE">
          <bind_instance_geometry type="environment" link="linka" url="#linka_env0"/>
          <bind_instance_geometry type="environment" link="linka" url="#linka_env1"/>
          <bind_instance_geometry type="self" link="linka" url="#linka_self"/>
          <bind_instance_geometry type="environment" link="linkb" url="#linkb_env0"/>
          <ignore_link_pair link0="linka" link1="linkb"/>
          <link_collision_state link="linka"><bool>true</bool></link_collision_state>
        </technique>
      </extra>
  </library_kinematics_models> 

Attributes for <link_visible_state>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  link | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>. One of the links defining the pair to be ignored.

Details
~~~~~~~

By default, all links are visible unless turned off via **<link_visible_state>** or the geometry definition **<visible>** says so.


.. _collada_geometry_info:

geometry_info
=============

Introduction
~~~~~~~~~~~~

Uses the COLLADA Physics specification of Analytical Shapes to summarize geometric information in simpler terms like box, cylinder, sphere, etc.

Concepts
~~~~~~~~

A simple geometric element like an oriented box can be very difficult to exactly define with
triangle meshes or brep presentations. Furthermore, elements like spheres must use brep, which
require a deep understanding of the new brep specification. By using the **<geometry_info>**
element, a user can give a hint to the user as to what shape the geometry mesh represents using the
physics specifications like **<box>**.

In order to represent an oriented bounding box, a coordinate system can be defined within
**<geometry_info>** by using the **<translate>** and **<rotate>** tags.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <geometry>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <visible> | Contains a **common_bool_or_param_type** that specifies whether the geometry is visible or not. | 0 or 1
  <translate> | Translate the simple geometry shape. See main entry in Core. | 0 or more
  <rotate> | Rotation axis for rotating the simple geometry. See main entry in Core. | 0 or more
  *geometry of the shape* | An inline definition using one of the following COLLADA Physics analytical shape elements: **<plane>**, **<box>**, **<sphere>**, **<cylinder>**, or **<capsule>** | 0 or 1

Attributes for <parameters>
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Details
~~~~~~~

It is possible to only define the local coordinate system of the geometry without defining an extra analytical shape.

Example
~~~~~~~

Translation box center to (0,0,0.5) and rotate 45 degrees around z axis.

.. code-block:: xml

  <geometry>
    <extra type="geometry_info">
      <technique profile="OpenRAVE">
        <box>
          <half_extents>0.1 0.2 0.3</half_extents>
        </box>
        <translate>0 0 0.5</translate>
        <rotate>0 0 1 45</rotate>
        <visible><bool>true</bool></visible>
      </technique>
    </extra>
  </geometry>

link_info
=========

Introduction
~~~~~~~~~~~~

Defines extra link parameters not part of the COLLADA 1.5 specification.

Concepts
~~~~~~~~

The parameters are key/value-array pairs.

Attributes
~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  name | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>. One of the links defining the pair to be ignored.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 80
  
  Parent elements | <kinematics_model>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances
  
  <float_array> | An array of floating-point values | 0 or more
  <int_array> | An array of floating-point values | 0 or more

Example
~~~~~~~

Sets **myparam** to **link0** and **newparam** to **link1**.

.. code-block:: xml

  <kinematics_model>
    <extra type="link_info" name="link0">
      <technique profile="OpenRAVE">
        <float_array link="link0" name="myparam">1.0 2.0 3.0</float_array>
      </technique>
    </extra>
    <extra type="link_info" name="link1">
      <technique profile="OpenRAVE">
        <int_array link="link1" name="newparam">-10</int_array>
      </technique>
    </extra>
  </kinematics_model>

joint_info
==========

Introduction
~~~~~~~~~~~~

Defines extra joint parameters not part of the COLLADA 1.5 specification.

Concepts
~~~~~~~~

The parameters are key/value-array pairs.

Attributes
~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  name | **xs:token** | Required. References the SID of a <joint> defined in <kinematics_model>. One of the joints defining the pair to be ignored.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 80
  
  Parent elements | <kinematics_model>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances
  
  <float_array> | An array of floating-point values | 0 or more
  <int_array> | An array of floating-point values | 0 or more

Example
~~~~~~~

Sets **myparam** to **joint0** and **newparam** to **joint1**.

.. code-block:: xml

  <kinematics_model>
    <extra type="joint_info" name="joint0">
      <technique profile="OpenRAVE">
        <float_array joint="joint0" name="myparam">1.0 2.0 3.0</float_array>
      </technique>
    </extra>
    <extra type="joint_info" name="joint0">
      <technique profile="OpenRAVE">
        <int_array joint="joint1" name="newparam">-10</int_array>
      </technique>
    </extra>
  </kinematics_model>

library_sensors
===============

Introduction
~~~~~~~~~~~~

Provides a library in which to place <sensor> elements.

Concepts
~~~~~~~~

Allows sensors to be stored as modular resources in libraries. Can be easily referenced through files.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <COLLADA>

sensor
======

Introduction
~~~~~~~~~~~~

Defines a sensor's type and the geometric and intrinsic parameters.

Concepts
~~~~~~~~

Each sensor will be associated with a particular sensor type; depending on the sensor type, the parameters that need to be set will change. The parameters should contain everything necessary to simulate the sensor accurately. They *should not* contain parameters that define the format and transfer of the data.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |

  Parent elements | <articulated_system>

Attributes
~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  type | **xs:token** | Required. The type of the sensor. Possible types are: **base_pinhole_camera, base_stereo_camera, base_laser2d, base_laser3d, base_flash_laser, base_encoder, base_force6d, base_imu, base_odometry**
  id | **xs:ID** | Required. A text string containing the unique identifier of the <sensor> element. This value must be unique within the instance document.

Child Elements
~~~~~~~~~~~~~~

**common**:

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <interface_type> | Optional. Contains the interface type to load the sensor with. | 0 or 1
  <hardware_id> | Optional. The hardware ID . | 0 or 1

**type base_pinhole_camera**:

Simple pin hole camera defined by an intrinsic matrix. The camera can support multiple image dimensions with multiple channel formats. It is not clear whether all supported formats for one camera should be enumerated in one <sensor> tag, or there should be multiple sensor tags for each different type where the sensors are exclusively mutual.

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  
  <image_dimensions> | Contains a **int3_type** that specifies the image width, height, and channels. | 1
  <format> | Contains a string that specifies the format of every value in the image. Possible types are **uint8, uint16, uint32, int8, int16, int32, float32, float64**. | 1
  <measurement_time> | Contains a **float_type** that specifies how long an image takes to grab (ie exposure time). | 0 or 1
  <gain> | Contains a **float_type** that specifies the gain of the camera. | 0 or 1
  <intrinsic> | Contains a **float2x3_type** that specifies the intrinsic parameters defining the principal point, field of view, and skew. | 1
  <focal_length> | Contains a **float_type** that specifies the physical focal length of the camera. | 0 or 1
  <distortion_model> | The distortion model to use. It has a **type** attribute specifying the actual model type, and contains a **list_of_floats_type** that specifies the distortion coefficients of the model. | 0 or 1
  <sensor_reference> | References another sensor's data when computing this sensor's data. The reference sensor is specified by a **url** attribute of type **xs:anyURI**. | 0 or more
  <target_region> | References an articulated system that describes the target region of interest for the sensor. The target region is specified by a **url** attribute of type **xs:anyURI**. | 0 or more

**type base_stereo_camera:**

Uses two cameras together to extract a depth map. The stereo camera's coordinate system is in the first instanced camera.

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  
  <instance_sensor> | The camera sensors, the scan time should be equal | 2

Attributes for <instance_sensor>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  url | **xs:anyURI** | Required. The URL of the location of the <sensor> element to instantiate.

Child Elements for <instance_sensor>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <rectification> | Contains a **float3x3_type** that specifies a homography which takes an image to the ideal stereo image plane so that epipolar lines in both stereo images are parallel. The homography transforms from the second image to the first image. | 1

**type base_laser2d**:

Single scan from a planar laser range-finder along the xy plane.

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  
  <angle_range> | Contains a **float2_type** that specifies the minimum and maximum angles (degrees) of the laser range. | 1
  <distance_range> | Contains a **float2_type** that specifies the minimum and maximum distance of the laser. | 1
  <angle_increment> | Contains a **float_type** that specifies the angular distance between measurements (degrees). | 1
  <time_increment> | Contains a **float_type** that specifies the time between measurements (seconds). If your scanner is moving, this will be used in interpolating position of 3d points. | 1
  <measurement_time> | Contains a **float_type** that specifies the time between scans (seconds) | 1

**type base_laser3d:**

**type base_flash_laser:**

**type base_encoder:**

**type base_force6d:**

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  
  <load_range_force> | Contains a **float3_type** that specifies the maximum force around the XYZ axes the sensor can accurately measure before saturating. Units are **Mass * Distance² * Time-²**. | 0 or 1
  <load_range_torque> | Contains a **float3_type** that specifies the maximum torque around the XYZ axes the sensor can accurately measure before saturating. Units are **Mass * Distance * Time-²**. | 0 or 1
  <load_resolution_force> | Contains a **float3_type** that specifies the sensing resolution of the forces being measured around the XYZ axes. Units are **Mass * Distance² * Time-²**. | 0 or 1
  <load_resolution_torque> | Contains a **float3_type** that specifies the sensing resolution of the torques being measured around the XYZ axes. Units are **Mass * Distance² * Time-²**. | 0 or 1
  <load_capacity_range_force> | Contains a **float3_type** that specifies the maximum force around the XYZ axes the sensor can withstand before breaking. Units are **Mass * Distance² * Time-²**. | 0 or 1
  <load_capacity_range_torque> | Contains a **float3_type** that specifies the maximum torque around the XYZ axes the sensor can withstand before breaking. Units are **Mass * Distance² * Time-²**. | 0 or 1

**type base_imu:**

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  
  <measurement_time> | Contains a **float_type** that specifies the time between scans (seconds). | 1
  <rotation_covariance> | The uncertainty covariance matrix (3x3 row-major matrix) in x, y, and z axes. | 1
  <angular_velocity_covariance> | The uncertainty covariance matrix (3x3 row-major matrix) in x, y, and z axes. | 1
  <linear_acceleration_covariance> | The uncertainty covariance matrix (3x3 row-major matrix) in x, y, and z axes. | 1

**type base_odometry:**

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  
  <measurement_time> | Contains a **float_type** that specifies the time between scans (seconds). | 1
  <target> | The name of the target whose odometry is being measured | 0 or 1

Example
~~~~~~~

Example using a default sensor with a custom interface

.. code-block:: xml

  <extra type="library_sensors" id="libsensors">
    <technique profile="OpenRAVE">
      <sensor type="base_laser2d" id="ExampleLaser1">
        <angle_min>-90</angle_min>
        <angle_max>90</angle_max>
        <range_min>0.01</range_min>
        <range_max>4.0</range_max>
        <angle_increment>1</angle_increment>
        <time_increment>0.0005</time_increment>
        <measurement_time>0.025</measurement_time>
        <interface_type>
          <technique profile="OpenRAVE">
            <interface>BaseLaser2D</interface>
          </technique>
        </interface_type>
      </sensor>
    </technique>
  </extra>

Using a non-default, custom sensor

.. code-block:: xml

  <extra type="library_sensors" id="libsensors">
    <technique profile="OpenRAVE">
      <sensor type="BaseLaser2D" id="ExampleLaser1">
        <minangle>-135</minangle>
        <maxangle>135</maxangle>
        <resolution>0.35</resolution>
        <maxrange>5</maxrange>
        <scantime>0.1</scantime>
        <color>0.5 0.5 1</color>
      </sensor>
    </technique>
  </extra>

Develop a formal sensor XML file format for different sensor types.

attach_sensor
=============

Introduction
~~~~~~~~~~~~

Attaches a sensor to a link of the robot.

Concepts
~~~~~~~~

The sensor comes from the sensor library. It can be attached anywhere onto a link defined from the kinematics section. The sensor will maintain a constant transformation between the link.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <articulated_system>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <instance_sensor> | Instantiate a sensor. | 1
  <frame_origin> | The base link that the sensor is attached to. | 1

Attributes for <frame_origin>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  link | **xs:token** | Required. References the SID of a <link> defined in <kinematics_model>.

Child Elements for <frame_origin>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <translate> | Translation. See main entry in Core. | 0 or more
  <rotate> | Rotation axis. See main entry in Core. | 0 or more

Example
~~~~~~~

.. code-block:: xml

  <extra type="attach_sensor" name="left_head_camera">
    <technique profile="OpenRAVE">
      <instance_sensor url="#pgr_camera"/>
      <frame_origin link="head">
        <translate>0 1 0</translate>
        <rotate>0 1 0 90</rotate>
      </frame_origin>
    </technique>
  </extra>

formula/technique
=================

Introduction
~~~~~~~~~~~~

Full specifies a formula for a joint and annotates it with extra information necessary for robotics.

Concepts
~~~~~~~~

The original <formula>/<technique_common> supports only one equation for the value of the joint. More complex kinematics systems have more than one degree of freedom per joint and use the partial derivatives of the equation to compute Jacobians and simulate physics. 

This "OpenRAVE" technique for <formula> can specify partial derivatives of the position 
equation for computing velocity and accelerations.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <formula>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <equation> | Equation in MathML format. Used to specify the position and partial derivatives. | 0 or more

Attributes for <equation>
~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  type | **xs:token** | Required. can be one of "position", "first_partial", or "second_partial".
  target | **xs:token** | If 'type' is "first_partial" or "second_partial", then fill this with the variable taking the partial derivative with respect to. 

Example
~~~~~~~

.. code-block:: xml

  <technique profile="OpenRAVE">
    <equation type="position">
      <math>
        <apply>
          <plus/>
          <apply>
            <times/>
            <cn>0.333330</cn>
            <csymbol encoding="COLLADA">kmodel1/joint0</csymbol>
          </apply>
          <cn>0.872700</cn>
        </apply>
      </math>
    </equation>
    <equation type="first_partial" target="kmodel1/joint0">
      <math>
        <cn>0.333330</cn>
      </math>
    </equation>
  </technique>

library_actuators
=================

Introduction
~~~~~~~~~~~~

Provides a library in which to place <actuator> elements.

Concepts
~~~~~~~~

Allows actuators to be stored as modular resources in libraries. Can be easily referenced through files.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <COLLADA>

actuator
========

Introduction
~~~~~~~~~~~~

An actuator provides force/momentum/action to kinematics joints.

Concepts
~~~~~~~~

Defines a actuator's physical properties necessary to simulate dynamics and control algorithms of a robot. They **should not** contain parameters that define the format and transfer of the data to and from actuators. 

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <articulated_system>

Attributes
~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  type | **xs:token** | Required. The type of the actuator. Possible types are: **electric_motor**
  id | **xs:ID** | Required. A text string containing the unique identifier of the <actuator> element. This value must be unique within the instance document.

Child Elements
~~~~~~~~~~~~~~

**common**:

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <interface_type> | Optional. Contains the interface type to load the actuator with. | 0 or 1

**type electric_motor**:

Converts electrical energy into mechanical energy usually using magnetic fields and conductors. The **speed** of a motor is measured in revolutions/Time (Time is defined by the <asset> tag and usually measured in seconds). DC Motor Theory References:

* http://hades.mech.northwestern.edu/index.php/Brushed_DC_Motor_Theory

* http://en.wikipedia.org/wiki/Brushed_DC_electric_motor


.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10

  <model_type> | Contains a string that specifies the the type of actuator it is. Usually the motor model name is ok, but can include other info like gear box, etc. | 0 or 1
  <gear_ratio> | Contains a **float_type** that specifies the ratio between the input speed of the transmission (the speed of the motor shaft) and the output speed of the transmission. | 1 
  <assigned_power_rating> | Contains a **float_type** that specifies the nominal power the electric motor can safely produce. Units are **Mass * Distance² * Time-³**. | 1
  <max_speed> | Contains a **float_type** that specifies the maximum speed of the motor. Units are **Time-¹**. | 1
  <no_load_speed> | Contains a **float_type** that specifies the speed of the motor powered by the nominal voltage when the motor provides zero torque. Units are **Time-¹**. | 0 or 1
  <stall_torque> | Contains a **float type** that specifies the maximum torque achievable by the motor at the nominal voltage. This torque is achieved at zero velocity (stall). Units are **Mass * Distance * Time-²**. | 1
  <nominal_speed_torque_point> | Contains a **float2_array** that specifies the speed and torque achievable when the motor is powered by the nominal voltage. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the stall torque directly (ideal). | 0 or more
  <max_speed_torque_point> | Contains a **float2_array** that specifies the speed and torque achievable when the motor is powered by the max voltage/current. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the max instantaneous torque directly (ideal). | 0 or more
  <rotor_inertia> | Contains a **float_type** that specifies the inertia of the rotating element about the axis of rotation. Units are **Mass * Distance²**. | 1
  <torque_constant> | Contains a **float_type** that specifies the proportion relating current to torque. Units are **Mass * Distance * Time-¹ * Charge-¹**. | 1
  <nominal_torque> | Contains a **float_type** that specifies the maximum torque the motor can provide continuously without overheating. Units are **Mass * Distance * Time-²**. | 1
  <nominal_voltage> | Contains a **float_type** that specifies the nominal voltage the electric motor can safely produce. Units are **Mass * Distance² * Time-² * Charge**. | 1
  <speed_constant> | Contains a **float_type** that specifies the constant of proportionality relating speed to voltage. Units are **Mass-¹ * Distance-² * Time * Charge-¹**. | 1
  <starting_current> | Contains a **float_type** that specifies the current through the motor at zero velocity, equal to the nominal voltage divided by the terminal resistance. Also called the stall current.  Units are **Time-¹ * Charge**. | 1
  <terminal_resistance> | Contains a **float_type** that specifies the resistance of the motor windings. Units are **Mass * Distance² * Time-¹ * Charge-²**. | 1
  <max_instantaneous_torque> | Contains a **float_type** that specifies the maximum instantenous torque achievable by the motor when voltage <= nominal voltage. Motor going between nominal_torque and max_instantaneous_torque can overheat, so should not be driven at it for a long time. Units are **Mass * Distance * Time-²**. | 1
  <coloumb_friction> | Contains a **float_type** that specifies the static coloumb friction on each joint after the gear box. Defaults to 0 if not specified. Units are **Mass * Distance * Time-²**. | 0 or 1
  <viscous_friction> | Contains a **float_type** that specifies the viscous friction on each joint after the gear box. Defaults to 0 if not specified. Units are **Mass * Distance * Time-²**. | 0 or 1

Related variables, but not inserted in the electric_motor specification:

* Mechanical time constant - The time it takes the unloaded motor to reach 63% of its no load speed under a constant voltage, starting from rest. Proportional to the inertia of the rotor and inversely proportional to the square of the the torque constant. 
* Max. efficiency - The maximum efficiency of the motor in converting electrical power to mechanical power. This maximum efficiency typically occurs at high speed and low torque; the efficiency is zero at zero speed and zero torque, since the mechanical power is τω. 
* No load current - The current required to spin the motor at the no load condition (i.e., the current needed to provide the torque necessary to overcome friction).
* Nominal current (max. continuous current) - The current that yields the maximum continuous torque. This maximum is determined by thermal characteristics of the motor. The power dissipated by the motor as heat is i2R. Larger currents are acceptable intermittently, but large continuous currents may cause the motor to overheat. 
* Mechanical time constant - The time it takes the unloaded motor to reach 63% of its no load speed under a constant voltage, starting from rest. Proportional to the inertia of the rotor and inversely proportional to the square of the the torque constant.
* Terminal inductance - The inductance of the motor windings. 
* Thermal resistance housing-ambient 
* Thermal resistance winding-housing
* Thermal time constant winding.

Example
~~~~~~~

.. code-block:: xml

  <extra type="library_actuators" id="libactuators">
    <technique profile="OpenRAVE">
      <actuator type="electric_motor" id="ExampleMotor1">
        <assigned_power_rating>1.0</assigned_power_rating>
        <max_speed>3000</max_speed>
        <no_load_speed>3990</no_load_speed>
        <stall_torque>0.012</stall_torque>
        <nominal_torque>0.012</nominal_torque>
        <nominal_voltage>24.0</nominal_voltage>
        <rotor_inertia>0.0000023</rotor_inertia>
        <speed_constant>173.0</speed_constant>
        <starting_current>0.578</starting_current>
        <terminal_resistance>41.5</terminal_resistance>
        <torque_constant>0.0552</torque_constant>
      </actuator>
    </technique>
  </extra>

attach_actuator
===============

Introduction
~~~~~~~~~~~~

Attaches an actuator to a joint.

Concepts
~~~~~~~~

The actuator comes from the actuator library.

Related Elements
~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  
  Parent elements | <articulated_system>

Child Elements
~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 70, 10
  :header: Element, Description, Occurances

  <instance_actuator> | Instantiate an actuator. | 1
  <bind_actuator> | Binds the actuator to a joint. | 1

Attributes for <bind_actuator>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. csv-table::
  :class: collada
  :delim: |
  :widths: 15, 15, 70

  joint | **xs:token** | Required. The reference of the joint in the instantiated kinematics model that is part of the manipulator chain.

Example
~~~~~~~

.. code-block:: xml

  <extra type="attach_actuator" name="motor0">
    <technique profile="OpenRAVE">
      <instance_actuator url="#ExampleMotor1"/>
      <bind_actuator joint="kmodel0/myjoint"/>
    </technique>
  </extra>


<articulated_system>/<kinematics>/<technique_common>/<axis_info>
================================================================

Extra parameters for each axis specified through the **<newparam>** element type.

.. csv-table::
  :class: collada
  :delim: |
  :widths: 20, 80
  :header: Name, Type, Description

  circular | **xs:bool** | Circular joint axes have the lower and upper limits identified like loops. This is not necessarily always -180 to 180 degrees.
  planning_weight | **xs:float** | For each joint, a measure of how much a joint's movement impacts the robot (base joints have more impact than end effector joints). this information should be used by all planners to evaluate importance of joints.
  discretization_resolution | **xs:float** | The maximum step size a joint range can be safely discretized in order to guarantee that any point on the kinematic body will not move beyond a certain specified range. For example, resolutions for each joint can be set in order to guarnatee to point moves more than 5mm when the joints move.

Notes/Usage
===========

* **articulated_system** tag is used for saving both Robot and KinBody objects
  *  if child is a **motion** tag, get accelerations and velocity limits from it
* If **<visual_scene>** tag present, but no kinematics, then add each root node tree as a rigid link.
* In order to set a static link in physics, use the **<instance_rigid_body>/<dynamic>** tag.

Hard and Soft Joint Limits
~~~~~~~~~~~~~~~~~~~~~~~~~~

In many scenarios, the controllers on the robots use joints limits which are smaller than the maximum limits. The controller limits are called **soft limits**, while the hardware limits are called **hard limits**. In COLLADA, the specification is:

* **hard limits** - specified inside the <joint> tag using the <limits> tag.

* **soft limits** - specified inside the <articulated_system>/<kinematics>/<technique_common>/<axis_info> tag using the <limits> tag.

Storing Convex Decompositions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each link is composed of a set of convex hulls. Need to create one geometry per convex hull (<convex_mesh>?) and specify multiple geometries per <node>.

Custom Data
~~~~~~~~~~~

OpenRAVE provides a user to hook up an XML writer and reader to a robot, which can be written as **<extra>** elements under any **<articulated_system>**.

.. code-block:: xml
  <articulated_system sid="body1_kinematics_inst" url="#body1_kinematics" name="box0">
    <motion/>
    <extra type="mycustomparams">
      <technique profile="myprofile">
        <mylocalparam>1</mylocalparam>
      </technique>
    </extra>
  </articulated_system>

.. _collada_openrave_uri:

OpenRAVE Database URI
~~~~~~~~~~~~~~~~~~~~~

OpenRAVE uses the **$OPENRAVE_DATA** environment variable to build its database of robots. Only files inside these directories can be accessed. Syntax::

  openrave://[user[:password]@]host[:port]/path

Saving Scenes with Resource References
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Although COLLADA is very flexible in terms of referencing libraries of models, in almost all cases, instantiating a robot will only change the following peices of information:

* robot joint values
* robot position
* what bodies the robot is grabbing (:ref:`collada_dynamic_rigid_constraints`)
* joint limits

For most cases, the **<kinematics_scene>** defined in the separate robot file can be left intact and overrides can happen inside **<instance_kinematics_scene>**. For example, a minimal file that references a robot and sets joint0's value to 1.5 is:

.. code-bloxk:: xml

  <COLLADA xmlns="http://www.collada.org/2008/03/COLLADASchema" version="1.5.0">
    <asset>
      <created/>
      <modified/>
    </asset>
    <scene>
      <instance_physics_scene url="./robot.dae#pscene" sid="pscene_inst"/>
      <instance_visual_scene url="./robot.dae#vscene" sid="vscene_inst"/>
      <instance_kinematics_scene url="./robot.dae#kscene" sid="kscene_inst">
        <bind_kinematics_model node="visual1/node0">
          <param>kscene_kmodel1_inst</param>
        </bind_kinematics_model>
        <bind_joint_axis target="visual1/node_joint0_axis0">
          <axis><param>kscene_kmodel1_inst_robot1_kinematics_kmodel1_inst_joint0.axis0</param></axis>
          <value><float>1.5</float></value>
        </bind_joint_axis>
      </instance_kinematics_scene>
    </scene>
  </COLLADA>

Unfortunately, COLLADA only allows one **instance_kinematics_scene** and one **instance_visual_scene**, which means that for multiple objects, the kinematics/physics/visual models have to be referenced directly instead of the scenes.

Instantiating Nodes with Similar Ids
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When the same visual hierarchy nodes are instanced in the same scene like this:

.. code-block:: xml

  <node name="body1">
    <translate>1 0 0</translate>
    <instance_node url="#somenode"/>
  </node>
  <node name="body2">
    <translate>1 0 0</translate>
    <instance_node url="#somenode"/>
  </node>

The Ids fo the child nodes in #somenode will clash, therefore add a **<extra>** tag to allow suffixing of the Ids:

.. code-block:: xml

  <node name="body1">
    <translate>1 0 0</translate>
    <instance_node url="#somenode">
      <extra type="idsuffix" name=".b1"/>
    </instance_node>
  </node>
  <node name="body2">
    <translate>1 0 0</translate>
    <instance_node url="#somenode">
      <extra type="idsuffix" name=".b2"/>
    </instance_node>
  </node>

Composing Robots from Multiple Parts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Robots usually have grippers, robot arms, and robot bases in separate files, then we have one file that references all of them and specifies the links to merge together (ie, we do not complicate things by creating dummy joints). This can be done with articulated systems since **<kinematics>** tag supports multiple **<instance_kinematics_model>** tags.

Visibility/Collision Flags
~~~~~~~~~~~~~~~~~~~~~~~~~~

Bodies can be initially invisible, but can still act as collision obstacles. Furthermore, there can exist nodes like plots and markers that should not act as kinbody elements or pertake in collision detection. 

* **visibility** is handled by :ref:`collada_geometry_info` and is per-geometry.

* **colliding** is handled by the :ref:`collada_collision`/**<link_collision_state>**

TODO
++++

Calibration vs Static Data
~~~~~~~~~~~~~~~~~~~~~~~~~~

One thing that separates a base description of the robot from the real robot that will be used in labs is calibration:

* where each sensor is with respect to the robot (6D pose)
* intrinsic parameters for each sensor
* joint offsets for encoder calibration
* controller parameters like PID gains for dynamic properties of motors
* possibly even link lengths depending on how much you trust the manufacturer

All these parameters will change per robot, and it won't be a good idea asking every person to go and modify their one robot file. Instead there should have a different calibration file that the main collada file always references. It should be setup in such a way that the calibration file becomes optional.

Trajectories
~~~~~~~~~~~~

Store the openrave XML trajectories, or possibly store as the COLLADA native **<animation>** elements.

Contributors
============

* Rosen Diankov
* Ryohei Ueda - `University of Tokyo JSK Lab <http://www.jsk.t.u-tokyo.ac.jp/>`_
