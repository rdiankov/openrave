.. _arch_trajectory:

Trajectory Concepts
===================

Reference: :class:`.Trajectory`

A trajectory is a path between a set of configuration space points. It performs smoothing and
filtering on this path.

A trajectory holds **waypoints** of different pieces of data and their corresponding **interpolation method**. A waypoint can specify robot joints, timestamps, body transformations, events,
etc. Subclasses of trajectories have differnet way of executing and interpolating the waypoints. A
trajectory is just the executor, it is **not** responsible for re-timing and re-adjusting its path,
which is a planner's job. **A trajectory cannot be executed without timestamps being present.**

Usage Cases
-----------

- Planner fills the trajectory class with timestamps, interpolation methods, and waypoints. It
  should be up to user to select trajectory class that will best execute this data.

- Users can create, append, retime, and serialize trajectories both in C++ and Python.

- Store arbitrary animations of an environment. Support for any configuration space similar to
  PlannerParameters so that multiple bodies and affine transformations are supported. For example, a robot
  opening door requires the robot joints and the door to move together.

- Sample a trajectory at any time. Easily set the scene as the trajectory dictates it. How would
  this affect robot controllers?

- It should be possible to play back the trajectory classes without loading any robots or any
  environment data. A real controller, on robot side, can link with openrave-core and use the
  trajectory class for interpolating incoming trajectory data.

Details
-------

Every trajectory has a :class:`.ConfigurationSpecification` that defines what is stored in each
point. This allows points to hold any type of custom information and to synchronize better with the
configuration spaces of the planners. Use the :meth:`.Trajectory.GetConfigurationSpecification` to
get the details of what data is in the trajectory and its dimensions. Every point should have a
**deltatime** value that specifies the time it takes to go to it given the previous point. With this
convention, the first point's deltatime should be 0.

.. _arch_trajectory_format:

Trajectory Serialization
------------------------

The file format for exporting the trajectory is in XML:

.. code-block:: xml

  <trajectory type="string">
    <configuration>
      <group name="string" offset="#OFF1" dof="#D1" interpolation="string"/>
      <group name="string" offset="#OFF2" dof="#D2" interpolation="string"/>
    </configuration>
    <data count="#N">
      1 2 3 4 5
    </data>
  </trajectory>

----

configuration tag
~~~~~~~~~~~~~~~~~

First tags specified. The configuration describes one type of data inside the trajectory and how it is stored and processed. See :class:`.ConfigurationSpecification` for details on the individual attributes. In trajectories a group **name="deltatime"** should be present if the trajectory is to be executed. The deltatime values always need to be positive.

----

data tag
~~~~~~~~

The raw data of the trajectory, there is **count** x **dof** values. Multiple data tags can be present

attributes:

- **count** - number of waypoints. although the number can be deduced from the size of the data, this is a precaution if the user wants to specify a subset.
