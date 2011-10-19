.. _geometric_conventions:

Conventions and Guidelines
==========================

This is the set of conventions all users are highly recommended to follow. Sticking to them will guarantee that OpenRAVE modules can inter-operate with each other.

Geometric Conventions
---------------------

* Internal matrices are in column-order, row-major format. If this is confusing, check out this guide. This means affine matrix representation is using the standard math way. All matrices are serialized in column-major format, this is to make it simpler for Octave/Matlab to convert between matrices. Note that python expects row-major matrices, which require transposition when passed around the two interfaces.

* Quaternions, the preferred way of representing rotations, are defined with the scalar value as the first component. For example [w x y z] or [cos sin*axis].

* A pose is an affine transformation specified as a quaternion and translation. It serializes into 7 values, the first 4 being the quaternion.

* Distances between two rotations is :math:`\cos^{-1} | q_1 \cdot q_2 |`, where each rotation is represented as a quaternion. For rotations close to each other, this is sometimes approximated as: :math:`\min\left( | q1 - q2 | \; , \; | q1 + q2 |\right)`.

* Joint axis rotation is counter-cockwise.

Robot Conventions
-----------------

* A robot's up direction is on the positive z-axis, forward direction is the positive x-axis.

* Mobile manipulation is performed in the XY plane.

* The origin of a robot should be defined so that its base perfectly rests on a floor at z=0, and its rotation around the z-axis is the center of rotation when the base makes a natural in-place turn.

* All objects/robots of human-environment scale should be specified in meters. There are many default thresholds and parameters that assume this convention, and not following it will result in explosion of computation. A more general convention is that the unit should be chosen so that the arm length of the robot is closest to 1.

* Every link/manipulator/sensor/joint in a robot/kinbody should have a name that will differentiate it from the others.

* The initial configuration of the robot when first loaded in the scene **cannot** be in self-collision.

Environment Conventions
-----------------------

* Every body added to the environment should have a unique name.
