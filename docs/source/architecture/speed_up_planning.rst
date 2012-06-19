.. _speed_up_planning:

Ways to Speed up Planning
=========================

This page attempts to answer a commonly asked question: *The planning algorithm is SLOW, what can we do?!*

* Create a :mod:`.databases.linkstatistics` model and tune the xyzdelta that controls the joint resolutions/weights. Can also manually tweak the joint resolutions and weights via `.KinBody.SetDOFWeights` and `.KinBody.Joint.SetResolution`.

* Each planner has the following parameters in its :class:`.Planner.PlannerParameters` structure that can be tweaked:

 * Planner.PlannerParameters._fStepLength - max step length for each tree extension
 * Planner.PlannerParameters._nMaxIterations - max number of iterations to search

* Each planner usually has a post-processing planner that does the smoothing/retiming of the path. The parameters to control are
 * Planner.PlannerParameters._sPostProcessingPlanner
 * Planner.PlannerParameters._sPostProcessingParameters

.. code-block:: c++

  _sPostProcessingParameters = "parabolicsmoother";
  _sPostProcessingParameters = <_nmaxiterations>100</_nmaxiterations>";

* The smoothers has several other parameters, which are written in the TrajectoryTimingParameters structure inside the rplanners plugin

* Add as many pairs of links of the robot that will never collide as "adjacent links" (both OpenRAVE XML and COLLADA specifications support this).

* Temporarily disable KinBody objects via :meth:`.KinBody.Enable` (False) for kinematic bodies that are too far from the robot or robot could never collide with them.

* Use primitive shapes (boxes/cylinders/spheres) as much as possible!
