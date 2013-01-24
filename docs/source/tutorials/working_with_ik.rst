Working with Inverse Kinematics
===============================

Debugging IK
------------

1. Set the debug level to VERBOSE:

  .. code-block:: python
  
    RaveSetDebugLevel(DebugLevel.Verbose)

That will give more output on why IK fails.

2. On FindIKSolution, add a "ikreturn=True" parameter. This will return an IkReturn object and call its GetAction() method in order to get a list of errors that failed. They will be part of IkReturnAction enum.

3. Turn env collision checking off and self-collision checking off by passing in a 0 or IkFilterOptions.IgnoreSelfCollisions as the second parameter.

