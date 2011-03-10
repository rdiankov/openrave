Beginning with openravepy
=========================

`openravepy` allows Python to use the C++ API seamlessly. The bindings are developed using the
`Boost.Python <http://www.boost.org/doc/libs/release/libs/python/doc>`_ library. Because
`openravepy` directly links with OpenRAVE instead of going through the network, it allows much
faster execution times in a much more natural setting. In fact, most of the python bindings match
the exactly C++ header files exactly.

`openravepy` can be found in ``${OPENRAVE_INSTALL_DIR}/share/openrave`` on unix-based systems and ``C:\\Program Files\\openrave\\share\\openrave`` on Windows. For unix-based systems, the follow command can be used to retrieve the path:

.. code-block:: bash

  openrave-config --python-dir

When directly importing openravepy, this path needs to be added into the **PYTHONPATH** environment variable. For unix-based systems, it would look like:

.. code-block:: bash

  export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`

All the examples are stored in the ``openravepy/examples`` directory. For example, the simplest
planning example hanoi can be found in ``openravepy/examples/hanoi.py`` and executed by:

.. code-block:: bash

  openrave.py --example hanoi

OpenRAVE should automatically pop up and the puma arm will attempt to grasp the pegs on the table.


The docstrings for each function and class are automatically compiled from the C++ documentation. In the python interpreter, simply type:

.. code-block:: python

  help env.CloneSelf
  help KinBoby.GetChain
  help Robot.Manipulator.FindIKSolution
