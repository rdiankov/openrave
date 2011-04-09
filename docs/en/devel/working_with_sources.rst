Working With the Source Code
============================

Most of these guidelines are for Linux users.

In order to compile debug, just do:

.. code-block:: bash

  make DEBUG=d

This will create a **builddebug** folder independent of the original **build** folder.

Can setup `colorgcc <https://github.com/johannes/colorgcc>`_ and `ccache <http://ccache.samba.org/>`_  for easier development by executing the following script inside the **~/.bashrc** file:

.. literalinclude:: ../../../sandbox/setupgcc.sh
  :language: bash

Note that this will create symlinks to the compilers inside **/usr/local/bin**.
