Working With the Source Code
============================

Most of these guidelines are for Linux users.

Building
--------

In order to compile debug, just do:

.. code-block:: bash

  make DEBUG=d

This will create a **builddebug** folder independent of the original **build** folder.

Can setup `colorgcc <https://github.com/johannes/colorgcc>`_ and `ccache <http://ccache.samba.org/>`_  for easier development by executing the following script inside the **~/.bashrc** file:

.. literalinclude:: ../../../sandbox/setupgcc.sh
  :language: bash

Note that this will create symlinks to the compilers inside **/usr/local/bin**.

Emacs
-----

When using `Collection of Emacs Development Environment Tools (CEDEC) <http://cedet.sourceforge.net/>`_, make sure to put the following Lisp code in your .emacs file in order to get auto-completion for the OpenRAVE C++ API.

.. code-block:: common-lisp

  (defun openrave-package-path ()
    (save-excursion
      (with-temp-buffer
        (call-process "openrave-config" nil t nil "--cflags-only-I")
        (goto-char (point-min))
        (re-search-forward "^-I\\(.*\\)[ \\|$]")
        (match-string 1))))

  (setq openrave-base-dir (openrave-package-path))
  (semantic-add-system-include openrave-base-dir 'c++-mode)
  (semantic-add-system-include openrave-base-dir 'c-mode)
  (add-to-list 'auto-mode-alist (cons openrave-base-dir 'c++-mode))
  (add-to-list 'semantic-lex-c-preprocessor-symbol-file (concat openrave-base-dir "/openrave/config.h"))
