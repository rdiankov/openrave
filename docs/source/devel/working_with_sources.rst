Working With the Source Code
============================

These guidelines are for OpenRAVE Developers using Linux.

Building
--------

In order to compile debug, just do:

.. code-block:: bash

  make DEBUG=d

This will create a **builddebug** folder independent of the original **build** folder.

Can setup `colorgcc <https://github.com/johannes/colorgcc>`_ and `ccache <http://ccache.samba.org/>`_  for easier development by executing the following **setupgcc.bash** script inside the **~/.bashrc** file:

.. literalinclude:: ../../../sandbox/setupgcc.bash
  :language: bash

Note that this will create symlinks to the compilers inside **/usr/local/bin**.

Indenting C++ Files
-------------------

OpenRAVE C++ code should automatically be run with `uncrustify <http://uncrustify.sourceforge.net/>`_ before being committed. Currently OpenRAVE requires uncrustify version >=0.57. First put :download:`.uncrustify.cfg <../../../sandbox/.uncrustify.cfg>` in your $HOME directory.

Run a C++ file before committing to OpenRAVE:

.. code-block:: bash

  uncrustify --no-backup myfile.cpp

:ref:`emacs_automatic_indention` shows how to automate calling uncrustify when saving the C++ file in Emacs.

Auto-Completion
###############

To get auto-completion for the OpenRAVE C++ API using `Collection of Emacs Development Environment Tools (CEDEC) <http://cedet.sourceforge.net/>`_, make sure to put the following Lisp code in your **.emacs** file:

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

.. _emacs_automatic_indention:

Automatic Indention
###################

It is possible to setup emacs to automatically call **uncrustify** when saving a file by downloading the `emacs-uncrustify <https://github.com/glima/Emacs-uncrustify>`_  pakcage, setting its path in the emacs load path, and putting the following in your **.emacs** file:

.. code-block:: common-lisp

  (require 'uncrustify)
  (setq uncrustify-uncrustify-on-save t)
  (setq uncrustify-args "-l CPP")

Bookmarks
#########

Using the `bm.el <http://www.nongnu.org/bm/>`_ library for getting bookmarks. If using bookmarks with uncrustify, have to add the following code to your **.emacs** file:

.. code-block:: common-lisp

  (add-hook 'uncrustify-init-hooks 'bm-buffer-save)
  (add-hook 'uncrustify-finish-hooks 'bm-buffer-restore)
