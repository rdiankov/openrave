Working With the Source Code
============================

These guidelines are for OpenRAVE Developers using Linux.

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

Auto-Completion
---------------

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

Indenting C++ Files
-------------------

**Do not use Hard TABS!!**. And have indents by 4 spaces. OpenRAVE C++ code should automatically be run with **uncrustify** before being committed. Currently OpenRAVE requires uncrustify version >=0.58. Put the following configuration in **~/.uncrustify.cfg**:

.. literalinclude:: ../../../sandbox/uncrustify.cfg

It is possible to setup emacs to automatically perform the uncrustify operation when saving a file by downloading the `emacs-uncrustify <https://github.com/glima/Emacs-uncrustify>`_  pakcage and putting the following in your **.emacs** file:

.. code-block:: common-lisp

  (require 'uncrustify)
  (setq uncrustify-uncrustify-on-save t)
  (setq uncrustify-args "-l CPP")
  (add-hook 'c++-mode-common-hook
           '(lambda()
              (make-local-variable 'write-contents-hooks)
              (add-hook 'write-contents-hooks
                        'uncrustify-uncrustify-buffer-on-save)))
