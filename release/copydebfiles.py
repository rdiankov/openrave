#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
"""copies necessary files for deb source package building to a destination directory. The destination directory must not exist
"""
import os, sys
import shutil

if __name__=='__main__':
    srcdir = sys.argv[1]
    destdir = sys.argv[2]

    removefiles = '.git docs release sandbox test sympy msvc_boost.tgz msvc_collada.tgz msvc_libxml2.tgz msvc_ode.tgz msvc_soqt.tgz msvc_include sympy_0.7.1.tgz'.split()
    def ignorefiles(src, names):
        if src == srcdir:
            return '.git docs release sandbox test sympy msvc_boost.tgz msvc_collada.tgz msvc_libxml2.tgz msvc_ode.tgz msvc_soqt.tgz msvc_include sympy_0.7.1.tgz'.split()
        
        if src == os.path.join(srcdir,'3rdparty'):
            return ['pcre-8.02', 'qhull', 'zlib', 'flann-1.6.6', 'collada']
        
        return []
    shutil.copytree(srcdir, destdir, ignore = ignorefiles)
