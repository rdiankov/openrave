#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
from optparse import OptionParser
import nose
from nose.plugins import failuredetail
from noseplugins import xunitmultiprocess, capture, callableclass, multiprocess

import multiprocessing

if __name__ == "__main__":
    import test_kinematics
    parser = OptionParser(description='OpenRAVE unit tests')
    parser.add_option('--timeout','-t', action='store', type='float', dest='timeout',default='600',
                      help='Timeout for each ikfast run, this includes time for generation and performance measurement. (default=%default)')
    parser.add_option('-j', action='store', type='int', dest='numprocesses',default=None,
                      help='Number of processors to run this in (default=%default).')
    parser.add_option('--with-coverage',action='store_true',dest='with_coverage',default=False,
                      help='set to create coverage statistics')
    parser.add_option('--os-only',action='store_true',dest='os_only',default=False,
                      help='set to run only tests that test program execution to make sure things run on the current OS')
    (options, args) = parser.parse_args()

    multiprocess._instantiate_plugins = [capture.Capture, xunitmultiprocess.Xunitmp,failuredetail.FailureDetail,callableclass.CallableClass]
    numprocesses = options.numprocesses if options.numprocesses is not None else multiprocessing.cpu_count()
    argv=['nosetests','-v','--with-xunitmp','--xunit-file=results.xml','--processes=%d'%numprocesses,'--process-timeout=%f'%options.timeout,'--process-restartworker','-d','--with-callableclass','-s']
    if options.os_only:
        argv.append('test_programs.py')
    if options.with_coverage:
        argv += ['--with-coverage', '--cover-package=openravepy','--cover-html']
    plugins=[capture.Capture(),multiprocess.MultiProcess(),xunitmultiprocess.Xunitmp(),failuredetail.FailureDetail(),callableclass.CallableClass()]
    prog=nose.core.TestProgram(argv=argv,plugins=plugins,exit=False)
