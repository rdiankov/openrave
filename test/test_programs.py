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
from common_test_openrave import *
import os, sys, platform
import shutil

def run_example(name):
    __doc__='testing example '+name
    example = getattr(examples,name)
    example.run(args=['--testmode',"--viewer="])

def test_examples():
    print "test if all the examples run"
    yield run_example, 'hanoi'
    #yield run_example, 'calibrationviews'
    yield run_example, 'graspplanning'
#     for name in dir(examples):
#         if not name.startswith('__'):
#             try:
#                 m=__import__('openravepy.examples.'+name)
#                 if type(m) is ModuleType:
#                     yield test_example, name
#             except ImportError:
#                 pass

# def test_databases():
#     """test if all the databases run on default parameters"""
#     pass

def test_createplugin():
    curdir = os.getcwd()
    try:
        shutil.rmtree('myplugin')
    except:
        pass

    cmakeoptions = ''
    makecommand = 'make %s'
    pythoncommand = 'python '
    runcommand = './'
    programdir = 'build'
    if openravepyCompilerVersion().startswith('msvc'):
        cmakeoptions += '-G "Visual Studio 10" '
        makecommand = '"C:\\Program Files\\Microsoft Visual Studio 10.0\\VC\\vcvarsall.bat" x86 && msbuild %s.sln /p:Configuration=RelWithDebInfo /maxcpucount:1'
    if sys.platform.startswith('win') or platform.system().lower() == 'windows':
        pythoncommand = ''
        runcommand = ''
        programdir = 'build\\RelWithDebInfo'
    try:
        assert(os.system('openrave-createplugin.py myplugin --module=MyTestModule') == 0)
        os.chdir('myplugin')
        os.mkdir('build')
        os.chdir('build')
        assert(os.system('cmake %s ..'%cmakeoptions) == 0)
        assert(os.system(makecommand%'myplugin') == 0)
        os.chdir('..')
        if programdir != 'build':
            shutil.copyfile(os.path.join(programdir,'myplugin.dll'),'build/myplugin.dll')
        assert(os.system('%stestplugin.py'%pythoncommand) == 0)
    finally:
        os.chdir(curdir)
        try:
            shutil.rmtree('myplugin')
        except:
            pass

    try:
        shutil.rmtree('myprogram')
    except:
        pass
    try:
        assert(os.system('openrave-createplugin.py myprogram --usecore') == 0)
        os.chdir('myprogram')
        os.mkdir('build')
        os.chdir('build')
        assert(os.system('cmake %s ..'%cmakeoptions) == 0)
        assert(os.system(makecommand%'myprogram') == 0)
        os.chdir('..')
        assert(os.system(runcommand+os.path.join(programdir,'myprogram')) == 0)
    finally:
        os.chdir(curdir)
        shutil.rmtree('myprogram')

