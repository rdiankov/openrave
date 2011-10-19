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

from openravepy import examples

def run_example(name,args=[]):
    __doc__='testing example '+name
    example = getattr(examples,name)
    example.run(args=args+['--testmode',"--viewer="])

def test_examples():
    print "test if all the examples run"
    yield run_example, 'hanoi', []
    #yield run_example, 'calibrationviews'
    #yield run_example, 'graspplanning', []

#     for name in dir(examples):
#         if not name.startswith('__'):
#             try:
#                 m=__import__('openravepy.examples.'+name)
#                 if type(m) is ModuleType:
#                     yield run_example, name
#             except ImportError:
#                 pass

def run_database(name,args=[]):
    __doc__='testing database '+name
    database = getattr(databases,name)
    database.run(args=args+["--viewer="])

# def test_databases():
#     """test if all the databases run on default parameters"""
#     yield run_database, 'kinematicreachability', ['--robot=robots/barrettwam.robot.xml','--quatdelta=1','--xyzdelta=0.1']
#     yield run_database, 'kinematicreachability', ['--robot=robots/pr2-beta-static.zae','--manipname=leftarm','--quatdelta=1','--xyzdelta=0.1','--ignorefreespace']
#     yield run_database, 'kinematicreachability', ['--robot=robots/kawada-hironx.zae','--manipname=leftarm','--quatdelta=1','--xyzdelta=0.2','--ignorefreespace']
# 
#     yield run_database, 'inversereachability', ['--robot=robots/barrettwam.robot.xml']
#     yield run_database, 'convexdecomposition', ['--robot=robots/kawada-hironx.zae']
#     yield run_database, 'linkstatistics', ['--robot=robots/kawada-hironx.zae']
#     yield run_database, 'inversekinematics', ['--robot=robots/barrettwam.robot.xml']
#     yield run_database, 'grasping', ['--robot=robots/barrettwam.robot.xml','--boxdelta=0.05']
#     yield run_database, 'grasping', ['--robot=robots/pr2-beta-static.zae','--manipname=leftarm','--target=data/box_frootloops.kinbody.xml','--boxdelta=0.05']
#     yield run_database, 'visibilitymodel', ['--robot=robots/pa10schunk.robot.xml','--target=data/box_frootloops.kinbody.xml']

    
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

