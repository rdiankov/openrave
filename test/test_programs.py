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
"""Note that Windows is also running this test
"""
from common_test_openrave import *
import os, sys, platform
import shutil

from openravepy import examples

def run_example(name,args=[]):
    __doc__='testing example '+name
    example = getattr(examples,name)
    example.run(args=args+['--testmode',"--viewer="])

# def test_examples():
#     print "test if all the examples run"
#     yield run_example, 'hanoi', []
#     yield run_example, 'calibrationviews', ['--noshowsensor']
#     #yield run_example, 'graspplanning', []


#     for name in dir(examples):
#         if not name.startswith('__'):
#             try:
#                 m=__import__('openravepy.examples.'+name)
#                 if type(m) is ModuleType:
#                     yield run_example, name
#             except ImportError:
#                 pass

def run_database(name,docname,args=[]):
    __doc__='database.%s.%s'%(name,docname)
    database = getattr(databases,name)
    database.run(args=args+["--viewer="])

def test_databases():
    """test if all the databases run on default parameters"""
    yield run_database, 'kinematicreachability', 'wam', ['--robot=robots/barrettwam.robot.xml','--quatdelta=1','--xyzdelta=0.2']
    yield run_database, 'kinematicreachability', 'pr2', ['--robot=robots/pr2-beta-static.zae','--manipname=leftarm','--quatdelta=1','--xyzdelta=0.4']
    yield run_database, 'convexdecomposition', 'hironx', ['--robot=robots/kawada-hironx.zae']
    yield run_database, 'linkstatistics', 'hironx', ['--robot=robots/kawada-hironx.zae']
    yield run_database, 'linkstatistics', 'pr2', ['--robot=robots/pr2-beta-static.zae']
    yield run_database, 'grasping', 'barrett', ['--robot=robots/barrettwam.robot.xml','--target=data/mug2.kinbody.xml','--boxdelta=0.1']
    yield run_database, 'grasping', 'barrett_multi', ['--robot=robots/barrettwam.robot.xml','--target=data/mug2.kinbody.xml','--boxdelta=0.1','--numthreads=2']
    yield run_database, 'inversekinematics', 'wam', ['--robot=robots/barrettwam.robot.xml','--iktests=100']
        
#     yield run_database, 'inversereachability', ['--robot=robots/barrettwam.robot.xml']
#     yield run_database, 'grasping', ['--robot=robots/pr2-beta-static.zae','--manipname=leftarm','--target=data/box_frootloops.kinbody.xml','--boxdelta=0.05']
#     yield run_database, 'visibilitymodel', ['--robot=robots/pa10schunk.robot.xml','--target=data/box_frootloops.kinbody.xml']

def GetRunCommand():
    return '' if sys.platform.startswith('win') or platform.system().lower() == 'windows' else './'

def GetPythonCommand():
    return '' if sys.platform.startswith('win') or platform.system().lower() == 'windows' else 'python '

def CompileProject(cmakedir):
    """will compile the cmake project and run testfn
    """
    cmakeoptions = ''
    makecommand = 'make %s'
    programdir = 'build'
    if openravepyCompilerVersion().startswith('msvc'):
        cmakeoptions += '-G "Visual Studio 10" '
        makecommand = '"C:\\Program Files\\Microsoft Visual Studio 10.0\\VC\\vcvarsall.bat" x86 && msbuild %s.sln /p:Configuration=RelWithDebInfo /maxcpucount:1'
    if sys.platform.startswith('win') or platform.system().lower() == 'windows':
        programdir = 'build\\RelWithDebInfo'
    os.chdir(cmakedir)
    os.mkdir('build')
    os.chdir('build')
    assert(os.system('cmake %s ..'%cmakeoptions) == 0)
    assert(os.system(makecommand%cmakedir) == 0)
    os.chdir('..')
    if programdir != 'build':
        dllfilename = os.path.join(programdir,cmakedir+'.dll')
        if os.path.exists(dllfilename):
            shutil.copyfile(dllfilename,'build/'+cmakedir+'.dll')
    return programdir

def CompileRunCPP(name,cppdata):
    curdir = os.getcwd()
    try:
        shutil.rmtree(name)
    except:
        pass    

    try:
        os.mkdir(name)
        open(os.path.join(name,name+'.cpp'),'w').write(cppdata)
        CMakeLists = """cmake_minimum_required (VERSION 2.6.0)
project(%(name)s)
find_package(OpenRAVE REQUIRED)
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)
include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_executable(%(name)s %(name)s.cpp)
set_target_properties(%(name)s PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(%(name)s ${OpenRAVE_LIBRARIES})
install(TARGETS %(name)s DESTINATION .)
"""%{'name':name}
        open(os.path.join(name,'CMakeLists.txt'),'w').write(CMakeLists)
        programdir = CompileProject(name)
        assert(os.system(GetRunCommand()+os.path.join(programdir,name)) == 0)
    finally:
        os.chdir(curdir)
        shutil.rmtree(name)

def test_cppgeometry_standalone():
    cppdata="""#include <openrave/geometry.h>
using namespace OpenRAVE::geometry;
int main()
{
    RaveTransformMatrix<double> m = matrixFromAxisAngle(RaveVector<double>(1,1,1));
    return 0;
}
    """
    CompileRunCPP('testgeometry',cppdata)
        
def test_cpputils_standalone():
    cppdata="""#include <openrave/utils.h>
#include <iostream>
using namespace OpenRAVE;
using namespace std;
int main()
{
    cout << utils::NormalizeCircularAngle(4.5,0.0,1.0) << endl;
    cout << utils::GetMD5HashString("this is a test md5 hash string") << endl;
    return 0;
}
    """
    CompileRunCPP('testutils',cppdata)
    
def test_createplugin():
    curdir = os.getcwd()
    try:
        shutil.rmtree('myplugin')
    except:
        pass

    try:
        assert(os.system('openrave-createplugin.py myplugin --module=MyTestModule') == 0)
        programdir=CompileProject('myplugin')
        assert(os.system(GetPythonCommand()+'testplugin.py') == 0)
    finally:
        os.chdir(curdir)
        shutil.rmtree('myplugin')

    try:
        shutil.rmtree('myprogram')
    except:
        pass
    try:
        assert(os.system('openrave-createplugin.py myprogram --usecore') == 0)
        programdir=CompileProject('myprogram')
        assert(os.system(GetRunCommand()+os.path.join(programdir,'myprogram')) == 0)
    finally:
        os.chdir(curdir)
        shutil.rmtree('myprogram')
