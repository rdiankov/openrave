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

class RunExample(object):
    __name__= 'test_programs.runexample'
    def __init__(self,name,docname,args=[]):
        self.name=name
        self.args=args
        self.description = 'test_programs.example.%s.%s'%(name,docname)
            
    def __call__(self):
        # turn off trajectory validation for now
        RaveSetDebugLevel(RaveGetDebugLevel())
        example = getattr(examples,self.name)
        example.run(args=self.args+['--testmode',"--viewer="])

# def test_examples():
#     yield RunExample('hanoi', 'default',[])
#     yield RunExample('calibrationviews', 'default',['--noshowsensor'])
#     yield RunExample('graspplanning', 'default', [])
# 
# class RunDatabase(object):
#     def __init__(self,name,docname,args=[]):
#         self.name=name
#         self.args=args
#         self.description = 'test_programs.database.%s.%s'%(name,docname)
#             
#     def __call__(self):
#         database = getattr(databases,self.name)
#         database.run(args=args+["--viewer="])
# 
# def test_databases():
#     """test if all the databases run on default parameters"""
#     yield RunDatabase('kinematicreachability', 'wam', ['--robot=robots/barrettwam.robot.xml','--quatdelta=1','--xyzdelta=0.2'])
#     yield RunDatabase('kinematicreachability', 'pr2', ['--robot=robots/pr2-beta-static.zae','--manipname=leftarm','--quatdelta=1','--xyzdelta=0.4'])
#     yield RunDatabase('convexdecomposition', 'hironx', ['--robot=robots/kawada-hironx.zae'])
#     yield RunDatabase('linkstatistics', 'hironx', ['--robot=robots/kawada-hironx.zae'])
#     yield RunDatabase('linkstatistics', 'pr2', ['--robot=robots/pr2-beta-static.zae'])
#     yield RunDatabase('grasping', 'barrett', ['--robot=robots/barrettwam.robot.xml','--target=data/mug2.kinbody.xml','--boxdelta=0.1'])
#     yield RunDatabase('grasping', 'barrett_multi', ['--robot=robots/barrettwam.robot.xml','--target=data/mug2.kinbody.xml','--boxdelta=0.1','--numthreads=2'])
#     yield RunDatabase('inversekinematics', 'wam', ['--robot=robots/barrettwam.robot.xml','--iktests=100'])
        
#     yield run_database, 'inversereachability', ['--robot=robots/barrettwam.robot.xml']
#     yield run_database, 'grasping', ['--robot=robots/pr2-beta-static.zae','--manipname=leftarm','--target=data/box_frootloops.kinbody.xml','--boxdelta=0.05']
#     yield run_database, 'visibilitymodel', ['--robot=robots/pa10schunk.robot.xml','--target=data/box_frootloops.kinbody.xml']

def GetRunCommand():
    return '' if sys.platform.startswith('win') or platform.system().lower() == 'windows' else './'

def GetPythonCommand():
    return '' if sys.platform.startswith('win') or platform.system().lower() == 'windows' else 'python '

def CompileProject(cmakedir,cmakebuilddir=None):
    """will compile the cmake project and run testfn
    """
    cmakeoptions = ''
    cmakedir = os.path.abspath(cmakedir)
    cmakename = os.path.split(cmakedir)[1]
    makecommand = 'make '
    if cmakebuilddir is None:
        cmakebuilddir = cmakename+'_build'
    programdir = cmakebuilddir
    if openravepyCompilerVersion().startswith('msvc'):
        cmakeoptions += '-G "Visual Studio 10" '
        makecommand = '"C:\\Program Files\\Microsoft Visual Studio 10.0\\VC\\vcvarsall.bat" x86 && msbuild %s.sln /p:Configuration=RelWithDebInfo /maxcpucount:1'%cmakename
    if sys.platform.startswith('win') or platform.system().lower() == 'windows':
        programdir += '\\RelWithDebInfo'

    try:
        # remove dir first since it could contain cached information of previous versions
        shutil.rmtree(cmakebuilddir)
    except:
        pass

    os.mkdir(cmakebuilddir)

    curdir = os.getcwd()
    try:
        os.chdir(cmakebuilddir)
        assert(os.system('cmake %s %s'%(cmakeoptions,cmakedir)) == 0)
        assert(os.system(makecommand) == 0)
        os.chdir(curdir)
        if openravepyCompilerVersion().startswith('msvc'):
            dllfilename = os.path.join(programdir,cmakename+'.dll')
            if os.path.exists(dllfilename):
                shutil.copyfile(dllfilename,os.path.join(cmakebuilddir,cmakename+'.dll'))
        return programdir

    finally:
        os.chdir(curdir)

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
        programdir = CompileProject(name,os.path.join(name,'build'))
        assert(os.system(GetRunCommand()+os.path.join(programdir,name)) == 0)
    finally:
        os.chdir(curdir)
        shutil.rmtree(name)
        
def test_cppexamples():
    programdir = CompileProject(os.path.join('..','src','cppexamples'))
    shutil.rmtree(programdir)
        
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
        programdir=CompileProject('myplugin',os.path.join('myplugin','build'))
        os.chdir('myplugin')
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
        programdir=CompileProject('myprogram',os.path.join('myprogram','build'))
        os.chdir('myprogram')
        assert(os.system(GetRunCommand()+os.path.join(os.path.relpath(programdir,'myprogram'),'myprogram')) == 0)
    finally:
        os.chdir(curdir)
        shutil.rmtree('myprogram')
