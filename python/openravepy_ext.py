# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
from __future__ import with_statement # for python 2.5
import openravepy
import os, optparse, numpy, metaclass
try:
   import cPickle as pickle
except:
   import pickle

class KinBodyStateSaver:
    def __init__(self,body):
        self.body = body
    def __enter__(self):
        self.handle = self.body.CreateKinBodyStateSaver()
    def __exit__(self, type, value, traceback):
        self.handle.close()

class RobotStateSaver:
    def __init__(self,robot):
        self.robot = robot
    def __enter__(self):
        self.handle = self.robot.CreateRobotStateSaver()
    def __exit__(self, type, value, traceback):
        self.handle.close()

def mkdir_recursive(newdir):
    """works the way a good mkdir should :)
        - already exists, silently complete
        - regular file in the way, raise an exception
        - parent directory(ies) does not exist, make them as well
    """
    if os.path.isdir(newdir):
        pass
    elif os.path.isfile(newdir):
        raise OSError("a file with the same name as the desired dir, '%s', already exists." % newdir)
    else:
        head, tail = os.path.split(newdir)
        if head and not os.path.isdir(head):
            mkdir_recursive(head)
        if tail:
            os.mkdir(newdir)


class OpenRAVEModel(metaclass.AutoReloader):
    def __init__(self,robot):
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.manip = self.robot.GetActiveManipulator()
    def has(self):
        raise NotImplementedError()
    def getfilename(self):
        return os.path.join(self.env.GetHomeDirectory(),'robot.'+self.robot.GetRobotStructureHash())
    def load(self):
        if not os.path.isfile(self.getfilename()):
            return None
        return pickle.load(open(self.getfilename(), 'r'))
    def save(self,params):
        print 'saving model to %s'%self.getfilename()
        mkdir_recursive(os.path.join(self.env.GetHomeDirectory(),'robot.'+self.robot.GetRobotStructureHash()))
        pickle.dump(params, open(self.getfilename(), 'w'))
    def generate(self):
        raise NotImplementedError()
    def generateFromOptions(self,options):
        return self.generate()
    def show(self):
        raise NotImplementedError()
    def autogenerate(self,forcegenerate):
        """Caches parameters for most commonly used robots/objects and starts the generation process for them"""
        raise NotImplementedError()
    @staticmethod
    def CreateOptionParser():
        parser = optparse.OptionParser(description='Computes an openrave model and caches into file.')
        parser.add_option('--robot',action='store',type='string',dest='robot',default='robots/barrettsegway.robot.xml',
                          help='OpenRAVE robot to load')
        parser.add_option('--manipname',action='store',type='string',dest='manipname',default=None,
                          help='The name of the manipulator to use')
        parser.add_option('--show',action='store_true',dest='show',default=False,
                          help='If set, uses mayavi (v3+) to display the reachability')
        return parser
    @staticmethod
    def RunFromParser(Model,env=None,parser=None):
        if parser is None:
            parser = OpenRAVEModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        destroyenv = False
        if env is None:
            env = openravepy.Environment()
            destroyenv = True
        try:
            with env:
                robot = env.ReadRobotXMLFile(options.robot)
                env.AddRobot(robot)
                robot.SetTransform(numpy.eye(4))
                if options.manipname is None:
                    # prioritize manipulators with ik solvers
                    indices = [i for i,m in enumerate(robot.GetManipulators()) if m.HasIKSolver()]
                    if len(indices) > 0:
                        robot.SetActiveManipulator(indices[0])
                else:
                    robot.SetActiveManipulator([i for i,m in robot.robot.GetManipulators() if m.GetName()==options.manipname][0])
            model = Model(robot=robot)
            if options.show:
                if not model.load():
                    print 'failed to find cached model %s'%model.getfilename()
                    sys.exit(1)
                model.show()
                return
            try:
                model.autogenerate(forcegenerate=False)
            except ValueError, e:
                print e
                print 'attempting preset values'
                model.generateFromOptions(options)
                model.save()
        finally:
            if destroyenv:
                env.Destroy()
