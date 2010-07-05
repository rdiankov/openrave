#!/usr/bin/env python
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
#
#Created: 5 January 2010 
#Modified: 20 March 2010
from __future__ import with_statement # for python 2.5
__author__ = 'Achint Aggarwal'
__copyright__ = '2010 Achint Aggarwal'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
from optparse import OptionParser
import time

class Schunkplanner:
   probsmanip = None
   def __init__(self,env):
      self.env = env
      self.robot = self.env.GetRobots()[0]
      self.probsmanip = self.env.CreateProblem('dualmanipulation')
      args = self.robot.GetName()
      #args += ' planner birrt' 
      self.env.LoadProblem(self.probsmanip,args)

      self.leftArm=self.robot.GetManipulators()[0]
      self.rightArm=self.robot.GetManipulators()[1]
      for imanip in [0,1]:
         self.robot.SetActiveManipulator(imanip)
         ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
         if not ikmodel.load():
            ikmodel.autogenerate()
            ikmodel.load()
      self.robot.SetActiveManipulator(0)                

   def WaitForController(self):
      while not self.robot.GetController().IsDone():
         self.robot.WaitForController(0)

   def Serialize(self, T):
      return 'goal %s'%(' '.join(str(f) for f in T))

   def MoveArmsToJointPosition(self, T):
      """Moves the two arms to the given joint position T"""
      success = self.probsmanip.SendCommand('movealljoints '+self.Serialize(T))
      return False if success is None or len(success) == 0 else True
   def MoveObjectToPosition(self, T):
      """Constrained movement of the arms to new position T while holding the object"""
      success = self.probsmanip.SendCommand('movealljoints '+self.Serialize(T)+' constrainterrorthresh 1')
      if success is None or (len(success) == 0):
         return False
      else: 
         return True
      
   def planDualPath(self,name):
      """this plans the trajectory for both the manipulators"""
      obj=self.env.GetKinBody(name)
      Tbody=obj.GetTransform()
      ab = obj.ComputeAABB().extents()
      halfwidth= ab[1] #this is y
      TRightGrasp= dot(Tbody,array([[0, 0, -1, 0],[1, 0, 0, (halfwidth+.1)],[0, -1, 0, 0],[0, 0, 0, 1]]))
      TLeftGrasp= dot(Tbody,array([[0, 0, -1, 0],[-1, 0, 0,-(halfwidth+.1)],[0, 1, 0, 0],[0, 0, 0, 1]])) #to determine the grasp for the eef given the transform of the object
      
      with self.env:
         jointsRight=self.rightArm.FindIKSolution(TRightGrasp,True)
         jointsLeft=self.leftArm.FindIKSolution(TLeftGrasp,True)
         if jointsRight is None: 
            print('Tgrasp invalid for Right Arm')
            if jointsLeft is None:
               print('Tgrasp invalid for Left Arm')
      
      T=array([jointsLeft[0],jointsLeft[1],jointsLeft[2],jointsLeft[3],jointsLeft[4],jointsLeft[5],jointsLeft[6],jointsRight[0],jointsRight[1],jointsRight[2],jointsRight[3],jointsRight[4],jointsRight[5],jointsRight[6]])
      
      if self.MoveArmsToJointPosition(T):
         print('failed to move to position next to object')         

   def moveObject(self,name,x,y,z):
      """this plans the trajectory for both the manipulators manipulating the object 'name' """
      obj=self.env.GetKinBody(name)
      Tbody=obj.GetTransform()
      ab = obj.ComputeAABB().extents()
      halfwidth= ab[1] #this is y
      Tbody[0,3]=Tbody[0,3]+x
      Tbody[1,3]=Tbody[1,3]+y
      Tbody[2,3]=Tbody[2,3]+z
      
      TRightGrasp= dot(Tbody,array([[0, 0, -1, 0],[1, 0, 0, (halfwidth+.04)],[0, -1, 0, 0 ],[0, 0, 0, 1]])) #.04 is just half the thickness of the EEF
      TLeftGrasp= dot(Tbody,array([[0, 0, -1, 0],[-1, 0, 0, -(halfwidth+.04)],[0, 1, 0, 0],[0, 0, 0, 1]])) #to determine the grasp for the eef given the transform of the object
      with self.env:
         jointsRight=self.rightArm.FindIKSolution(TRightGrasp,False)
         jointsLeft=self.leftArm.FindIKSolution(TLeftGrasp,False)
         if jointsRight is None: 
            print('Tgrasp invalid for Right Arm')
            if jointsLeft is None:
               print('Tgrasp invalid for Left Arm')

      T=array([jointsLeft[0],jointsLeft[1],jointsLeft[2],jointsLeft[3],jointsLeft[4],jointsLeft[5],jointsLeft[6],jointsRight[0],jointsRight[1],jointsRight[2],jointsRight[3],jointsRight[4],jointsRight[5],jointsRight[6]])
      self.MoveObjectToPosition(T) 
      
   def graspObject(self):
      ThandR=self.robot.GetManipulators()[0].GetEndEffectorTransform()
      ThandL=self.robot.GetManipulators()[1].GetEndEffectorTransform()
      self.probsmanip.SendCommand('movebothhandsstraight direction1 %lf ' %(ThandR[0,3]-ThandL[0,3]) +'%lf '%(ThandR[1,3]-ThandL[1,3]) +'%lf'%(ThandR[2,3]-ThandL[2,3]) +' direction0 %lf ' %(ThandL[0,3]-ThandR[0,3]) +'%lf '%(ThandL[1,3]-ThandR[1,3]) +'%lf'%(ThandL[2,3]-ThandR[2,3]))

   def releaseObject(self):
      ThandR=self.robot.GetManipulators()[0].GetEndEffectorTransform()
      ThandL=self.robot.GetManipulators()[1].GetEndEffectorTransform()
      self.probsmanip.SendCommand('movebothhandsstraight direction1 %lf ' %(ThandL[0,3]-ThandR[0,3]) +'%lf '%(ThandL[1,3]-ThandR[1,3]) +'%lf'%(ThandL[2,3]-ThandR[2,3]) +' direction0 %lf ' %(ThandR[0,3]-ThandL[0,3]) +'%lf '%(ThandR[1,3]-ThandL[1,3]) +'%lf'%(ThandR[2,3]-ThandL[2,3]) +' maxsteps 100')

   def graspAndMoveObject(self,T,name):
      mode='approach'
      print ('Moving to Grasping position for object: %s'%(name))
      self.planDualPath(name)
      self.WaitForController()
      time.sleep(1)

      mode='out'
      print ('Grasping body %s'%(name))
      self.graspObject()
      self.WaitForController()

      print ('Grabbing body %s'%(name))
      self.probsmanip.SendCommand('grabbody name %s'%(name))
      time.sleep(1)

      print ('Moving body %s out of the shelf'%(name))
      delta=array([.2,0.0,0.0])
      self.moveObject(name,delta[0],delta[1],delta[2])
      self.WaitForController()
      time.sleep(1)

      mode='move'
      print ('Moving body %s to final position'%(name))
      delta=array([-.20,-0.0,0])#change this to give a new position
      self.moveObject(name,delta[0],delta[1],delta[2])
      self.WaitForController()
      
      print ('Releasing body %s'%(name))
      self.probsmanip.SendCommand('releaseall')
      self.releaseObject()
      self.WaitForController()

      mode='return'
      print ('Returning to Starting position')
      self.MoveArmsToJointPosition(T)
      self.WaitForController()

      print ('Body %s successfully manipulated'%(name))

def run(args=None):
    """Executes the dualarmdemo_schunk demo
    @type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description="Schunk Manipulation planning example\nFor a dual arm robot with Schunk LWA3 arms, plan trajectories for grasping an object and manipulating it on a shelf.")
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/dualarmmanipulation.env.xml',
                      help='Scene file to load')   
    (options, args) = parser.parse_args(args=args)

    env = Environment()
    env.SetCollisionChecker(env.CreateCollisionChecker('ode'))
    env.SetViewer('qtcoin')
    env.Load(options.scene)
    schunk = Schunkplanner(env)
    time.sleep(1)
    
    try:
        T=array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])#Set initial position		
        schunk.robot.SetActiveDOFValues(T)
        time.sleep(1)
        name='Object1'
        schunk.robot.SetActiveManipulator(1)#Set left arm as the active manipulator	
        schunk.graspAndMoveObject(T,name)
        schunk.WaitForController()
        print "Path Planning complete...."
    finally:
        time.sleep(5)
        del schunk
        env.Destroy() # done with the environment

if __name__ == "__main__":
   run()
