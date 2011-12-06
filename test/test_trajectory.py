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

class TestTrajectory(EnvironmentSetup):
    def test_merging(self):
        env = self.env
        env.Load('robots/pr2-beta-static.zae')
        robot=env.GetRobots()[0]
        basemanip=interfaces.BaseManipulation(robot)
        manip1=robot.SetActiveManipulator('leftarm')
        Tgoal1 = manip1.GetTransform()
        Tgoal1[0,3] -= 0.3
        Tgoal1[2,3] += 0.4
        trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal1],execute=False,outputtraj=True)
        traj1=RaveCreateTrajectory(env,'').deserialize(trajdata)

        manip2=robot.SetActiveManipulator('rightarm')
        Tgoal2 = manip2.GetTransform()
        Tgoal2[0,3] -= 0.5
        Tgoal2[1,3] -= 0.5
        Tgoal2[2,3] += 0.2
        trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal2],execute=False,outputtraj=True)
        traj2=RaveCreateTrajectory(env,'').deserialize(trajdata)

        traj3=planningutils.MergeTrajectories([traj1,traj2])

        with robot:
            dofvalues=traj3.GetConfigurationSpecification().ExtractJointValues(traj3.GetWaypoint(-1),robot,range(robot.GetDOF()),0)
            robot.SetDOFValues(dofvalues)
            assert( transdist(manip1.GetTransform(),Tgoal1) <= g_epsilon)
            assert( transdist(manip2.GetTransform(),Tgoal2) <= g_epsilon)
            assert( abs(traj3.GetDuration() - max(traj1.GetDuration(),traj2.GetDuration())) <= g_epsilon )

    def test_grabbing(self):
        env = self.env
        with env:
            env.Load('robots/pr2-beta-static.zae')
            robot=env.GetRobots()[0]
            basemanip=interfaces.BaseManipulation(robot)
            manip1=robot.SetActiveManipulator('leftarm')
            Tgoal1 = manip1.GetTransform()
            Tgoal1[0,3] -= 0.3
            Tgoal1[2,3] += 0.4

        trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal1],execute=False,outputtraj=True)
        traj1=RaveCreateTrajectory(env,'').deserialize(trajdata)
        
        with env:
            body1=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.AddKinBody(body1,True)
            body1.SetTransform(manip1.GetTransform())
            
        newspec = traj1.GetConfigurationSpecification()
        newspec.AddGroup('grab %s %d'%(robot.GetName(),manip1.GetEndEffector().GetIndex()),1,'previous')
        graboffset = newspec.GetGroupFromName('grab').offset
        traj1grab = RaveCreateTrajectory(env,'')
        traj1grab.Init(newspec)
        data=traj1.GetWaypoints(0,traj1.GetNumWaypoints(),newspec)
        data[graboffset] = body1.GetEnvironmentId()
        data[-newspec.GetDOF()+graboffset] = -body1.GetEnvironmentId()
        traj1grab.Insert(0,data)

        # run the trajectory
        robot.GetController().SendCommand('SetThrowExceptions 1')
        env.StartSimulation(0.01,False)
        robot.GetController().SetPath(traj1grab)
        assert(robot.WaitForController(traj1grab.GetDuration()+1))
        assert(transdist(body1.GetTransform(),Tgoal1) <= g_epsilon )
        assert(len(robot.GetGrabbed())==0)

        # try with another arm
        with env:
            manip2=robot.SetActiveManipulator('rightarm')
            Tgoal2 = manip2.GetTransform()
            Tgoal2[0,3] -= 0.5
            Tgoal2[1,3] -= 0.5
            Tgoal2[2,3] += 0.2
            trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal2],execute=False,outputtraj=True)
            traj2=RaveCreateTrajectory(env,'').deserialize(trajdata)

        with env:
            body2=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.AddKinBody(body2,True)
            body2.SetTransform(manip2.GetTransform())

        newspec = traj2.GetConfigurationSpecification()
        newspec.AddGroup('grab %s %d'%(robot.GetName(),manip2.GetEndEffector().GetIndex()),1,'previous')
        graboffset = newspec.GetGroupFromName('grab').offset
        traj2grab = RaveCreateTrajectory(env,'')
        traj2grab.Init(newspec)
        data=traj2.GetWaypoints(0,traj2.GetNumWaypoints(),newspec)
        data[graboffset] = body2.GetEnvironmentId()
        data[-newspec.GetDOF()+graboffset] = -body2.GetEnvironmentId()
        traj2grab.Insert(0,data)
        
        traj3=planningutils.MergeTrajectories([traj1grab,traj2grab])

        # run the trajectory
        robot.GetController().SetPath(traj3)
        assert(robot.WaitForController(traj3.GetDuration()+1))
        assert(transdist(body1.GetTransform(),Tgoal1) <= g_epsilon )
        assert(transdist(body2.GetTransform(),Tgoal2) <= g_epsilon )
        assert(len(robot.GetGrabbed())==0)

    def test_grabonly(self):
        env = self.env
        with env:
            env.Load('robots/pr2-beta-static.zae')
            robot=env.GetRobots()[0]
            manip1=robot.SetActiveManipulator('leftarm')
            body1=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.AddKinBody(body1,True)
            body1.SetTransform(manip1.GetTransform())

            robot.GetController().SendCommand('SetThrowExceptions 1')
            env.StartSimulation(0.01,False)

            spec=ConfigurationSpecification()
            spec.AddGroup('grab %s %d'%(robot.GetName(),robot.GetActiveManipulator().GetEndEffector().GetIndex()),1,'previous')
            spec.AddGroup('deltatime',1,'linear')
            traj=RaveCreateTrajectory(self.env,'')
            traj.Init(spec)
            traj.Insert(0,[body1.GetEnvironmentId(),0])

        robot.GetController().SetPath(traj)
        assert(robot.WaitForController(0.1))
        assert(robot.GetGrabbed()[-1] == body1)
