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
try:
    import ultrajson as json
except ImportError as e:
    import json

import functools
import numpy as np
from common_test_openrave import *


class TestJSONSeralization(EnvironmentSetup):

    def _AssertEqual(self, lo, ro):
        if type(lo) != type(ro):
            assert False

        if type(lo) in [int, str, long]:
            assert lo == ro
        if type(lo) in [list, array]:
            for li in lo:
                for ri in ro:
                    _AssertEqual(li, ri)

        if isinstance(lo, np.ndarray):
            assert all(lo == ro)

    def test_AttachedSensorInfo(self):
        self.log.info('test serialize/deserialize AttachedSensorInfo')
        env = self.env
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                robot = self.LoadRobot(robotfile)
                attachedsensors = robot.GetAttachedSensors()
                if len(attachedsensors) == 0:
                    self.log.warn("AttachedSensorInfo is empty in %s" % robotfile)
                for attachedsensor in attachedsensors:
                    newInfo = AttachedSensorInfo()
                    oldInfo = attachedsensor.GetInfo()
                    newInfo.DeserializeJSON(oldInfo.SerializeJSON(), env)
                    _AssertEqual(newInfo._name, oldInfo._name)
                    _AssertEqual(newInfo._linkname, oldInfo._linkname)
                    _AssertEqual(newInfo._trelative, oldInfo._trelative)
                    _AssertEqual(newInfo._sensorname, oldInfo._sensorname)

    # # robot.h
    # def test_ManipulatorInfo(self):
    #     self.log.info('test serialize/deserialize ManipulatorInfo')
    #     env = self.env
    #     with env:
    #         for robotfile in g_robotfiles:
    #             env.Reset()
    #             robot = self.LoadRobot(robotfile)
    #             manips = robot.GetManipulators()
    #             for manip in manips:
    #                 emptyInfo =  ManipulatorInfo()
    #                 emptyInfo.DeserializeJSON(manip.GetInfo().SerializeJSON())
    #                 assert emptyInfo == manip.GetInfo()

    # def test_ConnectedBodyInfo(self):
    #     self.log.info('test serialize/deserialize ConnectedBodyInfo')
    #     env = self.env
    #     with env:
    #         for robotfile in g_robotfiles:
    #             env.Reset()
    #             robot = self.LoadRobot(robotfile)
    #             connectedbodies = robot.GetConnectedBodies()
    #             if len(connectedbodies) == 0:
    #                 self.log.warn("ConnectedBodyInfo is empty in %s" % robotfile)
    #             for connectedbody in connectedbodies:
    #                 emptyInfo = ConnectedBodyInfo()
    #                 emptyInfo.DeserializeJSON(connectedbody.GetInfo().SerializeJSON())
    #                 assert emptyInfo == connectedbody.GetInfO()

    # # trajectory.h
    # def test_TrajectoryBase(self):
    #     self.log.info('test serialize/deserialize Trajectory')
    #     env = self.env
    #     with env:
    #         trajxml = """
    #         <trajectory>
    #             <configuration>
    #                 <group name="string" offset="#OFF1" dof="#D1" interpolation="string"/>
    #                 <group name="string" offset="#OFF2" dof="#D2" interpolation="string"/>
    #             </configuration>
    #             <data count="6">
    #                 0 0 0 0 0 0 0
    #             </data>
    #             <description>My trajectory
    #             </description>
    #         </trajectory>
    #         """
    #         trajFromXML = RaveCreateTrajectory(env, '')
    #         trajFromXML.deserialize(trajxml)
    #         trajjson = traj.SerializeJSON()
    #         trajFromJSON = RaveCreateTrajectorY(env, 'json')
    #         trajFromJSON.DeserializeJSON(trajjson)
    #         assert trajFromXML == trajfromJSON

    # def test_GenericTrajectory(self):
    #     pass

    # # kinbody.h
    # def test_ElectricMotorActuatorInfo(self):
    #     pass

    # def test_GeometryInfo(self):
    #     pass

    # def test_LinkInfo(self):
    #     pass

    # def test_MimicInfo(self):
    #     pass

    # def test_JointInfo(self):
    #     pass

    # def test_GrabbedInfo(self):
    #     pass

