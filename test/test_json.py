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

        if type(lo) in [int, str, long, dict]:
            assert lo == ro
        if type(lo) in [list, tuple]:
            for li in lo:
                for ri in ro:
                    _AssertEqual(li, ri)

        if isinstance(lo, np.ndarray):
            assert all(lo == ro)


    def _TestAttachedSensorInfo(self, lo, ro):
        _AssertEqual(newInfo._name, oldInfo._name)
        _AssertEqual(newInfo._linkname, oldInfo._linkname)
        _AssertEqual(newInfo._trelative, oldInfo._trelative)
        _AssertEqual(newInfo._sensorname, oldInfo._sensorname)

    def _TestAttachedSensorInfoArray(self, lo, ro):
        assert len(lo) == len(ro)
        for linfo in lo:
            found = False
            for rinfo in ro:
                if linfo._name == rinfo._name:
                    _TestAttachedSensorInfo(linfo, rinfo)
                    found = true
            if not found:
                assert False

    def _TestConnectedBodyInfo(self, lo, ro):
        _AssertEqual(lo._name, ro._name)
        _AssertEqual(lo._linkename, ro._linkname)
        _AssertEqual(lo._trelative, ro._trelative)
        _AssertEqual(lo._url, ro._url)

        _TestLinkInfoArray(lo._linkInfos, ro._linkInfos)
        _TestJointInfoArray(lo._jointInfos, ro._jointInfos)
        _TestManipulatorInfoArray(lo._manipulatorInfos, ro._manipulatorInfos)
        _TestAttachedSensorInfoArray(lo._attachedSensorInfos, ro._attachedSensorInfos)


    def _TestElectricMotorActuatorInfo(self, lo, ro):
        _AssertEqual(lo.model_type, ro.model_type)
        _AssertEqual(lo.assigned_power_rating, ro.assigned_power_rating)
        _AssertEqual(lo.max_speed, ro.max_speed)
        _AssertEqual(lo.no_load_speed, ro.no_load_speed)
        _AssertEqual(lo.stall_torque, ro.stall_torque)
        _AssertEqual(lo.max_instantaneous_torque, ro.max_instantaneous_torque)
        _AssertEqual(lo.nominal_speed_torque_points, ro.nominal_speed_torque_points)
        _AssertEqual(lo.max_speed_torque_points, ro.max_speed_torque_points)
        _AssertEqual(lo.nominal_torque, ro.nominal_torque)
        _AssertEqual(lo.rotor_inertia, ro.rotor_inertia)
        _AssertEqual(lo.torque_constant, ro.torque_constant)
        _AssertEqual(lo.nominal_voltage, ro.nominal_voltage)
        _AssertEqual(lo.speed_constant, ro.speed_constant)
        _AssertEqual(lo.starting_current, ro.starting_current)
        _AssertEqual(lo.terminal_resistance, ro.terminal_resistance)
        _AssertEqual(lo.gear_ratio, ro.gear_ratio)
        _AssertEqual(lo.coloumb_friction, ro.coloumb_friction)
        _AssertEqual(lo.viscous_friction, ro.viscous_friction)


    def _TestJointInfo(self, lo, ro):
        _AssertEqual(lo._type, ro._type)
        _AssertEqual(lo._name, ro._name)
        _AssertEqual(lo._linkname0, ro._linkname0)
        _AssertEqual(lo._linkname, ro._linkname)
        _AssertEqual(lo._vanchor, ro._vanchor)
        _AssertEqual(lo._vaxes, ro._vaxes)
        _AssertEqual(lo._vcurrentvalues, ro._vcurrentvalues)
        _AssertEqual(lo._vresolution, ro._vresolution)
        _AssertEqual(lo._vmaxvel, ro._vmaxvel)
        _AssertEqual(lo._vhardmaxvel, ro._vhardmaxvel)
        _AssertEqual(lo._vmaxaccel, ro._vmaxaccel)
        _AssertEqual(lo._vhardmaxaccel, ro._vhardmaxaccel)
        _AssertEqual(lo._vmaxjerk, ro._vmaxjerk)
        _AssertEqual(lo._vhardmaxjerk, ro._vhardmaxjerk)
        _AssertEqual(lo._vmaxtorque, ro._vmaxtorque)
        _AssertEqual(lo._vmaxinertia, ro._vmaxinertia)
        _AssertEqual(lo._vweights, ro._vweights)
        _AssertEqual(lo._voffsets, ro._voffsets)
        _AssertEqual(lo._vupperlimit, ro._vupperlimit)
        #TODO: _TestTrajectory(lo._trajfollow, ro._trajfollow)
        _TestMimicInfoArray(lo._vmimic, ro._vmimic)
        _AssertEqual(lo._mapFloatParameters, ro._mapFloatParameters)
        _AssertEqual(lo._mapIntParameters, ro._mapIntParameters)
        _AssertEqual(lo._mapStringParameters, ro._mapStringParameters)
        _TestElectricMotorActuatorInfo(lo._infoElectricMotor, ro._infoElectricMotor)
        _AssertEqual(lo._bIsCircular, ro._bIsCircular)
        _AssertEqual(lo._bIsActive, ro._bIsActive)

    def _TestMimicInfo(self, lo, ro):
        """ MimicInfo only serialize and deserialize in JointInfo as an arry of string.
        """
        _AssertEqual(lo._equations, ro._equations)

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
                    _TestAttachedSensorInfo(newInfo, oldInfo)
                    #TODO: sensorGeometry


    def test_ConnectedBodyInfo(self):
        self.log.info('test serialize/deserialize ConnectedBodyInfo')
        env = self.env
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                robot = self.LoadRobot(robotfile)
                connectedbodies = robot.GetConnectedBodies()
                if len(connectedbodies) == 0:
                    self.log.warn("ConnectedBodyInfo is empty in %s" % robotfile)
                for connectedbody in connectedbodies:
                    newInfo = ConnectedBodyInfo()
                    oldInfo = connectedbody.GetInfo()
                    newInfo.DeserializeJSON(oldInfo.SerializeJSON())
                    _TestConnectedBodyInfo(newInfo, oldInfo)

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

