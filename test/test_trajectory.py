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
        self.LoadEnv('robots/pr2-beta-static.zae')
        robot=env.GetRobots()[0]
        basemanip=interfaces.BaseManipulation(robot)
        manip1=robot.SetActiveManipulator('leftarm')
        Tgoal1 = manip1.GetTransform()
        Tgoal1[0,3] -= 0.3
        Tgoal1[2,3] += 0.4
        traj1=basemanip.MoveToHandPosition(matrices=[Tgoal1],execute=False,outputtrajobj=True)

        manip2=robot.SetActiveManipulator('rightarm')
        Tgoal2 = manip2.GetTransform()
        Tgoal2[0,3] -= 0.5
        Tgoal2[1,3] -= 0.5
        Tgoal2[2,3] += 0.2
        traj2=basemanip.MoveToHandPosition(matrices=[Tgoal2],execute=False,outputtrajobj=True)
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
            self.LoadEnv('robots/pr2-beta-static.zae')
            robot=env.GetRobots()[0]
            originalvalues = robot.GetDOFValues()
            basemanip=interfaces.BaseManipulation(robot)
            manip1=robot.SetActiveManipulator('leftarm')
            Tgoal1 = manip1.GetTransform()
            Tgoal1[0,3] -= 0.3
            Tgoal1[2,3] += 0.4

        traj1=basemanip.MoveToHandPosition(matrices=[Tgoal1],execute=False,outputtrajobj=True)
        with env:
            body1=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(body1,True)
            body1.SetTransform(manip1.GetTransform())
            
        newspec = traj1.GetConfigurationSpecification()
        newspec.AddGroup('grab %s %d'%(robot.GetName(),manip1.GetEndEffector().GetIndex()),1,'previous')
        graboffset = newspec.GetGroupFromName('grab').offset
        traj1grab = RaveCreateTrajectory(env,'')
        traj1grab.Init(newspec)
        data=traj1.GetWaypoints(0,traj1.GetNumWaypoints(),newspec)
        data[graboffset] = body1.GetEnvironmentId()
        data[-newspec.GetDOF()+graboffset] = -body1.GetEnvironmentId() # release in the end
        traj1grab.Insert(0,data)

        # run the trajectory
        robot.GetController().SendCommand('SetThrowExceptions 1')
        env.StartSimulation(0.01,False)
        robot.GetController().SetPath(traj1grab)
        assert(robot.WaitForController(traj1grab.GetDuration()+1))
        with env:
            assert(transdist(body1.GetTransform(),Tgoal1) <= g_epsilon )
            assert(len(robot.GetGrabbed())==0)
        
        # try with another arm
        with env:
            manip2=robot.SetActiveManipulator('rightarm')
            Tgoal2 = manip2.GetTransform()
            Tgoal2[0,3] -= 0.5
            Tgoal2[1,3] -= 0.5
            Tgoal2[2,3] += 0.2
            traj2=basemanip.MoveToHandPosition(matrices=[Tgoal2],execute=False,outputtrajobj=True)
            # create body
            body2=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(body2,True)
            body2.SetTransform(manip2.GetTransform())
            # create traj
            newspec = traj2.GetConfigurationSpecification()
            newspec.AddGroup('grab %s %d'%(robot.GetName(),manip2.GetEndEffector().GetIndex()),1,'previous')
            graboffset = newspec.GetGroupFromName('grab').offset
            traj2grab = RaveCreateTrajectory(env,'')
            traj2grab.Init(newspec)
            data=traj2.GetWaypoints(0,traj2.GetNumWaypoints(),newspec)
            data[graboffset] = body2.GetEnvironmentId()
            data[-newspec.GetDOF()+graboffset] = -body2.GetEnvironmentId()
            traj2grab.Insert(0,data)
            
            # reset back to the original scene
            robot.GetController().Reset()
            robot.SetActiveManipulator(manip1)
            robot.Grab(body1)
            robot.SetDOFValues(originalvalues)
            robot.ReleaseAllGrabbed()
            # merge
            traj3=planningutils.MergeTrajectories([traj1grab,traj2grab])

        # run the trajectory
        robot.GetController().SetPath(traj3)
        assert(robot.WaitForController(0))#traj3.GetDuration()+1))
        with env:
            assert(transdist(body1.GetTransform(),Tgoal1) <= g_epsilon )
            assert(transdist(body2.GetTransform(),Tgoal2) <= g_epsilon )
            assert(len(robot.GetGrabbed())==0)

    def test_grabonly(self):
        env = self.env
        with env:
            self.LoadEnv('robots/pr2-beta-static.zae')
            robot=env.GetRobots()[0]
            manip1=robot.SetActiveManipulator('leftarm')
            body1=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(body1,True)
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

    def test_smoothingsamepoint(self):
        env = self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        assert(not env.CheckCollision(robot))
        plannernames=['parabolicsmoother','linearsmoother']
        smoothers=dict([(plannername,planningutils.ActiveDOFTrajectorySmoother(robot,plannername,'')) for plannername in plannernames])
        for delta in [0,1e-8,1e-9,1e-10,1e-11,1e-12,1e-13,1e-14,1e-15,1e-16]:
            self.log.debug('delta=%.16e',delta)
            traj = RaveCreateTrajectory(env,'')
            traj.Init(robot.GetActiveConfigurationSpecification('linear'))
            # get some values, hopefully non-zero
            basevalues = robot.GetActiveDOFValues()
            with robot:
                for delta in arange(0.1,0.5,0.1):
                    robot.SetActiveDOFValues(delta*ones(robot.GetActiveDOF()))
                    if not env.CheckCollision(robot):
                        basevalues = robot.GetActiveDOFValues()
                        break
            self.log.debug('smoothing')
            traj.Insert(0,basevalues)
            traj.Insert(1,basevalues+delta*ones(robot.GetActiveDOF()))
            for plannername in plannernames:
                ret=planningutils.SmoothActiveDOFTrajectory(traj,robot,maxvelmult=1,maxaccelmult=1,plannername=plannername)
                assert(ret==PlannerStatus.HasSolution)
                ret=planningutils.SmoothActiveDOFTrajectory(traj,robot,maxvelmult=1,maxaccelmult=1,plannername=plannername)
                assert(ret==PlannerStatus.HasSolution)
                assert(smoothers[plannername].PlanPath(traj)==PlannerStatus.HasSolution)
                assert(smoothers[plannername].PlanPath(traj)==PlannerStatus.HasSolution)
                self.RunTrajectory(robot,traj)
                self.RunTrajectory(robot,RaveCreateTrajectory(env,traj.GetXMLId()).deserialize(traj.serialize(0)))
            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,robot.GetActiveDOFValues())
            traj.Insert(1,robot.GetActiveDOFValues()+delta*ones(robot.GetActiveDOF()))
            self.log.debug('retiming')
            for plannername in ['parabolictrajectoryretimer', 'lineartrajectoryretimer']:
                ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,False,maxvelmult=1,maxaccelmult=1,plannername=plannername)
                assert(ret==PlannerStatus.HasSolution)
                ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,False,maxvelmult=1,maxaccelmult=1,plannername=plannername)
                assert(ret==PlannerStatus.HasSolution)
                self.RunTrajectory(robot,traj)
                self.RunTrajectory(robot,RaveCreateTrajectory(env,traj.GetXMLId()).deserialize(traj.serialize(0)))
                
    def test_smoothing(self):
        env = self.env
        self.LoadEnv('data/katanatable.env.xml')
        robot=env.GetRobots()[0]
        with env:
            robot.SetActiveDOFs(range(5))
            traj = RaveCreateTrajectory(env,'')
            origvalues = robot.GetActiveDOFValues()

            lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
            if not lmodel.load():
                lmodel.autogenerate()
            lmodel.setRobotWeights()
            lmodel.setRobotResolutions(xyzdelta=0.001) # the pegs are really thin

            smoother = planningutils.ActiveDOFTrajectorySmoother(robot,'','')
            # this fails by grazing an obstacle
#             traj.Init(robot.GetActiveConfigurationSpecification())
#             traj.Insert(0,origvalues)
#             traj.Insert(1,[2.437978870810338, -0.4588760200376995, -1.425257530151645, 1.257459103400449, 2.274410109574347])
#             planningutils.SmoothActiveDOFTrajectory(traj,robot)
#             self.RunTrajectory(robot,traj)

            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,[2.437978870810338, -0.4588760200376995, -1.425257530151645, 1.257459103400449, 2.274410109574347, 2.437978870810338, -0.4588760200376995, -1.425257530151644, 1.257459103400449, 2.274410109574348])
            trajclone = RaveClone(traj,0)
            ret=planningutils.SmoothActiveDOFTrajectory(traj,robot)
            assert(ret==PlannerStatus.HasSolution)
            assert(smoother.PlanPath(trajclone) == PlannerStatus.HasSolution)
            self.RunTrajectory(robot,traj)

            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,origvalues)
            traj.Insert(1,[ 2.299995  , -0.43290472, -1.34459131,  1.18628988,  2.14568385])
            trajclone = RaveClone(traj,0)
            ret=planningutils.SmoothActiveDOFTrajectory(traj,robot)
            assert(ret==PlannerStatus.HasSolution)
            assert(smoother.PlanPath(trajclone) == PlannerStatus.HasSolution)
            self.RunTrajectory(robot,traj)

            # test resampling
            spec = traj.GetConfigurationSpecification()
            stepsize=0.1
            curtime = 0
            trajdata = traj.Sample(0)
            for i in range(traj.GetNumWaypoints()-1):
                print curtime
                start = traj.GetWaypoint(i)
                end = traj.GetWaypoint(i+1)
                enddeltatime = spec.ExtractDeltaTime(end)
                if enddeltatime > stepsize:
                    for t in reversed(arange(curtime+enddeltatime,curtime,-stepsize)):
                        print t
                        newdata = traj.Sample(t)
                        spec.InsertDeltaTime(newdata,t-curtime)
                        curtime = t
                        trajdata = r_[trajdata,newdata]
                else:
                    trajdata = r_[trajdata,end]
                    curtime += spec.ExtractDeltaTime(end)

            traj2 = RaveCreateTrajectory(env,traj.GetXMLId())
            traj2.Init(spec)
            traj2.Insert(0,trajdata)

            assert(abs(traj2.GetDuration()-traj.GetDuration()) <= g_epsilon)
            for t in arange(0,traj.GetDuration(),stepsize*0.5):
                data1 = traj.Sample(t)
                data2 = traj2.Sample(t)
                assert( transdist(data1,data2) <= g_epsilon)

    def test_simpleretiming(self):
        env=self.env
        env.Load('robots/barrettwam.robot.xml')
        with env:
            robot=env.GetRobots()[0]
            robot.SetActiveDOFs(range(7))
            finalvalues = numpy.minimum(0.5,robot.GetActiveDOFLimits()[1])
            parameters = Planner.PlannerParameters()
            parameters.SetRobotActiveJoints(robot)
            retimer = planningutils.ActiveDOFTrajectoryRetimer(robot,'parabolictrajectoryretimer','')
            traj = RaveCreateTrajectory(env,'')
            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,zeros(robot.GetActiveDOF()))
            traj.Insert(1,finalvalues)
            traj2 = RaveClone(traj,0)
            try:
                ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,True,maxvelmult=1,maxaccelmult=1,plannername='parabolictrajectoryretimer')
                assert(ret==PlannerStatus.Failed)
                
            except openrave_exception,e:
                pass
            
            ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,False,maxvelmult=1,maxaccelmult=1,plannername='parabolictrajectoryretimer')
            assert(ret==PlannerStatus.HasSolution)
            planningutils.VerifyTrajectory(parameters,traj,samplingstep=0.002)
            self.RunTrajectory(robot,traj)

            assert(retimer.PlanPath(traj2,False)==PlannerStatus.HasSolution)
            assert(abs(traj.GetDuration()-traj2.GetDuration()) <= g_epsilon)
            
            self.log.info('try with timestamps')
            spec = robot.GetActiveConfigurationSpecification()
            timeoffset=spec.AddDeltaTimeGroup()
            traj = RaveCreateTrajectory(env,'')
            traj.Init(spec)
            traj.Insert(0,r_[zeros(robot.GetActiveDOF()),finalvalues],robot.GetActiveConfigurationSpecification())
            assert(traj.GetNumWaypoints()==2)
            assert(traj.GetDuration()==0)
            g=ConfigurationSpecification.Group()
            g.name='deltatime'
            g.dof=1
            traj.Insert(1,[3.0],g,True)
            assert(traj.GetNumWaypoints()==2 and traj.GetDuration()==3)
            traj2.Clone(traj,0)
            ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,True,maxvelmult=1,maxaccelmult=1,plannername='parabolictrajectoryretimer')
            assert(ret==PlannerStatus.HasSolution)
            assert(traj.GetDuration()==3)
            planningutils.VerifyTrajectory(parameters,traj,samplingstep=0.002)
            self.RunTrajectory(robot,traj)
            assert( transdist(robot.GetActiveDOFValues(),finalvalues) <= g_epsilon)

            assert(retimer.PlanPath(traj2,True)==PlannerStatus.HasSolution)
            assert( abs(traj.GetDuration()-traj2.GetDuration()) <= g_epsilon)
            planningutils.VerifyTrajectory(parameters,traj2,samplingstep=0.002)

            robot.SetActiveDOFs(range(7,11))
            traj = RaveCreateTrajectory(env,'')
            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,[0, -2.220446049250314e-16, 0, 1.047197551200003, 0.5, 0.5, 0.5, 1.0471975512])
            ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,False,maxvelmult=1,maxaccelmult=1,plannername='parabolictrajectoryretimer',plannerparameters='<multidofinterp>1</multidofinterp>')
            assert(ret==PlannerStatus.HasSolution)
            
    def test_ikparamretiming(self):
        self.log.info('retime workspace ikparam')
        env=self.env
        env.Load('robots/barrettwam.robot.xml')
        with env:
            robot=env.GetRobots()[0]
            manip = robot.GetActiveManipulator()
            robot.SetActiveDOFs(manip.GetArmIndices())
            robot.SetActiveDOFValues(zeros(len(manip.GetArmIndices())))
            ikparam0 = manip.GetIkParameterization(IkParameterizationType.Transform6D)
            robot.SetActiveDOFValues(ones(len(manip.GetArmIndices())))
            ikparam1 = manip.GetIkParameterization(ikparam0.GetType())
            assert(ikparam0.GetNumberOfValues()==7 and ikparam0.GetDOF()==6)
            assert(ikparam1.GetNumberOfValues(ikparam1.GetType())==7 and ikparam1.GetDOF(ikparam1.GetType())==6)
            rotaxis = axisAngleFromRotationMatrix(dot(linalg.inv(ikparam0.GetTransform6D()),ikparam1.GetTransform6D()))
            traj = RaveCreateTrajectory(env,'')
            plannerinfo = [('LinearTrajectoryRetimer',1,1.9851930205965249), ('ParabolicTrajectoryRetimer', 2,2.2014092368127409)]
            maxvel = 0.8
            maxaccel = 3.7
            sampletime = 0.005
            plannerparameters='<multidofinterp>1</multidofinterp>'
            for plannername, degree, expectedduration in plannerinfo:
                retimer = planningutils.AffineTrajectoryRetimer(plannername,plannerparameters)
                traj.Init(ikparam0.GetConfigurationSpecification())
                traj.Insert(0,ikparam0.GetValues())
                traj.Insert(1,ikparam1.GetValues())
                trajclone = RaveClone(traj,0)
                maxvelocities=tile(maxvel,ikparam0.GetNumberOfValues())
                maxaccelerations=tile(maxaccel,ikparam0.GetNumberOfValues())
                ret=planningutils.RetimeAffineTrajectory(traj,maxvelocities=maxvelocities,maxaccelerations=maxaccelerations,hastimestamps=False,plannername=plannername,plannerparameters=plannerparameters)
                assert(ret==PlannerStatus.HasSolution)
                assert(abs(traj.GetDuration()-expectedduration) < 0.01)
                assert(retimer.PlanPath(trajclone,maxvelocities,maxaccelerations,False)==PlannerStatus.HasSolution)
                assert(abs(traj.GetDuration()-trajclone.GetDuration()) < g_epsilon)
                gvel = traj.GetConfigurationSpecification().GetGroupFromName('ikparam_velocities')
                assert(gvel is not None)
                ikparams = []
                angledelta = []
                transdelta = []
                times = arange(0,traj.GetDuration()+sampletime,sampletime)
                for t in times:
                    ikparam = IkParameterization()
                    ikparam.SetValues(traj.Sample(t,ikparam0.GetConfigurationSpecification()),ikparam0.GetType())
                    quatvelocity = traj.Sample(t,gvel)[0:4]
                    # quatvelocity = 0.5 * angularvelocity * quat
                    angularvelocity = 2*quatMultiply(quatvelocity, quatInverse(ikparam.GetValues()[0:4]))[1:4]
                    assert( abs( abs(dot(rotaxis,angularvelocity))- linalg.norm(rotaxis)*linalg.norm(angularvelocity)) <= g_epsilon)
                    ikparams.append(ikparam)
                    # compute the change in angle, also check if valid transform can be extracted
                    T=ikparam.GetTransform6D()
                    angledelta.append(linalg.norm(axisAngleFromRotationMatrix(dot(linalg.inv(ikparam0.GetTransform6D()),T))))
                    transdelta.append(linalg.norm(T[0:3,3]-ikparam0.GetTransform6D()[0:3,3]))
                angledelta = array(angledelta)
                transdelta = array(transdelta)
                assert(ikparam0.ComputeDistanceSqr(ikparams[0]) <= g_epsilon)
                assert(ikparam1.ComputeDistanceSqr(ikparams[-1]) <= g_epsilon)
                gvel=traj.GetConfigurationSpecification().GetGroupFromName('ikparam_velocities')
                assert(sum(abs(traj.GetWaypoint(0,gvel)))<=g_epsilon)
                if degree == 1:
                    p, residuals, rank, singular_values, rcond = polyfit(times,c_[angledelta,transdelta],degree,full=True)
                    assert(all(residuals < 5e-5)) # make sure it matches the degree
                elif degree == 2:
                    assert(sum(abs(traj.GetWaypoint(-1,gvel)))<=g_epsilon)
                    startindex = int(round(maxvel/(maxaccel*sampletime)))
                    endindex = len(times)-startindex
                    p, residuals, rank, singular_values, rcond = polyfit(times[:startindex],c_[angledelta[:startindex],transdelta[:startindex]],2,full=True)
                    assert(all(residuals < 5e-5)) # make sure it matches the degree
                    p, residuals, rank, singular_values, rcond = polyfit(times[startindex:endindex],c_[angledelta[startindex:endindex],transdelta[startindex:endindex]],1,full=True)
                    assert(all(residuals < 5e-5)) # make sure it matches the degree
                    p, residuals, rank, singular_values, rcond = polyfit(times[endindex:],c_[angledelta[endindex:],transdelta[endindex:]],2,full=True)
                    assert(all(residuals < 5e-5)) # make sure it matches the degree
                    
    def drawhandles():
        handles = [misc.DrawAxes(env,ikparam.GetTransform6D()) for ikparam in ikparams]
        import matplotlib.pyplot as plt
        ax = plt.subplot(121)
        ax.plot(times, angledelta,'r')
        ax.plot(times, transdelta,'b')
        ax = plt.subplot(122)
        deltatimes = times[1:]-times[:-1]
        ax.plot(times[1:], (angledelta[1:]-angledelta[:-1])/deltatimes,'r')
        ax.plot(times[1:], (transdelta[1:]-transdelta[:-1])/deltatimes,'b')
        plt.show()
        
    def test_smoothwithcircular(self):
        self.log.info('test smoothing with circular joints')
        env=self.env
        self.LoadEnv('data/pa10calib.env.xml')
        robot=env.GetRobots()[0]
        traj=RaveCreateTrajectory(env,'')
        trajxml = """<trajectory>
<configuration>
<group name="joint_values PA10 0 1 2 3 4 5 6" offset="0" dof="7" interpolation="linear"/>
</configuration>

<data count="49">
0.6232796600868203 -0.8112710061292812 0.7999999999999998 0.6799631064508246 0.1091755354059138 1.440731827698615 2.922283401707198 0.6297659060914433 -0.791896844803094 0.8178285806447316 0.6926179013352842 0.08766435149958259 1.427920262546699 2.913468581341383 1.503972143656972 -0.9274864371467325 1.225573917311743 1.003437877648504 -0.4340282761288557 1.823200101638587 3.098016069378392 1.525185021612136 -0.9331519020556411 1.234868889346184 1.014691108733368 -0.4468302042491505 1.833329603939712 3.103930219009325 1.5463978995673 -0.9388173669645494 1.244163861380624 1.025944339818231 -0.4596321323694453 1.843459106240837 3.109844368640257 1.567610777522464 -0.9444828318734579 1.253458833415064 1.037197570903095 -0.4724340604897401 1.853588608541962 3.11575851827119 1.588823655477628 -0.9501482967823666 1.262753805449504 1.048450801987958 -0.4852359886100349 1.863718110843087 3.121672667902123 1.610036533432792 -0.955813761691275 1.272048777483944 1.059704033072821 -0.4980379167303295 1.873847613144212 3.127586817533055 1.631249411387956 -0.9614792266001836 1.281343749518385 1.070957264157685 -0.5108398448506243 1.883977115445337 3.133500967163988 1.65246228934312 -0.9671446915090921 1.290638721552825 1.082210495242548 -0.523641772970919 1.894106617746462 3.139415116794921 1.673675167298284 -0.9728101564180006 1.299933693587265 1.093463726327412 -0.5364437010912138 1.904236120047587 -3.137856040753733 1.694888045253446 -0.9784756213269096 1.309228665621706 1.104716957412276 -0.5492456292115089 1.914365622348711 3.151243416056785 1.768477491729069 -0.858623477800765 1.222218154809126 0.9641516389871772 -0.6920262133927459 1.904373843952331 3.255188040566058 1.778990269797015 -0.8415017430113158 1.209788081835901 0.944070879212163 -0.7124234397043513 1.902946447038562 3.270037272638811 1.789503047864962 -0.8243800082218665 1.197358008862675 0.9239901194371489 -0.7328206660159566 1.901519050124793 3.284886504711565 1.780997116546671 -0.8173747151353777 1.177261052644005 0.9223185059169084 -0.7400461535176325 1.897779926914885 -2.981038652082886 1.772491185228381 -0.810369422048889 1.157164096425336 0.9206468923966682 -0.747271641019308 1.894040803704977 -2.96377850169775 1.763985253910091 -0.8033641289624004 1.137067140206666 0.918975278876428 -0.7544971285209835 1.890301680495069 -2.946518351312615 1.755479322591801 -0.7963588358759117 1.116970183987996 0.9173036653561878 -0.7617226160226591 1.886562557285161 -2.929258200927479 1.74697339127351 -0.789353542789423 1.096873227769326 0.9156320518359472 -0.7689481035243347 1.882823434075253 -2.911998050542343 0.6667201138506444 0.1003186791946383 -1.455440212001736 0.7033371347654194 -1.686585016237153 1.407954786416909 -0.7199589516301085 0.6582141825323542 0.107323972281127 -1.475537168220406 0.7016655212451789 -1.693810503738829 1.404215663207 -0.7026988012449729 0.6497082512140638 0.1143292653676157 -1.495634124439075 0.6999939077249385 -1.701035991240505 1.400476539997092 -0.6854386508598372 0.6412023198957735 0.1213345584541043 -1.515731080657745 0.698322294204698 -1.70826147874218 1.396737416787184 -0.6681785004747015 0.6326963885774831 0.128339851540593 -1.535828036876415 0.696650680684458 -1.715486966243856 1.392998293577275 -0.6509183500895659 0.6241904572591926 0.1353451446270817 -1.555924993095084 0.6949790671642179 -1.722712453745532 1.389259170367367 -0.63365819970443 0.6156845259409023 0.1423504377135704 -1.576021949313754 0.6933074536439776 -1.729937941247208 1.385520047157459 -0.6163980493192943 0.6071785946226118 0.1493557308000591 -1.596118905532424 0.6916358401237372 -1.737163428748884 1.381780923947551 -0.5991378989341586 0.5986726633043216 0.1563610238865478 -1.616215861751093 0.689964226603497 -1.744388916250559 1.378041800737643 -0.581877748549023 0.5901667319860312 0.1633663169730365 -1.636312817969763 0.6882926130832564 -1.751614403752235 1.374302677527735 -0.5646175981638875 0.5816608006677335 0.1703716100595274 -1.656409774188434 0.6866209995630174 -1.75883989125391 1.370563554317825 5.735827859400842 -0.42814069596912 -0.4649467545775149 -1.751470087760962 0.03704785629781469 -1.778588776721555 0.625614420268662 5.15291743426412 -0.4158048805137168 -0.4872259586354415 -1.74684719374974 0.05051853910319864 -1.760254377573389 0.6055420435706921 5.156013190933851 -0.4174817282842055 -0.49483178664785 -1.72843953979616 0.05593516530178318 -1.740200278912081 0.6033089479882394 -1.142929810431415 -0.4191585760546943 -0.5024376146602585 -1.710031885842581 0.06135179150036778 -1.720146180250772 0.6010758524057866 -1.158687504617093 -0.4208354238251831 -0.5100434426726669 -1.691624231889002 0.06676841769895242 -1.700092081589464 0.598842756823334 -1.174445198802771 -0.4225122715956719 -0.5176492706850755 -1.673216577935423 0.07218504389753716 -1.680037982928156 0.5966096612408814 -1.190202892988449 -0.4241891193661606 -0.5252550986974842 -1.654808923981843 0.07760167009612177 -1.659983884266848 0.5943765656584287 -1.205960587174127 -0.543245311070862 -1.065268887578487 -0.3478654932777039 0.462182130195625 -0.2361428793139618 0.4358267793042803 -2.324756874357273 -0.5449221588413509 -1.072874715590896 -0.3294578393241245 0.4675987563942096 -0.2160887806526537 0.4335936837218275 -2.340514568542951 -0.5465990066118396 -1.080480543603304 -0.3110501853705449 0.4730153825927942 -0.1960346819913455 0.4313605881393751 -2.356272262728629 -0.5482758543823283 -1.088086371615713 -0.2926425314169657 0.4784320087913787 -0.1759805833300373 0.4291274925569223 -2.372029956914307 -0.5499527021528171 -1.095692199628121 -0.2742348774633862 0.4838486349899631 -0.1559264846687291 0.4268943969744696 -2.387787651099985 -0.5516295499233055 -1.10329802764053 -0.2558272235098042 0.4892652611885498 -0.1358723860074225 0.4246613013920161 3.879639961893922 -0.5486519599153192 -1.095411164822057 -0.2698588952376889 0.4764586768093309 -0.1095847329118121 0.4160472363161167 3.85957028218165 -0.545674369907333 -1.087524302003584 -0.2838905669655735 0.463652092430112 -0.08329707981620163 0.4074331712402174 3.839500602469378 -0.5426967798993466 -1.07963743918511 -0.2979222386934581 0.4508455080508931 -0.05700942672059117 0.398819106164318 3.819430922757106 -0.4306048612859366 -1.331235863773695 -0.6 0.9649208887187276 1.61258423001619 1.843347204006895 2.572631741801914 -0.4306048612859366 -1.331235863773695 -0.6 0.9649208887187276 1.61258423001619 1.843347204006895 2.572631741801914 </data>
<description>Not documented yet.</description>
</trajectory>
"""
        traj.deserialize(trajxml)
        with env:
            circularindices = [j.GetDOFIndex() for j in robot.GetJoints() if j.IsCircular(0)]
            assert(len(circularindices)>0)
            robot.SetActiveDOFs(circularindices)
            spec = robot.GetActiveConfigurationSpecification()
            parameters = Planner.PlannerParameters()
            parameters.SetRobotActiveJoints(robot)
            startconfig = traj.GetWaypoint(0,spec)
            endconfig = traj.GetWaypoint(-1,spec)
            # discontinuity happens between 47 and 48, double check
            data=traj.GetWaypoints(0,traj.GetNumWaypoints(),spec)
            disindices = flatnonzero(abs(data[1:]-data[0:-1])>1)
            disindex = disindices[0]
            assert(abs(traj.GetWaypoint(disindex,spec)-traj.GetWaypoint(disindex+1,spec)) > 4)
            plannernames = ['parabolictrajectoryretimer','lineartrajectoryretimer']
            for plannername in plannernames:
                self.log.debug('planner=%s',plannername)
                traj2 = RaveClone(traj,0)
                ret=planningutils.RetimeActiveDOFTrajectory(traj2,robot,False,maxvelmult=1,maxaccelmult=1,plannername=plannername)
                assert(ret==PlannerStatus.HasSolution)
                assert(traj2.GetDuration()<20)
                with robot:
                    planningutils.VerifyTrajectory(parameters,traj2,samplingstep=0.002)
                assert(transdist(traj2.GetWaypoint(0,spec),startconfig) <= g_epsilon)
                assert(transdist(traj2.GetWaypoint(-1,spec),endconfig) <= g_epsilon)
                # make sure discontinuity is covered
                timespec = ConfigurationSpecification()
                timespec.AddGroup('deltatime',1,'linear')
                times = cumsum(traj2.GetWaypoints(0,traj2.GetNumWaypoints(),timespec))
                for i in range(0,len(times)-1):
                    v0 = traj2.Sample(times[i],spec)
                    v1 = traj2.Sample((times[i]+times[i+1])/2,spec)
                    v2 = traj2.Sample(times[i+1],spec)
                    # care about distance centered at 0, add/subtract midpoint
                    dist01 = fmod((v0-v1)+3*pi,2*pi)-pi
                    dist02 = fmod((v0-v2)+3*pi,2*pi)-pi
                    assert(all(abs(dist01)<=abs(dist02)))
                self.RunTrajectory(robot,traj2)

    def test_affine_smoothing(self):
        env=self.env
        robot=self.LoadRobot('robots/barrettwam.robot.xml')
        robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y |DOFAffine.RotationAxis, [0,0,1])
        with env:
            allwaypoints = [[1,0,1,  1,0,1], [0,0,0,  1,0,0.7,  1,0,-5.58,  1,0,-3.2]]
            for waypoints in allwaypoints:
                traj=RaveCreateTrajectory(env,'')
                traj.Init(robot.GetActiveConfigurationSpecification())
                traj.Insert(0,waypoints)
                traj2=RaveClone(traj,0)

                for itraj in range(2):
                    T=robot.GetTransform()
                    ret=planningutils.RetimeAffineTrajectory(traj2,[2,2,1],[5,5,5],False,plannername='lineartrajectoryretimer')
                    assert(ret==PlannerStatus.HasSolution)
                    assert(transdist(robot.GetTransform(),T) <= g_epsilon)
                    assert(traj2.GetNumWaypoints()==traj.GetNumWaypoints())
                    for i in range(traj.GetNumWaypoints()):
                        waypoint0=traj.GetWaypoint(i,robot.GetActiveConfigurationSpecification())
                        waypoint1=traj2.GetWaypoint(i,robot.GetActiveConfigurationSpecification())
                        assert(transdist(waypoint0,waypoint1) <= g_epsilon)
                    self.RunTrajectory(robot,traj2)
                    self.RunTrajectory(robot,RaveCreateTrajectory(env,traj2.GetXMLId()).deserialize(traj2.serialize(0)))

                plannernames = ['parabolicsmoother','linearsmoother']
                for plannername in plannernames:
                    traj2=RaveClone(traj,0)
                    for itraj in range(2):
                        T=robot.GetTransform()
                        ret=planningutils.SmoothAffineTrajectory(traj2,[2,2,1],[5,5,5],plannername=plannername)
                        assert(ret==PlannerStatus.HasSolution)
                        assert(transdist(robot.GetTransform(),T) <= g_epsilon)
                        for i in [0,-1]:
                            waypoint0=traj.GetWaypoint(i,robot.GetActiveConfigurationSpecification())
                            waypoint1=traj2.GetWaypoint(i,robot.GetActiveConfigurationSpecification())
                            assert(transdist(waypoint0,waypoint1) <= g_epsilon)
                        self.RunTrajectory(robot,traj2)
                        self.RunTrajectory(robot,RaveCreateTrajectory(env,traj2.GetXMLId()).deserialize(traj2.serialize(0)))

    def test_overwritetraj(self):
        env=self.env
        trajspec = ConfigurationSpecification()
        trajspec.AddGroup('joint_values',3,'linear')
        trajspec.AddGroup('customgroup',1,'previous')
        traj = RaveCreateTrajectory(env,'')
        traj.Init(trajspec)
        orgpoints = [ 11.,  12.,  13.,   0.,  21.,  22.,  23.,   0.]
        traj.Insert(0,orgpoints)
        assert(traj.GetNumWaypoints()==2)
        
        blendspec = ConfigurationSpecification()
        blendspec.AddGroup('customgroup',1,'previous')        
        traj.Insert(0,[55,56],blendspec,True)
        assert(traj.GetNumWaypoints()==2)
        orgpoints[3] = 55
        orgpoints[7] = 56
        newwaypoints = traj.GetWaypoints(0,2)
        assert(transdist(orgpoints,newwaypoints) <= g_epsilon)
        
        g = blendspec.GetGroupFromName('customgroup')
        blendspec = ConfigurationSpecification(g)
        assert(traj.GetWaypoint(0,g)==55)
        assert(traj.GetWaypoint(1,ConfigurationSpecification(g))==56)

    def test_robotdoortraj(self):
        env=self.env
        self.LoadEnv('data/wam_cabinet.env.xml')
        robot=env.GetRobot('BarrettWAM')
        door=env.GetRobot('Cabinet')
        xml="""<trajectory>
<configuration>
<group name="deltatime" offset="16" dof="1" interpolation=""/>
<group name="joint_velocities BarrettWAM 0 1 2 3 4 5 6" offset="8" dof="7" interpolation="linear"/>
<group name="joint_velocities Cabinet 0" offset="15" dof="1" interpolation="linear"/>
<group name="joint_values BarrettWAM 0 1 2 3 4 5 6" offset="0" dof="7" interpolation="quadratic"/>
<group name="joint_values Cabinet 0" offset="7" dof="1" interpolation="quadratic"/>
<group name="iswaypoint" offset="17" dof="1" interpolation="next"/>
</configuration>

<data count="15">
-0.4796659160759716 1.381855978887731 0 1.556581126034964 -2.987873188394926 1.328643520719891 -1.601720209477435 0 0 0 0 0 0 0 0 0 0 1 -0.4796659160759716 1.381855978887731 0 1.556581126034964 -2.987873188394926 1.328643520719891 -1.601720209477435 0 0 0 0 0 0 0 0 0 0 1 -0.4807301269856455 1.381934947294078 0.0006828032483823076 1.558706027222438 -2.991323332345269 1.329205496760741 -1.612686487877435 0.002731212993529221 -0.1016244184180574 0.007540909696978189 0.06520275481114474 0.2029126420429954 -0.3294637080160062 0.05366463338900882 -1.0472 0.2608110192445781 0.020944 0 -0.5930439061785613 1.390269047304727 0.07274390746551115 1.782962028177999 -3.355441668570001 1.388514842671358 -1.81713483322047 0.2909756298620436 -1.048935048067039 0.0778348806182543 0.6730021762149523 2.0944 -3.400620006330173 0.5539093426526105 -1.0472 2.692008704859799 0.1952333320693609 0 -0.6516910056024267 1.394620880138109 0.1103721911363543 1.88790106734493 -3.545574031847927 1.419484516485022 -1.869604352803936 0.4414887645454156 -1.292052367263704 0.09587508963860639 0.8289875112425256 2.0944 -4.1888 0.6822918909446598 -1.0472 3.315950044970091 0.05010458325388259 0 -0.6744897749612386 1.396312633592774 0.1249999999999996 1.923708085004249 -3.617188067166564 1.431523823753951 -1.887507861633595 0.4999999999999964 -1.375008207252988 0.1020307214044164 0.8822124091476999 2.0944 -4.1888 0.7260982399481899 -1.0472 3.528849636590787 0.01709655159440343 0 -0.6744897749612393 1.396312633592774 0.1250000000000001 1.92370808500425 -3.617188067166567 1.431523823753951 -1.887507861633595 0.4999999999999984 -1.375008207252991 0.1020307214044162 0.8822124091477017 2.0944 -4.1888 0.7260982399481913 -1.0472 3.528849636590794 5.551115123125783e-16 0 -0.6744897749612399 1.396312633592774 0.1250000000000003 1.923708085004251 -3.617188067166568 1.431523823753952 -1.887507861633596 0.4999999999999994 -1.375008207252992 0.1020307214044161 0.8822124091477008 2.0944 -4.1888 0.726098239948192 -1.0472 3.528849636590798 2.775557561562891e-16 0 -0.6744897749612401 1.396312633592774 0.1250000000000004 1.923708085004251 -3.617188067166568 1.431523823753952 -1.887507861633596 0.4999999999999999 -1.375008207252991 0.1020307214044161 0.8822124091477003 2.0944 -4.1888 0.7260982399481924 -1.0472 3.5288496365908 1.665334536937735e-16 0 -0.6744897749612407 1.396312633592774 0.1250000000000009 1.923708085004252 -3.617188067166571 1.431523823753952 -1.887507861633597 0.500000000000002 -1.375008207252989 0.1020307214044159 0.8822124091476986 2.0944 -4.1888 0.7260982399481938 -1.0472 3.528849636590793 5.551115123125783e-16 0 -0.6972885443200532 1.398004387047438 0.1396278088636465 1.959515102663572 -3.68880210248521 1.443563131022882 -1.905411370463256 0.5585112354545843 -1.292052367263703 0.09587508963860572 0.828987511242523 2.0944 -4.188800000000001 0.6822918909446627 -1.0472 3.315950044970091 0.01709655159440382 0 -0.7559356437439184 1.402356219880821 0.1772560925344896 2.064454141830503 -3.878934465763136 1.474532804836545 -1.957880890046722 0.7090243701379564 -1.048935048067038 0.07783488061825361 0.6730021762149495 2.0944 -3.400620006330174 0.5539093426526134 -1.0472 2.6920087048598 0.05010458325388262 0 -0.8682494229368339 1.410690319891469 0.2493171967516178 2.288710142786064 -4.243052801987867 1.533842150747163 -2.162329235389757 0.9972687870064708 -0.101624418418056 0.007540909696977516 0.06520275481114221 0.2029126420429962 -0.3294637080160079 0.05366463338901174 -1.047200000000002 0.2608110192445791 0.1952333320693609 0 -0.8693136338465078 1.410769288297816 0.2500000000000001 2.290835043973538 -4.246502945938211 1.534404126788013 -2.173295513789757 1 1.776068212215671e-15 -6.995804509340939e-16 -2.775557561562891e-15 4.379099900964414e-17 -5.734408659227547e-16 2.728325028375593e-15 1.487867679846236e-15 1.26161707343746e-16 0.02094400000000007 1 -0.8693136338465078 1.410769288297816 0.2500000000000001 2.290835043973538 -4.246502945938211 1.534404126788013 -2.173295513789757 1 0 0 0 0 0 0 0 0 5.551115123125783e-17 1 </data>
<description>Not documented yet.</description>
</trajectory>
"""
        traj=RaveCreateTrajectory(env,'').deserialize(xml)

        with env:
            manip=robot.GetActiveManipulator()
            trajvalues=traj.Sample(0,robot.GetActiveConfigurationSpecification())
            robotvalues = robot.GetDOFValues()
            robotvalues[manip.GetArmIndices()] = [-0.4796659160759716, 1.381855978887731, 0, 1.556581126034964, -2.987873188394926, 1.328643520719891, -1.601720209477435]
            assert(transdist(robotvalues,trajvalues) <= g_epsilon)
            trajvalues=traj.Sample(0,door.GetActiveConfigurationSpecification())
            assert(transdist(trajvalues,door.GetDOFValues()) <= g_epsilon)
            robot.GetController().SetPath(traj)
            door.GetController().SetPath(traj)
            while not robot.GetController().IsDone():
                self.env.StepSimulation(0.01)
                

    def test_badinterpolation(self):
        self.log.info('test bad interpolation')
        env=self.env
        robot=self.LoadRobot('robots/pr2-beta-static.zae')
        with env:
            robot.SetActiveDOFs([0])
            traj = RaveCreateTrajectory(env,'')
            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,[0,1])
            ret=planningutils.RetimeActiveDOFTrajectory(traj,robot,False,maxvelmult=1,maxaccelmult=1,plannername='parabolictrajectoryretimer')
            assert(ret==PlannerStatus.HasSolution)
            self.RunTrajectory(robot,traj)
            data = traj.GetWaypoint(1)
            data[1] *= 2 # increase velocity by 2
            traj.Insert(1,data,True)
            try:
                self.RunTrajectory(robot,traj)
                raise ValueError('bad trajectory should throw an exception!')
            except openrave_exception,e:
                pass
            
    def test_reverse(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        with robot:
            basemanip = interfaces.BaseManipulation(robot)
            originalvalues = robot.GetConfigurationValues()
            robot.SetActiveDOFs([3,5])
            goal = [pi/2,pi/2]
            traj1 = basemanip.MoveActiveJoints(goal,execute=False,outputtrajobj=True)
            self.RunTrajectory(robot,traj1)
            assert( transdist(robot.GetActiveDOFValues(),goal) <= g_epsilon )
            
            traj2 = basemanip.MoveHandStraight(direction=[0,0,-1],stepsize=0.01,maxsteps=20,minsteps=20,execute=False,outputtrajobj=True)
            self.RunTrajectory(robot,traj2)
            rtraj2 = planningutils.ReverseTrajectory(traj2)
            self.RunTrajectory(robot,rtraj2)
            assert( transdist(robot.GetActiveDOFValues(),goal) <= g_epsilon )
            rtraj1 = planningutils.ReverseTrajectory(traj1)
            self.RunTrajectory(robot,rtraj1)
            assert( transdist(originalvalues, robot.GetConfigurationValues()) <= g_epsilon)

    def test_worktraj(self):
        self.log.debug('test workspace trajerctory with ikparameterization')
        env=self.env
        maxvelocities = array([ 0.62831853,  0.62831853,  0.62831853,  0.62831853,  0.3259    , 0.3259    ,  0.3259    ])
        maxaccelerations = array([ 4.24811395,  4.24811395,  4.24811395,  4.24811395,  2.2036    ,    2.2036    ,  2.2036    ])
        hastimestamps=False
        ikparam = IkParameterization(eye(4),IkParameterizationType.Transform6D)
        T = matrixFromAxisAngle([0,0,0])
        T[0:3,3] = [0,0,0.1]
        ikparam1 = ikparam.Transform(T)
        traj = RaveCreateTrajectory(env,'')
        traj.Init(ikparam.GetConfigurationSpecification())
        traj.Insert(0,ikparam.GetValues())
        traj.Insert(1,ikparam1.GetValues())
        trajclone = RaveCreateTrajectory(env,'')
        trajclone = RaveClone(traj,0)
        ret=planningutils.RetimeAffineTrajectory(traj,maxvelocities,maxaccelerations,hastimestamps=False,plannername='ParabolicTrajectoryRetimer',plannerparameters='<multidofinterp>2</multidofinterp>')
        assert(ret==PlannerStatus.HasSolution)
        assert(abs(traj.GetDuration()-0.45473694444377921) <= g_epsilon)

        retimer = planningutils.AffineTrajectoryRetimer(plannername='ParabolicTrajectoryRetimer',plannerparameters='<multidofinterp>2</multidofinterp>')
        assert(retimer.PlanPath(trajclone,maxvelocities,maxaccelerations,False)==PlannerStatus.HasSolution)
        assert(abs(traj.GetDuration()-trajclone.GetDuration()) <= g_epsilon )

    def test_affinetraj(self):
        self.log.debug('test workspace trajerctory with affine transform')
        env=self.env
        T=eye(4)
        extents=array([2.0,0,0.1])
        spec=RaveGetAffineConfigurationSpecification(DOFAffine.Transform)
        assert(spec==spec.GetTimeDerivativeSpecification(0))
        traj = RaveCreateTrajectory(env,'')
        traj.Init(spec)
        originaldir = array([extents[0],0,0])
        poseglobal = poseFromMatrix(T)
        for t in linspace(0,pi,100):
            pos = array([0,0,extents[2]])
            q=quatFromAxisAngle([0,t,0])
            newpos = quatRotate(q,pos)
            newpos[0] += extents[0]
            localpose = r_[q,newpos]
            pose = poseMult(poseglobal,localpose)
            traj.Insert(traj.GetNumWaypoints(),RaveGetAffineDOFValuesFromTransform(pose,DOFAffine.Transform))
        for t in linspace(pi,0,100):
            pos = array([0,0,extents[2]])
            q=quatFromAxisAngle([0,-t,0])
            newpos = quatRotate(q,pos)
            newpos[0] -= extents[0]
            localpose = r_[q,newpos]
            pose = poseMult(poseglobal,localpose)
            traj.Insert(traj.GetNumWaypoints(),RaveGetAffineDOFValuesFromTransform(pose,DOFAffine.Transform))
        # add the first point as the last point in oder to complete the loop
        traj.Insert(traj.GetNumWaypoints(),traj.GetWaypoint(0))
        velocities = ones(spec.GetDOF())
        quatindex = RaveGetIndexFromAffineDOF(DOFAffine.Transform,DOFAffine.RotationQuat)
        velocities[quatindex:(quatindex+4)] = 100
        ret=planningutils.RetimeAffineTrajectory(traj,velocities,1e6*ones(spec.GetDOF()), False, 'LinearTrajectoryRetimer')
        assert(ret==PlannerStatus.HasSolution)
        assert(abs(traj.GetDuration()-8.6282921678584152) <= g_epsilon)
        sampledata=traj.Sample(1.5,spec)
        assert(transdist(sampledata,array([  8.14146084e-01,   0.00000000e+00,  -1.00000000e-01, 6.12323400e-17,   0.00000000e+00,   4.07073042e-01, 0.00000000e+00])) <= g_epsilon)
        velspec = spec.ConvertToVelocitySpecification()
        sampledata=traj.Sample(1.5,velspec)
        assert(transdist(sampledata, array([-1. ,  0. ,  0. ,  0. ,  0. , -0.5,  0. ])) <= g_epsilon)

    def test_insertionsmoothing(self):
        env=self.env
        env.Load('robots/kawada-hironx.zae')
        robot=env.GetRobots()[0]

        with env:
            robot.SetActiveManipulator('leftarm')
            ikmodel=databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            Tmanip = matrixFromAxisAngle([0,pi/2,0])
            Tmanip[0:3,3] = [0.4,0.1,0.3]
            basemanip = interfaces.BaseManipulation(robot)
            traj=basemanip.MoveToHandPosition(matrices=[Tmanip],outputtrajobj=True,execute=False)
            
            Tmanip2 = array(Tmanip)
            Tmanip2[2,3] -= 0.04
            solution2 = ikmodel.manip.FindIKSolution(Tmanip2,IkFilterOptions.CheckEnvCollisions)

            trajclone = RaveClone(traj,0)
            starttime=time.time()
            planningutils.InsertWaypointWithSmoothing(trajclone.GetNumWaypoints(),solution2,zeros(len(solution2)),trajclone)
            endvalues=trajclone.GetConfigurationSpecification().ExtractJointValues(trajclone.GetWaypoint(-1),robot,ikmodel.manip.GetArmIndices(),0)
            assert(transdist(solution2,endvalues) <= g_epsilon)
            totaltime = time.time()-starttime
            # path should be a little faster
            assert(trajclone.GetDuration()<traj.GetDuration())

    def test_computederiv(self):
        env = self.env
        self.LoadEnv('data/katanatable.env.xml')
        robot=env.GetRobots()[0]
        with env:
            robot.SetActiveDOFs(range(5))
            robot.SetDOFVelocityLimits(linspace(1,5,robot.GetDOF()))
            robot.SetDOFAccelerationLimits(linspace(10,50,robot.GetDOF()))
            traj = RaveCreateTrajectory(env,'')
            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0,[-1,-0.5,-1,-1,-1,1,1,1,1,1])
            traj2 = RaveClone(traj,0)
            ret=planningutils.SmoothActiveDOFTrajectory(traj2,robot)
            assert(ret==PlannerStatus.HasSolution)
            ret=planningutils.SmoothTrajectory(traj)
            assert(ret==PlannerStatus.HasSolution)
            assert(abs(traj.GetDuration()-traj2.GetDuration()) <= g_epsilon)
            # try twice to make sure we compute same result
            for i in range(2):
                planningutils.ComputeTrajectoryDerivatives(traj,2)
                gaccel=traj.GetConfigurationSpecification().FindCompatibleGroup('joint_accelerations',False)
                acceldata=traj.GetWaypoints(0,traj.GetNumWaypoints(),gaccel)
                expectedaccel=array([  0.00000000e+00,   7.50000000e+00,   1.00000000e+01, 1.00000000e+01,   1.00000000e+01,   0.00000000e+00, 3.50596745e-16,   4.67462326e-16,   4.67462326e-16, 4.67462326e-16,   0.00000000e+00,  -7.50000000e+00, -1.00000000e+01,  -1.00000000e+01,  -1.00000000e+01, 0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 0.00000000e+00,   0.00000000e+00])
                assert(transdist(expectedaccel,acceldata) <= g_epsilon)

