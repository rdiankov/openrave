# -*- coding: utf-8 -*-
# Copyright (C) 2011-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
from openravepy import openravepy_configurationcache

class TestConfigurationCache(EnvironmentSetup):
    def setup(self):
        EnvironmentSetup.setup(self)
    
    def test_insertandquery(self):
        self.LoadEnv('data/lab1.env.xml')
        env=self.env
        robot=env.GetRobots()[0]
        robot.SetActiveDOFs(range(7))
        cache=openravepy_configurationcache.ConfigurationCache(robot)

        values = robot.GetActiveDOFValues()
        inserted = cache.InsertConfiguration(values, None)
        assert(inserted and cache.GetNumNodes()==1)
        values[1] = pi/2
        inserted=cache.InsertConfiguration(values, None)
        assert(inserted and cache.GetNumNodes()>=2)
        assert(cache.Validate())

        values[1] = pi/2-0.0001
        robot.SetActiveDOFValues(values)
        ret, closestdist, collisioninfo = cache.CheckCollision(values)
        assert(ret==0)

        originalvalues = array([0,pi/2,0,pi/6,0,0,0])

        sampler = RaveCreateSpaceSampler(env, u'MT19937')
        sampler.SetSpaceDOF(robot.GetActiveDOF())
        report=CollisionReport()

        with env:
            for iter in range(0, 10000):
                robot.SetActiveDOFValues(originalvalues + 0.05*(sampler.SampleSequence(SampleDataType.Real,1)-0.5))
                samplevalues = robot.GetActiveDOFValues()
                incollision = env.CheckCollision(robot, report=report)
                inserted = cache.InsertConfiguration(samplevalues, report if incollision else None)
            self.log.info('cache has %d nodes', cache.GetNumNodes())
            assert(cache.Validate())

        with env:
            numspurious = 0
            nummisses = 0
            numtests = 1000
            collisiontimes = []
            cachetimes = []
            for iter in range(0, numtests):
                robot.SetActiveDOFValues(originalvalues + 0.05*(sampler.SampleSequence(SampleDataType.Real,1)-0.5))
                samplevalues = robot.GetActiveDOFValues()
                starttime=time.time()
                ret, closestdist, collisioninfo = cache.CheckCollision(samplevalues)
                midtime=time.time()
                incollision = env.CheckCollision(robot, report=report)
                endtime=time.time()
                cachetimes.append(midtime-starttime)
                collisiontimes.append(endtime-midtime)
                if ret != -1:
                    assert(closestdist <= 1)
                    # might give spurious collision since cache is being conservative
                    if incollision != ret:
                        if ret == 1:
                            numspurious += 1
                        else:
                            # unexpected freespace
                            assert(0)
                else:
                    nummisses += 1
            self.log.info('num spurious colisions=%d/%d, num misses = %d/%d, meancache=%fs, meancollision=%fs', numspurious, numtests, nummisses, numtests, mean(cachetimes), mean(collisiontimes))
        assert(float(numspurious)/float(numtests)<=0.06)
        assert(float(nummisses)/float(numtests)>0.1) # space is pretty big
        #assert(mean(cachetimes) < mean(collisiontimes)) # caching not always faster and difficult to test performance anyway...
    
    def test_io(self):
        env = self.env
        with env:
            self.LoadEnv('data/hironxtable.env.xml')
            robot = env.GetRobots()[0]
            manip = robot.SetActiveManipulator('leftarm_torso')

            lmodel=databases.linkstatistics.LinkStatisticsModel(robot)
            if not lmodel.load():
                lmodel.autogenerate()
            lmodel.setRobotWeights()
            lmodel.setRobotResolutions(xyzdelta=0.04)
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetActiveDOFs(manip.GetArmIndices())
            goal = robot.GetActiveDOFValues()
            goal[0] = -0.556
            goal[3] = -1.86

            oldchecker = env.GetCollisionChecker()

            cachechecker = RaveCreateCollisionChecker(self.env,'CacheChecker')
            success=cachechecker.SendCommand('TrackRobotState %s'%robot.GetName())
            assert(success is not None)
            env.SetCollisionChecker(cachechecker)
            robot.SetSelfCollisionChecker(cachechecker)
            sampler = RaveCreateSpaceSampler(env, u'RobotConfiguration %s'%robot.GetName())
            sampler.SampleSequence(SampleDataType.Real,1)
            report=CollisionReport()

            cachechecker.SendCommand('ResetSelfCache')
            stime = time.time()
            confs = []
            for iter in range(0, 500):
                robot.SetActiveDOFValues(sampler.SampleSequence(SampleDataType.Real,1))
                samplevalues = robot.GetActiveDOFValues()
                confs.append(samplevalues)
                if (iter%10==0):
                    self.log.info('checking self collisions %s...',iter)

                    self.log.info('writing cache to file...')
                    cachechecker.SendCommand('SaveCache')
                env.GetCollisionChecker().CheckSelfCollision(robot, report=report)
            rawtime = time.time()-stime

            selfcachedcollisions, selfcachedcollisionhits, selfcachedfreehits, selfcachesize = cachechecker.SendCommand('GetSelfCacheStatistics').split()
            self.log.info('selfcollisionhits=%s selffreehits=%s selfcachesize=%s in %ss', selfcachedcollisionhits, selfcachedfreehits, selfcachesize, rawtime)

            self.log.info('writing cache to file...')
            cachechecker.SendCommand('SaveCache')

    def test_find_insert(self):

        self.LoadEnv('data/lab1.env.xml')
        env=self.env
        robot=env.GetRobots()[0]
        robot.SetActiveDOFs(range(7))
        cache=openravepy_configurationcache.ConfigurationCache(robot)
        cache.SetFreeSpaceThresh(1)
        values = robot.GetActiveDOFValues()
        inserted = cache.InsertConfiguration(values, None)
        sampler = RaveCreateSpaceSampler(env, u'MT19937')
        sampler.SetSpaceDOF(robot.GetActiveDOF())
        with env:
             self.log.info('testing exhaustive insertion...')
             for iter in range(0, 10000):
                 if iter%1000==0:
                     self.log.info('%d valid insertions %d nodes...',iter,cache.GetNumNodes())

                 samplevalues = 0.3*(sampler.SampleSequence(SampleDataType.Real,1)-0.5)
                 nn = cache.FindNearestNode(samplevalues, 4)
                 if nn is None:
                     cache.SetFreeSpaceThresh(8)
                     inserted = cache.InsertConfigurationDist(samplevalues, None, 1)
                     assert(inserted == 1)
                     cache.SetFreeSpaceThresh(1)

             self.log.info('exhaustive insertion test passed')

    def test_updates(self):
        env = self.env
        with env:
            self.LoadEnv('data/hironxtable.env.xml')
            robot = env.GetRobots()[0]
            manip = robot.SetActiveManipulator('leftarm_torso')

            lmodel=databases.linkstatistics.LinkStatisticsModel(robot)
            if not lmodel.load():
                lmodel.autogenerate()
            lmodel.setRobotWeights()
            lmodel.setRobotResolutions(xyzdelta=0.01)
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetActiveDOFs(manip.GetArmIndices())
            goal = robot.GetActiveDOFValues()
            goal[0] = -0.556
            goal[3] = -1.86

            self.log.info('testing cache updates...')
            oldchecker = env.GetCollisionChecker()

            cachechecker = RaveCreateCollisionChecker(self.env,'CacheChecker')
            success=cachechecker.SendCommand('TrackRobotState %s'%robot.GetName())
            assert(success is not None)
            env.SetCollisionChecker(cachechecker)
            robot.SetSelfCollisionChecker(cachechecker)

            traj = basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.01,maxtries=1,execute=False,outputtrajobj=True)

            cachedcollisions, cachedcollisionhits, cachedfreehits, oldcachesize = cachechecker.SendCommand('GetCacheStatistics').split()

            self.env.Remove(self.env.GetBodies()[1])
            cachedcollisions, cachedcollisionhits, cachedfreehits, cachesize = cachechecker.SendCommand('GetCacheStatistics').split()
            assert(oldcachesize>cachesize)
            self.log.info('environment removebody test passed (%s/%s)',cachesize,oldcachesize)

            assert(int(cachechecker.SendCommand('ValidateCache')) == 1)
            assert(int(cachechecker.SendCommand('ValidateSelfCache')) == 1)
            self.log.info('valid tests passed')

            traj = basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.01,maxtries=1,execute=False,outputtrajobj=True)

            cachedcollisions, cachedcollisionhits, cachedfreehits, cachesize = cachechecker.SendCommand('GetCacheStatistics').split()
            self.env.Reset()
            cachedcollisions, cachedcollisionhits, cachedfreehits, addcachesize = cachechecker.SendCommand('GetCacheStatistics').split()
            assert(cachesize>addcachesize)
            self.log.info('environment addbody test passed (%s/%s)',addcachesize,cachesize)

            assert(int(cachechecker.SendCommand('ValidateCache')) == 1)
            assert(int(cachechecker.SendCommand('ValidateSelfCache')) == 1)
            self.log.info('valid tests passed')

    def test_planning(self):
            env = self.env
            with env:
                self.LoadEnv('data/hironxtable.env.xml')
                robot = env.GetRobots()[0]
                manip = robot.SetActiveManipulator('leftarm_torso')
                lmodel=databases.linkstatistics.LinkStatisticsModel(robot)
                if not lmodel.load():
                    lmodel.autogenerate()
                lmodel.setRobotWeights()
                lmodel.setRobotResolutions(xyzdelta=0.004)
                basemanip = interfaces.BaseManipulation(robot)
                robot.SetActiveDOFs(manip.GetArmIndices())
                goal = robot.GetActiveDOFValues()
                goal[0] = -0.556
                goal[3] = -1.86

                self.log.info('testing planning...')
                oldchecker = env.GetCollisionChecker()

                cachechecker = RaveCreateCollisionChecker(self.env,'CacheChecker')
                success=cachechecker.SendCommand('TrackRobotState %s'%robot.GetName())
                assert(success is not None)

                starttime = time.time()
                traj = basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.01,maxtries=1,execute=False,outputtrajobj=True)
                regtime = time.time()-starttime

                self.log.info('time without cache %s',regtime)

                cachechecker.SendCommand('ResetSelfCache')

                env.SetCollisionChecker(cachechecker)
                robot.SetSelfCollisionChecker(cachechecker)
                cachedtimes = []
                prevtime = float('Inf')
                for runs in range(5):

                    starttime = time.time()
                    traj = basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.01,maxtries=1,execute=True,outputtrajobj=True)
                    cachetime = time.time()-starttime
                    cachedtimes.append(cachetime)

                    cachedcollisions, cachedcollisionhits, cachedfreehits, cachesize = cachechecker.SendCommand('GetCacheStatistics').split()
                    cacherate = float((float(cachedfreehits)+float(cachedcollisionhits))/float(cachedcollisions))
                    selfcachedcollisions, selfcachedcollisionhits, selfcachedfreehits, selfcachesize = cachechecker.SendCommand('GetSelfCacheStatistics').split()
                    selfcacherate = float((float(selfcachedfreehits)+float(selfcachedcollisionhits))/float(selfcachedcollisions))

                    self.log.info('planning time=%fs collisionhits=%s/%s freehits=%s/%s cachesize=%s selfcollisionhits=%s/%s selffreehits=%s/%s selfcachesize=%s', cachetime, cachedcollisionhits, cachedcollisions, cachedfreehits, cachedcollisions, cachesize, selfcachedcollisionhits, selfcachedcollisions, selfcachedfreehits, selfcachedcollisions, selfcachesize)
                    self.log.info('cacherate=%f selfcacherate=%f',cacherate,selfcacherate)

                    self.log.info('%s',cachechecker.SendCommand('GetCacheTimes'))

                    self.log.info('run %s', runs)
                    with robot:
                        parameters = Planner.PlannerParameters()
                        parameters.SetRobotActiveJoints(robot)
                        planningutils.VerifyTrajectory(parameters,traj,samplingstep=0.001)
                        self.log.info('trajectory test passed')


                    assert(cachetime < prevtime*1.5)
                    self.log.info('monotonic decrease test passed (%fs/%fs)',cachetime, prevtime)
                    prevtime = cachetime

                assert(cacherate > 0.9 and selfcacherate > 0.9)
                self.log.info('hitrate test passed (%f)(%f)',cacherate,selfcacherate)

                env.SetCollisionChecker(oldchecker)

                starttime = time.time()
                traj2 = basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.01,maxtries=1,execute=False,outputtrajobj=True)
                originaltime = time.time()-starttime
                assert(originaltime*1.5 > cachedtimes[0])
                self.log.info('speedup test passed (%fs/%fs)',originaltime, cachedtimes[0])
                with robot:
                    parameters = Planner.PlannerParameters()
                    parameters.SetRobotActiveJoints(robot)
                    planningutils.VerifyTrajectory(parameters,traj2,samplingstep=0.002)

                spec = manip.GetArmConfigurationSpecification()
                usedbodies = spec.ExtractUsedBodies(env)
                assert(len(usedbodies) == 1 and usedbodies[0] == robot)
                useddofindices, usedconfigindices = spec.ExtractUsedIndices(robot)
                assert(sorted(useddofindices) == sorted(manip.GetArmIndices()))

                cachechecker.SendCommand('ResetCache')
                cachedcollisions, cachedcollisionhits, cachedfreehits, cachesize = cachechecker.SendCommand('GetCacheStatistics').split()
                assert(int(cachesize)==0)
                self.log.info('cache reset test passed')

                cachechecker.SendCommand('ResetSelfCache')
                cachedcollisions, cachedcollisionhits, cachedfreehits, cachesize = cachechecker.SendCommand('GetSelfCacheStatistics').split()
                assert(int(cachesize)==0)
                self.log.info('self cache reset test passed')
