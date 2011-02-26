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
# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from openravepy.examples import mobilemanipulation,graspplanning
from openravepy.databases import inversereachability,linkstatistics
from numpy import *
import numpy,time,os,pickle
from itertools import izip
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy import stats

class StatisticsData:
    pass

class OpenRAVEEvaluator(metaclass.AutoReloader):
    @staticmethod
    def fntimer(f, *args, **kwargs):
        starttime=time.time()
        res = f(*args,**kwargs)
        return time.time()-starttime,res
    @staticmethod
    def getdatafiles(dataprefix):
        robotdir,prefix = os.path.split(dataprefix)
        allnames = os.listdir(robotdir)
        return [os.path.join(robotdir,name) for name in allnames if name.startswith(prefix)]
    @staticmethod
    def drawbarcomparison(statdata,stattitle,statylabel,datalabels=('Inverse Reachability', 'Random')):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ind = arange(len(statdata))  # the x locations for the groups
        width = 0.35       # the width of the bars
        rects1 = ax.bar(ind, [mean(d[1]) for d in statdata], width, color='r', yerr=[std(d[1]) for d in statdata])
        rects2 = ax.bar(ind+width, [mean(d[2]) for d in statdata], width, color='y', yerr=[std(d[2]) for d in statdata])

        ax.set_ylabel(statylabel)
        ax.set_title(stattitle)
        ax.set_xticks(ind+width)
        ax.set_ylim(0,1.3*numpy.max([max(mean(d[1]),mean(d[2])) for d in statdata]))
        ax.set_xticklabels([d[0] for d in statdata])
        ax.legend( (rects1[0], rects2[0]), datalabels, loc='upper left' )
        for i,rects in enumerate((rects1,rects2)):
            for j,rect in enumerate(rects):
                height = rect.get_height()
                ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%.3f'%height, ha='center', va='bottom')
                # draw median
                m = median(statdata[j][i+1])
                ax.plot([rect.get_x(),rect.get_x()+rect.get_width()], [m,m],color='k')
                ax.text(rect.get_x()+rect.get_width()/2., 0.95*m, 'median: %.3f'%m, ha='center', va='top')
        plt.show()
        return fig
        
class EvaluateGrasping(OpenRAVEEvaluator):
    pass
class EvaluateInverseKinematics(OpenRAVEEvaluator):
    pass
class EvaluateReachability(OpenRAVEEvaluator):
    pass
class EvaluateInverseReachability(OpenRAVEEvaluator):
    def __init__(self,env,scenename):
        self.env = env
        self.env.Reset()
        #self.env.StopSimulation()
        self.scenename = scenename
        self.env.Load(scenename)
        self.robot = self.env.GetRobots()[0]
        self.planning = graspplanning.GraspPlanning(self.robot,dests=[])
        sceneprefix = os.path.split(self.scenename)[1]
        if sceneprefix.find('.') >= 0:
            sceneprefix = sceneprefix[0:sceneprefix.find('.')]
        self.dataprefix = self.getdataprefix(self.robot)+sceneprefix

    @staticmethod
    def getdataprefix(robot):
        return os.path.join(robot.GetEnv().GetHomeDirectory(),'robot.'+robot.GetKinematicsGeometryHash(),'irstats.')
    def testgraspables(self,Nsamples=1000,logllthresh=2.4,weight=1.0):
        for gmodel,dests in self.planning.graspables:
            self.robot.SetActiveManipulator(gmodel.manip)
            irmodel=databases.inversereachability.InverseReachabilityModel (self.robot)
            if not irmodel.load():
                irmodel.autogenerate()
            gr = mobilemanipulation.GraspReachability(robot=self.robot,irgmodels=[(irmodel,gmodel)])
            starttime = time.time()
            densityfn,samplerfn,bounds = gr.computeGraspDistribution(logllthresh=logllthresh)
            print 'time to build distribution: %fs'%(time.time()-starttime)
            #h = gr.irmodel.showBaseDistribution(densityfn,bounds,self.target.GetTransform()[2,3],thresh=1.0)
            data = StatisticsData()
            data.Nsamples = Nsamples
            data.samplingavg = array(())
            data.samplingfailures = array(())
            for i in range(10):
                starttime = time.time()
                goals,numfailures = gr.sampleGoals(lambda N: samplerfn(N=N,weight=weight),N=Nsamples,timeout=Nsamples)
                data.samplingavg = r_[data.samplingavg,min(Nsamples,(time.time()-starttime)/(len(goals)+1e-8))]
                data.samplingfailures = r_[data.samplingfailures,numfailures/float(len(goals)+numfailures)]

            data.randomavg = array(())
            data.randomfailures = array(())
            Trobot = self.robot.GetTransform()
            validgrasps,validindices = gmodel.computeValidGrasps(checkik=False)
            Tgrasps = [(gmodel.getGlobalGraspTransform(grasp),i) for i,grasp in enumerate(validgrasps)]
            bounds = array(((0,-1.2,-1.2),(2*pi,1.2,1.2)))
            def randomsampler(N,weight=1.0):
                indices = random.randint(0,len(Tgrasps),N)
                angles = 0.5*random.rand(N)*(bounds[1,0]-bounds[0,0])+bounds[0,0]
                XY = [Tgrasps[i][0][0:2,3]+random.rand(2)*(bounds[1,1:3]-bounds[0,1:3])+bounds[0,1:3]  for i in indices]
                return c_[cos(angles),zeros((N,2)),sin(angles),array(XY),tile(Trobot[2,3],N)],[(gmodel,i) for i in indices],([],[])

            for i in range(10):
                starttime = time.time()
                goals,numfailures = gr.sampleGoals(randomsampler,N=Nsamples,timeout=Nsamples)
                data.randomavg = r_[data.randomavg,min(Nsamples,(time.time()-starttime)/(len(goals)+1e-8))]
                data.randomfailures = r_[data.randomfailures,numfailures/float(len(goals)+numfailures)]

            with self.env:
                data.samplingtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=False).next)[0] for i in range(Nsamples)]
                # remove the last %1
                data.samplingtimes = sort(data.samplingtimes)[0:floor(Nsamples*0.99)]
                data.randomtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=True).next)[0] for i in range(Nsamples)]
                data.randomtimes = sort(data.randomtimes)[0:floor(Nsamples*0.99)]

            data.robotname = self.robot.GetName()
            data.targetname = gmodel.target.GetName()
            datafilename = self.dataprefix+'.'+gmodel.target.GetName()+'.pp'
            pickle.dump(data,open(datafilename,'w'))
        print 'finished inversereachability'
            
    @staticmethod
    def gatherdata(robotname):
        env = Environment()
        robot = env.ReadRobotXMLFile(robotname)
        env.AddRobot(robot)

        dataprefix = EvaluateInverseReachability.getdataprefix(robot)
        datafiles = EvaluateInverseReachability.getdatafiles(dataprefix)
        allavg = []
        allfailures = []
        alltimes = []
        for datafile in datafiles:
            try:
                data = pickle.load(open(datafile,'r'))
            except:
                continue
            allavg.append((data.targetname,data.samplingavg,data.randomavg))
            allfailures.append((data.targetname,data.samplingfailures,data.randomfailures))
            alltimes.append((data.targetname,data.samplingtimes,data.randomtimes))
        
        # find all files starting with dataprefix and combine their data
        fig = EvaluateInverseReachability.drawbarcomparison(allavg,'Robot %s\nAverage Valid Configuration Sampling Time'%robot.GetName(),'seconds')
        fig.savefig(dataprefix+'average.pdf',format='pdf')
        plt.close(fig)
        fig = EvaluateInverseReachability.drawbarcomparison(allfailures,'Robot %s\Sample Failure Probability'%robot.GetName(),'samples')
        fig.savefig(dataprefix+'failures.pdf',format='pdf')
        plt.close(fig)
        fig = EvaluateInverseReachability.drawbarcomparison(alltimes,'Robot %s\nTime to First Valid Configuration in New Scene'%robot.GetName(),'seconds')
        fig.savefig(dataprefix+'times.pdf',format='pdf')
        plt.close(fig)
        
        robot = None
        env.Destroy()

    @staticmethod
    def plot(avg,firsttimes):
        """histogram plotting"""
        fig = plt.figure()
        ax = fig.add_subplot(111)
        delta = 0.2
        maxtime = 20.0
        n, bins, patches = ax.hist(firsttimes, bins=maxtime/delta,range=[0,maxtime], normed=1, facecolor='green', alpha=0.75)
        # add a 'best fit' line
#             params = stats.genexpon.fit(firsttimes)
#             bincenters = 0.5*(bins[1:]+bins[:-1])
#             y = stats.genexpon.pdf( bincenters, *params)
#             l = ax.plot(bincenters, y, 'r--', linewidth=1)
        l = ax.plot(bincenters, y, 'r--', linewidth=1)
        ax.set_xlabel('Time to First Solution (Average: %f)')
        ax.set_ylabel('Probability')
        #ax.set_title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
        ax.set_xlim(0.0,maxtime)
        ax.set_ylim(0,1/delta)
        ax.grid(True)
        plt.show()
        return fig
    @staticmethod
    def run():
        import evaluateplanning
        logllthresh = 2.4
        Nsamples = 100
        weight = 0.5
        env = Environment()
        self = evaluateplanning.EvaluateInverseReachability(env,'data/wamtest1.env.xml')
        self.testgraspables(Nsamples=Nsamples,logllthresh=logllthresh)
        self = evaluateplanning.EvaluateInverseReachability(env,'data/wamtest2.env.xml')
        self.testgraspables(Nsamples=Nsamples,logllthresh=logllthresh)
        env.Destroy()

        # save data
        evaluateplanning.EvaluateInverseReachability.gatherdata('robots/barrettsegway.robot.xml')
        
class EvaluateDistanceMetric(OpenRAVEEvaluator):
    def __init__(self,env,scenename):
        self.env = env
        self.env.Reset()
        self.env.StopSimulation()
        self.scenename = scenename
        self.env.Load(scenename)
        self.robot = self.env.GetRobots()[0]
        self.planning = graspplanning.GraspPlanning(self.robot,dests=[])
        sceneprefix = os.path.split(self.scenename)[1]
        if sceneprefix.find('.') >= 0:
            sceneprefix = sceneprefix[0:sceneprefix.find('.')]
        self.dataprefix = self.getdataprefix(self.robot)+sceneprefix
        self.lsmodel = linkstatistics.LinkStatisticsModel(self.robot)
        if not self.lsmodel.load():
            self.lsmodel.autogenerate()
    @staticmethod
    def getdataprefix(robot):
        return os.path.join(robot.GetEnv().GetHomeDirectory(),'robot.'+robot.GetKinematicsGeometryHash(),'diststats.')

    def testgraspables(self,weightexp=0.005,N=10):
        gmodel,dests = self.planning.graspables[0]
        validgrasps,validindices = gmodel.computeValidGrasps()
        Tgrasps = [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]
        data = StatisticsData()
        data.robotname = self.robot.GetName()
        data.catimes = []
        for type in range(2):
            self.lsmodel.setRobotWeights(weightexp=weightexp,type=type)
            data.catimes.append([self.fntimer(self.planning.basemanip.MoveToHandPosition,matrices=Tgrasps[0:1],maxtries=1,seedik=4,maxiter=10000,execute=False)[0] for i in range(N)])
        datafilename = self.dataprefix+'.pp'
        pickle.dump(data,open(datafilename,'w'))
    @staticmethod
    def gatherdata(robotnames):
        env = Environment()
        alltimes = []
        for robotname in robotnames:
            env.Reset()
            robot = env.ReadRobotXMLFile(robotname)
            env.AddRobot(robot)
            dataprefix = EvaluateDistanceMetric.getdataprefix(robot)
            datafiles = EvaluateDistanceMetric.getdatafiles(dataprefix)
            catimes = [[],[]]
            for datafile in datafiles:
                try:
                    data = pickle.load(open(datafile,'r'))
                except:
                    continue
                catimes[0] += data.catimes[0]
                catimes[1] += data.catimes[1]
            alltimes.append((robot.GetName(),array(catimes[0]),array(catimes[1])))
        # find all files starting with dataprefix and combine their data
        fig = EvaluateInverseReachability.drawbarcomparison(alltimes,'Robot %s\nPlanning Times (Weights)'%robot.GetName(),'seconds',datalabels=('Volume-Dependent Weights', 'Uniform Weight (%f)'))
        fig.savefig(dataprefix+'pdf',format='pdf')
        plt.close(fig)
        env.Destroy()
    @staticmethod
    def run():
        import evaluateplanning
        env = Environment()
        self = evaluateplanning.EvaluateDistanceMetric(env,'data/wamtest1.env.xml')
        self.testgraspables(N=100)
        env.Destroy()

        # save data
        evaluateplanning.EvaluateDistanceMetric.gatherdata(['robots/barrettsegway.robot.xml'])

class EvaluateResolutions(OpenRAVEEvaluator):
    def __init__(self,env,scenename):
        self.env = env
        self.env.Reset()
        self.env.StopSimulation()
        self.scenename = scenename
        self.env.Load(scenename)
        self.robot = self.env.GetRobots()[0]
        self.planning = graspplanning.GraspPlanning(self.robot,dests=[])
        sceneprefix = os.path.split(self.scenename)[1]
        if sceneprefix.find('.') >= 0:
            sceneprefix = sceneprefix[0:sceneprefix.find('.')]
        self.dataprefix = self.getdataprefix(self.robot)+sceneprefix
        self.lsmodel = linkstatistics.LinkStatisticsModel(self.robot)
        if not self.lsmodel.load():
            self.lsmodel.autogenerate()
    @staticmethod
    def getdataprefix(robot):
        return os.path.join(robot.GetEnv().GetHomeDirectory(),'robot.'+robot.GetKinematicsGeometryHash(),'resstats.')

    def testgraspables(self,xyzdelta=0.005,N=10):
        gmodel,dests = self.planning.graspables[0]
        validgrasps,validindices = gmodel.computeValidGrasps()
        Tgrasps = [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]
        data = StatisticsData()
        data.robotname = self.robot.GetName()
        self.lsmodel.setRobotResolution(xyzdelta)
        resolutions1 = [self.robot.GetJointResolutions(),self.robot.GetAffineTranslationResolution(),self.robot.GetAffineRotationAxisResolution()]
        minrotres = min(r_[resolutions1[0], resolutions1[2]])
        resolutions2 = [tile(minrotres,len(self.robot.GetJoints())),[xyzdelta,xyzdelta,xyzdelta],tile(minrotres,4)]
        data.resolutions = (resolutions1,resolutions2)
        data.catimes = []
        for resolutions in data.resolutions:
            for r,j in izip(resolutions[0],self.robot.GetJoints()):
                j.SetResolution(r)
            self.robot.SetAffineTranslationResolution(resolutions[1])
            self.robot.SetAffineRotationAxisResolution(resolutions[2])
            data.catimes.append([self.fntimer(self.planning.basemanip.MoveToHandPosition,matrices=Tgrasps[0:1],maxtries=1,seedik=4,maxiter=10000,execute=False)[0] for i in range(N)])
        datafilename = self.dataprefix+'.pp'
        pickle.dump(data,open(datafilename,'w'))
    @staticmethod
    def gatherdata(robotnames):
        env = Environment()
        alltimes = []
        for robotname in robotnames:
            env.Reset()
            robot = env.ReadRobotXMLFile(robotname)
            env.AddRobot(robot)
            dataprefix = EvaluateResolutions.getdataprefix(robot)
            datafiles = EvaluateResolutions.getdatafiles(dataprefix)
            catimes = [[],[]]
            for datafile in datafiles:
                try:
                    data = pickle.load(open(datafile,'r'))
                except:
                    continue
                catimes[0] += data.catimes[0]
                catimes[1] += data.catimes[1]
            alltimes.append((robot.GetName(),array(catimes[0]),array(catimes[1])))        
        # find all files starting with dataprefix and combine their data
        fig = EvaluateInverseReachability.drawbarcomparison(alltimes,'Robot %s\nPlanning Times (Resolution)'%robot.GetName(),'seconds',datalabels=('Swept Volume Resolutions', 'Minimum Resolution (%f)'%data.resolutions[1][0][0]))
        fig.savefig(dataprefix+'pdf',format='pdf')
        plt.close(fig)
        env.Destroy()
    @staticmethod
    def run():
        import evaluateplanning
        env = Environment()
        self = evaluateplanning.EvaluateResolutions(env,'data/wamtest1.env.xml')
        self.testgraspables(N=100)
        env.Destroy()

        # save data
        evaluateplanning.EvaluateResolutions.gatherdata(['robots/barrettsegway.robot.xml'])

class EvaluateManipulation(OpenRAVEEvaluator):
    pass
class EvaluateMobileManipulation(OpenRAVEEvaluator):
    pass

if __name__ == "__main__":
    pass
