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
from openravepy.examples import mobilemanipulation,graspplanning,inversereachability
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
        return [os.path.join(robotdir,name) for name in allnames if name.find(prefix) == 0]
        
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
        return os.path.join(robot.GetEnv().GetHomeDirectory(),'robot.'+robot.GetRobotStructureHash(),'irstats.')
    def testgraspables(self,Nsamples=1000,logllthresh=2.4,weight=1.0):
        for gmodel,dests in self.planning.graspables:
            gr = mobilemanipulation.GraspReachability(robot=self.robot,gmodel=gmodel)
            starttime = time.time()
            densityfn,samplerfn,bounds,validgrasps = gr.computeGraspDistribution(logllthresh=logllthresh)
            print 'time to build distribution: %fs'%(time.time()-starttime)
            #h = gr.irmodel.showBaseDistribution(densityfn,bounds,self.target.GetTransform()[2,3],thresh=1.0)

            print '1'
            data = StatisticsData()
            data.Nsamples = Nsamples
            data.samplingavg = array(())
            data.samplingfailures = array(())
            for i in range(10):
                starttime = time.time()
                goals,numfailures = gr.sampleGoals(lambda N: samplerfn(N,weight=weight),N=Nsamples)
                data.samplingavg = r_[data.samplingavg,(time.time()-starttime)/Nsamples]
                data.samplingfailures = r_[data.samplingfailures,numfailures/float(Nsamples+numfailures)]

            print '2'
            data.randomavg = array(())
            data.randomfailures = array(())
            Trobot = self.robot.GetTransform()
            Tgrasps = [(gr.gmodel.getGlobalGraspTransform(grasp),i) for i,grasp in enumerate(validgrasps)]
            bounds = array(((0,-1.0,-1.0),(2*pi,1.0,1.0)))
            def randomsampler(N,weight=1.0):
                indices = random.randint(0,len(Tgrasps),N)
                angles = random.rand(N)*(bounds[1,0]-bounds[0,0])+bounds[0,0]
                XY = [Tgrasps[i][0][0:2,3]+random.rand(2)*(bounds[1,1:3]-bounds[0,1:3])+bounds[0,1:3]  for i in indices]
                return c_[cos(angles),zeros((N,2)),sin(angles),array(XY),tile(Trobot[2,3],N)],array([Tgrasps[i][1] for i in indices])

            for i in range(10):
                starttime = time.time()
                goals,numfailures = gr.sampleGoals(randomsampler,N=Nsamples)
                data.randomavg = r_[data.randomavg,(time.time()-starttime)/Nsamples]
                data.randomfailures = r_[data.randomfailures,numfailures/float(Nsamples+numfailures)]

            with self.env:
                print '3'
                data.samplingtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=False).next)[0] for i in range(Nsamples)]
                # remove the last %1
                data.samplingtimes = sort(data.samplingtimes)[0:floor(Nsamples*0.99)]
                print '4'
                data.randomtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=True).next)[0] for i in range(Nsamples)]
                data.randomtimes = sort(data.randomtimes)[0:floor(Nsamples*0.99)]

            data.robotname = self.robot.GetName()
            data.targetname = gmodel.target.GetName()
            datafilename = self.dataprefix+'.'+gmodel.target.GetName()+'.pp'
            pickle.dump(data,open(datafilename,'w'))
        print 'finished inversereachability'
            
    @staticmethod
    def gatherdata(robot):
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
        fig = EvaluateInverseReachability.drawcomparison(allavg,'Robot %s\nAverage Valid Configuration Sampling Time'%robot.GetName(),'seconds')
        fig.savefig(dataprefix+'average.pdf',format='pdf')
        plt.close(fig)
        fig = EvaluateInverseReachability.drawcomparison(allfailures,'Robot %s\nBad Samples per Good Sample Generated'%robot.GetName(),'samples')
        fig.savefig(dataprefix+'failures.pdf',format='pdf')
        plt.close(fig)
        fig = EvaluateInverseReachability.drawcomparison(alltimes,'Robot %s\nTime to First Valid Configuration in New Scene'%robot.GetName(),'seconds')
        fig.savefig(dataprefix+'times.pdf',format='pdf')
        plt.close(fig)

    @staticmethod
    def drawcomparison(statdata,stattitle,statylabel):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ind = arange(len(statdata))  # the x locations for the groups
        width = 0.35       # the width of the bars
        rects1 = ax.bar(ind, [mean(d[1]) for d in statdata], width, color='r', yerr=[std(d[1]) for d in statdata])
        rects2 = plt.bar(ind+width, [mean(d[2]) for d in statdata], width, color='y', yerr=[std(d[2]) for d in statdata])

        ax.set_ylabel(statylabel)
        ax.set_title(stattitle)
        ax.set_xticks(ind+width)
        ax.set_xticklabels([d[0] for d in statdata])
        ax.legend( (rects1[0], rects2[0]), ('Inverse Reachability', 'Random'), loc='upper left' )
        for rect in rects1:
            height = rect.get_height()
            ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%.3f'%height, ha='center', va='bottom')
        for rect in rects2:
            height = rect.get_height()
            ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%.3f'%height, ha='center', va='bottom')
        plt.show()
        return fig

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

class EvaluateDistanceMetric(OpenRAVEEvaluator):
    pass
class EvaluateManipulation(OpenRAVEEvaluator):
    pass
class EvaluateMobileManipulation(OpenRAVEEvaluator):
    pass

def test():
    import evaluateplanning
    logllthresh = 2.4
    Nsamples = 10
    weight = 0.5
    env = Environment()
    self = evaluateplanning.EvaluateInverseReachability(env,'data/wamtest1.env.xml')
    self.testgraspables(Nsamples=Nsamples,logllthresh=logllthresh)
    self = evaluateplanning.EvaluateInverseReachability(env,'data/wamtest2.env.xml')
    self.testgraspables(Nsamples=Nsamples,logllthresh=logllthresh)

def savedata():
    import evaluateplanning
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    evaluateplanning.EvaluateInverseReachability.gatherdata(robot)

if __name__ == "__main__":
    pass
