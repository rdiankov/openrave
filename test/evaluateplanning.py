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

class OpenRAVEEvaluator(metaclass.AutoReloader):
    @staticmethod
    def fntimer(f, *args, **kwargs):
        starttime=time.time()
        res = f(*args,**kwargs)
        return time.time()-starttime,res

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
        self.dataprefix = os.path.join(self.env.GetHomeDirectory(),'robot.'+self.robot.GetRobotStructureHash(),'irstats.'+sceneprefix)

    def testgraspables(self,Nsamples=1000,logllthresh=2.4,weight=1.0):
        for gmodel,dests in self.planning.graspables:
            gr = mobilemanipulation.GraspReachability(robot=self.robot,gmodel=gmodel)
            starttime = time.time()
            densityfn,samplerfn,bounds,validgrasps = gr.computeGraspDistribution(logllthresh=logllthresh)
            print 'time to build distribution: %fs'%(time.time()-starttime)
            #h = gr.irmodel.showBaseDistribution(densityfn,bounds,self.target.GetTransform()[2,3],thresh=1.0)
            
            self.avgsampling = zeros((2,0))
            for i in range(10):
                starttime = time.time()
                goals,numfailures = gr.sampleGoals(lambda N: samplerfn(N,weight=weight),N=Nsamples)
                self.avgsampling = c_[self.avgsampling,array((time.time()-starttime,numfailures))/Nsamples]
            
            self.avgrandom = zeros((2,0))
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
                self.avgrandom = c_[self.avgrandom,array((time.time()-starttime,numfailures))/Nsamples]

            with self.env:
                self.samplingtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=False).next)[0] for i in range(Nsamples)]
                # remove the last %1
                self.samplingtimes = sort(self.samplingtimes)[0:floor(Nsamples*0.99)]
                self.randomtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=True).next)[0] for i in range(Nsamples)]
                self.randomtimes = sort(self.randomtimes)[0:floor(Nsamples*0.99)]

            datafilename = self.dataprefix+'.'+gmodel.target.GetName()+'.pp'
            pickle.dump((self.avgsampling,self.samplingtimes,self.avgrandom,self.randomtimes),open(datafilename,'w'))
            fig = self.plot(self.avgsampling,self.samplingtimes)

    def plot(self,avg,firsttimes):
        # plot the data
        fig = plt.figure()
        fig.clf()
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
    env = Environment()
    self = evaluateplanning.EvaluateInverseReachability(env,'data/lab1.env.xml')
    logllthresh = 2.4
    Nsamples = 1000
    weight = 0.5
    self.testgraspables(Nsamples=Nsamples,logllthresh=logllthresh)

if __name__ == "__main__":
    pass
