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

    def testgraspables(self,Nsamples=1000,logllthresh=2.4,weight=1.0):
        sceneprefix = os.path.split(self.scenename)[1]
        if sceneprefix.find('.') >= 0:
            sceneprefix = sceneprefix[0:sceneprefix.find('.')]
        for gmodel,dests in self.planning.graspables:
            gr = mobilemanipulation.GraspReachability(robot=self.robot,gmodel=gmodel)
            starttime = time.time()
            densityfn,samplerfn,bounds,validgrasps = self.computeGraspDistribution(logllthresh=logllthresh)
            print 'time to build distribution: %fs'%(time.time()-starttime)
            #h = gr.irmodel.showBaseDistribution(densityfn,bounds,self.target.GetTransform()[2,3],thresh=1.0)
            
            avgsampling = zeros((2,0))
            for i in range(10):
                starttime = time.time()
                goals,numfailures = self.sampleGoals(lambda N: samplerfn(N,weight=weight),validgrasps,N=Nsamples)
                avgsampling = c_[avgsampling,array((time.time()-starttime,numfailures))]
            
            avgrandom = zeros((2,0))
            randomiterfn = gr.randomBaseDistributionIterator(izip(validgrasps,range(len(validgrasps))),Nprematuresamples=0)
            for i in range(10):
                starttime = time.time()
                goals,numfailures = self.sampleGoals(lambda N: array([pose for pose,index in randomiterfn]),validgrasps,N=Nsamples)
                avgrandom = c_[avgrandom,array((time.time()-starttime,numfailures))]
            
            with self.env:
                samplingtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=False).next)[0] for i in range(Nsamples)]
                randomtimes = [self.fntimer(gr.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True).next)[0] for i in range(Nsamples)]

            pickle.dump(((avgsampling,samplingtimes),(avgrandom,randomtimes)),open(sceneprefix+'.'+gmodel.target.GetName()
            fig = plt.figure()
            ax = fig.add_subplot(111)

            # the histogram of the data
            n, bins, patches = ax.hist(x, 50, normed=1, facecolor='green', alpha=0.75)

            # hist uses np.histogram under the hood to create 'n' and 'bins'.
            # np.histogram returns the bin edges, so there will be 50 probability
            # density values in n, 51 bin edges in bins and 50 patches.  To get
            # everything lined up, we'll compute the bin centers
            bincenters = 0.5*(bins[1:]+bins[:-1])
            # add a 'best fit' line for the normal PDF
            y = mlab.normpdf( bincenters, mu, sigma)
            l = ax.plot(bincenters, y, 'r--', linewidth=1)

            ax.set_xlabel('Smarts')
            ax.set_ylabel('Probability')
            #ax.set_title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
            ax.set_xlim(40, 160)
            ax.set_ylim(0, 0.03)
            ax.grid(True)

            plt.show()
            
            print 'time to first good placement: ',mean(samplingtimes),std(samplingtimes)

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
