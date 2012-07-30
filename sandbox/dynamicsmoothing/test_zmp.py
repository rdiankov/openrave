from openravepy import *
from numpy import *
from pylab import *
import scipy
import time
import ZMP
import Trajectory
import MinimumTimeZMP
set_printoptions(precision=5)



################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
#env.Load('/home/cuong/Dropbox/Code/mintime/robots/arm.robot.xml')
env.Load('/home/cuong/Dropbox/Code/mintime/robots/hrp4r.dae')

robot=env.GetRobots()[0]
g=9.8
env.GetPhysicsEngine().SetGravity([0,0,-g])
params={'robot':robot,'gravity':g,'moment_coef':1}


q0=array([0,0,0,0])
q1=array([1, 0.5,  0.5,  1])
qd0=array([0.1,0.1,0.1,0.1])
qd1=array([0.1,0.1,0.1,0.1])
q_list=[q0,q1]
qd_list=[qd0,qd1]
T_list=[1]
pwp_traj=Trajectory.Interpolate(q_list,qd_list,T_list)

T=sum(T_list)
n_discr=100.
t_step=T/n_discr
traj=pwp_traj.GetSampleTraj(t_step)



#figure(1)
#clf()
#plot(transpose(traj.q_vect))
#plot(transpose(traj.qd_vect))
#grid('on')


#Trajectory.Execute(robot,traj,0.01)


xmin=-0.02
xmax=0.01
ymin=0.0
ymax=0.03
bounds=[xmin,xmax,ymin,ymax]
tunings={'':0}
tunings['i_threshold']=10 # threshold in the tangent points search
tunings['slope_threshold']=1 # threshold in the tangent points search
tunings['sdot_threshold']=0.05 # threshold in the discontinuity points search
tunings['a_threshold']=0.01 # threshold in the zero inertia points search
tunings['width']=5 # window to test if we can get through a switching point
tunings['tolerance']=0.005 #tolerance above the max_curve
tunings['t_step_integrate']=t_step/10 # time step to integrate the limiting curves
tunings['sdot_init']=1
tunings['sdot_final']=1
tunings['threshold_final']=1e-2
tunings['threshold_waive']=1e-2

pb=MinimumTimeZMP.RobotMinimumTime(robot,traj,bounds,tunings,params)
s_res=pb.s_res
sdot_res=pb.sdot_res
undersample_coef=int(round(t_step/tunings['t_step_integrate']))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=pwp_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)


figure(2)
clf()
[xzmp,yzmp]=Trajectory.ComputeZMP(traj,params)
[xzmp2,yzmp2]=Trajectory.ComputeZMP(traj2,params)
[xcom,ycom]=Trajectory.ComputeCOM(traj,params)
plot([xmin,xmin,xmax,xmax,xmin],[ymin,ymax,ymax,ymin,ymin],'k--',linewidth=2)
plot(xzmp,yzmp,'r')
plot(xzmp2,yzmp2,'b')
plot(xcom,ycom,'k')
axis('equal')
grid('on')


figure(3)
pb.plot_limiting_curves()













###### Test numerical differentiations

q=array([-1,pi/4,-1,1])
qd=array([0.1,-0.1,-0.2,.1])
qdd=array([0,1000,0,0])

robot.SetDOFValues(q)
robot.SetDOFVelocities(2*qd)
v2=robot.GetLinkVelocities()


J=robot.CalculateJacobian(2,[0,0,0,0])


J=Numdiff.ComputeJacobian(com,q,1e-5,{'linkindex':2})
H=Numdiff.ComputeJacobian(comJacobian,q,1e-5,{'linkindex':2})
romegaq=Numdiff.ComputeJacobian(romega,q,1e-5,{'linkindex':2})

i=4
r1=Numdiff.ComputeJacobian(ZMP.com,q,1e-5,{'linkindex':i,'robot':robot})
with robot:
    robot.SetDOFValues(q)
    r2=robot.CalculateJacobian(i,robot.GetLinks()[i].GetGlobalCOM())

print r1
print r2



i=4
with robot:
    robot.SetDOFValues(q)
    robot.SetDOFVelocities(qd)
    vel=robot.GetLinkVelocities()
    #ri=robot.GetLinks()[i].GetGlobalCOM()-robot.GetLinks()[i].GetTransform()[0:3,3]
    R=robot.GetLinks()[i].GetTransform()[0:3,0:3]
    ri=dot(R,robot.GetLinks()[i].GetLocalCOM())

vi=vel[i,0:3]
omegai=vel[i,3:6]
com_vel=vi+cross(omegai,ri)
print com_vel

rq=Numdiff.ComputeJacobian(ZMP.com,q,1e-7,{'linkindex':i,'robot':robot})
print rq
print dot(rq,qd)



#### Test the basic ZMP computation


sd=2
sdd=1.5
f_=array([0.2,-0.3,-0.4,0.5])
fs=array([0.7,0.4,0.9,-1])
fss=array([0.9,-0.5,-0.4,-2])
q=f_
qd=sd*fs
qdd=sdd*fs+sd*sd*fss



ZMP.ComputeZMP(q,qd,qdd,params)
[ax,bx,cx,ay,by,cy,d,e,f]=ZMP.ComputeCoefsFraction(f_,fs,fss,params)
(ax*sdd+bx*sd*sd+cx)/(d*sdd+e*sd*sd+f)
(ay*sdd+by*sd*sd+cy)/(d*sdd+e*sd*sd+f)

deb=time.time()
for i in range(100):
    [ax,bx,cx,ay,by,cy,d,e,f]=ZMP.ComputeCoefsFraction(f_,fs,fss,params)

print time.time()-deb




