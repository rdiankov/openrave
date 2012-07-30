from openravepy import *
from numpy import *
from pylab import *
import sys
import time
import Trajectory
import MinimumTime
import Shortcutting
import sys
import cStringIO


set_printoptions(precision=5)
save_stdout = sys.stdout


################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/cuong/Dropbox/Code/mintime/robots/twodof.robot.xml')

robot=env.GetRobots()[0]
grav=[0,9.8,0]



# DOF and velocity limits
n=robot.GetDOF()
dof_lim=robot.GetDOFLimits()
vel_lim=robot.GetDOFVelocityLimits()
robot.SetDOFLimits(-10*ones(n),10*ones(n))
robot.SetDOFVelocityLimits(100*vel_lim)



################# Initialize a trajectory ########################

P0=poly1d([1,-1,1,0.5])
P1=poly1d([1,-3,1])
pwp_traj=Trajectory.PieceWisePolyTrajectory([[P0,P1]],[1])

T=1
n_discr=500.
t_step=T/n_discr
a=time.clock()
traj=pwp_traj.GetSampleTraj(T,t_step)

# Compute the inverse dynamics of the original trajectory
tau=Trajectory.ComputeTorques(robot,traj,grav)
#mpl.axes.set_default_color_cycle(['r','b'])
figure(3)
clf()
Trajectory.PlotTorques(traj.t_vect,tau,tau_min,tau_max)







################# Compute a dynamically-feasible timing ########################

# Torques limits
tau_min=[-30,-30]
tau_max=[10,10]

# Some tuning parameters
tunings={'':0}
tunings['i_threshold']=10 # threshold in the tangent points search
tunings['slope_threshold']=1 # threshold in the tangent points search
tunings['sdot_threshold']=0.05 # threshold in the discontinuity points search
tunings['a_threshold']=0.01 # threshold in the zero inertia points search
tunings['width']=5 # window to test if we can get through a switching point
tunings['tolerance']=0.001 #tolerance above the max_curve
tunings['t_step_integrate']=t_step/8 # time step to integrate the limiting curves
tunings['sdot_init']=0.01
tunings['sdot_final']=0.01
tunings['threshold_final']=1e-2
tunings['threshold_waive']=1e-2
tunings['palier']=10 #length of the palier around disc and zero inertia points

# Compute a feasible timing
pb=MinimumTime.RobotMinimumTime(robot,traj,tau_min,tau_max,tunings,grav)
pb.plot_limiting_curves()



# Compute the unique acceleration

s0=pb.s_list[0]
sd0=pb.sdot_list[0]
delta=0.01
i_sat=0
[a,b,c]=pb.dynamics_coefficients(s0)
a=a[i_sat]
b=b[i_sat]
c=c[i_sat]
[a2,b2,c2]=pb.dynamics_coefficients(s0-delta)
a2=a2[i_sat]
b2=b2[i_sat]
c2=c2[i_sat]
ap=-(a2-a)/delta
bp=-(b2-b)/delta
cp=-(c2-c)/delta
lam=(-bp*sd0*sd0-cp)/(2*b*sd0+ap*sd0)

rod=0.01

ps=[s0,sd0]
p1=[s0-rod,sd0-rod*lam]
p2=[s0+rod,sd0+rod*lam]
n_discr=20
v_shift=0.005
dt=4e-6
n_int=1000

figure(3)
clf()
plot(pb.sample_t_vect,pb.sample_max_curve,'k',linewidth=2)
for i in range(0,n_discr):
    r1=i/float(n_discr)
    r2=1-r1
    x_i=r2*ps[0]+r1*p1[0]
    y_i=r2*ps[1]+r1*p1[1]
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i-v_shift,dt,n_int,'alpha')
    plot(s_flow,sdot_flow,'b')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i-v_shift,-dt,n_int,'alpha')
    plot(s_flow,sdot_flow,'b')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i+v_shift,dt,n_int,'alpha')
    plot(s_flow,sdot_flow,'b')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i+v_shift,-dt,n_int,'alpha')
    plot(s_flow,sdot_flow,'b')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i,dt,n_int,'beta')
    plot(s_flow,sdot_flow,'c')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i,-dt,n_int,'beta')
    plot(s_flow,sdot_flow,'c')
    x_i=r2*ps[0]+r1*p2[0]
    y_i=r2*ps[1]+r1*p2[1]
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i-v_shift,dt,n_int,'beta')
    plot(s_flow,sdot_flow,'g')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i-v_shift,-dt,n_int,'beta')
    plot(s_flow,sdot_flow,'g')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i+v_shift,dt,n_int,'beta')
    plot(s_flow,sdot_flow,'g')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i+v_shift,-dt,n_int,'beta')
    plot(s_flow,sdot_flow,'g')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i,dt,n_int,'alpha')
    plot(s_flow,sdot_flow,'y')
    [s_flow,sdot_flow]=pb.compute_flow(x_i,y_i,-dt,n_int,'alpha')
    plot(s_flow,sdot_flow,'y')




s=pb.s_list[0]
i_sat=0
x_v=[]
y_v=[]
for i in range(0,n_discr):
    r1=i/float(n_discr)
    r2=1-r1
    x_i=r2*(s-0.1)+r1*(s+0.1)
    [a,b,c]=pb.dynamics_coefficients(x_i)
    y_i=sqrt((tau_min[i_sat]-c[i_sat])/b[i_sat])
    x_v.append(x_i)
    y_v.append(y_i)




rod2=0.1
plot([s0-rod2,s0+rod2],[sd0-lam*rod2,sd0+lam*rod2],'r',linewidth=2)
plot(x_v,y_v,'m--',linewidth=2)
plot(pb.s_list[0],pb.sdot_list[0],'ro',markersize=10)
axis([0,1,0,20])






















# Compute the final trajectory


s_res=pb.s_res
sdot_res=pb.sdot_res
undersample_coef=int(round(t_step/tunings['t_step_integrate']))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=pwp_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)

# Inverse dynamics of the retimed trajectory
tau2=Trajectory.ComputeTorques(robot,traj2,grav)
mpl.axes.set_default_color_cycle(['r','b'])
figure(1)
clf()
Trajectory.PlotTorques(traj2.t_vect,tau2,tau_min,tau_max)

print 'Duration of the original trajectory: '+str(traj.duration)+' seconds'
print 'Duration of the retimed trajectory: '+str(traj2.duration) +' seconds'





########################## Visualizing and plotting #######################

s=[0.431,0.932]
sd=[1.155,1.31]
figure(0)
clf()
hold('on')
plot(pb.sample_t_vect,pb.sample_max_curve,'k--',linewidth=2)
for i in range(len(pb.s_traj_list)):
    plot(pb.s_traj_list[i],pb.sdot_traj_list[i],'r',linewidth=2)

plot(pb.s_res,pb.sdot_res,'b',linewidth=2)
for i in range(len(pb.s_list)):
    if pb.type_list[i]=='t':
        plot(pb.s_list[i],pb.sdot_list[i],'ks')
    if pb.type_list[i]=='d':
        plot(pb.s_list[i],pb.sdot_list[i],'bo')
    if pb.type_list[i]=='z':
        plot(pb.s_list[i],pb.sdot_list[i],'go')


plot(s,sd,'ko')
axis([0,T,0,3])
grid('on')
show()       







# Plot q (joint angles)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(1)
clf()
hold('on')
plot(traj.t_vect,transpose(traj.q_vect),'--')
plot(traj2.t_vect,transpose(traj2.q_vect))
plot(traj3.t_vect,transpose(traj3.q_vect),linewidth=3)
grid('on')


# Plot qd (speed)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(2)
clf()
hold('on')
plot(traj.t_vect,transpose(traj.qd_vect),'--')
plot(traj2.t_vect,transpose(traj2.qd_vect))
plot(traj3.t_vect,transpose(traj3.qd_vect),linewidth=3)
grid('on')


# Plot qdd (acc)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(3)
clf()
hold('on')
plot(traj.t_vect,transpose(traj.qdd_vect),'--')
plot(traj2.t_vect,transpose(traj2.qdd_vect))
plot(traj3.t_vect,transpose(traj3.qdd_vect),linewidth=3)
grid('on')


# Plot the torques
tv=[0.67,1.04,1.11]
mpl.axes.set_default_color_cycle(['r','b'])
T2=max(traj2.t_vect)
N=len(traj2.t_vect)
r=1.2
figure(4)
clf()
hold('on')
plot(traj2.t_vect[2:N-2],transpose(tau2[:,2:N-2]),linewidth=2)
#plot(traj.t_vect,transpose(tau),linewidth=1)
plot([0,T2],array([tau_min,tau_min]),'--',linewidth=2)
plot([0,T2],array([tau_max,tau_max]),'--',linewidth=2)
plot([tv[0],tv[0]],[-m*r,m*r],'k--',linewidth=1.3)
plot([tv[1],tv[1]],[-m*r,m*r],'k--',linewidth=1.3)
plot([tv[2],tv[2]],[-m*r,m*r],'k--',linewidth=1.3)
axis([0,T2,-m*1.2,m*1.2])
grid('on')


show()





# Visualize the algorithms (see the OpenRave window)

robot.SetDOFValues([0,0,0,0])
time.sleep(0.5)
Trajectory.Execute(robot,traj,0.03)
time.sleep(0.5)

robot.SetDOFValues([0,0,0,0])
time.sleep(0.5)
Trajectory.Execute(robot,traj2,0.03)
time.sleep(0.5)

robot.SetDOFValues([0,0,0,0])
time.sleep(0.5)
Trajectory.Execute(robot,traj3,0.03)
time.sleep(0.5)

