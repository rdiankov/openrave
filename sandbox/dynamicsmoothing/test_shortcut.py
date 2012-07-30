from openravepy import *
from numpy import *
from pylab import *
import scipy
import time
import Image
import Trajectory
import MinimumTime
import Shortcutting
set_printoptions(precision=5)



################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/cuong/Dropbox/Code/mintime/robots/arm.robot.xml')
env.Load('robots/table.kinbody.xml')


collisionChecker = RaveCreateCollisionChecker(env,'pqp')
env.SetCollisionChecker(collisionChecker)

robot=env.GetRobots()[0]
table=env.GetKinBody('table').GetLinks()[0]
table.SetTransform(array([[0,0,0,-1],[0,0,0,0.4],[0,0,0,0.4],[0,0,0,1]]))

grav=[0,0,-9.8]

# DOF and velocity limits
n=robot.GetDOF()
dof_lim=robot.GetDOFLimits()
vel_lim=robot.GetDOFVelocityLimits()
robot.SetDOFLimits(-10*ones(n),10*ones(n))
robot.SetDOFVelocityLimits(100*vel_lim)




################# Initialize a trajectory ########################

q0=[0,0,0,0]
q1=[ 0.8345 , 0,  0,  0]
q2=[ 0.8345 , -0.13497,  0.97706,  1.94169]
q3=[ 0.8345 ,  1.61082,  0.97706,  1.94169]
q4=[ 2.32883,  1.61082,  0.97706,  1.94169]
v=0.1
v2=0.1
qd0=[v,v,v,v]
qd1=[v2,v2,v2,v2]
qd2=[v2,v2,v2,v2]
qd3=[v2,v2,v2,v2]
qd4=[v,v,v,v]

q_list=[q0,q1,q2,q3,q4]
qd_list=[qd0,qd1,qd2,qd3,qd4]
T_list=[1,1,2,1]
pwp_traj=Trajectory.Interpolate(q_list,qd_list,T_list)

#pwp_traj=Trajectory.Interpolate([[0,0,0,pi/2],[0,0,0,pi/2]],[[0,0,0,0],[0,0,0,0]],[1])



T=sum(T_list)
n_discr=500.
t_step=T/n_discr
a=time.time()
traj=pwp_traj.GetSampleTraj(T,t_step)

# Compute the inverse dynamics of the original trajectory and plot
tau=Trajectory.ComputeTorques(robot,traj,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
plot(transpose(tau))
show()



# Visualize the trajectory (see the OpenRave window)
#robot.SetDOFValues([0,0,0,0])
#Trajectory.Execute(robot,traj,0.01)



################# Compute a dynamically-feasible timing ########################

# Torques limits
m0=6
m1=15
m2=5
m3=4
m=max([m0,m1,m2,m3])
tau_min=[-m0,-m1,-m2,-m3]
tau_max=[m0,m1,m2,m3]

# Some tuning parameters
tunings={'':0}
tunings['i_threshold']=10 # threshold in the tangent points search
tunings['slope_threshold']=1 # threshold in the tangent points search
tunings['sdot_threshold']=0.05 # threshold in the discontinuity points search
tunings['a_threshold']=0.01 # threshold in the zero inertia points search
tunings['width']=20 # window to test if we can get through a switching point
tunings['tolerance']=0.001 #tolerance above the max_curve
tunings['t_step_integrate']=t_step/20 # time step to integrate the limiting curves
tunings['sdot_init']=1
tunings['sdot_final']=1
tunings['threshold_final']=1e-2
tunings['threshold_waive']=1e-2
tunings['palier']=0 #length of the palier around disc and zero inertia points

# Compute a feasible timing
pb=MinimumTime.RobotMinimumTime(robot,traj,tau_min,tau_max,tunings,grav)
s_res=pb.s_res
sdot_res=pb.sdot_res
undersample_coef=int(round(t_step/tunings['t_step_integrate']))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=pwp_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)

print 'Duration of the original trajectory: '+str(traj.duration)+' seconds'
print 'Duration of the retimed trajectory: '+str(traj2.duration) +' seconds'

# Inverse dynamics of the retimed trajectory and plot
tau2=Trajectory.ComputeTorques(robot,traj2,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(4)
Trajectory.PlotTorques(traj2.t_vect,tau2,tau_min,tau_max)


# si=pb.s_list[11]
# sdoti=pb.sdot_list[11]
# #si=pb.s_res[200]
# #sdoti=pb.sdot_res[200]
# si=0.98
# sdoti=4
# ns=30
# nsdot=10
# dt=0.005
# dtdot=0.1
# pb.plot_arrows(si,sdoti,ns,nsdot,dt,dtdot)



################# Run the shortcutting algorithm ########################


# Tuning parameters
tunings['t_step_sample']=0.005 # time step to sample the shortcut
tunings['t_step_integrate']=0.001 # time step to integrate the limiting curves

max_time=20 # Upper bound for the execution of the algorithm in seconds
mean_len=traj2.duration/2 # Mean length of each shortcut
std_len=traj2.duration/2 # Std of the length of the shortcut

# Run the algorithm
n_runs=100
trajlist=[]
ls=[]
la=[]
lc=[]
coeft=1.1
i=0
while i <n_runs:
    print i
    [traj3,d_list_s,d_list_all,n_collisions]=Shortcutting.IterateSmooth(robot,traj2,tau_min,tau_max,tunings,grav,max_time,mean_len,std_len)
    tau3=Trajectory.ComputeTorques(robot,traj3,grav)
    print [max(abs(tau3[k,:])) for k in range(4)]
    if True not in [max(abs(tau3[k,:]))/coeft>tau_max[k] for k in range(4)]:  
        trajlist.append(traj3)
        ls.append(d_list_s)
        la.append(d_list_all)
        lc.append(n_collisions)
        i+=1

r=[k[-1] for k in la]
mean(r)
min(r)



# Save or load the result
saveload='nil'
if saveload=='save':
    data={'trajlist':trajlist,'la':la,'ls':ls,'lc':lc}
    f=open('data.sav','w')
    pickle.dump(data,f)
    f.close()

if saveload=='load':
    f=open('data.sav','r')
    data=pickle.load(f)
    f.close()
    la=data['la']
    ls=data['ls']
    lc=data['lc']
    trajlist=data['trajlist']


dd=[k[-1] for k in la]
i=dd.index(min(dd))
i=0
traj3=trajlist[i]

#dd2=[min(dd[10*i:10*i+9]) for i in range(9)]



# Inverse dynamics of the shortcutted trajectory and plot
tau3=Trajectory.ComputeTorques(robot,traj3,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(4)
Trajectory.PlotTorques(traj3.t_vect,tau3,tau_min,tau_max)







# ################# Resample the shortcutted trajectory ###########


# spline_traj=Trajectory.SplineInterpolateTrajectory(traj3.t_vect,traj3.q_vect,traj3.t_step)

# # Compute a feasible timing
# tunings['tolerance']=0.005
# tunings['width']=50
# pb2=MinimumTime.RobotMinimumTime(robot,traj3,tau_min,tau_max,tunings,grav)
# s_res=pb2.s_res
# sdot_res=pb2.sdot_res
# undersample_coef=int(round(t_step/tunings['t_step_integrate']))
# s_res_u=s_res[range(1,len(s_res),undersample_coef)]
# sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
# traj5=spline_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)

# # Inverse dynamics of the retimed trajectory
# tau5=Trajectory.ComputeTorques(robot,traj5,grav)

# print 'Duration of the original trajectory: '+str(traj3.duration)+' seconds'
# print 'Duration of the retimed trajectory: '+str(traj5.duration) +' seconds'


# # Plot the torques
# T2=traj2.duration
# mpl.axes.set_default_color_cycle(['r','b','g','m'])
# figure(4)
# clf()
# hold('on')
# #plot(traj.t_vect,transpose(tau),'--')
# plot(traj2.t_vect,transpose(tau2),linewidth=2)
# #plot(traj3.t_vect,transpose(tau3),linewidth=2)
# plot([0,T2],array([tau_min,tau_min]),'--',linewidth=2)
# plot([0,T2],array([tau_max,tau_max]),'--',linewidth=2)
# for k in gca().get_xticklabels():
#     k.set_fontsize(18)

# for k in gca().get_yticklabels():
#     k.set_fontsize(18)

# xlabel('Time (s)',fontsize=20)
# ylabel('Torque (Nm)',fontsize=20)
# axis([0,T2,-m*1.2,m*1.2])
# grid('on')
# show()



######################## Convert to parabola #############################


(P_list,T_list)=MinimumTime.ApproximateDichotomy(robot,traj3,tau_min,tau_max,grav,0,len(traj3.t_vect)-1,1.4)
pwptraj4=Trajectory.PieceWisePolyTrajectory(P_list,T_list)
traj4=pwptraj4.GetSampleTraj(0.01)
tau4=Trajectory.ComputeTorques(robot,traj4,grav)


T4=traj4.duration
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(4)
clf()
hold('on')
#plot(traj.t_vect,transpose(tau),'--')
#plot(traj2.t_vect,transpose(tau2),linewidth=2)
plot(traj4.t_vect,transpose(tau4),linewidth=2)
plot([0,T4],array([tau_min,tau_min]),'--',linewidth=2)
plot([0,T4],array([tau_max,tau_max]),'--',linewidth=2)
axis([0,T4,-m*1.2,m*1.2])
for k in gca().get_xticklabels():
    k.set_fontsize(18)

for k in gca().get_yticklabels():
    k.set_fontsize(18)

xlabel('Time (s)',fontsize=20)
ylabel('Torque (Nm)',fontsize=20)
grid('on')
show()



########################## Visualizing and plotting #######################


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







#################### Stats ###########################



ll=la
lenmax=max([len(k) for k in ll])
mint=min([k[-1] for k in la])
ortho=[]
for i in range(lenmax):
    col=[]
    for k in ll:
        if len(k)>=i+1:
            col.append(k[i])
        else:
            col.append(k[-1])
    ortho.append(col)

meanv=[mean(k) for k in ortho]
stdv=[std(k) for k in ortho]
meanv.insert(0,traj2.duration)
stdv.insert(0,0)

figure(5)
clf()
errorbar(range(lenmax+1),meanv,stdv,None,'k',linewidth=2)
plot([0,lenmax],[mint,mint],'k--',linewidth=2)
for k in gca().get_xticklabels():
    k.set_fontsize(18)

for k in gca().get_yticklabels():
    k.set_fontsize(18)

xlabel('Number of shortcuts attempted', fontsize=20)
ylabel('Trajectory duration (s)', fontsize=20)
axis([0,lenmax,0,2.5])
grid('on')


bins=[]
for i in range(10):
    bins.append(min(dd[10*i:10*i+10]))

mean(bins)
std(bins)


###################### Images #####################

V=env.GetViewer()
M=array([[-0.57592,  0.05446, -0.81569,  1.31467],
       [ 0.81748,  0.04612, -0.57411,  1.75801],
       [ 0.00635, -0.99745, -0.07108,  0.45978],
       [ 0.     ,  0.     ,  0.     ,  1.     ]])
V.SetCamera(M)

n_snaps=11
box=[20,0,330,480]
ct=traj3
ni=0
for i in [int(round(k)) for k in linspace(0,ct.n_steps-1,n_snaps)]:
    robot.SetDOFValues(ct.q_vect[:,i])
    I=V.GetCameraImage(640,480,M,[640,640,320,240])
    scipy.misc.imsave('tmp.jpg',I)
    im=Image.open('tmp.jpg')
    im2=im.crop(box)
    im2.save('../../Reda/mintime/fig/snap2-'+str(ni)+'.eps')
    ni+=1










