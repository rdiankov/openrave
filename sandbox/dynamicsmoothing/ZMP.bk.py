#import Numdiff
from openravepy import *
from numpy import *


# Compute the angular velocity mapping matrix
def romega(q,params):
    linkindex=params['linkindex']
    robot=params['robot']
    with robot:
        robot.SetDOFValues(q)
        return robot.CalculateAngularVelocityJacobian(linkindex)


def romega2(q,params):
    linkindex=params['linkindex']
    robot=params['robot']
    n=len(q)
    r=zeros((3,n))
    zv=zeros(n)
    with robot:
        robot.SetDOFValues(q)
        for i in range(n):
            zv[i]=1
            robot.SetDOFVelocities(zv)            
            zv[i]=0
            r[:,i]=robot.GetLinks()[linkindex].GetVelocity()[1]
    return r
            


# Com mapping function
def com(q,params):
    linkindex=params['linkindex']
    robot=params['robot']
    with robot:
        robot.SetDOFValues(q)
        return robot.GetLinks()[linkindex].GetGlobalCOM()

# Origin mapping function
def origin(q,params):
    linkindex=params['linkindex']
    robot=params['robot']
    with robot:
        robot.SetDOFValues(q)
        return robot.GetLinks()[linkindex].GetTransform()[0:3,3]




# Com Jacobian mapping function
def comJacobian2(q,params):
    return Numdiff.ComputeJacobian(com,q,1e-5,params)



def comJacobian(q,params):
    robot=params['robot']
    linkindex=params['linkindex']
    with robot:
        robot.SetDOFValues(q)
        return robot.CalculateJacobian(linkindex,robot.GetLinks()[linkindex].GetGlobalCOM())









#############################################

def ComputeJacobians(q,delta,i,params):
    robot=params['robot']
    with robot:
        robot.SetDOFValues(q)
        rpq_0=robot.CalculateJacobian(i,robot.GetLinks()[i].GetGlobalCOM())
        ro_0=robot.CalculateAngularVelocityJacobian(i)

    n=len(q)
    zv=zeros(n)
    m=list(shape(ro_0))
    m.append(n)
    rpqq=zeros(m)
    roq=zeros(m)
    for j in range(n):
        zv[j]=delta
        with robot:
            robot.SetDOFValues(q+zv)        
            rpq_j=robot.CalculateJacobian(i,robot.GetLinks()[i].GetGlobalCOM())
            ro_j=robot.CalculateAngularVelocityJacobian(i)
                                          
        zv[j]=0
        rpqq[:,j]=(rpq_j-rpq_0)/delta
        roq[:,j]=(ro_j-ro_0)/delta
    
    return [rpq_0,ro_0,rpqq,roq]



################################################

def ComputeZMP(q,qd,qdd,params_init):

    robot=params_init['robot']
    g=params_init['gravity']
    moment_coef=params_init['moment_coef']
    exclude_list=params_init['exclude_list']

    n=len(q)
    with robot:
        robot.SetDOFValues(q)
        robot.SetDOFVelocities(qd)

        com_pos=array([k.GetGlobalCOM() for k in robot.GetLinks()])
        vel=robot.GetLinkVelocities()
        acc=robot.GetLinkAccelerations(qdd) # Includes gravity term

        transforms=[k.GetTransform()[0:3,0:3] for k in robot.GetLinks()]
        masses=[k.GetMass() for k in robot.GetLinks()]
        inertiae=[k.GetLocalInertia() for k in robot.GetLinks()]
        localCOM=[k.GetLocalCOM() for k in robot.GetLinks()]
    

    xnum=0
    ynum=0
    denum=0

    for i in range(n+1):
        
        #print '--------------------'
        #print i
        if i in exclude_list:
            continue

        # Compute the inertia matrix in the global frame
        R=transforms[i]
        Ii=dot(R,dot(inertiae[i],transpose(R)))
        ri=dot(R,localCOM[i])

        # Compute the inertia moment
        omegai=vel[i,3:6]
        omegadi=acc[i,3:6]
        Mi=dot(Ii,omegadi)+cross(omegai,dot(Ii,omegai))

        com_vel=vel[i,0:3]+cross(omegai,ri)
        com_acc=acc[i,0:3]+cross(omegai,cross(omegai,ri))+cross(omegadi,ri)

        # Extract the position and accelerations
        xi=com_pos[i,0]
        yi=com_pos[i,1]
        zi=com_pos[i,2]
        xddi=com_acc[0]
        yddi=com_acc[1]
        zddi=com_acc[2]

        # Compute the numerators and denominator
        xnum+=masses[i]*(zddi*xi-xddi*zi)-moment_coef*Mi[1]
        ynum+=masses[i]*(zddi*yi-yddi*zi)-moment_coef*Mi[0]
        denum+=masses[i]*zddi
        
    #print xnum
    #print ynum
    return [xnum/denum,ynum/denum]



def ComputeCOM(q,params_init):
    robot=params_init['robot']
    g=params_init['gravity']
    exclude_list=params_init['exclude_list']

    n=len(q)
    with robot:
        robot.SetDOFValues(q)
        com_pos=array([k.GetGlobalCOM() for k in robot.GetLinks()])
        masses=[k.GetMass() for k in robot.GetLinks()]
     
    M=sum(masses)
    weighted_com=zeros(3)
    for i in range(n+1):
        if i in exclude_list:
            continue
        weighted_com+=masses[i]*array(com_pos[i])

    res=weighted_com/M
    return res[0:2]
    
    

        



################################################


def ComputeCoefsFraction(f_,fs,fss,params_init):
    n=len(f_)
    robot=params_init['robot']
    g=params_init['gravity']
    exclude_list=params_init['exclude_list']


    #Initialize the coefficients
    sd=2
    sdd=1.5
    ax,bx,cx=0,0,0
    ay,by,cy=0,0,0
    d,e,f=0,0,0
    
    for i in range(n+1):
        #print '--------------------'
        #print i
        if i in exclude_list:
            continue

        params={'linkindex':i,'robot':robot}

        with robot:
            robot.SetDOFValues(f_)
            LocalI=robot.GetLinks()[i].GetLocalInertia()
            R=robot.GetLinks()[i].GetTransform()[0:3,0:3]
            I=dot(R,dot(LocalI,transpose(R)))
            m=robot.GetLinks()[i].GetMass()

        # Terms in x,y,z
        rp=com(f_,params)
        x=rp[0]
        y=rp[1]
        z=rp[2]

        # Compute the Jacobians
        # with robot:
        #     robot.SetDOFValues(f_)
        #     rpq=robot.CalculateJacobian(i,robot.GetLinks()[i].GetGlobalCOM())
        
        # rpqq=Numdiff.ComputeJacobian(comJacobian,f_,1e-5,params)
        # ro=romega(f_,params)
        # roq=Numdiff.ComputeJacobian(romega,f_,1e-5,params)
        [rpq,ro,rpqq,roq]=ComputeJacobians(f_,1e-6,i,params)


        # Terms in xdd, ydd, zdd
        #rpq=Numdiff.ComputeJacobian(com,f_,1e-5,params)
 
        apdd=dot(rpq,fs)
        bpdd=dot(rpq,fss)+dot(fs,transpose(dot(rpqq,fs)))

        axdd=apdd[0]
        bxdd=bpdd[0]
        aydd=apdd[1]
        bydd=bpdd[1]
        azdd=apdd[2]
        bzdd=bpdd[2]
       
        # Terms in omega

        ro_fs=dot(ro,fs)
        aM=dot(dot(I,ro),fs)        
        bM=dot(I,dot(ro,fss)+dot(fs,transpose(dot(roq,fs))))+cross(ro_fs,dot(I,ro_fs))

        aMx=aM[0]
        bMx=bM[0]
        aMy=aM[1]
        bMy=bM[1]
       
        # Computations of the coefficients of the numerators and denominator
        ax+=m*(azdd*x-axdd*z)-aMy
        bx+=m*(bzdd*x-bxdd*z)-bMy
        cx+=m*g*x

        ay+=m*(azdd*y-aydd*z)-aMx
        by+=m*(bzdd*y-bydd*z)-bMx
        cy+=m*g*y

        d+=m*azdd
        e+=m*bzdd
        f+=m*g

    return [ax,bx,cx,ay,by,cy,d,e,f]
