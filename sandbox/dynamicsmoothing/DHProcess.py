"""
Compute the Denavit-Hartenberg parameters and associated DH frames
"""


from numpy import *
from openravepy import *


###########################
# This class holds the DH parameter for the last link
class DHParameterDummy():
    def __init__(self,d,a,theta,alpha):
        self.d=d
        self.a=a
        self.theta=theta
        self.alpha=alpha


###########################
# Compute the DH transform corresponding to given DH parameters
def DHtoMat(d,a,theta,alpha):
    ca=cos(alpha)
    sa=sin(alpha)
    ct=cos(theta)
    st=sin(theta)
    M=[[ct,-ca*st,sa*st,a*ct],
       [st,ca*ct,-sa*ct,a*st],
       [0,sa,ca,d],
       [0,0,0,1]]
    return(array(M))


###########################
# Compute the DH global and local frames
def ComputeDHFrames(dhParametersZeros,transform0,q):
    n=len(q)
    locDH=[]
    gloDH=[]
    M_cur=transform0
    for i in range(n+1):
        if i==0:
            qi=0
        else:
            qi=q[i-1]
        d=dhParametersZeros[i].d
        a=dhParametersZeros[i].a
        theta=dhParametersZeros[i].theta+qi
        alpha=dhParametersZeros[i].alpha        
        M=DHtoMat(d,a,theta,alpha)
        M_cur=dot(M_cur,M)
        locDH.append(M)
        gloDH.append(M_cur)
    return(gloDH,locDH)


############################
# Compute the local inertia and COM in the DH frames
def ComputeDHCOMInertia(robot,gloDH):
    inertiaDH=[]
    comDH=[]
    for i in range(len(gloDH)):
        link=robot.GetLinks()[i]
        localCOM=link.GetLocalCOM()
        localInertia=link.GetLocalInertia()
        R=dot(InvTransform(gloDH[i]),link.GetTransform())        
        comDH.append(dot(R,Add1(localCOM))[0:3])
        inertiaDH.append(dot(dot(R[0:3,0:3],localInertia),transpose(R[0:3,0:3])))
        #inertiaDH.append(dot(R[0:3,0:3],localInertia))
    return(comDH,inertiaDH)


###########################
# Invert a transformation matrix
def InvTransform(M):
    M1=numpy.zeros((4,4))
    R=M[0:3,0:3]
    T=M[0:3,3]
    M1[0:3,0:3]=transpose(R)
    M1[0:3,3]=-dot(transpose(R),T)
    M1[3,3]=1
    return(M1)


###########################
# Add 1 to the end of a vector
def Add1(x):
    n=len(x)
    x1=zeros(n+1)
    x1[0:n]=x
    x1[n]=1
    return(x1)

def Rot(a):
    return(array([[cos(a),sin(a)],[-sin(a),cos(a)]]))
