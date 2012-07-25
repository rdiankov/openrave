from openravepy import *
from numpy import *

def ComputeJacobian(f,q,delta,params=None):
    n=len(q)
    zv=zeros(n)
    p0=f(q,params)
    m=list(shape(p0))
    m.append(n)
    J=zeros(m)
    for j in range(n):
        zv[j]=delta
        pdelta=f(q+zv,params)
        zv[j]=0
        J[:,j]=(pdelta-p0)/delta

    return J
