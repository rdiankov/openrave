"""updates the cached ikfast files in the plugins/ikfastsolvers directory
"""
from openravepy import *
import time,platform,os,sys
import logging
from openravepy import ikfast

def updateik(robotfilename,manipname,iktype,destfilename,freeindices=None):
    print robotfilename, manipname, iktype, destfilename
    robotid = os.path.split(destfilename)[1][:-4]
    env=Environment()
    try:
        with env:
            robot = env.ReadRobotXMLFile(robotfilename,{'skipgeometry':'1'})
            env.AddRobot(robot)
            if manipname is not None:
                manip = robot.SetActiveManipulator(manipname)
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=iktype,freeindices=freeindices)
            ikmodel.manip.SetIKSolver(None)
            if not ikmodel.load():
                ikmodel.autogenerate()
            ikmodel.setrobot()
            successrate, wrongrate = ikmodel.testik('100' if ikmodel.manip.GetIkSolver().GetNumFreeParameters() <= 1 else '10') # sanity check
            assert(wrongrate==0)
            print 'success: ',successrate
            code = """#define IKFAST_NAMESPACE %s
#include "plugindefs.h"

"""%robotid
            code += open(ikmodel.getsourcefilename(True),'r').read()
            code += """
#include "ikbase.h"
namespace IKFAST_NAMESPACE {
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(IKSolution)
#endif
IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr penv, const std::vector<dReal>& vfreeinc) {
    std::vector<int> vfree(getNumFreeParameters());
    for(size_t i = 0; i < vfree.size(); ++i) {
        vfree[i] = getFreeParameters()[i];
    }
    return IkSolverBasePtr(new IkFastSolver<IKReal,IKSolution>(ik,vfree,vfreeinc,getNumJoints(),static_cast<IkParameterizationType>(getIKType()), boost::shared_ptr<void>(), getKinematicsHash(), penv));
}
} // end namespace
"""
            open(destfilename,'w').write(code)
    finally:
        env.Destroy()

if __name__ == "__main__":
    RaveInitialize()
    try:
        destdir = '../plugins/ikfastsolvers'
        updateik('robots/puma.robot.xml',None,IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_puma.cpp'))
        updateik('robots/barrettwam.robot.xml',None,IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_barrettwam.cpp'))
        updateik('robots/pa10schunk.robot.xml','arm',IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_pa10.cpp'))
        updateik('robots/pr2-beta-static.zae','head',IkParameterization.Type.Lookat3D,os.path.join(destdir,'ik_pr2_head.cpp'))
        updateik('robots/pr2-beta-static.zae','head_torso',IkParameterization.Type.Lookat3D,os.path.join(destdir,'ik_pr2_head_torso.cpp'))
        updateik('robots/pr2-beta-static.zae','leftarm',IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_pr2_leftarm.cpp'))
        updateik('robots/pr2-beta-static.zae','rightarm',IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_pr2_rightarm.cpp'))
        updateik('robots/pr2-beta-static.zae','leftarm_torso',IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_pr2_leftarm_torso.cpp'))
        updateik('robots/pr2-beta-static.zae','rightarm_torso',IkParameterization.Type.Transform6D,destdir+'/ik_pr2_rightarm_torso.cpp')
        updateik('robots/schunk-lwa3.zae',None,IkParameterization.Type.Transform6D,os.path.join(destdir,'ik_schunk_lwa3.cpp'))
        updateik('robots/neuronics-katana.zae','arm',IkParameterization.Type.TranslationDirection5D,os.path.join(destdir,'ik_katana5d.cpp'))
        updateik('robots/neuronics-katana.zae','armgrasp',IkParameterization.Type.Translation3D,os.path.join(destdir,'ik_katana5d_trans.cpp'),[3,4])
        print 'finished updating all files'
    finally:
        RaveDestroy()
