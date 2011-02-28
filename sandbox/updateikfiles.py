"""updates the cached ikfast files in the plugins/ikfastsolvers directory
"""
from openravepy import *
import time,platform,shutil,os
import logging

def updateik(robotfilename,manipname,iktype,destfilename):
    print robotfilename, manipname, iktype, destfilename
    env=Environment()
    try:
        with env:
            robot = env.ReadRobotXMLFile(robotfilename,{'skipgeometry':'1'})
            env.AddRobot(robot)
            if manipname is not None:
                manip = robot.SetActiveManipulator(manipname)
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=iktype)
            ikmodel.manip.SetIKSolver(None)
            if not ikmodel.load():
                ikmodel.autogenerate()
            ikmodel.setrobot()
            successrate, wrongrate = ikmodel.testik('10') # sanity check
            assert(wrongrate==0)
            shutil.copyfile(ikmodel.getsourcefilename(True), destfilename)
    finally:
        env.Destroy()

if __name__ == "__main__":
    RaveInitialize()
    format = logging.Formatter('%(name)s: %(message)s')
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(format)
    ikfast.log.addHandler(handler)
    ikfast.log.setLevel(logging.ERROR)
    destdir = '../plugins/ikfastsolvers'
    updateik('robots/puma.robot.xml',None,IkParameterization.Type.Transform6D,destdir+'/ik_puma.cpp')
    updateik('robots/barrettwam.robot.xml',None,IkParameterization.Type.Transform6D,destdir+'/ik_barrettwam.cpp')
    updateik('robots/pa10schunk.robot.xml',None,IkParameterization.Type.Transform6D,destdir+'/ik_pa10.cpp')
    updateik('robots/pr2-beta-static.zae','head',IkParameterization.Type.Lookat3D,destdir+'/ik_pr2_head.cpp')
    updateik('robots/pr2-beta-static.zae','head_torso',IkParameterization.Type.Lookat3D,destdir+'/ik_pr2_head_torso.cpp')
    updateik('robots/pr2-beta-static.zae','leftarm',IkParameterization.Type.Transform6D,destdir+'/ik_pr2_leftarm.cpp')
    updateik('robots/pr2-beta-static.zae','rightarm',IkParameterization.Type.Transform6D,destdir+'/ik_pr2_rightarm.cpp')
    updateik('robots/pr2-beta-static.zae','leftarm_torso',IkParameterization.Type.Transform6D,destdir+'/ik_pr2_leftarm_torso.cpp')
    updateik('robots/pr2-beta-static.zae','rightarm_torso',IkParameterization.Type.Transform6D,destdir+'/ik_pr2_rightarm_torso.cpp')
    updateik('robots/schunk-lwa3.zae',None,IkParameterization.Type.Transform6D,destdir+'/ik_schunk_lwa3.cpp')
    updateik('robots/neuronics-katana.zae',None,IkParameterization.Type.TranslationDirection5D,destdir+'/ik_katana5d.cpp')
    
