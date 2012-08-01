# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
# 
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
from common_test_openrave import *
import imp

log=logging.getLogger('openravepytest')

@with_destroy
def test_pluginloading():
    RaveInitialize(load_all_plugins=False)
    assert(RaveLoadPlugin('ikfastsolvers'))
    assert(RaveLoadPlugin('libikfastsolvers'))
    env=Environment()
    assert(RaveCreateProblem(env,'ikfast') is not None)

class RunTutorialExample(object):
    __name__= 'test_global.tutorialexample'
    def __call__(self,modulepath):
        # import the module, and at the end call RaveDestroy?
        modulename = os.path.split(modulepath)[1]
        description = 'test_programs.tutorialexample.%s'%(modulename)
        log.info('execute tutorial %s',modulepath)
        RaveDestroy()
        try:
            fp, pathname, description = imp.find_module(modulepath)
            module=imp.load_module(modulename, fp, pathname, description)
        finally:
            log.info('tutorial %s finished',modulepath)
            if fp:
                fp.close()
            RaveDestroy()

# test all scripts in source/tutorials/openravepy_examples/*.py
def test_tutorialexamples():
    ignore_examples = ['saving_viewer_image', 'recording_videos']
    examplesdir=os.path.join('..','docs','source','tutorials','openravepy_examples')
    for name in os.listdir(examplesdir):
        basename,ext = os.path.splitext(name)
        if not basename in ignore_examples and ext.lower() == '.py':
                yield RunTutorialExample(), os.path.join(examplesdir,basename)

def test_ikparam():
    ikparam = IkParameterization(Ray([1,2,3],[1,0,0]), IkParameterizationType.TranslationDirection5D)
    T = matrixFromAxisAngle([0,pi/4,0])
    T[0:3,3] = [0.1,0.2,0.3]
    # left mult
    ikparam2 = IkParameterization(ikparam)
    ikparam2.MultiplyTransform(T)
    ikparam3 = ikparam.__rmul__(T)
    assert(ikparam2.ComputeDistanceSqr(ikparam3) <= g_epsilon)
    # right mult
    
    ikparam2 = ikparam*T
    ikparam2.GetTranslationDirection5D().pos()
