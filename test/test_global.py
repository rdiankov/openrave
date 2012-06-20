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
    examplesdir=os.path.join('..','docs','source','tutorials','openravepy_examples')
    for name in os.listdir(examplesdir):
        basename,ext = os.path.splitext(name)
        if ext.lower() == '.py':
            yield RunTutorialExample(), os.path.join(examplesdir,basename)
