# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
from __future__ import with_statement # for python 2.5
import os

class KinBodyStateSaver:
    def __init__(self,body):
        self.body = body
    def __enter__(self):
        self.handle = self.body.CreateKinBodyStateSaver()
    def __exit__(self, type, value, traceback):
        self.handle.close()

class RobotStateSaver:
    def __init__(self,robot):
        self.robot = robot
    def __enter__(self):
        self.handle = self.robot.CreateRobotStateSaver()
    def __exit__(self, type, value, traceback):
        self.handle.close()

def mkdir_recursive(newdir):
    """works the way a good mkdir should :)
        - already exists, silently complete
        - regular file in the way, raise an exception
        - parent directory(ies) does not exist, make them as well
    """
    if os.path.isdir(newdir):
        pass
    elif os.path.isfile(newdir):
        raise OSError("a file with the same name as the desired dir, '%s', already exists." % newdir)
    else:
        head, tail = os.path.split(newdir)
        if head and not os.path.isdir(head):
            mkdir_recursive(head)
        if tail:
            os.mkdir(newdir)
