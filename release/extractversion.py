#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
"""extracts the openrave version from the root CMakeLists.txt file and returns it
"""
import sys

if __name__=='__main__':
    data = open(sys.argv[1],'r').read()
    indices = [data.find('OPENRAVE_VERSION_MAJOR'), data.find('OPENRAVE_VERSION_MINOR'), data.find('OPENRAVE_VERSION_PATCH')]
    versions = []
    for index in indices:
        startindex = data.find(' ',index)
        endindex = data.find(')',startindex)
        versions.append(data[startindex:endindex].strip())
    print '.'.join(versions)
