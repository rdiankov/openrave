#!/bin/bash
# source openrave into the path given its install prefix
if test $# -eq 0; then
    echo 'need a path'
else
    export PATH=$1/bin:$PATH
    export PYTHONPATH=`openrave-config --python-dir`:$PYTHONPATH
    export LD_LIBRARY_PATH=$1/lib:$LD_LIBRARY_PATH
fi
