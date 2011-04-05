#!/bin/sh
# this is the main script needed to be executed by jenkins
# assuming the workspace has 'openrave' folder for sources and 'data' folder for scenes and robots.
export PARALLEL_JOBS=-j4
export OPENRAVE_DATA=`pwd`/data
export OPENRAVE_DATABASE=`pwd`/.openrave
mkdir -p .openrave
rm -rf localinstall
cd openrave
make prefix=`pwd`/../localinstall $PARALLEL_JOBS
make install $PARALLEL_JOBS
export PATH=`sh build/openrave-config --prefix`/bin:$PATH
export PYTHONPATH=`openrave-config --python-dir`:$PYTHONPATH
export LD_LIBRARY_PATH=`openrave-config --prefix`/lib:$LD_LIBRARY_PATH
cd test
python test_ikfast.py $PARALLEL_JOBS
