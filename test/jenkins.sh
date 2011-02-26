#!/bin/sh
# this is the main script needed to be executed by jenkins
# assuming the workspace has 'openrave' folder for sources and 'data' folder for scenes and robots.
export PARALLEL_JOBS=-j4
export OPENRAVE_DATA=`pwd`/data
export OPENRAVE_DATABASE=`pwd`/.openrave
mkdir -p .openrave
rm -rf localinstall
cd openrave
make test prefix=`pwd`/../localinstall
