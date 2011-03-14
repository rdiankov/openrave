#!/bin/bash
# argument $1 is language: en, ja
python build_interfaces.py --outdir=$1
rm -rf $1/openravepy
mkdir -p $1/openravepy
python sphinx-autopackage-script/generate_modules.py --dest-dir=$1/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `openrave-config --python-dir`/openravepy pyflann
sphinx-build -b html -c . $1 build/$1/main
