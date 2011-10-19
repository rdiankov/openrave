#!/bin/bash
# argument $1 is language: en, ja
if [ "$1" == "ja" ]; then
    export LANG=ja_JP.UTF-8
else
    export LANG=en_US.UTF-8
fi
ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
python sphinx-autopackage-script/generate_modules.py --dest-dir=$1/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `pwd`/openravepy pyflann
sphinx-build -b html -c . $1 build/$1/main
