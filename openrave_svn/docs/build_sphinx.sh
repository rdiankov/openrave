#!/bin/bash
# argument $1 is language: en, ja
# build the python documentation and openrave homepage
if [ "$1" == "ja" ]; then
    export LANG=ja_JP.UTF-8
else
    export LANG=en_US.UTF-8
fi
rm -rf _templates/examples.html _templates/databases.html _templates/database_generator_template.rst $1/openravepy build/$1/main
python build_interfaces.py --outdir=$1
mkdir -p $1/openravepy
python sphinx-autopackage-script/generate_modules.py --dest-dir=$1/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `openrave-config --python-dir`/openravepy pyflann
python build_ikdatabase.py --lang=$1 --ikfaststats=../test/ikfaststats.pp
sphinx-build -b html -c . $1 build/$1/main
