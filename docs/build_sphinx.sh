#!/bin/bash
# argument $1 is language: en, ja
rm -rf _templates/examples.html _templates/databases.html _templates/database_generator_template.rst $1/openravepy build/$1/main
python build_interfaces.py --outdir=$1
mkdir -p $1/openravepy
python sphinx-autopackage-script/generate_modules.py --dest-dir=$1/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `openrave-config --python-dir`/openravepy pyflann
python build_ikdatabase.py
sphinx-build -b html -c . $1 build/$1/main
