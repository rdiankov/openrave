#!/bin/bash
# build the python documentation and openrave homepage
rm -rf _templates/examples.html _templates/databases.html _templates/database_generator_template.rst source/openravepy build/locale
python build_interfaces.py --outdir=source
if [ "$?" -ne 0 ]; then echo "build_interfaces.py failed"; exit 1; fi 
mkdir -p source/openravepy
ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
export PYTHONPATH=`pwd`:$PYTHONPATH
python sphinx-autopackage-script/generate_modules.py --dest-dir=source/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `pwd`/openravepy
rm openravepy
if [ "$?" -ne 0 ]; then echo "build_interfaces.py failed"; exit 1; fi 
python build_ikdatabase.py --ikfaststats=ikfaststats.pp
if [ "$?" -ne 0 ]; then echo "build_ikdatabase.py failed"; fi 
ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy

# create/update internationalization files
# have to set LANG to en, or otherwise the automodule stuff might not return english translations
# set OPENRAVE_INTERNAL_COMMENTS=0 in order to avoid the doxygen translated comments from popping up in the translations
OPENRAVE_INTERNAL_COMMENTS=0 LANG=en_US.UTF-8 sphinx-build -D language=en -b gettext -c . source build/locale

./build_sphinx_lang.sh en
if [ "$?" -ne 0 ]; then echo "sphinx-build failed"; exit 1; fi 

./build_sphinx_lang.sh ja
if [ "$?" -ne 0 ]; then echo "sphinx-build failed"; exit 1; fi 

rm openravepy
