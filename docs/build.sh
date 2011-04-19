#!/bin/bash
# Internal use, see documentation_system.rst
rm -rf build
mkdir -p build
curdir=`pwd`
cd ..
rootdir=`pwd`
cd $curdir
doxycommands="STRIP_FROM_PATH        = $rootdir
PROJECT_NUMBER = `openrave-config --version`
ALIASES += openraveversion=`openrave-config --version`
"
echo "$doxycommands" | cat Doxyfile.html Doxyfile.en - > build/Doxyfile.html.en
echo "$doxycommands" | cat Doxyfile.latex Doxyfile.en - > build/Doxyfile.latex.en
echo "$doxycommands" | cat Doxyfile.html Doxyfile.ja - > build/Doxyfile.html.ja
echo "$doxycommands" | cat Doxyfile.latex Doxyfile.ja - > build/Doxyfile.latex.ja

bash build_images.sh

# doxygen
python build_doxygen.py --lang=en
cp build/en/latex/refman.pdf build/en/openrave.pdf

python build_doxygen.py --lang=ja
#cp ja/latex/refman.pdf build/ja/openrave_ja.pdf

if [ "$1" == "internal" ]; then
    # build internal openravepy docs
    python build_openravepy_internal.py --languagecode en --languagecode ja
    # have to rebuild openravepy_int to get correct docstrings!
    cd ../build
    make install
    cd $curdir
fi

./build_sphinx.sh en
#./build_sphinx.sh ja
