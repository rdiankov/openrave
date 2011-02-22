#!/bin/bash
# Internal use
# script creates the doc files, and sends them over to the programmingvision server
# packages used: doxygen, python-docutils, python-pygments, python-epydoc, python-sphinx, python-lxml, python-sphinx
# for latex: dot2tex, texlive-base, texlive-latex-base, texlive-pictures, texlive-fonts-recommended
# for japanese: latex-cjk-japanese

# create all the temporary doxygen files
curdir=`pwd`
cd ..
rootdir=`pwd`
cd $curdir
doxycommands="STRIP_FROM_PATH        = $rootdir
PROJECT_NUMBER = `openrave-config --version`
ALIASES += openraveversion=`openrave-config --version`
"
echo "$doxycommands" | cat Doxyfile.html Doxyfile.en - > Doxyfile.html.en
echo "$doxycommands" | cat Doxyfile.latex Doxyfile.en - > Doxyfile.latex.en
echo "$doxycommands" | cat Doxyfile.html Doxyfile.ja - > Doxyfile.html.ja
echo "$doxycommands" | cat Doxyfile.latex Doxyfile.ja - > Doxyfile.latex.ja

#bash makeimages.sh

rm -rf en ja openrave.pdf ordocs.tgz

# run doxygen, assuming v1.7.1+
doxygen Doxyfile.html.en
doxygen Doxyfile.latex.en
python build_latex.py en/latex
cp en/latex/refman.pdf en/openrave.pdf

doxygen Doxyfile.html.ja
#python build_latex.py ja/latex
#cp ja/latex/refman.pdf ja/openrave_ja.pdf

# build internal openravepy docs
python build_openravepy_internal.py --languagecode en --languagecode ja
# have to rebuild openravepy_int!!

# build openravepy documentation
prevlang=$LANG
export LANG=en_US.UTF-8
python build_doc.py build_doc --outdir="en/openravepy-html" --languagecode=en
export LANG=ja_JP.UTF-8
python build_doc.py build_doc --outdir="ja/openravepy-html" --languagecode=ja
export LANG=$prevlang

# build interfaces
rm -rf sphinx/interfaces sphinx/sphinx-docs
python build_interfaces.py
sphinx-build sphinx sphinx/sphinx-docs
