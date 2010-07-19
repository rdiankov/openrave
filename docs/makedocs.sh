#!/bin/sh
# Internal use
# script creates the doc files, and sends them over to the programmingvision server
# packages used: doxygen, python-docutils, python-pygments, python-epydoc, python-sphinx
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
echo "$doxycommands" | cat Doxyfile.html Doxyfile.english - > Doxyfile.html.english
echo "$doxycommands" | cat Doxyfile.latex Doxyfile.english - > Doxyfile.latex.english
echo "$doxycommands" | cat Doxyfile.html Doxyfile.japanese - > Doxyfile.html.japanese
echo "$doxycommands" | cat Doxyfile.latex Doxyfile.japanese - > Doxyfile.latex.japanese

sh makeimages.sh

# run doxygen, assuming v1.7.1+
rm -rf english japanese openrave.pdf ordocs.tgz
doxygen Doxyfile.html.english
doxygen Doxyfile.latex.english
python build_latex.py english/latex
cp english/latex/refman.pdf english/openrave.pdf

doxygen Doxyfile.html.japanese
#python build_latex.py japanese/latex
#cp japanese/latex/refman.pdf japanese/openrave_japanese.pdf

# build openravepy documentation
python build_doc.py build_doc --outdir="english/openravepy-html" --languagecode=en
python build_doc.py build_doc --outdir="japanese/openravepy-html" --languagecode=ja

# build interfaces
rm -rf sphinx/interfaces
sphinx-build sphinx sphinx/sphinx-docs

# send to server
tar czf ordocs.tgz english/html english/openravepy-html english/openrave.pdf images/*.jpg images/*.png japanese/html japanese/openravepy-html sphinx/sphinx-docs
scp ordocs.tgz diankov@programmingvision.com:~/openrave/ordocs/
ssh diankov@programmingvision.com "cd ~/openrave/ordocs; rm -rf english japanese; tar xzf ordocs.tgz; rm -rf ordocs.tgz"
