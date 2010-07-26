#!/bin/sh
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

sh makeimages.sh

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

# build openravepy documentation
python build_doc.py build_doc --outdir="en/openravepy-html" --languagecode=en
python build_doc.py build_doc --outdir="ja/openravepy-html" --languagecode=ja

# build interfaces
rm -rf sphinx/interfaces sphinx/sphinx-docs
python build_interfaces.py
sphinx-build sphinx sphinx/sphinx-docs

# send to server
tar czf ordocs.tgz en/html en/openravepy-html en/openrave.pdf images/*.jpg images/*.png ja/html ja/openravepy-html sphinx/sphinx-docs
scp ordocs.tgz diankov@programmingvision.com:~/openrave/ordocs/
ssh diankov@programmingvision.com "cd ~/openrave/ordocs; rm -rf en ja; tar xzf ordocs.tgz; rm -rf ordocs.tgz"
