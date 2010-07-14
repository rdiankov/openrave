#!/bin/sh
# Internal use
# script creates the doc files, and sends them over to the programmingvision server
# packages used: doxygen, python-docutils, python-pygments, python-epydoc
# for latex: dot2tex, texlive-base, texlive-latex-base, texlive-pictures, texlive-fonts-recommended
# for japanese: latex-cjk-japanese
rm -rf english japanese openrave.pdf ordocs.tgz
echo "OUTPUT_LANGUAGE = English
OUTPUT_DIRECTORY = ./english/
PROJECT_NUMBER = `openrave-config --version`" | cat Doxyfile - > Doxyfile.english
doxygen Doxyfile.english

# build latex
cd english/latex
# yes, 3 times
pdflatex refman.tex
pdflatex refman.tex
pdflatex refman.tex
cd ..
cp latex/refman.pdf openrave.pdf
cd ..

echo "OUTPUT_LANGUAGE = Japanese
OUTPUT_DIRECTORY = ./japanese/
PROJECT_NUMBER = `openrave-config --version`" | cat Doxyfile - > Doxyfile.japanese
doxygen Doxyfile.japanese

# need to figure out how latex can handle japanese...
# cd japanese/latex
# # yes, 3 times
# pdflatex refman.tex
# pdflatex refman.tex
# pdflatex refman.tex
# cd ../..
#cp japanese/latex/refman.pdf japanese/openrave_japanese.pdf

# build openravepy documentation
python build_doc.py build_doc --outdir="english/openravepy-html" --languagecode=en
python build_doc.py build_doc --outdir="japanese/openravepy-html" --languagecode=ja

# send to server
tar czf ordocs.tgz english/html english/openravepy-html english/openrave.pdf images/*.jpg japanese/html japanese/openravepy-html 
scp ordocs.tgz diankov@programmingvision.com:~/openrave/ordocs/
ssh diankov@programmingvision.com "cd ~/openrave/ordocs; rm -rf english japanese; tar xzf ordocs.tgz; rm -rf ordocs.tgz"
