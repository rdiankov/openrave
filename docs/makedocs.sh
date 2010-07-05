#!/bin/sh
# Internal use
# script creates the doc files, and sends them over to the programmingvision server
rm -rf english japanese openrave.pdf ordocs.tgz
echo "OUTPUT_LANGUAGE = English
OUTPUT_DIRECTORY = ./english/" | cat Doxyfile - > Doxyfile.english
doxygen Doxyfile.english
cd english/latex
# yes, 3 times
pdflatex refman.tex
pdflatex refman.tex
pdflatex refman.tex
cd ..
cp latex/refman.pdf openrave.pdf

epydoc --output=openravepy-html --config=../epydoc.config --css=../epydoc.css
cd ..

echo "OUTPUT_LANGUAGE = Japanese
OUTPUT_DIRECTORY = ./japanese/" | cat Doxyfile - > Doxyfile.japanese
doxygen Doxyfile.japanese
# need to figure out how latex can handle japanese...
# cd japanese/latex
# # yes, 3 times
# pdflatex refman.tex
# pdflatex refman.tex
# pdflatex refman.tex
# cd ../..
#cp japanese/latex/refman.pdf japanese/openrave_japanese.pdf

tar czf ordocs.tgz english/html english/openravepy-html japanese/html english/openrave.pdf
scp ordocs.tgz diankov@programmingvision.com:~/openrave/ordocs/
ssh diankov@programmingvision.com "cd ~/openrave/ordocs; rm -rf english japanese; tar xzf ordocs.tgz; rm -rf ordocs.tgz"
