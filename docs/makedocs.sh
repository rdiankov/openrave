#!/bin/sh
# Internal use
# script creates the doc files, and sends them over to the programmingvision server
rm -rf html latex openrave.pdf ordocs.tgz
doxygen
cd latex
# yes, 3 times
pdflatex refman.tex
pdflatex refman.tex
pdflatex refman.tex
cd ..
cp latex/refman.pdf openrave.pdf
tar czf ordocs.tgz html openrave.pdf
scp ordocs.tgz diankov@programmingvision.com:~/openrave/ordocs/
ssh diankov@programmingvision.com "cd ~/openrave/ordocs; rm -rf openrave.pdf html; tar xzf ordocs.tgz; rm -rf ordocs.tgz"
