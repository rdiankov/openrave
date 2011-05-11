#!/bin/bash
# compiles all necessary images
dia -e images/openrave_architecture.eps images/openrave_architecture.dia
dia -e images/openrave_architecture.png -s 700 images/openrave_architecture.dia
dia -e images/examples/visibilityplanning_framework.png -s 1024 images/examples/visibilityplanning_framework.dia
ps2pdf -dEPSCrop images/openrave_architecture.eps images/openrave_architecture.pdf

dia -e images/openrave_architecture_jp.eps images/openrave_architecture_jp.dia
dia -e images/openrave_architecture_jp.png -s 700 images/openrave_architecture_jp.dia
ps2pdf -dEPSCrop images/openrave_architecture_jp.eps images/openrave_architecture_jp.pdf

dia -e images/openrave_documentation.eps images/openrave_documentation.dia
dia -e images/openrave_documentation.png -s 700 images/openrave_documentation.dia

cp -f ../resources/openrave_banner_400.png _static/
cp -f ../resources/openrave_icon*.png _static/
cp -f ../resources/openrave_banner_dark.png build/en/coreapihtml/
cp -f ../resources/openrave_banner_dark.png build/ja/coreapihtml/
