# compiles all necessary images
dia -e images/openrave_architecture.eps images/openrave_architecture.dia
dia -e images/openrave_architecture.png -s 700 images/openrave_architecture.dia
ps2pdf -dEPSCrop images/openrave_architecture.eps images/openrave_architecture.pdf

dia -e images/openrave_architecture_jp.eps images/openrave_architecture_jp.dia
dia -e images/openrave_architecture_jp.png -s 700 images/openrave_architecture_jp.dia
ps2pdf -dEPSCrop images/openrave_architecture_jp.eps images/openrave_architecture_jp.pdf
