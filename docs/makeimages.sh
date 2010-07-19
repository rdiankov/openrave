# compiles all necessary images
dia -e images/openrave_architecture.eps images/openrave_architecture.dia
dia -e images/openrave_architecture.png -s 700 images/openrave_architecture.dia
ps2pdf -dEPSCrop images/openrave_architecture.eps images/openrave_architecture.pdf

dia -e images/openrave_architecture_jp.eps images/openrave_architecture_jp.dia
dia -e images/openrave_architecture_jp.png -s 700 images/openrave_architecture_jp.dia
ps2pdf -dEPSCrop images/openrave_architecture_jp.eps images/openrave_architecture_jp.pdf

./build_robots.py --robot=robots/barretthand.robot.xml --robot=robots/barrettsegway.robot.xml --robot=robots/kuka-kr5-r850.robot.xml --robot=robots/katana.robot.xml --robot=robots/man1.robot.xml --robot=robots/manusarm_left.robot.xml --robot=robots/pa10schunk.robot.xml --robot=robots/pr2-beta-sim.robot.xml --robot=robots/puma.robot.xml --robot=robots/schunk-lwa3-dual.robot.xml --robot=robots/shadowhand.robot.xml
