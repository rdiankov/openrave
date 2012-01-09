#!/bin/bash
sudo apt-get install g++ libqt4-dev qt4-dev-tools ffmpeg libavcodec-dev libavformat-dev libxvidcore-dev libx264-dev libfaac-dev libogg-dev libvorbis-dev libdc1394-dev liblame-dev libgsm1-dev libboost-dev libboost-regex-dev libxml2-dev libglew-dev  libboost-graph-dev libboost-wave-dev libboost-serialization-dev libboost-filesystem-dev libpcre3-dev libboost-thread-dev libmpfr-dev libboost-date-time-dev libqhull-dev libswscale-dev lapack3-dev
sudo apt-get remove cmake qt3-dev-tools libqt3-headers libode0debian1 python-sympy

# cmake - need to use version >=2.6.0, default cmake in ubuntu 8.04 is 2.47, which will not work.
wget http://www.cmake.org/files/v2.8/cmake-2.8.7.tar.gz
cd cmake-2.8.7
./configure
make
sudo make install

# ode - Will need to install ODE from sources (do not use the one in the package manager)
wget https://downloads.sourceforge.net/project/opende/ODE/0.11.1/ode-0.11.1.tar.bz2
tar xjf ode-0.11.1.tar.bz2
cd ode-0.11.1
./configure --with-libccd=cyl-cyl --with-trimesh=opcode --enable-new-trimesh --disable-demos --enable-shared --with-arch=nocona --enable-release --enable-malloc --enable-ou --disable-asserts --with-pic --enable-double-precision
make -j4
sudo make install

# soqt
wget http://ftp.coin3d.org/coin/src/all/SoQt-1.5.0.tar.gz
tar xzf SoQt-1.5.0.tar.gz
cd SoQt-1.5.0
./configure
make
sudo make install

