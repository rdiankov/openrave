===================================================================================
Install instructions for OpenRave in Ubuntu/Ubuntu Mate 14.04.2 x64
===================================================================================
Installing ODE and Bullet is Optional, must be downloaded and installed. This Instructions work for branches master & latest_stable.

Info sources from:

- OpenRave: http://openrave.org/docs/latest_stable/coreapihtml/installation_linux.html
- André Dietrich: http://www.aizac.info/installing-openrave0-9-on-ubuntu-trusty-14-04-64bit/
- Own Experience

Tutorial made by Carlos Rubert: cescuder@uji.es

Last Update 15 October, 2015

Install Package Dependences
---------------------------
Some packages (like pkg-config) are required just in case your using Ubuntu Mate, so in Ubuntu are already installed:

1.	sudo add-apt-repository ppa:openrave/testing
2.	sudo apt-get install collada-dom2.4-dp*
3.	sudo apt-get install libsoqt4-dev
4.	sudo apt-get install cmake-curses-gui
5.	sudo apt-get install build-essential -y
6.	sudo apt-get install libboost-python-dev python python-dev python-numpy ipython python-sympy python-scipy
7.	sudo apt-get install liblinearmath2.81
8.	sudo apt-get install libpcre++-dev
9.	sudo apt-get install libboost-all-dev
10.	sudo apt-get install libassimp-dev
11.	sudo apt-get install pkg-config

Install Bullet (optional, needs version 2.80, ubuntu official repo has ver. 2.81)
---------------------------------------------------------------------------------

1.	sudo add-apt-repository ppa:joern-schoenyan/libbullet280
2.	sudo apt-get install libbullet2.80
3.	sudo apt-get install libbullet-dev=2.80\*

Install Ode(optional, manually installed)
-----------------------------------------
1.	sudo apt-get install libode1 (then remove, just needed .so files)
2.	wget https://bitbucket.org/odedevs/ode/downloads/ode-0.13.tar.gz
3.	tar -zxvf ode-0.13.tar.gz 
4.	cd ode-0.13/
5.	./configure --enable-double-precision --enable-shared
6.	make
7.	sudo make install


Installing Openrave
-------------------
Get latest source code:

git clone https://github.com/rdiankov/openrave.git


Build and Install OpenRave:

1. cd openrave
2. mkdir build
3. cd build
4. ccmake .. [(c)onfigure & (c)onfigure & (g)enerate)]
5. make
6. sudo make install

If you're using branch latest_stable, probably you'll get an error when compiling sources for bulletrave. This has been already solve in this issue by Stéphane Caron:

- https://github.com/rdiankov/openrave/issues/333



