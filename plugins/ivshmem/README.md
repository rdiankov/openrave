
## Building
First you need to build custom OpenRAVE dependencies.
- RapidJSON https://github.com/mujin/rapidjson branch master
- fcl https://github.com/rdiankov/fcl branch trimeshContactPoints20200813
- ccd https://github.com/danfis/libccd commit 2ddebf8
- fparser https://github.com/rdiankov/fparser branch master
- log4cxx ssh://git@git.mujin.co.jp/dev/log4cxx branch master


```
export OPENRAVE_DEPS=/home/mujin/.openrave_deps
export PATH="${OPENRAVE_DEPS}/bin":${PATH}
export PKG_CONFIG_PATH=${OPENRAVE_DEPS}/lib/pkgconfig/
export OPENRAVE_PLUGINS=${OPENRAVE_DEPS}/lib/openrave0.100-plugins ### Change this to openrave0.101 when you switch versions

git clone -b production git@github.com:rdiankov/openrave.git
cd openrave && mkdir build && cd build
cmake .. \
    -DODE_USE_MULTITHREAD=ON \
    -DOPT_IKFAST_FLOAT32=OFF \
    -DOPENRAVE_MSGPACK=OFF \
    -DNATIVE_COMPILE_FLAGS="-march=native -mtune=native" \
    -DOPT_LOG4CXX=ON \
    -DBoost_NO_BOOST_CMAKE=OFF \
    -DUSE_PYBIND11_PYTHON_BINDINGS=OFF \
    -DOPT_INSTALL_3DMODELDATA=OFF \
    -DOPT_MSGPACK=OFF \
    -DOPT_CURL=OFF \
    -DOPT_OCTAVE=OFF \
    -DOPT_COLLADA=OFF \
    -DOPT_MATLAB=OFF \
    -DOPT_STATIC=OFF \
    -DOPT_PYTHON=OFF \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX=/home/mujin/.openrave_deps/

```