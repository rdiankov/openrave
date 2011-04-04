if not exist msvc_boost/boost rmdir inc \s \q && rmdir bin \s \q && rmdir lib \s \q && cmake -E tar zxf msvc_files.tgz
mkdir build
cd build
cmake ..
cd ..
