#! /bin/bash


export CPU_CORES=$(grep -c ^processor /proc/cpuinfo);


git submodule init
git submodule update

cd Codigos/Tesis_ws/src/Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make -j$CPU_CORES -l$CPU_CORES

cd ../../fast
mkdir build
cd build
cmake ..
make -j$CPU_CORES -l$CPU_CORES

cd ../../../
catkin_make