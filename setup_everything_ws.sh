#! /bin/bash


export CPU_CORES=$(grep -c ^processor /proc/cpuinfo);


if [ -d Codigos/Tesis_ws/build ]; then
	rm -rf Codigos/Tesis_ws/build;
fi
if [ -d Codigos/Tesis_ws/devel ]; then
	rm -rf Codigos/Tesis_ws/devel;
fi

# if [ -d Codigos/Tesis_ws/src/Sophus ]; then
# 	rm -rf Codigos/Tesis_ws/src/Sophus;
# fi
# if [ -d Codigos/Tesis_ws/src/fast ]; then
# 	rm -rf Codigos/Tesis_ws/src/fast;
# fi
# if [ -d Codigos/Tesis_ws/src/rpg_svo ]; then
# 	rm -rf Codigos/Tesis_ws/src/rpg_svo;
# fi
# if [ -d Codigos/Tesis_ws/src/rpg_vikit ]; then
# 	rm -rf Codigos/Tesis_ws/src/rpg_vikit;
# fi
# if [ -d Codigos/Tesis_ws/src/ORB_SLAM ]; then
# 	rm -rf Codigos/Tesis_ws/src/ORB_SLAM;
# fi

# git submodule init
# git submodule update --depth=1

# cd Codigos/Tesis_ws/src/Sophus
# git checkout a621ff
# mkdir build
# cd build
# cmake ..
# make -j$CPU_CORES -l$CPU_CORES

# cd ../../fast
# mkdir build
# cd build
# cmake ..
# make -j$CPU_CORES -l$CPU_CORES


# cd ../../ORB_SLAM/Thirdparty/g2o/
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j$CPU_CORES -l$CPU_CORES


# cd ../../DBoW2/
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j$CPU_CORES -l$CPU_CORES

# cd ../../../
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j$CPU_CORES -l$CPU_CORES

# cd ../../../
cd ../../
catkin_make

sudo echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc

# REFERENCE: http://answers.ros.org/question/53353/autocomplete-not-working-anymore/
sudo echo 'export LC_ALL="C"' >> ~/.bashrc