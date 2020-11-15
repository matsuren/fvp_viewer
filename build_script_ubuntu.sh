#!/bin/sh
DEV_DIR="$HOME/dev"
mkdir $DEV_DIR

SCRIPT_DIR=$(cd $(dirname $0); pwd)

cd $DEV_DIR
git clone https://github.com/glfw/glfw.git -b 3.3.2 --depth 1
cd glfw
mkdir build && cd build
cmake -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_DOCS=OFF ..
make -j8
sudo make install

cd $DEV_DIR
git clone https://github.com/assimp/assimp.git -b v5.0.1 --depth 1
cd assimp
mkdir build && cd build
cmake -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_ASSIMP_TOOLS=OFF -DASSIMP_NO_EXPORT=ON -DASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT=OFF -DASSIMP_BUILD_PLY_IMPORTER=ON -DASSIMP_BUILD_STL_IMPORTER=ON ..
make -j8
sudo make install


cd $DEV_DIR
git clone https://github.com/gabime/spdlog.git -b v1.8.1 --depth 1
cd spdlog
mkdir build && cd build
cmake ..
make -j8
sudo make install

cd $SCRIPT_DIR
mkdir build && cd build
cmake ..
make -j8