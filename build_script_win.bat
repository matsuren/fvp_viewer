set CURRENT_DIR=%~dp0

set DEV_DIR="C:/dev"
mkdir %DEV_DIR%

::set GENERATOR_NAME="Visual Studio 14 2015"
::set GENERATOR_NAME="Visual Studio 15 2017"
set GENERATOR_NAME="Visual Studio 16 2019"

cd %DEV_DIR%
git clone https://github.com/glfw/glfw.git -b 3.3 --depth 1
cd glfw
mkdir build
cd build
cmake -A x64 -G %GENERATOR_NAME% -DCMAKE_INSTALL_PREFIX=./install -DCMAKE_DEBUG_POSTFIX=d -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_DOCS=OFF .. 
cmake --build . --config Debug --target INSTALL
cmake --build . --config Release --target INSTALL

cd %DEV_DIR%
git clone https://github.com/assimp/assimp.git -b v5.0.0 --depth 1
cd assimp
mkdir build
cd build

cmake -A x64 -G %GENERATOR_NAME% -DCMAKE_INSTALL_PREFIX=./install -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_ASSIMP_TOOLS=OFF -DASSIMP_NO_EXPORT=ON -DASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT=OFF -DASSIMP_BUILD_PLY_IMPORTER=ON -DASSIMP_BUILD_STL_IMPORTER=ON .. 
cmake --build . --config Debug --target INSTALL
cmake --build . --config Release --target INSTALL


cd %CURRENT_DIR%
mkdir build
cd build
cmake -A x64 -G %GENERATOR_NAME% -DGLFW3_DIR=%DEV_DIR%/glfw/build/install/lib/cmake/glfw3 -DAssimp_DIR=%DEV_DIR%/assimp/build/install/lib/cmake/assimp-5.0 ..
cmake --build . --config Release
pause