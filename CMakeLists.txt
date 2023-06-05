# 文件结构:
# wordspace下设inc,src,build三个文件夹
# inc:存放头文件
# src:存放源文件
# build:存放编译后的文件
# 程序使用Eigen库,需要在/usr/include下添加eigen3文件夹


cmake_minimum_required(VERSION 3.0)

project(main)


# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

# 添加头文件和Eigen
include_directories(inc)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#添加boost库
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

# 添加源文件
aux_source_directory(src DIR_SRCS)

# 添加可执行文件
# add_executable(Path_Planning src/main.cpp src/InputStream.cpp src/Path_Planning.cpp)
add_executable(main ${DIR_SRCS})

# 添加链接库
# target_link_libraries(Path_Planning -lpthread)

# 添加编译输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)