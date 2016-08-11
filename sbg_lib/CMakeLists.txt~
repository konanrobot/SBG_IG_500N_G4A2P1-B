# Version required
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project Name
PROJECT( sbg_lib )
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif(COMMAND cmake_policy)
# Basic Settings
SET(CMAKE_CXX_COMPILER "g++")
# c++编译器参数
#CMAKE_CXX_FLAGS_DEBUG debug版本对应的编译器参数
#CMAKE_CXX_FLAGS_RELEASE release版本对应的编译器参数
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread -O2")
#PROJECT_SOURCE_DIR:工程的源文件目录，通常是包含CMakeLists.txt（有Project命令的）的目录
MESSAGE(STATUS "This is SOURCE dir "${CMAKE_YEL_SOURCE_DIR})
MESSAGE(STATUS "This is BINARY dir " ${CMAKE_YEL_BINARY_DIR})

#LIBRARY_OUTPUT_PATH:链接库的输出目录
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Debug Settings控制 Debug 和 Release 模式的构建
SET( CMAKE_BUILD_TYPE Release )

# Include directories包含目录
#将./include加入包含目录列表
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)


#这个指令用于向当前工程添加存放源文件的子目录，并可以指定中间二进制和目标二进制存放的位置。
ADD_SUBDIRECTORY(lib)
#INCLUDE_DIRECTORIES(${includedir}) #相当与gcc的-I。
#LINK_DIRECTORIES(${libdir}) #相当与gcc的-L
#TARGET_LINK_LIBRARIES(helloworld ${linkflags}) #相当与gcc的-l
#ADD_DEFINITIONS(${cflags}) #相当与gcc的-D，向C/CPP添加宏定义
#ADD_DEPENDICIES(target_name, depend_name)：定义target对其他target的依赖关系
