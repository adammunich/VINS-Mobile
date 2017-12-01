# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_VERBOSE_MAKEFILE on)
set(PREBUILT_LIBS "${CMAKE_SOURCE_DIR}/src/main/jniLibs")

include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/OpenCV)
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/Boost)
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/Eigen)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/Ceres)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/Ceres/include)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/Ceres/config)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/Ceres/internal/ceres/miniglog)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/VINS)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/VINS/loop)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/VINS/delaunay)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/ThirdParty/DBoW)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/ThirdParty/DUtils)
#include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/ThirdParty/DVision)

add_library(android_opencv_java3 SHARED IMPORTED )
set_target_properties(android_opencv_java3 PROPERTIES
                      IMPORTED_LOCATION "${PREBUILT_LIBS}/OpenCV/${ANDROID_ABI}/libopencv_java3.so")

add_library(android_boost_system SHARED IMPORTED )
set_target_properties(android_boost_system PROPERTIES
                      IMPORTED_LOCATION "${PREBUILT_LIBS}/Boost/libboost_system.so")

add_library(android_boost_thread SHARED IMPORTED )
set_target_properties(android_boost_thread PROPERTIES
                      IMPORTED_LOCATION "${PREBUILT_LIBS}/Boost/libboost_thread.so")

add_library(android_ceres_solver STATIC IMPORTED )
set_target_properties(android_ceres_solver PROPERTIES
                      IMPORTED_LOCATION "${PREBUILT_LIBS}/Ceres/${ANDROID_ABI}/libceres.a")

#add_library(android_vins_core SHARED IMPORTED )
#set_target_properties(android_vins_core PROPERTIES
#                      IMPORTED_LOCATION "${PREBUILT_LIBS}/VINS/${ANDROID_ABI}/libvins.so")

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

add_library( # Sets the name of the library.
             native-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/native-lib.cpp )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       native-lib android log
                       android_opencv_java3
                       android_boost_system
                       android_boost_thread
                       #android_ceres_solver
                       #android_vins_core

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )