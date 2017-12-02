set(PROJECT_NAME vins)
cmake_minimum_required(VERSION 2.8.0)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release)

if (NOT CMAKE_BUILD_TYPE)
  	message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  	set(CMAKE_BUILD_TYPE Release CACHE STRING
    	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    	FORCE)
else (NOT CMAKE_BUILD_TYPE)
  	if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    	message("\n=================================================================================")
    	message("\n-- Build type: Debug. Performance will be terrible!")
    	message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    	message("\n=================================================================================")
  	endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	message("Build VINS-Mobile Linux ...")

	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs/linux)

	set(BOOST_INCLUDE_DIR ${BOOST_DIR}/include/)

	set(BOOST_LIBS 
		${BOOST_DIR}/lib/libboost_system.so
		${BOOST_DIR}/lib/libboost_thread.so
		${BOOST_DIR}/lib/libboost_filesystem.so
	)

	set(OpenCV_INCLUDE_DIR ${OPENCV_DIR}/include/)

	set(OpenCV_LIBS 
		${OPENCV_DIR}/lib/libopencv_calib3d.so
		${OPENCV_DIR}/lib/libopencv_core.so
		${OPENCV_DIR}/lib/libopencv_highgui.so
		${OPENCV_DIR}/lib/libopencv_imgcodecs.so
		${OPENCV_DIR}/lib/libopencv_imgproc.so
		${OPENCV_DIR}/lib/libopencv_features2d.so
		${OPENCV_DIR}/lib/libopencv_video.so
		${OPENCV_DIR}/lib/libopencv_videostab.so
		${OPENCV_DIR}/lib/libopencv_videoio.so
	)

	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		message("Build VINS-Mobile Linux with GNU ...")
		set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

		set(CERES_LIBS 
			${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/build_linux/install/lib/libceres.so
		)

	elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		message("Build VINS-Mobile Linux with Clang ...")
		set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

		set(CERES_LIBS 
			${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/build_clang/install/lib/libceres.so
		)
    endif ()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
	message("Build VINS-Mobile MacOS ...")

	set(CMAKE_MACOSX_RPATH 1)

	set (CMAKE_CXX_STANDARD 11)

	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs/macos)

	set(BOOST_INCLUDE_DIR ${BOOST_DIR}/include/)

	set(BOOST_LIBS 
		${BOOST_DIR}/lib/libboost_system.a
		${BOOST_DIR}/lib/libboost_thread.a
		${BOOST_DIR}/lib/libboost_filesystem.a
	)

	set(OpenCV_INCLUDE_DIR ${OPENCV_DIR}/include/)

	set(OpenCV_LIBS 
		${OPENCV_DIR}/lib/libopencv_calib3d.dylib
		${OPENCV_DIR}/lib/libopencv_core.dylib
		${OPENCV_DIR}/lib/libopencv_highgui.dylib
		${OPENCV_DIR}/lib/libopencv_imgcodecs.dylib
		${OPENCV_DIR}/lib/libopencv_imgproc.dylib
		${OPENCV_DIR}/lib/libopencv_features2d.dylib
		${OPENCV_DIR}/lib/libopencv_video.dylib
		${OPENCV_DIR}/lib/libopencv_videostab.dylib
		${OPENCV_DIR}/lib/libopencv_videoio.dylib
	)

	set(CERES_LIBS 
		${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/build_macos/install/lib/libceres.dylib
	)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Android")
	message("Build VINS-Mobile Android ...")

	add_definitions(-DANDROID)	

	set(CMAKE_CXX_STANDARD 11)

	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs/android/armeabi-v7a)

	set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

	set(BOOST_INCLUDE_DIR
		${BOOST_DIR}/armeabi-v7a/include
	)
	
	set(BOOST_LIBS 
		${BOOST_DIR}/armeabi-v7a/lib/libboost_system.a
		${BOOST_DIR}/armeabi-v7a/lib/libboost_thread.a
	)
	
	set(OpenCV_INCLUDE_DIR ${OPENCV_DIR}/native/jni/include)
	
	set(OpenCV_LIBS
		${OPENCV_DIR}/native/libs/armeabi-v7a/libopencv_java3.so
	)
		
	set(CERES_LIBS
		 ${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/obj/local/armeabi-v7a/libceres.a
	)
endif()


# Set source files
set(VINS_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/delaunay/delaunay.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/delaunay/triangle.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/loop/keyframe_database.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/loop/keyframe.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/loop/loop_closure.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/loop/VocabularyBinary.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/BowVector.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/VINS.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/vins_system.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/draw_result.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/feature_manager.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/feature_tracker.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/global_param.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/inital_sfm.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/initial_aligment.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/marginalization_factor.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/motion_estimator.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/perspective_factor.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/pose_local_parameterization.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/projection_facor.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/utility.cpp
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/vins_pnp.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW/FBrief.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW/FeatureVector.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW/QueryResults.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW/ScoringObject.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DUtils/Random.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DUtils/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/ThirdParty/DVision/BRIEF.cpp
)

include_directories(
  ${BOOST_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR} 
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src	
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/delaunay
  ${PROJECT_SOURCE_DIR}/VINS_Lib/src/loop
  ${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib
  ${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/include
  ${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/config
  ${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/ceres-solver/internal/ceres/miniglog/
  ${PROJECT_SOURCE_DIR}/VINS_ThirdPartyLib/eigen3
  ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW/
  ${PROJECT_SOURCE_DIR}/ThirdParty/DUtils/
  ${PROJECT_SOURCE_DIR}/ThirdParty/DVision/
  
)

# build shared library.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	add_library(${PROJECT_NAME} SHARED ${VINS_SOURCE_FILES})
	target_link_libraries(${PROJECT_NAME} ${CERES_LIBS} ${OpenCV_LIBS} ${BOOST_LIBS})

	add_executable(test_vins_dataset ${PROJECT_SOURCE_DIR}/src/test_vins_dataset.cpp)
	target_link_libraries(test_vins_dataset ${PROJECT_NAME} ${BOOST_LIBS} ${CERES_LIBS} ${OpenCV_LIBS})
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
	add_library(${PROJECT_NAME} SHARED ${VINS_SOURCE_FILES})
	target_link_libraries(${PROJECT_NAME} ${CERES_LIBS} ${OpenCV_LIBS} ${BOOST_LIBS})

	add_executable(test_vins_dataset ${PROJECT_SOURCE_DIR}/src/test_vins_dataset.cpp)
	target_link_libraries(test_vins_dataset ${PROJECT_NAME} ${BOOST_LIBS} ${CERES_LIBS} ${OpenCV_LIBS})
elseif(CMAKE_SYSTEM_NAME STREQUAL "Android")
	add_library(${PROJECT_NAME} SHARED ${VINS_SOURCE_FILES})
	target_link_libraries(${PROJECT_NAME} ${CERES_LIBS} ${OpenCV_LIBS} ${BOOST_LIBS} -llog)
endif()