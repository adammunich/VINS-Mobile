# VINS-Mobile
## Monocular Visual-Inertial State Estimator on Mobile Phones

**13 Nov 2017**: Wrap VINS-Mobile into a library to be used on multiple platforms.

Dependencies: 
	Boost 
	OpenCV 
	Ceres-Solver

Test with Boost 1.63, OpenCV 3.2, Ceres-Solver 1.12 on Ubuntu 16.04, GCC 4.9, Clang 3.8.

Build procedure:

	Linux/Ubuntu:

	1. Build Ceres-Solver (a minimal version). 
		
		cd VINS_ThirdPartyLib/ceres-solver/
		
		export EIGEN_DIRS=absolute_path_to_VINS_ThirdPartyLib_eigen3

		To build with GCC:

			mkdir build_linu && cd build_linux

			cmake -DEIGEN_DIRS=${EIGEN_DIRS} -DMINIGLOG=ON -DSUITESPARSE=OFF -DLAPACK=OFF -DOPENMP=OFF -DCXX11=ON 
				-DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=./install ..

		To build with Clang:

			mkdir build_clang && cd build_clang

			cmake -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DEIGEN_DIRS=${EIGEN_DIRS} 
				-DMINIGLOG=ON -DSUITESPARSE=OFF -DLAPACK=OFF -DOPENMP=OFF -DCXX11=ON 
				-DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=./install ..
		
		make 

		make install

	2. Prepare Boost and OpenCV

		Suppose Boost directory structure is:

			boost_1_63_0
					|
					|---- install
					|		|
					|		|---- include 
					|		|
					|		|---- lib
					|
					|---- install_clang
							|
							|---- include 
							|
							|---- lib

		OpenCV directory structure is:

			opencv
				|
				|---- release
				|		|
				|		|---- install
				|				|
				|				|---- bin
				|				|
				|				|---- include
				|				|
				|				|---- lib
				|				|
				|				|---- share
				|
				|---- release_clang
						|
						|---- install
								|
								|---- bin
								|
								|---- include
								|
								|---- lib
								|
								|---- share		

		To build with GCC:

			export BOOST_DIR=absolute_path_to_boost_1_63_0/install/

			export OPENCV_DIR=absolute_path_to_opencv/release/install/

		To build with Clang:

			export BOOST_DIR=absolute_path_to_boost_1_63_0/install_clang/

			export OPENCV_DIR=absolute_path_to_opencv/release_clang/install/

	3. Build VINS-Mobile library and test executable

		cd VINS-Mobile

		To build with GCC:

			mkdir build_linux && cd build_linux 

			cmake -DCMAKE_BUILD_TYPE=Release -DBOOST_DIR=${BOOST_DIR} -DOPENCV_DIR=${OPENCV_DIR} ..

		To build with Clang:

			mkdir builc_clang && cd build_clang

			cmake -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ 
				-DCMAKE_BUILD_TYPE=Release -DBOOST_DIR=${BOOST_DIR} -DOPENCV_DIR=${OPENCV_DIR} .. 

		make 

**27 Jun 2017**: We upgrade the pose outputs and AR rendering to 30 Hz by motion-only 3D tracking in front-end and improve the loop-closure procedure(See our [technical report](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper/tro_technical_report.pdf) for detail).

**22 May 2017**:  **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator** is released. It is the **Linux** version and is fully integrated with **ROS**. Available at: [link](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)


VINS-Mobile is a real-time monocular visual-inertial state estimator developed by members of the [HKUST Aerial Robotics Group](http://uav.ust.hk/). It runs on compatible iOS devices, and provides localization services for augmented reality (AR) applications. It is also tested for state estimation and feedback control for autonomous drones. VINS-Mobile uses sliding window optimization-based formulation for providing high-accuracy visual-inertial odometry with automatic initialization and failure recovery. The accumulated odometry errors are corrected in real-time using global pose graph SLAM. An AR demonstration is provided to showcase its capability.

**Authors:** [Peiliang LI](https://github.com/PeiliangLi), [Tong QIN](https://github.com/qintony), [Zhenfei YANG](https://github.com/dvorak0), Kejie QIU, and [Shaojie SHEN](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HKUST Aerial Robotics Group](http://uav.ust.hk/)

**Videos:** https://youtu.be/0mTXnIfFisI https://youtu.be/CI01qbPWlYY ([Video1](http://www.bilibili.com/video/av10813373/) [Video2](http://www.bilibili.com/video/av10813030/) for mainland China friends)

**Related Papers:**
* [**Monocular Visual-Inertial State Estimation for Mobile Augmented Reality**](http://www.ece.ust.hk/~eeshaojie/ismar2017peiliang.pdf), *P.Li et al (ISMAR 2017, accepted)*
* [**Robust Initialization of Monocular Visual-Inertial Estimation on Aerial Robots**](http://www.ece.ust.hk/~eeshaojie/iros2017tong.pdf), *T.Qin et al (IROS 2017, accepted)*
* [**Monocular Visual-Inertial State Estimation With Online Initialization and Camera-IMU Extrinsic Calibration**](http://ieeexplore.ieee.org/document/7463059/), *Z.Yang et al (T-ASE 2017)*

*If you use VINS-Mobile for your academic research, please cite at least one of our related papers.*

## 1. Build

The code has been compiled on macOS Sierra with Xcode 8.3.1 and tested with iOS 10.2.1 on iPhone7 Plus.

1.1 Install boost for macOS
```
$ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
$ brew install boost
```

1.2 Download specific **opencv2.framework** from [here](http://uav.ust.hk/storage/opencv2.framework.zip), then unzip it to VINS_ThirdPartyLib/opencv2.framework
    **(Please make sure you haven't installed opencv for your OSX)**

1.3 In your Xcode, select **Product**-> **Scheme**-> **Edit Scheme**-> **Run**-> **Info**, set **Build Configuration** to **Release** (not debug)

1.4 **Slect your device** at upper left corner, then **choose your device size** at Main.storyboard, build and run

1.5 Compatible Devices and iOS version requiements

	iPhone7 Plus, iPhone7, iPhone6s Plus, iPhone6s, iPad Pro
	iOS 10.2.1 and above

## 2. Acknowledgements

We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBow](https://github.com/dorian3d/DBoW2) for loop detection.

Thanks the contributions of [Botao Hu](http://amber.botao.hu/) (from [Amber Garage](https://ambergarage.com/)) and [Yang Liu](https://github.com/wandermyz).

## 3. Licence

The source code is released under [GPLv3](http://www.gnu.org/licenses/) licence.

We are still working for improving the code readability. Welcome to contribute to VINS-Mobile or ask any issues via Github or contacting Peiliang LI <pliapATconnect.ust.hk> or Tong QIN <tong.qinATconnect.ust.hk>.

For commercial inqueries, please contact Shaojie SHEN <eeshaojieATust.hk>
