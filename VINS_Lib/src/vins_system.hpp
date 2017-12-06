#ifndef VINS_SYSTEM_H_
#define VINS_SYSTEM_H_

#include <thread>
#include <condition_variable>
#include <boost/thread.hpp>
#include "VINS.hpp"
#include "loop_closure.h"
#include "keyframe_database.h"

struct IMU_MSG {
	double header;
	Vector3d acc;
	Vector3d gyr;
};

struct IMG_MSG {
	double header;
	map<int, Vector3d> point_clouds;
};

struct IMG_DATA {
	double header;
	cv::Mat image;
};

struct IMG_DATA_CACHE {
	double header;
	cv::Mat equ_image;
	cv::Mat image;
};

struct VINS_DATA_CACHE {
	double header;
	Vector3f P;
	Matrix3f R;
};

struct VINS_STATUS {
	std::string init_status;
	int failed_times;
	int parallax;
	int init_progress;
	float x_view;
	float y_view;
	float z_view;
	int waiting_lists;
	int feature_num;
};

struct VINS_PARAMS {
	int frame_width;
	int frame_height;
	double focal_length_x;
	double focal_length_y;
	double px;
	double py;
	double tic_x; 
	double tic_y;
	double tic_z;
	double ric_y;
	double ric_p;
	double ric_r;
	double acc_n;
	double acc_w;
	double gyr_n;
	double gyr_w;
	double solver_time;
	int freq;
};

class VinsSystem {
public:

	VinsSystem(const char* voc_file_path, const char* pattern_file_path, const char* config_file_path);

	~VinsSystem();

	void processFrame(double img_timestamp, cv::Mat& input_frame);

	void putAccelData(double imu_timestamp, double accel_x, double accel_y, double accel_z);

	void putGyroData(double imu_timestamp, double gyro_x, double gyro_y, double gyro_z);

	void drawTrajectory(cv::Mat& input_frame);

	void drawAr(cv::Mat& input_frame);

	void setArCubePosition(float ratio_x, float ratio_y);

	void getVinsStatus(VINS_STATUS& vins_status);

	void reinitSystem();

	void shutdownSystem();

private:

	bool is_vins_running = false;

	const char* voc_file;

	const char* pattern_file;

	const char* config_file;

	std::vector<Point2f> good_pts;

	std::vector<double> track_len;

	bool vins_normal = false;

	boost::thread fusion_thread;

	boost::thread loop_closing_thread;

	boost::thread global_optimization_thread;

	VINS_PARAMS vins_params;

	int kf_global_index = -1;

	bool start_global_optimization = false;

	bool imuDataFinished = false;

	bool vinsDataFinished = false;

	std::vector<IMU_MSG> gyro_buf;  // for Interpolation

	/*
	Main process image thread: this thread detects and track feature between two continuous images
	and takes the newest VINS result and the corresponding image to draw AR and trajectory.
	*/
	queue<IMG_DATA_CACHE> image_pool;

	queue<VINS_DATA_CACHE> vins_pool;

	IMG_DATA_CACHE image_data_cache;

	Vector3f lateast_P = Vector3f::Zero();

	Matrix3f lateast_R = Matrix3f::Zero();

	Vector3d pnp_P = Vector3d::Zero();

	Matrix3d pnp_R = Matrix3d::Zero();

	IMU_MSG cur_acc;

	/******************************* UI CONFIG *******************************/

	FeatureTracker *feature_tracker = nullptr;

	VINS *vins = nullptr;

	// Store the fesature data processed by featuretracker
	queue<IMG_MSG> img_msg_buf;

	// Store the IMU data for vins
	queue<IMU_MSG> imu_msg_buf;

	// Store the IMU data for motion-only vins
	queue<IMU_MSG_LOCAL> local_imu_msg_buf;

	// The number of measurements waiting to be processed
	int waiting_lists = 0;

	int frame_cnt = 0;

	// Lock the feature and imu data buffer
	std::mutex m_buf;

	std::condition_variable con;

	double current_time = -1;

	double lateast_imu_time = -1;

	int imu_prepare = 0;

	// Segment the trajectory using color when re-initialize
	int segmentation_index = 0;

	// Set true:  30 HZ pose output and AR rendering in front-end (very low latency)
	// Set false: 10 HZ pose output and AR rendering in back-end
	bool USE_PNP = false;

	// Lock the solved VINS data feedback to featuretracker
	std::mutex m_depth_feedback;

	// Lock the IMU data feedback to featuretracker
	std::mutex m_imu_feedback;

	// Solved VINS feature feedback to featuretracker
	list<IMG_MSG_LOCAL> solved_features;

	// Solved VINS status feedback to featuretracker
	VINS_RESULT solved_vins;

	/******************************* Feature Tracking ******************************/
	cv::Mat gray;

	cv::Mat img_with_feature;
		
	cv::Ptr<cv::CLAHE> clahe;

	/******************************* Fusion ******************************/

	std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> > measurements;

	map<int, Vector3d> image;

	/******************************* Loop Closure ******************************/

	// Raw image data buffer for extracting FAST feature
	queue<pair<cv::Mat, double> > image_buf_loop;

	// Lock the image_buf_loop
	std::mutex m_image_buf_loop;

	// Detect loop
	LoopClosure *loop_closure = nullptr;

	// Keyframe database
	KeyFrameDatabase keyframe_database;

	// Control the loop detection frequency
	int keyframe_freq = 0;

	// Index the keyframe
	int global_frame_cnt = 0;

	// Record the checked loop frame
	int loop_check_cnt = 0;

	// Indicate if breif vocabulary read finish
	bool voc_init_ok = false;

	// Indicate the loop frame index
	int old_index = -1;

	int loop_old_index = -1;

	// Translation drift
	Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);

	// Rotation drift
	Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

	std::vector<cv::Point2f> measurements_old;

	std::vector<cv::Point2f> measurements_old_norm;

	std::vector<cv::Point2f> measurements_cur;

	std::vector<int> features_id;

	std::vector<cv::Point2f> measurements_cur_origin;

	cv::Mat current_image;

	vector<cv::Point2f> cur_pts;

	vector<cv::Point2f> old_pts;

	/******************************* Loop Closure ******************************/

	// MARK: Unity Camera Mode Switching
	// Ground truth from UI switch property "self.switchUIAREnabled"

	// Implied, updated by updateCameraMode()
	bool imuPredictEnabled = false;

	// Implied, updated by updateCameraMode()
	bool cameraMode = true;

	// Implied, updated by updateCameraMode()
	bool imageCacheEnabled = cameraMode && !USE_PNP;

	/******************************* Render ******************************/

	// for draw trajectory
	cv::Mat thumbnail_frame;
	cv::Mat thumbnail_region;

	// for draw AR
	cv::Mat ar_mask;

private:

	// Loop detection thread: this thread detect loop for newest keyframe and retrieve features
	void detectLoopClosure();

	// GLobal Pose graph thread: optimize global pose graph based on realative pose 
	// from vins and update the keyframe database
	void globalOptimization();

	// Send imu data and visual data into VINS
	void getMeasurements(std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> >& measurements);

	void sendImu(IMU_MSG &imu_msg);

	void fusion();

	void getImuMeasurements(double header, std::vector<IMU_MSG_LOCAL>& imu_measurements);

	void readVinsConfigFile(VINS_PARAMS &params, const char *params_file_path);

};

#endif