#include "vins_system.hpp"

#ifdef ANDROID
#include <android/log.h>
#define printf(x...) __android_log_print(ANDROID_LOG_DEBUG, "vins_system", x)
#endif

VinsSystem::VinsSystem(const char* voc_file_path, 
						const char* pattern_file_path,
						const char* config_file_path)
	: voc_file(voc_file_path), 
	pattern_file(pattern_file_path),
	config_file(config_file_path) {

		vins_params = { // use iPhone 6s config as default
			480, 640,
			548.813, 549.477,
			238.520, 320.379,
			0.0, 0.065, 0.0,
			0.0, 0.0, 180.0,
			0.5, 0.002, 0.2, 4.0e-5,
<<<<<<< HEAD
                        0.06, 3
=======
			0.06, 3
>>>>>>> android
		};

		readVinsConfigFile(vins_params, config_file);

		setGlobalParam(vins_params.focal_length_x, vins_params.focal_length_y,
						vins_params.px, vins_params.py,
						vins_params.tic_x, vins_params.tic_y, vins_params.tic_z,
<<<<<<< HEAD
                                                vins_params.ric_y, vins_params.ric_p, vins_params.ric_r,
=======
						vins_params.ric_y, vins_params.ric_p, vins_params.ric_r,
>>>>>>> android
						vins_params.acc_n, vins_params.acc_w, vins_params.gyr_n, vins_params.gyr_w,
						vins_params.solver_time, vins_params.freq);

		feature_tracker = new FeatureTracker(vins_params.frame_width, vins_params.frame_height);

		vins = new VINS();

		vins->setIMUModel();

		vins->setExtrinsic();

		vins->setIMUModel();

		feature_tracker->vins_pnp.setExtrinsic();

		feature_tracker->vins_pnp.setIMUModel();

		if (LOOP_CLOSURE) {

			loop_closing_thread = boost::thread(&VinsSystem::detectLoopClosure, this);

			global_optimization_thread = boost::thread(&VinsSystem::globalOptimization, this);

		}

		fusion_thread = boost::thread(&VinsSystem::fusion, this);

		is_vins_running = true;

}

VinsSystem::~VinsSystem() {
	delete feature_tracker;
	delete vins;
	delete loop_closure;
}

void VinsSystem::detectLoopClosure() {

	if(LOOP_CLOSURE && !loop_closure)
	{
		printf("loop start load voc\n");
		TS(load_voc);
		loop_closure = new LoopClosure(voc_file, vins_params.frame_width, vins_params.frame_height);
		TE(load_voc);
		printf("loop load voc finish\n");

		voc_init_ok = true;
	}

	while (is_vins_running) {

		if(!LOOP_CLOSURE)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			continue;
		}

		bool loop_succ = false;
		if (loop_check_cnt < global_frame_cnt)
		{
			KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
			//assert(loop_check_cnt == cur_kf->global_index);
			loop_check_cnt++;
			cur_kf->check_loop = 1;

			cv::Mat current_image;
			current_image = cur_kf->image;

			std::vector<cv::Point2f> measurements_old;
			std::vector<cv::Point2f> measurements_old_norm;
			std::vector<cv::Point2f> measurements_cur;
			std::vector<int> features_id;
			std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;

			vector<cv::Point2f> cur_pts;
			vector<cv::Point2f> old_pts;
			cur_kf->extractBrief(current_image);
			printf("loop extract %lu feature\n", cur_kf->keypoints.size());
			loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
			if(loop_succ)
			{
				KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
				if (old_kf == NULL)
				{
					printf("NO such %dth frame in keyframe_database\n", old_index);
					assert(false);
				}
				printf("loop succ with %drd image\n", old_index);
				assert(old_index!=-1);

				Vector3d T_w_i_old;
				Matrix3d R_w_i_old;

				old_kf->getPose(T_w_i_old, R_w_i_old);
				cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
					measurements_old, measurements_old_norm);
				measurements_cur = cur_kf->measurements;
				features_id = cur_kf->features_id;

				if(measurements_old_norm.size()>MIN_LOOP_NUM)
				{

					Quaterniond Q_loop_old(R_w_i_old);
					RetriveData retrive_data;
					retrive_data.cur_index = cur_kf->global_index;
					retrive_data.header = cur_kf->header;
					retrive_data.P_old = T_w_i_old;
					retrive_data.Q_old = Q_loop_old;
					retrive_data.use = true;
					retrive_data.measurements = measurements_old_norm;
					retrive_data.features_ids = features_id;
					vins->retrive_pose_data = (retrive_data);

					//cout << "old pose " << T_w_i_old.transpose() << endl;
					//cout << "refinded pose " << T_w_i_refine.transpose() << endl;
					// add loop edge in current frame
					cur_kf->detectLoop(old_index);
					keyframe_database.addLoop(old_index);
					old_kf->is_looped = 1;
					loop_old_index = old_index;
				}
			}
			cur_kf->image.release();
		}

		if(loop_succ) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	}

}

void VinsSystem::globalOptimization() {

	while (is_vins_running) {
		if (start_global_optimization) {
			start_global_optimization = false;
			TS(global_optimization_thread);
			keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,
				loop_correct_t,
				loop_correct_r);
			vins->t_drift = loop_correct_t;
			vins->r_drift = loop_correct_r;
			TE(global_optimization_thread);
			boost::this_thread::sleep(boost::posix_time::milliseconds(1170));
		} 
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
	}

}

void VinsSystem::getMeasurements(std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> >& measurements) {
		while (true)
		{
			if (imu_msg_buf.empty() || img_msg_buf.empty())
				return;

			if (!(imu_msg_buf.back().header > img_msg_buf.front().header))
			{
				printf("wait for imu, only should happen at the beginning\n");
				return;
			}

			if (!(imu_msg_buf.front().header < img_msg_buf.front().header))
			{
				printf("throw img, only should happen at the beginning\n");
				img_msg_buf.pop();
				continue;
			}

			IMG_MSG img_msg = img_msg_buf.front();
			
			img_msg_buf.pop();

			std::vector<IMU_MSG> IMUs;
			while (imu_msg_buf.front().header <= img_msg.header)
			{
				IMUs.push_back(imu_msg_buf.front());
				imu_msg_buf.pop();
			}
			//printf("IMU_buf = %d\n", IMUs.size());
			measurements.emplace_back(IMUs, img_msg);
		}
		return;
}

void VinsSystem::sendImu(IMU_MSG &imu_msg) {
	double t = imu_msg.header;
	if (current_time < 0)
		current_time = t;
	double dt = (t - current_time);
	current_time = t;

	double ba[]{0.0, 0.0, 0.0};
	double bg[]{0.0, 0.0, 0.0};

	double dx = imu_msg.acc.x() - ba[0];
	double dy = imu_msg.acc.y() - ba[1];
	double dz = imu_msg.acc.z() - ba[2];

	double rx = imu_msg.gyr.x() - bg[0];
	double ry = imu_msg.gyr.y() - bg[1];
	double rz = imu_msg.gyr.z() - bg[2];
	//printf("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f\n", t, dt, dx, dy, dz, rx, ry, rz);

	vins->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void VinsSystem::fusion() {
	std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> > measurements;
	while (is_vins_running) {
		measurements.clear();
		std::unique_lock<std::mutex> lk(m_buf);
		con.wait(lk, [&]
		{
			getMeasurements(measurements);
			return measurements.size() != 0;
		});
		lk.unlock();
		waiting_lists = measurements.size();

		for(auto &measurement : measurements)
		{
			for(auto &imu_msg : measurement.first)
			{
				sendImu(imu_msg);
			}

			auto img_msg = measurement.second;
			map<int, Vector3d> image = img_msg.point_clouds;
			double header = img_msg.header;
			TS(process_image);
			vins->processImage(image, header, waiting_lists);
			TE(process_image);
			double time_vins = vins->Headers[WINDOW_SIZE];

			//update feature position for front-end
			if(vins->solver_flag == VINS::SolverFlag::NON_LINEAR)
			{
				m_depth_feedback.lock();
				solved_vins.header = vins->Headers[WINDOW_SIZE - 1];
				solved_vins.Ba = vins->Bas[WINDOW_SIZE - 1];
				solved_vins.Bg = vins->Bgs[WINDOW_SIZE - 1];
				solved_vins.P = vins->correct_Ps[WINDOW_SIZE-1].cast<double>();
				solved_vins.R = vins->correct_Rs[WINDOW_SIZE-1].cast<double>();
				solved_vins.V = vins->Vs[WINDOW_SIZE - 1];
				Vector3d R_ypr = Utility::R2ypr(solved_vins.R);
				solved_features.clear();
				for (auto &it_per_id : vins->f_manager.feature)
				{
					it_per_id.used_num = it_per_id.feature_per_frame.size();
					if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
						continue;
					if (it_per_id.solve_flag != 1)
						continue;
					int imu_i = it_per_id.start_frame;
					Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
					IMG_MSG_LOCAL tmp_feature;
					tmp_feature.id = it_per_id.feature_id;
					tmp_feature.position = vins->r_drift * vins->Rs[imu_i] * (vins->ric * pts_i + vins->tic) + vins->r_drift * vins->Ps[imu_i] + vins->t_drift;
					tmp_feature.track_num = (int)it_per_id.feature_per_frame.size();
					solved_features.push_back(tmp_feature);
				}
				m_depth_feedback.unlock();
			}

			if(imageCacheEnabled)
			{
				//add state into vins buff for alignwith image
				if(vins->solver_flag == VINS::SolverFlag::NON_LINEAR)
				{
					VINS_DATA_CACHE vins_data_cache;
					vins_data_cache.header = vins->Headers[WINDOW_SIZE-1];
					vins_data_cache.P = vins->correct_Ps[WINDOW_SIZE-1];
					vins_data_cache.R = vins->correct_Rs[WINDOW_SIZE-1];
					vins_pool.push(vins_data_cache);
				}
				else if(vins->failure_occur == true)
				{
					vins->drawresult.change_color = true;
					vins->drawresult.indexs.push_back(vins->drawresult.pose.size());
					segmentation_index++;
					keyframe_database.max_seg_index++;
					keyframe_database.cur_seg_index = keyframe_database.max_seg_index;

					while(!vins_pool.empty())
						vins_pool.pop();
				}
			}
			/**
			*** start build keyframe database for loop closure
			**/
			if(LOOP_CLOSURE)
			{
				static bool first_frame = true;
				if(vins->solver_flag != VINS::SolverFlag::NON_LINEAR)
					first_frame = true;
				if(vins->marginalization_flag == VINS::MarginalizationFlag::MARGIN_OLD && 
					vins->solver_flag == VINS::SolverFlag::NON_LINEAR && 
					!image_buf_loop.empty())
				{
					first_frame = false;
					if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
					{
						keyframe_freq = 0;
						/**
						** save the newest keyframe to the keyframe database
						** only need to save the pose to the keyframe database
						**/
						Vector3d T_w_i = vins->Ps[WINDOW_SIZE - 2];
						Matrix3d R_w_i = vins->Rs[WINDOW_SIZE - 2];
						m_image_buf_loop.lock();
						while(!image_buf_loop.empty() && image_buf_loop.front().second < vins->Headers[WINDOW_SIZE - 2])
						{
							image_buf_loop.pop();
						}
						//assert(vins->Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
						if(vins->Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
						{
							KeyFrame* keyframe = new KeyFrame(vins->Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
							keyframe->setExtrinsic(vins->tic, vins->ric);
							/*
							** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
							** and save the pointcloud to the keyframe for reprojection search correspondance
							*/
							keyframe->buildKeyFrameFeatures(*vins);
							keyframe_database.add(keyframe);

							global_frame_cnt++;
						}
						m_image_buf_loop.unlock();

					}
					else
					{
						first_frame = false;
					}
					// update loop info
					for (int i = 0; i < WINDOW_SIZE; i++)
					{
						if(vins->Headers[i] == vins->front_pose.header)
						{
							KeyFrame* cur_kf = keyframe_database.getKeyframe(vins->front_pose.cur_index);
							if (abs(vins->front_pose.relative_yaw) > 30.0 || vins->front_pose.relative_t.norm() > 10.0)
							{
								printf("Wrong loop\n");
								cur_kf->removeLoop();
								break;
							}
							cur_kf->updateLoopConnection(vins->front_pose.relative_t,
								vins->front_pose.relative_q,
								vins->front_pose.relative_yaw);
							break;
						}
					}
					/*
					** update the keyframe pose when this frame slides out the window and optimize loop graph
					*/
					int search_cnt = 0;
					for(int i = 0; i < keyframe_database.size(); i++)
					{
						search_cnt++;
						KeyFrame* kf = keyframe_database.getLastKeyframe(i);
						if(kf->header == vins->Headers[0])
						{
							kf->updateOriginPose(vins->Ps[0], vins->Rs[0]);
							//update edge
							// if loop happens in this frame, update pose graph;
							if (kf->has_loop)
							{
								kf_global_index = kf->global_index;
								start_global_optimization = true;
							}
							break;
						}
						else
						{
							if(search_cnt > 2 * WINDOW_SIZE)
								break;
						}
					}
					keyframe_freq++;
				}
			}
			waiting_lists--;

			//finish solve one frame
		}
	}
}

void VinsSystem::getImuMeasurements(double header, std::vector<IMU_MSG_LOCAL>& imu_measurements) {
	imu_measurements.clear();
	static double last_header = -1;
	if (last_header < 0 || local_imu_msg_buf.empty()) {
		last_header = header;
		return;
	}

	while (!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= last_header) {
		local_imu_msg_buf.pop();
	}

	while (!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= header) {
		imu_measurements.emplace_back(local_imu_msg_buf.front());
		local_imu_msg_buf.pop();
	}
	last_header = header;
}

void VinsSystem::readVinsConfigFile(VINS_PARAMS &params, const char *params_file_path) {
	if (!params_file_path || params_file_path[0] == '\0') {
		printf("Empty file path ...\n");
		return;
	}

	std::ifstream params_file(params_file_path);

	if (params_file.good()) {

		printf("Found VINS config file\n");

		std::string l1, l2, l3, l4, l5, l6;
		std::getline(params_file, l1);
		std::getline(params_file, l2);
		std::getline(params_file, l3);
		std::getline(params_file, l4);
		std::getline(params_file, l5);
		std::getline(params_file, l6);

		float intrinsics[4], extrinsics[10];
		float input_solver_time;
		int input_freq, input_frame_width, input_frame_height;
		// line 1: fx, fy, cx, cy
		if (std::sscanf(l1.c_str(), "%f %f %f %f",
				&intrinsics[0], &intrinsics[1], &intrinsics[2], &intrinsics[3]) == 4) {

			printf("Found VINS params fx: %.3f, fy: %.3f, cx: %.3f, cy: %.3f\n",
					intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);

			params.focal_length_x = intrinsics[0];
			params.focal_length_y = intrinsics[1];
			params.px = intrinsics[2];
			params.py = intrinsics[3];

		}
		// line 2: FREQ, solver time
		if (std::sscanf(l2.c_str(), "%d %f",
				&input_freq, &input_solver_time) == 2) {

			printf("Found VINS params freq: %d, solver time: %.3f\n",
					input_freq, input_solver_time);

			params.freq = input_freq;
			params.solver_time =  input_solver_time;

		}
		// line 3: tic_x, tic_y, tic_z
		if (std::sscanf(l3.c_str(), "%f %f %f",
				&extrinsics[0], &extrinsics[1], &extrinsics[2]) == 3) {

			printf("Found VINS params tic_x: %.3f, tic_y: %.3f, tic_z: %.3f\n",
					extrinsics[0], extrinsics[1], extrinsics[2]);

			params.tic_x = extrinsics[0];
			params.tic_y = extrinsics[1];
			params.tic_z = extrinsics[2];

		}
		// line 4: ric_y, ric_p, ric_r
		if (std::sscanf(l4.c_str(), "%f %f %f",
				&extrinsics[3], &extrinsics[4], &extrinsics[5]) == 3) {

			printf("Found VINS params ric_y: %.3f, ric_p: %.3f, ric_r: %.3f\n",
					extrinsics[3], extrinsics[4], extrinsics[5]);

			params.ric_y = extrinsics[3];
			params.ric_p = extrinsics[4];
			params.ric_r = extrinsics[5];

		}
		// line 5: acc_n, acc_w, gyr_n, gyr_w
		if (std::sscanf(l5.c_str(), "%f %f %f %f",
				&extrinsics[6], &extrinsics[7], &extrinsics[8], &extrinsics[9]) == 4) {

			printf("Found VINS params acc_n: %.3f, acc_w: %.3f, gyr_n: %.3f, gyr_w: %.3f\n",
					extrinsics[6], extrinsics[7], extrinsics[8], extrinsics[9]);

			params.acc_n = extrinsics[6];
			params.acc_w = extrinsics[7];
			params.gyr_n = extrinsics[8];
			params.gyr_w = extrinsics[9];

		}
		// line 6: width, height
		if (std::sscanf(l6.c_str(), "%d %d",
				&input_frame_width, &input_frame_height) == 2) {

			printf("Found VINS params width: %d, height: %d\n",
					input_frame_width, input_frame_height);

			params.frame_width = input_frame_width;
			params.frame_height = input_frame_height;

        }

	}

	params_file.close();
}

void VinsSystem::putAccelData(double imu_timestamp, double accel_x, double accel_y, double accel_z) {
	if (imu_prepare < 10) {
		imu_prepare++;
	}
	cur_acc.header = imu_timestamp;
	cur_acc.acc << accel_x,
		accel_y,
		accel_z;
}

void VinsSystem::putGyroData(double imu_timestamp, double gyro_x, double gyro_y, double gyro_z) {
	//The time stamp is the amount of time in seconds since the device booted.
	double header = imu_timestamp;
	if (header <= 0) {
		return;
	}
	if (imu_prepare < 10) {
		return;
	}

	IMU_MSG gyro_msg;
	gyro_msg.header = header;
	gyro_msg.gyr << gyro_x,
		gyro_y,
		gyro_z;

	if (gyro_buf.size() == 0) {
		gyro_buf.push_back(gyro_msg);
		gyro_buf.push_back(gyro_msg);
		return;
	} else {
		gyro_buf[0] = gyro_buf[1];
		gyro_buf[1] = gyro_msg;
	}
	//interpolation
	IMU_MSG imu_msg;
	if(cur_acc.header >= gyro_buf[0].header && cur_acc.header <= gyro_buf[1].header) {
		imu_msg.header = cur_acc.header;
		imu_msg.acc = cur_acc.acc;
		imu_msg.gyr = gyro_buf[0].gyr + (cur_acc.header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
		//printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].header, imu_msg->header, gyro_buf[1].header);
		//printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, gyro_buf[0].gyr.x(), imu_msg->gyr.x(), gyro_buf[1].gyr.x());
	} else {
		printf("imu error %lf %lf %lf\n", gyro_buf[0].header, cur_acc.header, gyro_buf[1].header);
		return;
	}

	lateast_imu_time = imu_msg.header;

	//img_msg callback
	IMU_MSG_LOCAL imu_msg_local;
	imu_msg_local.header = imu_msg.header;
	imu_msg_local.acc = imu_msg.acc;
	imu_msg_local.gyr = imu_msg.gyr;

	m_imu_feedback.lock();
	local_imu_msg_buf.push(imu_msg_local);
	m_imu_feedback.unlock();
	m_buf.lock();
	imu_msg_buf.push(imu_msg);
	//printf("IMU_buf timestamp %lf, acc_x = %lf\n",imu_msg_buf.front()->header,imu_msg_buf.front()->acc.x());
	m_buf.unlock();
	con.notify_one();
}

void VinsSystem::processFrame(double img_timestamp, cv::Mat& input_frame) {
	if (!input_frame.empty())
	{
		//printf("processFrame\n");
		IMG_MSG img_msg;

		if (lateast_imu_time <= 0) {
			cv::cvtColor(input_frame, input_frame, CV_BGRA2RGB);
			cv::flip(input_frame, input_frame, -1);
			return;
		}

		img_msg.header = img_timestamp;
		bool isNeedRotation = false; //input_frame.size() != frameSize;

		cv::Mat gray;
		if (input_frame.channels() == 4) // for mobile camera input 
		{
			cv::cvtColor(input_frame, gray, CV_RGBA2GRAY);
		}
		else if (input_frame.channels() == 3) // for dataset input
		{
			cv::cvtColor(input_frame, gray, CV_BGR2GRAY);
		}
		else 
		{
			gray = input_frame.clone();
		}
		cv::Mat img_with_feature;
		cv::Mat img_equa;
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
		clahe->setClipLimit(3);
		clahe->apply(gray, img_equa);
		//img_equa = gray;
		TS(time_feature);

		m_depth_feedback.lock();
		feature_tracker->solved_features = solved_features;
		feature_tracker->solved_vins = solved_vins;
		m_depth_feedback.unlock();

		m_imu_feedback.lock();
		getImuMeasurements(img_timestamp, feature_tracker->imu_msgs);
		m_imu_feedback.unlock();

		good_pts.clear();
		track_len.clear();
		vins_normal = (vins->solver_flag == VINS::SolverFlag::NON_LINEAR);
		feature_tracker->use_pnp = USE_PNP;
		feature_tracker->readImage(img_equa, img_with_feature, frame_cnt, good_pts, track_len, img_timestamp, pnp_P, pnp_R, vins_normal);
		TE(time_feature);
		//cvtColor(img_equa, img_equa, CV_GRAY2BGR);

		//image msg buf
		if (feature_tracker->img_cnt == 0) {
			img_msg.point_clouds = feature_tracker->image_msg;
			//img_msg callback
			m_buf.lock();
			img_msg_buf.push(img_msg);
			//printf("Img timestamp %lf\n",img_msg_buf.front()->header);
			m_buf.unlock();
			con.notify_one();
			if (imageCacheEnabled) {
				image_data_cache.header = img_timestamp;
				image_data_cache.image = input_frame;
				image_pool.push(image_data_cache);
			}

			if (LOOP_CLOSURE) {
				m_image_buf_loop.lock();
				cv::Mat loop_image = gray.clone();
				image_buf_loop.push(make_pair(loop_image, img_timestamp));
				if(image_buf_loop.size() > WINDOW_SIZE)
					image_buf_loop.pop();
				m_image_buf_loop.unlock();
			}
		}

		feature_tracker->img_cnt = (feature_tracker->img_cnt + 1) % FREQ;

		for (int i = 0; i < good_pts.size(); i++) {
			cv::circle(input_frame, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
		}

		TS(visualize);
		if (imageCacheEnabled) {
			//use aligned vins and image
			if (!vins_pool.empty() && !image_pool.empty()) {
				while (vins_pool.size() > 1) {
					vins_pool.pop();
				}

				while (!image_pool.empty() && image_pool.front().header < vins_pool.front().header) {
					image_pool.pop();
				}

				if (!vins_pool.empty() && !image_pool.empty()) {
					input_frame = image_pool.front().image;
					lateast_P = vins_pool.front().P;
					lateast_R = vins_pool.front().R;
				}

			} else if (!image_pool.empty()) {
				if (image_pool.size() > 10) {
					image_pool.pop();
				}
			}
		}

		if (USE_PNP) {
			lateast_P = pnp_P.cast<float>();
			lateast_R = pnp_R.cast<float>();
		}

		TE(visualize);
	} 
}

void VinsSystem::drawTrajectory(cv::Mat& input_frame) {

	if(vins->solver_flag == VINS::SolverFlag::NON_LINEAR) {
		vins->drawresult.pose.clear();
		vins->drawresult.pose = keyframe_database.refine_path;
		vins->drawresult.segment_indexs = keyframe_database.segment_indexs;
		vins->drawresult.Reprojection(vins->image_show, vins->correct_point_cloud, vins->correct_Rs, vins->correct_Ps, false);

		input_frame = vins->image_show;
	}

}

void VinsSystem::getVinsStatus(VINS_STATUS& vins_status) {

	std::string curr_status = "STA: SUCC!";

	if(vins->solver_flag != VINS::SolverFlag::NON_LINEAR)
	{
		switch (vins->init_status) {
		case VINS::InitStatus::FAIL_IMU:
			curr_status = "STA: FAIL_IMU";
			break;
		case VINS::InitStatus::FAIL_PARALLAX:
			curr_status = "STA: FAIL_PARA";
			break;
		case VINS::InitStatus::FAIL_RELATIVE:
			curr_status = "STA: FAIL_RELA";
			break;
		case VINS::InitStatus::FAIL_SFM:
			curr_status = "STA: FAIL_SFM";
			break;
		case VINS::InitStatus::FAIL_PNP:
			curr_status = "STA: FAIL_PNP";
			break;
		case VINS::InitStatus::FAIL_ALIGN:
			curr_status = "STA: FAIL_ALIGN";
			break;
		case VINS::InitStatus::FAIL_CHECK:
			curr_status = "STA: FAIL_COST";
			break;
		case VINS::InitStatus::SUCC:
			//curr_status = "STA: SUCC!";
			break;
		default:
			break;
		}

		vins_status.failed_times = vins->fail_times;

		vins_status.parallax = vins->parallax_num_view;

		vins_status.init_progress = vins->initProgress;

		vins_status.x_view = 0.0f;

		vins_status.y_view = 0.0f;

		vins_status.z_view = 0.0f;
	}
	else
	{
		vins_status.x_view = (float)vins->correct_Ps[frame_cnt][0];

		vins_status.y_view = (float)vins->correct_Ps[frame_cnt][1];

		vins_status.z_view = (float)vins->correct_Ps[frame_cnt][2];
	}

	vins_status.init_status = curr_status;

	vins_status.waiting_lists = waiting_lists;

	vins_status.feature_num = vins->feature_num;
}

void VinsSystem::reinitSystem() {
	vins->drawresult.planeInit = false;
	vins->failure_hand = true;
	vins->drawresult.change_color = true;
	vins->drawresult.indexs.push_back(vins->drawresult.pose.size());
	segmentation_index++;
	keyframe_database.max_seg_index++;
	keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
}

void VinsSystem::shutdownSystem() {
	is_vins_running = true;
	fusion_thread.join();
	loop_closing_thread.join();
	global_optimization_thread.join();
}
