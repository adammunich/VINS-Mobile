#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <sstream>
#include <math.h>
#include "boost/filesystem.hpp"
#include "boost/thread.hpp"
#include "vins_system.hpp"

#define PROCESS_EUROC_DATASET
//#define PROCESS_IPHONE_DATASET
//#define PROCESS_ANDROID_DATASET

void fusionLoop(bool running);

void imuLoop(bool running);

void imgLoop(bool running, const std::string& dataset_path);

void renderLoop(bool running);

size_t countIphoneImgNumber(const std::string& dir_path);

double readIphoneImgTimestamp(const std::string& file_path);

void readIphoneImuData(const std::string& file_path);

void readEurocTimestamp(const std::string& file_path);

void readEurocImuData(const std::string& file_path);

void readAndroidTimestamp(const std::string& file_path);

void readAndroidImuData(const std::string& file_path);

void drawSomething();

VinsSystem* vins_system;
std::vector< std::vector<double> > imu_storage;
std::map<unsigned int, double> iphone_timestamp_storage;
size_t iphone_img_counter = 0;
size_t iphone_img_num = 0;
std::vector<long> euroc_timestamp_storage;
std::vector<long> android_timestamp_storage;
std::queue<cv::Mat> tracked_frames;

int main(int argc, char** argv) {

    std::cout << "Test VINS dataset mode...\n";

    std::string m_dataset_path = argv[1];
    const char* config_path = argv[2];
    const char* voc_path = "../Resources/brief_k10L6.bin";
    const char* pattern_path = "../Resources/brief_pattern.yml";

#ifdef PROCESS_EUROC_DATASET
    const std::string euroc_dataset_path = m_dataset_path + "/cam0/data.csv";
    const std::string euroc_imu_path = m_dataset_path + "/imu0/data.csv";

    readEurocImuData(euroc_imu_path);

    readEurocTimestamp(euroc_dataset_path);
#endif 

#ifdef PROCESS_IPHONE_DATASET
    const std::string iphone_imu_path(m_dataset_path + "/IMU");
    const std::string iphone_timestamps_dir(m_dataset_path + "/IMAGE_TIME/");

    iphone_img_num = countIphoneImgNumber(iphone_timestamps_dir);

    readIphoneImuData(iphone_imu_path);
#endif 

#ifdef PROCESS_ANDROID_DATASET
    const std::string android_dataset_path = m_dataset_path + "/image_timestamps.txt";
    const std::string android_imu_path = m_dataset_path + "/imu.txt";

    readAndroidImuData(android_imu_path);

    readAndroidTimestamp(android_dataset_path);
#endif 

    std::cout << "Create new VINS system ...\n";

    vins_system = new VinsSystem(voc_path, pattern_path, config_path);

    volatile bool is_running = true;
                                                                  
    boost::thread imu_thread = boost::thread(&imuLoop, is_running);

    boost::thread img_thread = boost::thread(&imgLoop, is_running, m_dataset_path);

    cv::namedWindow("traj", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("tracked", CV_WINDOW_AUTOSIZE);

    std::cout << "Start processing images ...\n";

    while (is_running) {
        drawSomething();        
    }

    is_running = false;

    cv::destroyAllWindows();

    return 0;

}

size_t countIphoneImgNumber(const std::string &dir_path) {

    std::cout << "countIphoneImgNumber starts ...\n";

    std::vector<boost::filesystem::directory_entry> files;

    if (boost::filesystem::is_directory(dir_path)) {
        std::copy(
            boost::filesystem::directory_iterator(dir_path),
            boost::filesystem::directory_iterator(),
            std::back_inserter(files)
        );    

        std::cout << dir_path << " is a directory containing:\n";

        for (std::vector<boost::filesystem::directory_entry>::const_iterator iter = files.begin(); 
                iter != files.end(); ++iter) {

            //std::cout <<(*iter).path().stem().string() << "\n";
            unsigned int img_idx = std::stoi((*iter).path().stem().string());
            const std::string timestamp_path = (*iter).path().string();
            double img_timestamp = readIphoneImgTimestamp(timestamp_path);

            iphone_timestamp_storage.insert(std::pair<int, double>(img_idx, img_timestamp));
        }
    }

    std::cout << "Total timestamps: " << iphone_timestamp_storage.size() << "\n";

    return files.size();
}

void drawSomething() {
    cv::Mat show_image = cv::Mat::ones(cv::Size(640, 480), CV_8UC4);

    vins_system->drawTrajectory(show_image);

    VINS_STATUS vins_info;
    vins_system->getVinsStatus(vins_info);
    
    cv::putText(show_image, vins_info.init_status, cv::Point(10, 300), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "failed times: " + std::to_string(vins_info.failed_times), cv::Point(10, 320), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "parallax: " + std::to_string(vins_info.parallax), cv::Point(10, 340), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "features: " + std::to_string(vins_info.feature_num), cv::Point(10, 360), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "init progress: " + std::to_string(vins_info.init_progress), cv::Point(10, 380), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "waiting list: " + std::to_string(vins_info.waiting_lists), cv::Point(10, 400), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);    
    cv::putText(show_image, "x: " + std::to_string(vins_info.x_view), cv::Point(10, 420), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "y: " + std::to_string(vins_info.y_view), cv::Point(10, 440), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::putText(show_image, "z: " + std::to_string(vins_info.z_view), cv::Point(10, 460), 
                FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8, false);
    cv::imshow("traj", show_image);

    if (!tracked_frames.empty()) {
        cv::imshow("tracked", tracked_frames.front());
        tracked_frames.pop();
    }

    cv::waitKey(15);
}

void imgLoop(bool running, const std::string& dataset_path) {

#ifdef PROCESS_IPHONE_DATASET
    while (running && iphone_img_counter < iphone_img_num) {
        
        const std::string img_path(dataset_path + "/IMAGE/" + std::to_string(iphone_img_counter));

        // std::cout << "Image path: " << img_path << "\n";

        cv::Mat input_frame = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);

        if (input_frame.cols == 0 || input_frame.rows == 0) continue;

        // std::cout << "Image w: " << input_frame.cols << " h: " << input_frame.rows << "\n";

        // cv::rotate(input_frame, input_frame, cv::ROTATE_90_COUNTERCLOCKWISE);

        vins_system->processFrame(iphone_timestamp_storage.find(iphone_img_counter)->second, input_frame);

        iphone_img_counter++;

        tracked_frames.push(input_frame);

        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
#endif

#ifdef PROCESS_EUROC_DATASET  
    for (size_t i = 0; i < euroc_timestamp_storage.size(); ++i) {
        
        const std::string img_path(dataset_path + "/cam0/data/" + std::to_string(euroc_timestamp_storage[i]) + ".png");
        
        //std::cout << "Image path: " << img_path << "\n";
        
        cv::Mat input_frame = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);

        if (input_frame.cols == 0 || input_frame.rows == 0) continue;
        
        //std::cout << "Image w: " << input_frame.cols << " h: " << input_frame.rows << "\n";

        vins_system->processFrame(euroc_timestamp_storage[i] / pow(10, 9), input_frame);

        tracked_frames.push(input_frame);

        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
#endif 

#ifdef PROCESS_ANDROID_DATASET  
    for (size_t i = 0; i < android_timestamp_storage.size(); ++i) {
        
        const std::string img_path(dataset_path + "/image/" + std::to_string(android_timestamp_storage[i]) + ".png");
        
        //std::cout << "Image path: " << img_path << "\n";
        
        cv::Mat input_frame = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);

        if (input_frame.cols == 0 || input_frame.rows == 0) continue;
        
        //std::cout << "Image w: " << input_frame.cols << " h: " << input_frame.rows << "\n";

        vins_system->processFrame(android_timestamp_storage[i] / pow(10, 9), input_frame);

        tracked_frames.push(input_frame);

        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
#endif 

    std::cout << "imgLoop exits ...\n";
}

void imuLoop(bool running) {
    size_t imu_counter = 0;
    while (running && imu_counter < imu_storage.size()) {
        //printf("imu loop is running imu_counter = %lu...\n", imu_counter);
        
#ifdef PROCESS_IPHONE_DATASET
        vins_system->putAccelData(imu_storage[imu_counter][0], imu_storage[imu_counter][1], 
                                imu_storage[imu_counter][2], imu_storage[imu_counter][3]);
        vins_system->putGyroData(imu_storage[imu_counter][0], imu_storage[imu_counter][4], 
                                imu_storage[imu_counter][5], imu_storage[imu_counter][6]);
                
        // printf("imu: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", imu_storage[imu_counter][0], 
        //         imu_storage[imu_counter][4], imu_storage[imu_counter][5], imu_storage[imu_counter][6], 
        //         imu_storage[imu_counter][1], imu_storage[imu_counter][2], imu_storage[imu_counter][3]);
#endif

#ifdef PROCESS_EUROC_DATASET
        vins_system->putAccelData(imu_storage[imu_counter][0] / pow(10, 9), imu_storage[imu_counter][4], 
                                imu_storage[imu_counter][5], imu_storage[imu_counter][6]);
        vins_system->putGyroData(imu_storage[imu_counter][0] / pow(10, 9), imu_storage[imu_counter][1], 
                                imu_storage[imu_counter][2], imu_storage[imu_counter][3]);

        // printf("imu: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", imu_storage[imu_counter][0] / pow(10, 9), 
        //         imu_storage[imu_counter][4], imu_storage[imu_counter][5], imu_storage[imu_counter][6], 
        //         imu_storage[imu_counter][1], imu_storage[imu_counter][2], imu_storage[imu_counter][3]);

#endif

#ifdef PROCESS_ANDROID_DATASET
        vins_system->putAccelData(imu_storage[imu_counter][0], imu_storage[imu_counter][1], 
            imu_storage[imu_counter][2], imu_storage[imu_counter][3]);
        vins_system->putGyroData(imu_storage[imu_counter][4], imu_storage[imu_counter][5], 
            imu_storage[imu_counter][6], imu_storage[imu_counter][7]);  
        // printf("imu: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
        //         imu_storage[imu_counter][0], imu_storage[imu_counter][1], imu_storage[imu_counter][2], imu_storage[imu_counter][3], 
        //         imu_storage[imu_counter][4], imu_storage[imu_counter][5], imu_storage[imu_counter][6], imu_storage[imu_counter][7]);
#endif

        imu_counter++;

        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }

    std::cout << "imuLoop exits ...\n";
}

double readIphoneImgTimestamp(const std::string& file_path) {

    std::cout << "readIphoneImgTimestamp from: " << file_path << "\n";

    double timestamp = 0;

    std::ifstream in_stream(file_path, ios::in | ios::binary);

    if (in_stream.is_open()) {
        std::cout << "Timestamp file is opened..\n";

        char time_buf[8];

        in_stream.read(time_buf, 8);

        timestamp = *reinterpret_cast<double*>(time_buf);

        printf("Image timestamp: %f\n", timestamp);

        in_stream.close();
    }

    return timestamp;

}

void readIphoneImuData(const std::string& file_path) {

    std::cout << "readIphoneImuData from: " << file_path << "\n";

    std::ifstream in_stream(file_path, ios::in | ios::binary); 

    if (in_stream.is_open()) {
        std::cout << "IMU file is opened..\n";

        char time_buf[8], accel_x_buf[8], accel_y_buf[8], accel_z_buf[8], 
                gyro_x_buf[8], gyro_y_buf[8], gyro_z_buf[8];

        std::vector<double> imu_data(7);

        while (!in_stream.eof()) {

        imu_data.clear();

        in_stream.read(time_buf, 8);
        in_stream.read(accel_x_buf, 8);
        in_stream.read(accel_y_buf, 8);
        in_stream.read(accel_z_buf, 8);
        in_stream.read(gyro_x_buf, 8);
        in_stream.read(gyro_y_buf, 8);
        in_stream.read(gyro_z_buf, 8);

        double timestamp = *reinterpret_cast<double*>(&time_buf);
        double accel_x = *reinterpret_cast<double*>(&accel_x_buf);
        double accel_y = *reinterpret_cast<double*>(&accel_y_buf);
        double accel_z = *reinterpret_cast<double*>(&accel_z_buf);
        double gyro_x = *reinterpret_cast<double*>(&gyro_x_buf);
        double gyro_y = *reinterpret_cast<double*>(&gyro_y_buf);
        double gyro_z = *reinterpret_cast<double*>(&gyro_z_buf);

        printf("Imu timestamp: %f %f %f %f %f %f %f\n", timestamp, accel_x, accel_y, accel_z,
                                                    gyro_x, gyro_y, gyro_z);

         imu_data = {
            timestamp,
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z
        };

        imu_storage.push_back(imu_data);

        }

        in_stream.close();
    }

    std::cout << "IMU data total: " << imu_storage.size() << "\n";

}

void readEurocTimestamp(const std::string& file_path) {

    ifstream in_stream(file_path, ios::in);

    std::string curr_line;

    if (in_stream.is_open()) {

        getline(in_stream, curr_line);
        
        while (getline(in_stream, curr_line)) {

            //std::cout << curr_line << "\n";

            istringstream iss(curr_line);

            long timestamp_long;

            while (iss >> timestamp_long) {

                //std::cout << "Timestamp is: " << timestamp_long << "\n";

                euroc_timestamp_storage.push_back(timestamp_long);

                if (iss.peek() == ',') {
                    break;
                }

            }
        }

        in_stream.close();
    }

    printf("readEurocTimestamp found %lu images\n", euroc_timestamp_storage.size());

}

void readEurocImuData(const std::string& file_path) {

    ifstream in_stream(file_path, ios::in);

    std::string curr_line;

    if (in_stream.is_open()) {

        getline(in_stream, curr_line);

        std::vector<double> imu_data(7);

        while (getline(in_stream, curr_line)) {

            imu_data.clear();
            
            istringstream iss(curr_line);

            double data;

            while (iss >> data) {

                imu_data.push_back(data);

                if (iss.peek() == ',') {
                    iss.ignore();
                }

            }

            printf("%f %f %f %f %f %f %f\n", imu_data[0], imu_data[1], imu_data[2],
                                imu_data[3], imu_data[4], imu_data[5], imu_data[6]);

            imu_storage.push_back(imu_data);

        }

        in_stream.close();
    }

    printf("readEurocImuData found %lu imu data\n", imu_storage.size());

}

void readAndroidTimestamp(const std::string& file_path) {
    ifstream in_stream(file_path, ios::in);
    
        std::string curr_line;
    
        if (in_stream.is_open()) {
                
            while (getline(in_stream, curr_line)) {
    
                //std::cout << curr_line << "\n";
    
                istringstream iss(curr_line);
    
                long timestamp_long;
    
                while (iss >> timestamp_long) {
    
                    //std::cout << "Timestamp is: " << timestamp_long << "\n";
    
                    android_timestamp_storage.push_back(timestamp_long);
    
                    break;
    
                }
            }
    
            in_stream.close();
        }
    
        printf("readAndroidTimestamp found %lu images\n", android_timestamp_storage.size());
}

void readAndroidImuData(const std::string& file_path) {
    ifstream in_stream(file_path, ios::in);
    
        std::string curr_line;
    
        if (in_stream.is_open()) {
    
            getline(in_stream, curr_line);
    
            std::vector<double> imu_data(8);
    
            while (getline(in_stream, curr_line)) {
    
                imu_data.clear();
                
                istringstream iss(curr_line);
    
                double data;
    
                while (iss >> data) {
    
                    imu_data.push_back(data);
    
                    if (iss.peek() == ' ') {
                        iss.ignore();
                    }
    
                }
    
                printf("%f %f %f %f %f %f %f %f\n", imu_data[0], imu_data[1], imu_data[2],
                                    imu_data[3], imu_data[4], imu_data[5], imu_data[6], imu_data[7]);
    
                imu_storage.push_back(imu_data);
    
            }
    
            in_stream.close();
        }
    
        printf("readAndroidImuData found %lu imu data\n", imu_storage.size());
}