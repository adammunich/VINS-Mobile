#include <jni.h>
#include <string>

#include <android/log.h>

#include "opencv2/opencv.hpp"
#include "vins_system.hpp"

#define printf(x...) __android_log_print(ANDROID_LOG_DEBUG, "vins_executor", x)

VinsSystem* vins_system = nullptr;

extern "C" {
JNIEXPORT void JNICALL
Java_me_li_ginger_vinsmobileandroid_MainActivity_initSystem(
        JNIEnv *env, jobject obj,
        jstring vocabulary_file_path,
        jstring pattern_file_path,
        jstring config_file_path) {
    const char *voc_file = env->GetStringUTFChars(vocabulary_file_path, 0);
    const char *pattern_file = env->GetStringUTFChars(pattern_file_path, 0);
    const char *config_file = env->GetStringUTFChars(config_file_path, 0);

    vins_system = new VinsSystem(voc_file, pattern_file, config_file);

//    env->ReleaseStringUTFChars(vocabulary_file_path, voc_file);
//    env->ReleaseStringUTFChars(pattern_file_path, pattern_file);
//    env->ReleaseStringUTFChars(config_file_path, config_file);
}

JNIEXPORT void JNICALL
Java_me_li_ginger_vinsmobileandroid_MainActivity_processFrame(
        JNIEnv *env, jobject obj,
        jdouble img_timestamp,
        jlong addr_rgba) {
    if (!vins_system) return;

    cv::Mat &input_frame = *(cv::Mat *) addr_rgba;

    vins_system->processFrame(img_timestamp, input_frame);

    vins_system->drawTrajectory(input_frame);

}

JNIEXPORT void JNICALL
Java_me_li_ginger_vinsmobileandroid_MainActivity_putAccelData(
        JNIEnv *env, jobject obj,
        jdouble accel_timestamp, jdouble accel_x,
        jdouble accel_y, jdouble accel_z) {
    if (!vins_system) return;

    vins_system->putAccelData(accel_timestamp, accel_x, accel_y, accel_z);
}

JNIEXPORT void JNICALL
Java_me_li_ginger_vinsmobileandroid_MainActivity_putGyroData(
        JNIEnv *env, jobject obj,
        jdouble gyro_timestamp, jdouble gyro_x,
        jdouble gyro_y, jdouble gyro_z) {
    if (!vins_system) return;

    vins_system->putGyroData(gyro_timestamp, gyro_x, gyro_y, gyro_z);
}

JNIEXPORT void JNICALL
Java_me_li_ginger_vinsmobileandroid_MainActivity_shutdownSystem(
        JNIEnv *env, jobject obj) {
    if (!vins_system) return;

    delete vins_system;
}
}