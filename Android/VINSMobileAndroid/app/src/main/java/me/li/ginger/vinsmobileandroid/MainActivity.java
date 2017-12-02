package me.li.ginger.vinsmobileandroid;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.os.SystemClock;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

public class MainActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2, SensorEventListener {

    private static final String TAG = "MainActivity";
    private static final int INIT_FINISHED = 0x00010001;
    private CameraBridgeViewBase mOpenCvCameraView;
    private SensorManager mSensorManager;
    private Sensor mAccelSensor, mGyroSensor;
    private String mVocabularyFilePath, mPatternFilePath, mConfigFilePath;
    private volatile boolean isVinsRunning = false;
    private int firstNCameraFrames = 0;
    private int prevImuDataType = Sensor.TYPE_GYROSCOPE;

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("opencv_java3");
        System.loadLibrary("vins");
        System.loadLibrary("vins_executor");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.java_surface_view);

        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);

        mOpenCvCameraView.enableFpsMeter();

        mOpenCvCameraView.setCameraIndex(0);

        mOpenCvCameraView.setMaxFrameSize(640, 480);

        mOpenCvCameraView.setCvCameraViewListener(this);

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        mAccelSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        mGyroSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        mVocabularyFilePath = Environment.getExternalStorageDirectory() + "/brief_k10L6.bin";
        mPatternFilePath = Environment.getExternalStorageDirectory() + "/brief_pattern.yml";
        mConfigFilePath = Environment.getExternalStorageDirectory() + "/camparas.yml";

        new Thread(new Runnable() {
            @Override
            public void run() {
                initSystem(mVocabularyFilePath, mPatternFilePath, mConfigFilePath);
                mHandler.sendEmptyMessage(INIT_FINISHED);
            }
        }).start();

    }

    Handler mHandler = new Handler() {
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case INIT_FINISHED:
                    isVinsRunning = true;
                    Toast.makeText(MainActivity.this, "System Initialized", Toast.LENGTH_SHORT).show();
                    break;
            }
            super.handleMessage(msg);
        }
    };

    @Override
    protected void onResume() {
        super.onResume();
        mOpenCvCameraView.enableView();
        mSensorManager.registerListener(this, mAccelSensor, 10000);
        mSensorManager.registerListener(this, mGyroSensor, 10000);
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null) {
            mOpenCvCameraView.disableView();
        }
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        if (firstNCameraFrames < 30) {
            firstNCameraFrames++;
        } else {
            if (isVinsRunning) {
                double imgTimestamp = SystemClock.elapsedRealtimeNanos() / Math.pow(10, 9);
                Mat mRgba = inputFrame.rgba();
                processFrame(imgTimestamp, mRgba.getNativeObjAddr());
                //Log.d(TAG, "20171202 img: " + imgTimestamp);
                return mRgba;
            }
        }

        return inputFrame.rgba();
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        Sensor sensor = sensorEvent.sensor;

        if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

            if (prevImuDataType == Sensor.TYPE_GYROSCOPE) {
                double accelTimestamp = SystemClock.elapsedRealtimeNanos() / Math.pow(10, 9);
                putAccelData(accelTimestamp, sensorEvent.values[0], sensorEvent.values[1], sensorEvent.values[2]);
                prevImuDataType = Sensor.TYPE_ACCELEROMETER;
//                Log.d(TAG, "20171202 acc: " + accelTimestamp + " " + sensorEvent.values[0] + " " +
//                        sensorEvent.values[1] + " " + sensorEvent.values[2]);
            }

        } else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {

            if (prevImuDataType == Sensor.TYPE_ACCELEROMETER) {
                double gyroTimestamp = SystemClock.elapsedRealtimeNanos() / Math.pow(10, 9);
                putGyroData(gyroTimestamp, sensorEvent.values[0], sensorEvent.values[1], sensorEvent.values[2]);
                prevImuDataType = Sensor.TYPE_GYROSCOPE;
//                Log.d(TAG, "20171202 gyr: " + gyroTimestamp + " " + sensorEvent.values[0] + " " +
//                        sensorEvent.values[1] + " " + sensorEvent.values[2]);
            }

        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native void initSystem(String vocabularyFilePath, String patternFilePath, String configFilePath);

    public native void processFrame(double imgTimestamp, long addrRgba);

    public native void putAccelData(double accelTimestamp, double accelX, double accelY, double accelZ);

    public native void putGyroData(double gyroTimestamp, double gyroX, double gyroY, double gyroZ);

    public native void shutdownSystem();
}
