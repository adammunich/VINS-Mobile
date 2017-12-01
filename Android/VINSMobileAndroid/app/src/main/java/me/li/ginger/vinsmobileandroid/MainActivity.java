package me.li.ginger.vinsmobileandroid;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.TextView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

public class MainActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2, SensorEventListener {

    private static final String TAG = "MainActivity";
    private CameraBridgeViewBase mOpenCvCameraView;
    private SensorManager mSensorManager;
    private Sensor mAccelSensor, mGyroSensor;
    private String mVocabularyFilePath, mPatternFilePath, mConfigFilePath;
    private volatile boolean isVinsRunning = false;
    private int firstNCameraFrames = 0;

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
        System.loadLibrary("opencv_java3");
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

        mOpenCvCameraView.setCvCameraViewListener(this);

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        mAccelSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        mGyroSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        mVocabularyFilePath = Environment.getExternalStorageState() + "/brief_k10L6.bin";
        mPatternFilePath = Environment.getExternalStorageDirectory() + "/brief_pattern.yml";
        mConfigFilePath = Environment.getExternalStorageDirectory() + "/camparas.yml";

        new Thread(new Runnable() {
            @Override
            public void run() {
                initSystem();
            }
        }).start();

        // Example of a call to a native method
//        TextView tv = (TextView) findViewById(R.id.sample_text);
//        tv.setText(stringFromJNI());
    }

    Handler mHandler = new Handler() {
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case INIT_FINISHED:
                    isVinsRunning = true;
                    break;
            }
            super.handlerMessage(msg);
        }
    }

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
        return inputFrame.rgba();
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();

    public native void initSystem(String vocabularyFilePath, String patternFilePath, String configFilePath);

    public native void processFrame(double imgTimestamp, long addrRgba);

    public native void putAccelData(double accelTimestamp, double accelX, double accelY, double accelZ);

    public native void putGyroData(double gyroTimestamp, double gyroX, double gyroY, double gyroZ);

    public native void shutdownSystem();
}
