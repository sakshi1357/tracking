package com.vehicletracking;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;

import com.vehicletracking.fusion.orientationcomplimentaryfusion;

import org.apache.commons.math3.complex.Quaternion;

public class VehicleTrackingActivity extends AppCompatActivity implements SensorEventListener {
    private static final float NS2S = 1.0f / 1000000000.0f;
    public final float EPSILON = 0.000000001f;
    private final String APPLICATION_LOG_TAG = "vehicle_tracking";
    private SensorManager mSensorManager;
    private float timestamp;
    public boolean initState = true;

    public com.vehicletracking.kalman.rotationkalmanfilter kalmanFilter;
    public com.vehicletracking.kalman.rotationprocessmodel pm;
    public com.vehicletracking.kalman.rotationmeasurementtool mm;
    public orientationfusedkalman ofk;
    // angular speeds from gyro
    private float[] gyro = new float[3];

    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];

    // orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];

    // magnetic field vector
    private float[] magnetic = new float[3];

    // accelerometer vector
    private float[] accel = new float[3];

    private float[] gravity = new float[3];
    private float[] linear_acceleration = new  float[3];

    // orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];

    // final orientation angles from sensor fusion
    private float[] fusedOrientation = new float[3];

    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];
    public  float[] newAccMagValue = new float[3];
    private long factor;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;

        // initialise gyroMatrix with identity matrix
        gyroMatrix[0] = 1.0f;
        gyroMatrix[1] = 0.0f;
        gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f;
        gyroMatrix[4] = 1.0f;
        gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f;
        gyroMatrix[7] = 0.0f;
        gyroMatrix[8] = 1.0f;

        mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
        initSensorListeners();
    }

    private void initSensorListeners() {
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_FASTEST);

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                // copy new accelerometer data into accel array
                // then calculate new orientation
                System.arraycopy(event.values, 0, accel, 0, 3);
                Log.i(APPLICATION_LOG_TAG, "ACCELEROMETER values x: " + accel[0] + " y:" + accel[1] + " z : " + accel[2]);
                calculateAccMagOrientation();
                break;

            case Sensor.TYPE_GYROSCOPE:
                // process gyro data
                Log.i(APPLICATION_LOG_TAG, "GYROSCOPE values x: " + event.values[0] + " y:" + event.values[1] + " z : " + event.values[2]);
                gyrofunction(event);
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                // copy new magnetometer data into magnet array
                System.arraycopy(event.values, 0, magnetic, 0, 3);
                Log.i(APPLICATION_LOG_TAG, "magnetic values x: " + magnetic[0] + " y:" + magnetic[1] + " z : " + magnetic[2]);
                break;


                case  Sensor.TYPE_LINEAR_ACCELERATION:

                    final float alpha = (float) 0.8;
                    // Isolate the force of gravity with the low-pass filter.
                    gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
                    gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
                    gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

                    // Remove the gravity contribution with the high-pass filter.
                    linear_acceleration[0] = event.values[0] - gravity[0];
                    linear_acceleration[1] = event.values[1] - gravity[1];
                    linear_acceleration[2] = event.values[2] - gravity[2];


                    Log.i(APPLICATION_LOG_TAG, "linear acceleration value x:" + event.values[0] + "y:" + event.values[1] +"z:" + event.values[2] );
                    break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    private void calculateAccMagOrientation() {
        Log.i(APPLICATION_LOG_TAG, "calculateAccMagOrientation");
        if (SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnetic)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);

        }
    }

    private void getRotationVectorFromGyro(float[] gyroValues,float[] deltaRotationVector, float timeFactor) {
        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude =
                (float) Math.sqrt(gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);

        // Normalize the rotation vector if it's big enough to get the axis
        if (omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }


       // double[] deltaRotationVector = new double[4];
        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;

       // return gyroOrientation. new Quaternion(deltaRotationVector[4],Arrays.copyOfRange(deltaRotationVector,0,3)));


    }

    public void gyrofunction(SensorEvent event) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;

        // initialisation of the gyroscope based rotation matrix

            float[] initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;


        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if (timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            System.arraycopy(event.values, 0, gyro, 0, 3);
            Log.i(APPLICATION_LOG_TAG, "getRotationVectorFromGyro x:" + event.values[0] + "y:" + event.values[1] +"z:" + event.values[2] );
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);


        }

        // measurement done, save current time for next interval
        timestamp = event.timestamp;

        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
     ;
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float) Math.sin(o[1]);
        float cosX = (float) Math.cos(o[1]);
        float sinY = (float) Math.sin(o[2]);
        float cosY = (float) Math.cos(o[2]);
        float sinZ = (float) Math.sin(o[0]);
        float cosZ = (float) Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f;
        xM[1] = 0.0f;
        xM[2] = 0.0f;
        xM[3] = 0.0f;
        xM[4] = cosX;
        xM[5] = sinX;
        xM[6] = 0.0f;
        xM[7] = -sinX;
        xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY;
        yM[1] = 0.0f;
        yM[2] = sinY;
        yM[3] = 0.0f;
        yM[4] = 1.0f;
        yM[5] = 0.0f;
        yM[6] = -sinY;
        yM[7] = 0.0f;
        yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ;
        zM[1] = sinZ;
        zM[2] = 0.0f;
        zM[3] = -sinZ;
        zM[4] = cosZ;
        zM[5] = 0.0f;
        zM[6] = 0.0f;
        zM[7] = 0.0f;
        zM[8] = 1.0f;

        //rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    public static class orientationfusedkalman extends com.vehicletracking.fusion.orientationfused {

        public static final String tag = orientationcomplimentaryfusion.class.getSimpleName();

        public com.vehicletracking.kalman.rotationkalmanfilter kalmanFilter;
        public com.vehicletracking.kalman.rotationprocessmodel pm;
        public com.vehicletracking.kalman.rotationmeasurementtool mm;
        public volatile boolean run;
        public volatile float dt;
        public volatile float[] fusedOrientation = new float[3];
        public volatile float[] accel = new float[3];
        public volatile float[] magnetic = new float[3];
        public volatile float[] gyro = new float[4];
        private Thread thread;

        public orientationfusedkalman() {
            this(DEFAULT_TIME_CONSTANT);
        }

        public orientationfusedkalman(float timeConstant) {
            super(timeConstant);

            pm = new com.vehicletracking.kalman.rotationprocessmodel();
            mm = new com.vehicletracking.kalman.rotationmeasurementtool();

            kalmanFilter = new com.vehicletracking.kalman.rotationkalmanfilter(pm, mm);


        }


        @Override
        public void reset() {
            super.reset();
        }

        @Override
        public boolean isBaseOrientationSet() {
            return super.isBaseOrientationSet();
        }

        @Override
        public void setBaseOrientation(Quaternion baseOrientation) {
            super.setBaseOrientation(baseOrientation);
        }




        /**
         * Calculate the fused orientation of the device.
         *
         * @param gyro    the gyroscope measurements.
         * @param timestamp    the gyroscope timestamp
         * @param accel the acceleration measurements
         * @param magnetic     the magnetic measurements
         * @return the fused orientation estimation.
         */
        @Override
        public float[] calculateFusedOrientation(float[] gyro, long timestamp, float[] accel, float[] magnetic) {
            return new float[0];
        }




        @Override
        public float[] filter(float[] values) {
            return new float[0];
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mSensorManager.unregisterListener(this);
    }




    }