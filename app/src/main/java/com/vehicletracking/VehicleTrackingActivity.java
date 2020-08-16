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
import android.renderscript.Matrix3f;

import com.vehicletracking.fusion.orientationcomplimentaryfusion;
import com.vehicletracking.fusion.orientationfused;
import com.vehicletracking.fusion.orientationfusedkalman;
import com.vehicletracking.linearacceleration.linearacceleration;
import com.vehicletracking.linearacceleration.linearaccelerationfusion;
import com.vehicletracking.util.rotationutil;
import com.vehicletracking.util.util;

import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.complex.RootsOfUnity;

import java.sql.Array;
import java.util.Arrays;

import static android.view.KeyCharacterMap.ALPHA;

public class VehicleTrackingActivity extends AppCompatActivity implements SensorEventListener {
    private static final float NS2S = 1.0f / 1000000000.0f;
    public final float EPSILON = 0.000000001f;
    private final String APPLICATION_LOG_TAG = "vehicle_tracking";
    public SensorManager mSensorManager;
    // private float timestamp;
    //public boolean initState = true;
    public orientationfused ofk;
    public  float [] linearaccelerationfusion = new float[3];
    public orientationcomplimentaryfusion ocf;
    private boolean hasAcceleration = false;
    private int sensorType = Sensor.TYPE_GYROSCOPE;
    private float startTime = 0;
    private int count = 0;
    public rotationutil rotationutil;


    // angular speeds from gyro
    private float[] gyroscope = new float[3];

    // magnetic field vector
    private float[] magnetic = new float[3];

    // accelerometer vector
    private float[] acceleration = new float[3];

    private float[] gravity = new float[3];
    private float[] linear_acceleration = new  float[3];

    // orientation angles from accel and magnet
    public Quaternion previousRotationVector;

    // final orientation angles from sensor fusion

    public float[] rateOfRotation = new float[3];
    public  volatile float dt;
    // accelerometer and magnetometer based rotation matrix

    public volatile  Quaternion baseOrientation;
    public volatile Quaternion prevoiusRotationVector;
    public float epilson;
    public volatile Quaternion rotationVectorAccelerationMagnetic;
    public volatile long timestamp;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
        initSensorListeners();

    }
    protected float[] lowPass( float[] input, float[] output ) {
        if ( output == null ) return input;

        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return output;
    }
    public void setSensorType(int sensorType) {
        if(sensorType != Sensor.TYPE_GYROSCOPE && sensorType != Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {
            throw new IllegalStateException("Sensor Type must be Sensor.TYPE_GYROSCOPE or Sensor.TYPE_GYROSCOPE_UNCALIBRATED");
        }

        this.sensorType = sensorType;
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

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(sensorType),
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
                System.arraycopy(event.values, 0, acceleration, 0, 3);
                Log.i(APPLICATION_LOG_TAG, "ACCELEROMETER values x: " + acceleration[0] + " y:" + acceleration[1] + " z : " + acceleration[2]);
               getBaseOrientation(acceleration,magnetic);
                break;

            case Sensor.TYPE_GYROSCOPE:
                // process gyro data
                Log.i(APPLICATION_LOG_TAG, "GYROSCOPE values x: " + event.values[0] + " y:" + event.values[1] + " z : " + event.values[2]);



                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    magnetic = lowPass(event.values.clone(), magnetic);
                }
                // copy new magnetometer data into magnet array
                System.arraycopy(event.values, 0, magnetic, 0, 3);
                Log.i(APPLICATION_LOG_TAG, "magnetic values x: " + magnetic[0] + " y:" + magnetic[1] + " z : " + magnetic[2]);
                getBaseOrientation(acceleration, magnetic);

                //getAccelerationMagneticRotationVector(orientation);


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
                getBaseOrientation(acceleration, magnetic);
                orientationfusedkalman ok = new orientationfusedkalman();



                ok.calculate();
                ok.calculateFusedOrientation(gyroscope,timestamp,acceleration,magnetic);

                float[] arr= ok.calculate() ;
                for (float f:arr){
                    System.out.println(f);

                }




                break;






        }
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
    public float[] getBaseOrientation(float[] acceleration, float[] magnetic) {
        // To get the orientation vector from the acceleration and magnetic
        // sensors, we let Android do the heavy lifting. This call will
        // automatically compensate for the tilt of the compass and fail if the
        // magnitude of the acceleration is not close to 9.82m/sec^2. You could
        // perform these steps yourself, but in my opinion, this is the best way
        // to do it.
        float[] rotationMatrix = new float[9];
        if (SensorManager.getRotationMatrix(rotationMatrix, null, acceleration, magnetic)) {
            float[] baseOrientation;
            baseOrientation = new float[3];
            SensorManager.getOrientation(rotationMatrix, baseOrientation);
            return baseOrientation;
            //float[] arr= baseOrientation ;
           // for (float f:arr){
                //System.out.println(f);

            //}

        }

        return null;
    }


    public class linearaccelerationfusion extends linearacceleration {

        public linearaccelerationfusion( orientationfusedkalman orientationfusedkalman ) {
            super(orientationfusedkalman);
            float[] arr= linearaccelerationfusion ;
             for (float f:arr){
            System.out.println(f);

            }
        }

        @Override
        public float[] getGravity(float[] values) {
            return util.getGravityFromOrientation(filter.filter(values));
        }
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
        mSensorManager.unregisterListener(this);
    }




}

