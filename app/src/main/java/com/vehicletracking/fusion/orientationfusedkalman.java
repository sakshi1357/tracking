package com.vehicletracking.fusion;
import android.hardware.SensorManager;
import android.util.Log;

import com.vehicletracking.kalman.rotationkalmanfilter;
import com.vehicletracking.kalman.rotationmeasurementtool;
import com.vehicletracking.kalman.rotationprocessmodel;
import com.vehicletracking.util.angleutil;
import com.vehicletracking.util.rotationutil;

import org.apache.commons.math3.complex.Quaternion;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

public class orientationfusedkalman extends orientationfused {

    private static final String TAG = orientationcomplimentaryfusion.class.getSimpleName();

    private final rotationkalmanfilter kalmanFilter;
    private final AtomicBoolean run;
    private volatile float dT;
    private volatile float[] output = new float[3];
    private Thread thread;

    private volatile Quaternion rotationVectorAccelerationMagnetic;
    private final double[] vectorGyroscope = new double[4];
    private final double[] vectorAccelerationMagnetic = new double[4];

    public orientationfusedkalman() {
        this(DEFAULT_TIME_CONSTANT);
    }

    public orientationfusedkalman(float timeConstant) {
        super(timeConstant);
        run = new AtomicBoolean(false);
        kalmanFilter = new rotationkalmanfilter(new rotationprocessmodel(), new rotationmeasurementtool());
    }

    public void startFusion() {
        if (!run.get() && thread == null) {
            run.set(true);

            thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    while (run.get() && !Thread.interrupted()) {

                        output = calculate();

                        try {
                            Thread.sleep(20);
                        } catch (InterruptedException e) {
                            Log.e(TAG, "Kalman Thread", e);
                            Thread.currentThread().interrupt();
                        }
                    }

                    Thread.currentThread().interrupt();
                }
            });

            thread.start();
        }
    }

    public void stopFusion() {
        if (run.get() && thread != null) {
            run.set(false);
            thread.interrupt();
            thread = null;
        }
    }

    public float[] getOutput() {
        return output;
    }

    /**
     * Calculate the fused orientation of the device.
     *
     * Rotation is positive in the counterclockwise direction (right-hand rule). That is, an observer looking from some positive location on the x, y, or z axis at
     * a device positioned on the origin would report positive rotation if the device appeared to be rotating counter clockwise. Note that this is the
     * standard mathematical definition of positive rotation and does not agree with the aerospace definition of roll.
     *
     * See: https://source.android.com/devices/sensors/sensor-types#rotation_vector
     *
     * Returns a vector of size 3 ordered as:
     * [0]X points east and is tangential to the ground.
     * [1]Y points north and is tangential to the ground.
     * [2]Z points towards the sky and is perpendicular to the ground.
     *
     * @return An orientation vector -> @link SensorManager#getOrientation(float[], float[])}
     */
    public float[] calculate() {
        if (rotationVector != null && rotationVectorAccelerationMagnetic != null && dT != 0) {
            vectorGyroscope[0] = (float) rotationVector.getVectorPart()[0];
            vectorGyroscope[1] = (float) rotationVector.getVectorPart()[1];
            vectorGyroscope[2] = (float) rotationVector.getVectorPart()[2];
            vectorGyroscope[3] = (float) rotationVector.getScalarPart();

            vectorAccelerationMagnetic[0] = (float) rotationVectorAccelerationMagnetic.getVectorPart()[0];
            vectorAccelerationMagnetic[1] = (float) rotationVectorAccelerationMagnetic.getVectorPart()[1];
            vectorAccelerationMagnetic[2] = (float) rotationVectorAccelerationMagnetic.getVectorPart()[2];
            vectorAccelerationMagnetic[3] = (float) rotationVectorAccelerationMagnetic.getScalarPart();

            // Apply the Kalman fusedOrientation... Note that the prediction and correction
            // inputs could be swapped, but the fusedOrientation is much more stable in this
            // configuration.
            kalmanFilter.predict(vectorGyroscope);
            kalmanFilter.correct(vectorAccelerationMagnetic);

            // rotation estimation.
            Quaternion result = new Quaternion(kalmanFilter.getStateEstimation()[3], Arrays.copyOfRange(kalmanFilter.getStateEstimation(), 0, 3));

            output = angleutil.getAngles(result.getQ0(), result.getQ1(), result.getQ2(), result.getQ3());
        }

        return output;
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
        if(isBaseOrientationSet()) {
            if (this.timestamp != 0) {
                dT = (timestamp - this.timestamp) * NS2S;

                rotationVectorAccelerationMagnetic = rotationutil.getOrientationVectorFromAccelerationMagnetic(accel, magnetic);
                rotationVector = rotationutil.integrateGyroscopeRotation(rotationVector, gyro, dT, EPSILON);
            }
            this.timestamp = timestamp;

            return output;
        }  else {
            throw new IllegalStateException("You must call setBaseOrientation() before calling calculateFusedOrientation()!");
        }
    }

    @Override
    public float[] filter(float[] values) {
        return new float[0];
    }
}