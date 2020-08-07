package com.vehicletracking.fusion;



import android.annotation.TargetApi;
import android.hardware.SensorManager;
import android.os.Build;

import com.vehicletracking.Basefilter;

import org.apache.commons.math3.complex.Quaternion;

import java.util.Arrays;

/*
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * A base implementation for an orientation IMU sensor fusion.
 * Created by kaleb on 7/6/17.
 */

public abstract class orientationfused implements Basefilter {

    protected static final float EPSILON = 0.000000001f;
    // Nano-second to second conversion
    protected static final float NS2S = 1.0f / 1000000000.0f;
    private static final String tag = orientationfused.class.getSimpleName();
    public static float DEFAULT_TIME_CONSTANT = 0.18f;
    // The coefficient for the fusedOrientation... 0.5 = means it is averaging the two
    // transfer functions (rotations from the gyroscope and
    // acceleration/magnetic, respectively).
    public float timeConstant;
    protected Quaternion rotationVector;
    protected long timestamp = 0;
    protected float[] output = new float[3];

    /**
     * Initialize a singleton instance.
     */
    public orientationfused() {
        this(DEFAULT_TIME_CONSTANT);
    }

    public orientationfused(float timeConstant) {
        this.timeConstant = timeConstant;

    }



    public void reset() {
        timestamp = 0;
        rotationVector = null;
    }

    public boolean isBaseOrientationSet() {
        return rotationVector != null;
    }

    /**
     * The complementary fusedOrientation coefficient, a floating point value between 0-1,
     * exclusive of 0, inclusive of 1.
     *
     * @param timeConstant
     */
    public void setTimeConstant(float timeConstant) {
        this.timeConstant = timeConstant;
    }

    /**
     * Calculate the fused orientation of the device.
     * @param gyro the gyroscope measurements.
     * @param timestamp the gyroscope timestamp
     * @param accel the acceleration measurements
     * @param magnetic the magnetic measurements
     * @return the fused orientation estimation.
     */
    public abstract float[] calculateFusedOrientation(float[] gyro, long timestamp, float[] accel, float[] magnetic);

    public void setBaseOrientation(Quaternion baseOrientation) {
        rotationVector = baseOrientation;
    }
}