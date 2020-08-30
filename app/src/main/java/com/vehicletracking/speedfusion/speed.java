package com.vehicletracking.speedfusion;

import android.os.Handler;
import android.util.Log;
import android.widget.TextView;

public class speed {

    private final String TAG = getClass().getName().toString();

    int sampleCounter = 0;
    final int totalSamples = 10;
    long time0;
    static int i = 0;
    double aDelT0 = 0, v0 = 0, v = 0;
    TextView spd, spd_kmph;
    speed velocity;
    Handler handler;

    final int totalVelocityValues = 1000;
    double[] velocityValues = new double[totalVelocityValues];

    float[] linear_acceleration = new float[3];

    //final int totalAccl = 5;
    double[] accel = new double[totalSamples];


    public double getlinear_acceleration(float[] linear_acceleration) {

        return Math.sqrt(Math.pow(linear_acceleration[0], 2) + Math.pow(linear_acceleration[0], 2) + Math.pow(linear_acceleration[0], 2));

    }

    public double getAvg(double[] accel) {
        accel[sampleCounter] = getlinear_acceleration(linear_acceleration);
        double total = 0;
        for (int i = 0; i <= accel.length; i++) {
            total = total + accel[i];
        }
        return (total / accel.length);
    }


    public double getVelocity(float[] linear_acceleration, long time1) {

        //this.linearAcceleration = linearAcceleration;

        try {
            if (sampleCounter < (totalSamples - 1)) {
                if (sampleCounter == 0)
                    time0 = time1;
                accel[sampleCounter] = getlinear_acceleration(linear_acceleration);
                sampleCounter++;
            } else if (sampleCounter == (totalSamples - 1)) {
                accel[sampleCounter] = getlinear_acceleration(linear_acceleration);

                double avgAccel = getAvg(accel);
                long timeDelta = ((time1 - time0) / 1000);
                double aDelT1 = (avgAccel * timeDelta);
                Log.d(TAG, "aDelT1 = " + avgAccel + " * " + timeDelta + " = " + aDelT1);

                v = calculateVelovity(aDelT1);
                if (i != totalVelocityValues) {
                    velocityValues[i] = v;
                    i++;
                } else {
                    for (int j = 0; j < (totalVelocityValues - 1); j++)
                        velocityValues[j] = velocityValues[j + 1];
                    velocityValues[totalVelocityValues - 1] = v;
                }
                sampleCounter = 0;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return v;
    }

    private double calculateVelovity(double aDelT1) {
        double v = v0 + (aDelT1 - aDelT0);
        Log.d(TAG, "v = " + v0 + "+ (" + aDelT1 + " - " + aDelT0 + ") = " + v);
        v0 = v;
        aDelT0 = aDelT1;
        return v;
    }


    public double[] getVlArray() {
        return velocityValues;
    }


}