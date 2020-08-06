package com.vehicletracking;

public interface Basefilter {
    float[] filter(float[] values);
    void setTimeConstant(float timeConstant);
    void reset();
}