package com.vehicletracking.linearacceleration;
import com.vehicletracking.fusion.orientationfused;
import com.vehicletracking.fusion.orientationfusedkalman;
import com.vehicletracking.util.util;

public class linearaccelerationfusion extends linearacceleration {

    public linearaccelerationfusion( orientationfusedkalman orientationfusedkalman ) {
        super(orientationfusedkalman);

    }

    @Override
    public float[] getGravity(float[] values) {
        return util.getGravityFromOrientation(filter.filter(values));
    }
}
