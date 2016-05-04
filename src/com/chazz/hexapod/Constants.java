package com.chazz.hexapod;


class Constants {
    static final float PI = 3.14159265f;
    static final float DEGTORAD = 0.01745329251f;
    static final float RADTODEG = 57.2957795131f;

    static final float MAX_SPEED = 1.5f;
    static final float SS_DURATION = 2.0f;
    static final float TURN_TOL = 0.005f;
    static final int MAXITER = 300;
    //position tolerance in cm
    static final float TOLERANCE = 0.05f;
    // angle to move things away from bounds by, in radians
    static final float ANGEPS = 0.01f;

    // in turning value per second
    static final float TURN_SLEW = 0.8f;
    static final float SPEED_SLEW = 0.4f;
}
