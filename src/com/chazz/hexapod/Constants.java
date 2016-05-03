package com.chazz.hexapod;


public class Constants {
    static final double PI = 3.14159265;
    static final double DEGTORAD = 0.01745329251;
    static final double RADTODEG = 57.2957795131;

    static final double MAX_SPEED = 1.5;
    static final double SS_DURATION = 2.0;
    static final double TURN_TOL = 0.005;
    static final int MAXITER = 300;
    //position tolerance in cm
    static final double TOLERANCE = 0.05;
    // angle to move things away from bounds by, in radians
    static final double ANGEPS = 0.01;

    // in turning value per second
    static final double TURN_SLEW = 0.8;
    static final double SPEED_SLEW = 0.4;
}
