package com.chazz;

import com.chazz.hexapod.Hexapod;

public class Main {

    public static void main(String[] args) {
        Hexapod hex = new Hexapod(true);

        double[] servoAngles = new double[18];
        for (int ii=0; ii<18; ii++)
        {
            servoAngles[ii] = 90;
        }


        hex.setServoAngle(servoAngles).setAngles();
        hex.setSpeed(0.0).setTurning(0.0);


        hex.safeStand();

        for(double t = 0.0; t < 35.0d; t+=0.5){
            hex.step(t);

            System.out.println(hex);
        }



    }
}
