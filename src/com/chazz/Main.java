package com.chazz;

import com.chazz.hexapod.Hexapod;

import java.util.Date;

public class Main {

    private static double getTime(){
        Date d = new Date();
        double sec = d.getTime()/1000;
        double usec = ((d.getTime()%1000)*1e-6);
        return (sec+usec);

    }
    public static void main(String[] args) {

        int ii;

        Hexapod hex = new Hexapod();
        double lasttime, dt, inittime;

        inittime = Main.getTime();
        System.out.println(inittime);
        float[] servoAngles = new float[18];
        for (ii=0; ii<18; ii++)
        {
            servoAngles[ii] = 90;
        }
        hex.setServoAngle(servoAngles);
        hex.setAngles();

        hex.setSpeed(0.0f);
        hex.setTurning(0.0f);

        lasttime = Main.getTime();
        System.out.println(hex);
        hex.safeStand();
        System.out.println(hex);
        while (hex.isSsRunning()){
            dt = (Main.getTime() - lasttime);
//            System.out.println(dt);
            hex.step((float) dt);


            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        System.out.println(hex);



    }
}
