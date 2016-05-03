package com.chazz.hexapod;


/* servo IDs:
 *           back
 *	06 10 08 ---- 07 09 11
 *	12 16 14 ---- 13 15 17
 *	18 04 02 ---- 01 03 05
 *           front
 *
 * convert to index:
 *  (LEG 5) 17 16 15 ---- 06 07 08 (LEG 2)
 *  (LEG 4) 14 13 12 ---- 03 04 05 (LEG 1)
 *  (LEG 3) 11 10 09 ---- 00 01 02 (LEG 0)
 *
 * tibia, femur, coxa ---- coxa, femur, tibia
 *
 * limits (ignoring intersection with other limbs):
 * 	coxa: 245 - 75
 * 	femur: 250 - 55
 * 	tibia: 215 - 40
 */

/* Coordinate system:
 *  Origin is bottom of main chassis, center horizontally
 *    Z OO==> Y
 *      ||
 *      \/ X
 */


public class Hexapod {


    // both sets of angles are relative to leg root and leg angle
    private double[] servoangle = new double[18]; // in degrees, 0 to 300 or so (subject to more constraint)
    private double[] angle = new double[18]; // in radians, direction and offset corrected
     // Hexapod body
    private double[] length = new double[3]; // length of coxa, femur, tibia
    private double femurangle;
    private double tibiaangle;
    private double[] angleub = new double[3];
    private double[] anglelb = new double[3]; // angle bounds
    private double[][]  legpos = new double[6][3]; // root of leg in xyz
    private double[][] legpos1 = new double[6][3]; // default resting position of leg
    private double[] legang = new double[6]; // root angle of leg
     // for walking
    private double time, speed;
    private double smoothspeed, fdf;
    private double turning, smoothturning;
    private double standheight;
    private double sweepmodifier, speedmodifier, maxsweep;
     // for safestand
    private double sstime;
    private boolean ssrunning;
    private double[] ssx0 = new double[6]; // initial positions
    private double[] ssy0 = new double[6]; // initial positions
    private double[] ssz0 = new double[6]; // initial positions
     // dead-reckoning
    private double dr_xpos, dr_ypos, dr_ang;
    private boolean debug;
    // for walking
    private Bezier2D b2d_walk_up, b2d_walk_down;

    public Hexapod(boolean debug) {
        this.debug = debug;
        initialSetup();
    }

    public Hexapod(){
        this.debug = false;
        initialSetup();
    }


    private synchronized void initialSetup(){
        int ii;


        // init constants for each leg
        length[0] = 5.26; // in cm
        length[1] = 6.53;
        length[2] = 13.2;
        femurangle = 12.3*Constants.DEGTORAD; // old: 11.5, 9.53
        tibiaangle = 40.0*Constants.DEGTORAD; // old_47.3, 45.0

        anglelb[0] = 75.0*Constants.DEGTORAD;
        angleub[0] = 245.*Constants.DEGTORAD;
        anglelb[1] = 55.0*Constants.DEGTORAD;
        angleub[1] = 250.*Constants.DEGTORAD;
        anglelb[2] = 40.0*Constants.DEGTORAD;
        angleub[2] = 215.*Constants.DEGTORAD;
        for (ii=0; ii<3; ii++)
        {
            anglelb[ii] -= 150.*Constants.DEGTORAD; // because servos are straight at 150 degrees
            angleub[ii] -= 150.*Constants.DEGTORAD;
            //anglelb[ii] *= 0.75;
            //angleub[ii] *= 0.75;
        }

        legpos[0][0] = 12.50;
        legpos[0][1] = 5.95;
        legpos[0][2] = 1.82;
        legpos[1][0] = 0.0;
        legpos[1][1] = 9.957;
        legpos[1][2] = legpos[0][2];
        // symmetry
        legpos[2][0] = -legpos[0][0];
        legpos[2][1] = legpos[0][1];
        legpos[2][2] = legpos[0][2];
        legpos[3][0] = legpos[0][0];
        legpos[3][1] = -legpos[0][1];
        legpos[3][2] = legpos[0][2];
        legpos[4][0] = legpos[1][0];
        legpos[4][1] = -legpos[1][1];
        legpos[4][2] = legpos[1][2];
        legpos[5][0] = legpos[2][0];
        legpos[5][1] = legpos[3][1];
        legpos[5][2] = legpos[0][2];

        // which way do the coxa servos point
        legang[0] = 45.0*Constants.DEGTORAD;
        legang[1] = 90.0*Constants.DEGTORAD;
        legang[2] = 135.*Constants.DEGTORAD;
        legang[3] =-45.*Constants.DEGTORAD;
        legang[4] =-90.0*Constants.DEGTORAD;
        legang[5] =-135.0*Constants.DEGTORAD;

        // default target for each leg
        for (ii=0; ii<6; ii++)
        {
            legpos1[ii][0] = legpos[ii][0] + 12.0 * Math.cos(legang[ii]);
            legpos1[ii][1] = legpos[ii][1] + 12.0 * Math.sin(legang[ii]);
            legpos1[ii][2] = -10.0;
        }

        // initialize bezier curve gait
        // goes from +-1 in x and 0 to 1 in z
        b2d_walk_up.addPoint(-0.83775,0);
        b2d_walk_up.addPoint(-1.11701,0);
        b2d_walk_up.addPoint(-1.39626,0);
        b2d_walk_up.addPoint(0,3.2);
        b2d_walk_up.addPoint(1.39626,0);
        b2d_walk_up.addPoint(1.11701,0);
        b2d_walk_up.addPoint(0.83775,0);

        b2d_walk_down.addPoint(0.83775,0);
        b2d_walk_down.addPoint(-0.83775,0);

//        hexlock.lock();
        speed = 0.0;
        turning = 0.0;
        standheight = 2.0;
//        hexlock.unlock();
        smoothspeed = 0.0;
        time = 0.0;
        fdf = 0.50;

        dr_xpos = 0.0;
        dr_ypos = 0.0;
        dr_ang = 0.0;
    }

    public synchronized void step (double dt)
    {
        int ii, modt;
        double absspeed, speedsgn, ssfrac, ssfrac2;
        double cycletime, xpos, ypos;
        Position[] target = new Position[3];
        Position zpos = new Position();
        double legraise;
        double turn_dist = 0, rpos, maxdist;
        Position thtpos = new Position();
        double dist, tht0, turn_step, speed_step;

        // clamp speed and turning, just in case
//        hexlock.lock();
        if (speed > Constants.MAX_SPEED) speed = Constants.MAX_SPEED;
        if (speed < -Constants.MAX_SPEED) speed = -Constants.MAX_SPEED;
        if (turning > 1.0) turning = 1.0;
        if (turning < -1.0) turning = -1.0;
        if (standheight < -2.0) standheight = -2.0;
        if (standheight > 2.0) standheight = 2.0;

        // make sure speed doesnt change too rapidly
        speed_step = -(smoothspeed - speed);
        if (speed_step > Constants.SPEED_SLEW*dt) speed_step = Constants.SPEED_SLEW*dt;
        if (speed_step < -Constants.SPEED_SLEW*dt) speed_step = -Constants.SPEED_SLEW*dt;
        smoothspeed += speed_step;
        // cap the rate at which turning can change

        turn_step = -(smoothturning - turning);
        if (turn_step > Constants.TURN_SLEW*dt) turn_step = Constants.TURN_SLEW*dt;
        if (turn_step < -Constants.TURN_SLEW*dt) turn_step = -Constants.TURN_SLEW*dt;
        smoothturning += turn_step;


//        hexlock.unlock();

        // to control walking, modify speed and turning
        absspeed = Math.abs(smoothspeed);
        speedsgn = 1.0;
        if (smoothspeed < 0.0) speedsgn = -1.0;

        // walking speed is influenced by leg sweep and movement speed
        legraise = 1.0;
        if (absspeed < 0.05)
            legraise = absspeed/0.05;
        if (absspeed < 0.2)
        {
            sweepmodifier = absspeed*0.8/0.2;
            speedmodifier = 0.25;
        } else if (absspeed < 0.8) {
            sweepmodifier = 0.8;
            speedmodifier = absspeed/sweepmodifier;
        } else if (absspeed < 1.0) {
            sweepmodifier = absspeed;
            speedmodifier = 1.0;
        } else {
            sweepmodifier = 1.0;
            speedmodifier = absspeed;
        }
        speedmodifier *= speedsgn;

        if (ssrunning)
        {
            sstime += dt;
            ssfrac = sstime/Constants.SS_DURATION;
            for (ii=0; ii<6; ii++)
            {
                // compute final target
                target[0].setPos(legpos[ii][0]*1.5);
                if (ii == 0 || ii == 2) target[1].setPos(14.0);
                if (ii == 1) target[1].setPos(18.0);
                if (ii == 3 || ii == 5) target[1].setPos(-14.0);
                if (ii == 4) target[1].setPos(-18.0);
                target[0].setPos(legpos1[ii][0]);
                target[1].setPos(legpos1[ii][1]);
                target[2].setPos(-10. + standheight);
                // given final target, turn into current target
                if (ssfrac < 0.5)
                {
                    ssfrac2 = ssfrac*2.0;
                    if (ii % 2 == 0)
                    {
                        target[0].setPos(ssx0[ii] + ssfrac2*(target[0].getPos()-ssx0[ii]));
                        target[1].setPos(ssy0[ii] + ssfrac2*(target[1].getPos()-ssy0[ii]));
                        target[2].setPos(ssz0[ii] + ssfrac2*(target[2].getPos()-ssz0[ii]) +
                                2.0*Math.pow(Math.sin(3.1416*ssfrac2),2));
                    } else {
                        target[0].setPos(ssx0[ii]);
                        target[1].setPos(ssy0[ii]);
                        target[2].setPos(ssz0[ii]);
                    }
                } else {
                    ssfrac2 = (ssfrac-0.5)*2.0;
                    if (ii % 2 == 0)
                    {
                        // don't modify targets
                    } else {
                        target[0].setPos(ssx0[ii] + ssfrac2*(target[0].getPos()-ssx0[ii]));
                        target[1].setPos(ssy0[ii] + ssfrac2*(target[1].getPos()-ssy0[ii]));
                        target[2].setPos(ssz0[ii] + ssfrac2*(target[2].getPos()-ssz0[ii]) +
                                2.0*Math.pow(Math.sin(3.1416*ssfrac2),2));
                    }
                }
                IKSolve(ii,target);
            }
            if (sstime > Constants.SS_DURATION)
            {
                ssrunning = false;
                sstime = 0.0;
            }
        } else {


            // based on current turning, compute turning math
            if (Math.abs(smoothturning) <= Constants.TURN_TOL)
                turn_dist = Math.tan((1.0-Constants.TURN_TOL)*3.1416/2.0)*50.;
            else if (Math.abs(smoothturning) > Constants.TURN_TOL)
                turn_dist = Math.tan((1.0-smoothturning)*3.1416/2.0)*50.;
            // compute dist between turn_dist and farthest leg
            maxdist = 0.0;
            for (ii=0; ii<6; ii++)
            {
                dist = Math.sqrt(Math.pow(legpos1[ii][0],2) + Math.pow(legpos1[ii][1]-turn_dist,2));
                if (dist > maxdist) maxdist = dist;
            }
            // each leg can only sweep so much, so use this farthest leg
            // to determine the angle of sweep that every leg must do
            maxsweep = 8.*sweepmodifier/maxdist;
            if (turn_dist < 0.0) maxsweep = -maxsweep;
            // maxsweep is the angle of sweep for every leg, in radians

            // increment dead-reckoning position
            // turning radius is "turn_dist"
            dr_ang += dt*speedmodifier*maxsweep*2.0*0.83775/fdf;
            if (dr_ang > Constants.PI) dr_ang -= 2.*Constants.PI;
            if (dr_ang < -Constants.PI) dr_ang += 2.*Constants.PI;
            dr_xpos += maxsweep*turn_dist*dt*speedmodifier*2.0*Math.cos(dr_ang)*0.83775/fdf;
            dr_ypos += maxsweep*turn_dist*dt*speedmodifier*2.0*Math.sin(dr_ang)*0.83775/fdf;

            // dr_ang has about 20% error, likely systematic
            // dr_xpos,dr_ypos have more like 5% error, also likely systematic

            // increment fake time
            time += dt*speedmodifier;
            if (time > 1.0) time -= 1.0;
            if (time < 0.0) time += 1.0;

            // loop through each leg to figure out where it should be right now
            for (ii=0; ii<6; ii++)
            {
                // where is this leg in the cycle of stepping?
                // the 0.5*ii is to completely de-sync legs
                // the other part is to adjust it more
                cycletime = Math.IEEEremainder(time + 0.5*ii + (legpos[ii][0]-legpos[0][0])*0.0125, 1.0);

                // use bezier curve to either be up or down
                // b2d_walk_down goes between +/- 0.83775
                if (cycletime < fdf) b2d_walk_down.getPos(cycletime/fdf, thtpos, zpos);
                else b2d_walk_up.getPos((cycletime-fdf)/(1.-fdf), thtpos, zpos);
                // convert thtpos into angle?
                thtpos.setPos(thtpos.getPos()* maxsweep);

                // convert rpos to xpos,ypos
                dist = Math.sqrt(Math.pow(legpos1[ii][0],2) + Math.pow(legpos1[ii][1]-turn_dist,2));
                tht0 = Math.atan2(legpos1[ii][1]-turn_dist,legpos1[ii][0]);
                xpos = dist*Math.cos(thtpos.getPos()+tht0);
                ypos = turn_dist + dist*Math.sin(thtpos.getPos()+tht0);

                // set up the IK target
                target[0].setPos(xpos);
                target[1].setPos(ypos);
                target[2].setPos(-10.0 + zpos.getPos()*3.0*legraise + standheight);
                if (standheight < 0.0) target[2].setPos(target[2].getPos()-1.7*zpos.getPos()*standheight);


                // perform IK solve
                IKSolve(ii,target);
                // TODO: add error handling if IK fails to converge
            }

        }
        setServoAngles();
    }

    public void safeStand ()
    {
        int ii;
        Position[] fkangles = new Position[3];
        Position[] fkpos = new Position[3];

        // initialize
        sstime = 0.0;
        ssrunning = true;
        // store FK leg positions
        for (ii=0; ii<6; ii++)
        {
            fkangles[0].setPos(angle[ii*3+0]);
            fkangles[1].setPos(angle[ii*3+1]);
            fkangles[2].setPos(angle[ii*3+2]);
            FKSolve(ii,fkangles,fkpos);
            ssx0[ii] = fkpos[0].getPos();
            ssy0[ii] = fkpos[1].getPos();
            ssz0[ii] = fkpos[2].getPos();
        }
    }

    public void setAngles ()
    {
        int ii;
        for (ii=0; ii<6; ii++)
        {
            angle[ii*3] = (servoangle[ii*3]-150.)*Constants.DEGTORAD;
            angle[ii*3+1] = (servoangle[ii*3+1]-150.)*Constants.DEGTORAD;
            angle[ii*3+2] = (servoangle[ii*3+2]-150.)*Constants.DEGTORAD;
        }
    }

    private void setServoAngles()
    {
        int ii;
        for (ii=0; ii<6; ii++)
        {
            servoangle[ii*3] = angle[ii*3]*Constants.RADTODEG + 150.;
            servoangle[ii*3+1] = angle[ii*3+1]*Constants.RADTODEG + 150.;
            servoangle[ii*3+2] = angle[ii*3+2]*Constants.RADTODEG + 150.;
        }
    }

    // target is a double[3] giving absolute position
// return whether or not the solver was successful
    private boolean IKSolve(int leg, Position[] target)
    {
        int iter;
        boolean converged;
        double diff;
        double targetr, targetz, targetang;
        Position[] fkpos = new Position[3];
        Position[] fkangles = new Position[3];
        double[][] J = new double[2][2];
        double[][] inv = new double[2][2];
        double[] delta = new double[2];
        double[] p = new double[2];
        double posr, posz, ang1, ang2, det;

        // convert absolute position to polar around leg root
        targetz = target[2].getPos() - legpos[leg][2];
        targetr = Math.sqrt(Math.pow(target[0].getPos()-legpos[leg][0],2) + Math.pow(target[1].getPos()-legpos[leg][1],2));
        targetang = Math.atan2(target[1].getPos()-legpos[leg][1],target[0].getPos()-legpos[leg][0]) - legang[leg]; // atan2 [-pi:pi]

        // easy part: can the coxa servo get to the right angle?
        if (targetang > angleub[0] || targetang < anglelb[0]) return false;
        // else, go ahead and set coxa servo. One out of three angles done!
        angle[leg*3] = targetang;

        // begin 2-joint IK solver using jacobian pseudo-inverse
        // whenever we call FKSolve, need to convert to polar coords around leg root
        fkangles[0].setPos(angle[leg*3]); // already solved for
        // starting point is influenced by actual current point
        // but this makes it safer in case the leg has somehow gone out of bounds
        fkangles[1].setPos(angle[leg*3+1]*0.5);
        fkangles[2].setPos(angle[leg*3+2]*0.5);
        FKSolve(leg, fkangles, fkpos);
        posz = fkpos[2].getPos() - legpos[leg][2];
        posr = Math.sqrt(Math.pow(fkpos[0].getPos()-legpos[leg][0],2) + Math.pow(fkpos[1].getPos()-legpos[leg][1],2));

        diff = Math.sqrt(Math.pow(targetr-posr,2) + Math.pow(targetz-posz,2));
        // ITERATE
        converged = false;
        for (iter=0; iter<Constants.MAXITER && !converged; iter++)
        {
            // compute jacobian
            p[0] = targetr - posr;
            p[1] = targetz - posz;
            ang1 = fkangles[1].getPos()-femurangle;
            ang2 = fkangles[2].getPos()-tibiaangle;
            J[0][0] = -length[1]*Math.sin(ang1) - length[2]*Math.sin(ang1+ang2); // dr/dang1
            J[1][0] = -length[2]*Math.sin(ang1+ang2); // dr/dang2
            J[0][1] = length[1]*Math.cos(ang1) + length[2]*Math.cos(ang1+ang2); // dz/dang2
            J[1][1] = length[2]*Math.cos(ang1+ang2); // dz/dang2
            // compute inverse
            det = 1.0/(J[0][0]*J[1][1]-J[0][1]*J[1][0]);
            inv[0][0] = J[1][1]*det;
            inv[1][0] = -J[1][0]*det;
            inv[0][1] = -J[0][1]*det;
            inv[1][1] = J[0][0]*det;
            delta[0] = p[0]*inv[0][0] + p[1]*inv[0][1];
            delta[1] = p[0]*inv[1][0] + p[1]*inv[1][1];
            fkangles[1].setPos(fkangles[1].getPos()+delta[0]*0.5);
            fkangles[2].setPos(fkangles[2].getPos()+delta[1]*0.5);
            // enforce bounds
            if (fkangles[1].getPos() >= angleub[1]) {
                fkangles[1].setPos(angleub[1] - Constants.ANGEPS);
                if (debug) System.out.println("ang1ub"+leg+" "+angleub[1]);
            }
            if (fkangles[1].getPos() <= anglelb[1]) {
                fkangles[1].setPos(anglelb[1] + Constants.ANGEPS);
                if (debug) System.out.println("ang1lb"+leg+" "+anglelb[1]);
            }
            if (fkangles[2].getPos() >= angleub[2]) {
                fkangles[2].setPos(angleub[2] - Constants.ANGEPS);
                if (debug) System.out.println("ang2ub"+leg+" "+angleub[2]);
            }
            if (fkangles[2].getPos() <= anglelb[2]) {
                fkangles[2].setPos(anglelb[2] + Constants.ANGEPS);
                if (debug) System.out.println("ang2lb"+leg+" "+anglelb[2]);
            }
            // FK
            FKSolve(leg, fkangles, fkpos);
            posz = fkpos[2].getPos() - legpos[leg][2];
            posr = Math.sqrt(Math.pow(fkpos[0].getPos()-legpos[leg][0],2) + Math.pow(fkpos[1].getPos()-legpos[leg][1],2));
            // convergence criteria
            diff = Math.sqrt(Math.pow(targetr-posr,2) + Math.pow(targetz-posz,2));
            //cout << iter << " " << diff << " " << posr << " " << posz << endl;
            if (diff < Constants.TOLERANCE) converged = true; // 1 mm tolerance
        }

        // converged?
        if (converged)
        {
            angle[leg*3+1] = fkangles[1].getPos();
            angle[leg*3+2] = fkangles[2].getPos();
        }

        //if (converged) cout << iter << endl;

        return converged;
    }

    // forward kinematics in absolute coordinate system
// given a flat[3] angles, compute position and put in double[3] pos
// input angles are in radians and are offset properly
    private void FKSolve(int leg, Position[] angles, Position[] pos)
    {
        double r, ang0, ang1, ang2;

        ang0 = angles[0].getPos()+legang[leg];
        ang1 = angles[1].getPos()-femurangle;
        ang2 = angles[2].getPos()-tibiaangle;

        r = length[0] + length[1]*Math.cos(ang1) + length[2]*Math.cos(ang1+ang2);
        pos[0].setPos(legpos[leg][0] + r*Math.cos(ang0));
        pos[1].setPos(legpos[leg][1] + r*Math.sin(ang0));
        pos[2].setPos(legpos[leg][2] + length[1]*Math.sin(ang1) + length[2]*Math.sin(ang1+ang2));
    }

    public void stand ()
    {
        int ii;
        Position[] target = new Position[3];
        for (ii=0; ii<6; ii++)
        {
            target[0].setPos(legpos[ii][0] + 10.0*Math.cos(legang[ii]));
            target[1].setPos(legpos[ii][1] + 10.0*Math.sin(legang[ii]));
            target[2].setPos(-10.0);
            IKSolve(ii,target);
        }
        setServoAngles();
    }

    public void sit ()
    {
        int ii;
        Position[] target = new Position[3];
        for (ii=0; ii<6; ii++)
        {
            target[0].setPos(legpos[ii][0] + 10.0*Math.cos(legang[ii]));
            target[1].setPos(legpos[ii][1] + 10.0*Math.sin(legang[ii]));
            target[2].setPos(-5.0);
            IKSolve(ii,target);
        }
        setServoAngles();
    }
}
