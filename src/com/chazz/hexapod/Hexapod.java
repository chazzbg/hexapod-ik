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
    private double[] servoAngle = new double[18]; // in degrees, 0 to 300 or so (subject to more constraint)
    private double[] angle = new double[18]; // in radians, direction and offset corrected
     // Hexapod body
    private double[] length = new double[3]; // length of coxa, femur, tibia
    private double femurAngle;
    private double tibiaAngle;
    private double[] angleub = new double[3];
    private double[] anglelb = new double[3]; // angle bounds
    private double[][] legPos = new double[6][3]; // root of leg in xyz
    private double[][] legPos1 = new double[6][3]; // default resting position of leg
    private double[] legAng = new double[6]; // root angle of leg
     // for walking
    private double time, speed;
    private double smoothSpeed, fdf;
    private double turning, smoothTurning;
    private double standHeight;
    private double sweepModifier, speedModifier, maxSweep;
     // for safestand
    private double ssTime;
    private boolean ssRunning;
    private double[] ssx0 = new double[6]; // initial positions
    private double[] ssy0 = new double[6]; // initial positions
    private double[] ssz0 = new double[6]; // initial positions
     // dead-reckoning
    private double drXpos;
    private double drYpos;
    private double drAng;
    private boolean debug;
    // for walking
    private Bezier2D bezierWalkUp = new Bezier2D(), bezierWalkDOwn = new Bezier2D();

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
        femurAngle = 12.3*Constants.DEGTORAD; // old: 11.5, 9.53
        tibiaAngle = 40.0*Constants.DEGTORAD; // old_47.3, 45.0

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

        legPos[0][0] = 12.50;
        legPos[0][1] = 5.95;
        legPos[0][2] = 1.82;
        legPos[1][0] = 0.0;
        legPos[1][1] = 9.957;
        legPos[1][2] = legPos[0][2];
        // symmetry
        legPos[2][0] = -legPos[0][0];
        legPos[2][1] = legPos[0][1];
        legPos[2][2] = legPos[0][2];
        legPos[3][0] = legPos[0][0];
        legPos[3][1] = -legPos[0][1];
        legPos[3][2] = legPos[0][2];
        legPos[4][0] = legPos[1][0];
        legPos[4][1] = -legPos[1][1];
        legPos[4][2] = legPos[1][2];
        legPos[5][0] = legPos[2][0];
        legPos[5][1] = legPos[3][1];
        legPos[5][2] = legPos[0][2];

        // which way do the coxa servos point
        legAng[0] = 45.0*Constants.DEGTORAD;
        legAng[1] = 90.0*Constants.DEGTORAD;
        legAng[2] = 135.*Constants.DEGTORAD;
        legAng[3] =-45.*Constants.DEGTORAD;
        legAng[4] =-90.0*Constants.DEGTORAD;
        legAng[5] =-135.0*Constants.DEGTORAD;

        // default target for each leg
        for (ii=0; ii<6; ii++)
        {
            legPos1[ii][0] = legPos[ii][0] + 12.0 * Math.cos(legAng[ii]);
            legPos1[ii][1] = legPos[ii][1] + 12.0 * Math.sin(legAng[ii]);
            legPos1[ii][2] = -10.0;
        }

        // initialize bezier curve gait
        // goes from +-1 in x and 0 to 1 in z
        bezierWalkUp.addPoint(-0.83775,0);
        bezierWalkUp.addPoint(-1.11701,0);
        bezierWalkUp.addPoint(-1.39626,0);
        bezierWalkUp.addPoint(0,3.2);
        bezierWalkUp.addPoint(1.39626,0);
        bezierWalkUp.addPoint(1.11701,0);
        bezierWalkUp.addPoint(0.83775,0);

        bezierWalkDOwn.addPoint(0.83775,0);
        bezierWalkDOwn.addPoint(-0.83775,0);

//        hexlock.lock();
        speed = 0.0;
        turning = 0.0;
        standHeight = 2.0;
//        hexlock.unlock();
        smoothSpeed = 0.0;
        time = 0.0;
        fdf = 0.50;

        drXpos = 0.0;
        drYpos = 0.0;
        drAng = 0.0;
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
        if (standHeight < -2.0) standHeight = -2.0;
        if (standHeight > 2.0) standHeight = 2.0;

        // make sure speed doesnt change too rapidly
        speed_step = -(smoothSpeed - speed);
        if (speed_step > Constants.SPEED_SLEW*dt) speed_step = Constants.SPEED_SLEW*dt;
        if (speed_step < -Constants.SPEED_SLEW*dt) speed_step = -Constants.SPEED_SLEW*dt;
        smoothSpeed += speed_step;
        // cap the rate at which turning can change

        turn_step = -(smoothTurning - turning);
        if (turn_step > Constants.TURN_SLEW*dt) turn_step = Constants.TURN_SLEW*dt;
        if (turn_step < -Constants.TURN_SLEW*dt) turn_step = -Constants.TURN_SLEW*dt;
        smoothTurning += turn_step;


//        hexlock.unlock();

        // to control walking, modify speed and turning
        absspeed = Math.abs(smoothSpeed);
        speedsgn = 1.0;
        if (smoothSpeed < 0.0) speedsgn = -1.0;

        // walking speed is influenced by leg sweep and movement speed
        legraise = 1.0;
        if (absspeed < 0.05)
            legraise = absspeed/0.05;
        if (absspeed < 0.2)
        {
            sweepModifier = absspeed*0.8/0.2;
            speedModifier = 0.25;
        } else if (absspeed < 0.8) {
            sweepModifier = 0.8;
            speedModifier = absspeed/ sweepModifier;
        } else if (absspeed < 1.0) {
            sweepModifier = absspeed;
            speedModifier = 1.0;
        } else {
            sweepModifier = 1.0;
            speedModifier = absspeed;
        }
        speedModifier *= speedsgn;

        if (ssRunning)
        {
            ssTime += dt;
            ssfrac = ssTime /Constants.SS_DURATION;
            for (ii=0; ii<6; ii++)
            {
                // compute final target
                target[0].setPos(legPos[ii][0]*1.5);
                if (ii == 0 || ii == 2) target[1].setPos(14.0);
                if (ii == 1) target[1].setPos(18.0);
                if (ii == 3 || ii == 5) target[1].setPos(-14.0);
                if (ii == 4) target[1].setPos(-18.0);
                target[0].setPos(legPos1[ii][0]);
                target[1].setPos(legPos1[ii][1]);
                target[2].setPos(-10. + standHeight);
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
            if (ssTime > Constants.SS_DURATION)
            {
                ssRunning = false;
                ssTime = 0.0;
            }
        } else {


            // based on current turning, compute turning math
            if (Math.abs(smoothTurning) <= Constants.TURN_TOL)
                turn_dist = Math.tan((1.0-Constants.TURN_TOL)*3.1416/2.0)*50.;
            else if (Math.abs(smoothTurning) > Constants.TURN_TOL)
                turn_dist = Math.tan((1.0- smoothTurning)*3.1416/2.0)*50.;
            // compute dist between turn_dist and farthest leg
            maxdist = 0.0;
            for (ii=0; ii<6; ii++)
            {
                dist = Math.sqrt(Math.pow(legPos1[ii][0],2) + Math.pow(legPos1[ii][1]-turn_dist,2));
                if (dist > maxdist) maxdist = dist;
            }
            // each leg can only sweep so much, so use this farthest leg
            // to determine the angle of sweep that every leg must do
            maxSweep = 8.* sweepModifier /maxdist;
            if (turn_dist < 0.0) maxSweep = -maxSweep;
            // maxSweep is the angle of sweep for every leg, in radians

            // increment dead-reckoning position
            // turning radius is "turn_dist"
            drAng += dt* speedModifier * maxSweep *2.0*0.83775/fdf;
            if (drAng > Constants.PI) drAng -= 2.*Constants.PI;
            if (drAng < -Constants.PI) drAng += 2.*Constants.PI;
            drXpos += maxSweep *turn_dist*dt* speedModifier *2.0*Math.cos(drAng)*0.83775/fdf;
            drYpos += maxSweep *turn_dist*dt* speedModifier *2.0*Math.sin(drAng)*0.83775/fdf;

            // drAng has about 20% error, likely systematic
            // drXpos,drYpos have more like 5% error, also likely systematic

            // increment fake time
            time += dt* speedModifier;
            if (time > 1.0) time -= 1.0;
            if (time < 0.0) time += 1.0;

            // loop through each leg to figure out where it should be right now
            for (ii=0; ii<6; ii++)
            {
                // where is this leg in the cycle of stepping?
                // the 0.5*ii is to completely de-sync legs
                // the other part is to adjust it more
                cycletime = Math.IEEEremainder(time + 0.5*ii + (legPos[ii][0]- legPos[0][0])*0.0125, 1.0);

                // use bezier curve to either be up or down
                // bezierWalkDOwn goes between +/- 0.83775
                if (cycletime < fdf) bezierWalkDOwn.getPos(cycletime/fdf, thtpos, zpos);
                else bezierWalkUp.getPos((cycletime-fdf)/(1.-fdf), thtpos, zpos);
                // convert thtpos into angle?
                thtpos.setPos(thtpos.getPos()* maxSweep);

                // convert rpos to xpos,ypos
                dist = Math.sqrt(Math.pow(legPos1[ii][0],2) + Math.pow(legPos1[ii][1]-turn_dist,2));
                tht0 = Math.atan2(legPos1[ii][1]-turn_dist, legPos1[ii][0]);
                xpos = dist*Math.cos(thtpos.getPos()+tht0);
                ypos = turn_dist + dist*Math.sin(thtpos.getPos()+tht0);

                // set up the IK target
                target[0].setPos(xpos);
                target[1].setPos(ypos);
                target[2].setPos(-10.0 + zpos.getPos()*3.0*legraise + standHeight);
                if (standHeight < 0.0) target[2].setPos(target[2].getPos()-1.7*zpos.getPos()* standHeight);


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
        ssTime = 0.0;
        ssRunning = true;
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
            angle[ii*3] = (servoAngle[ii*3]-150.)*Constants.DEGTORAD;
            angle[ii*3+1] = (servoAngle[ii*3+1]-150.)*Constants.DEGTORAD;
            angle[ii*3+2] = (servoAngle[ii*3+2]-150.)*Constants.DEGTORAD;
        }
    }

    private void setServoAngles()
    {
        int ii;
        for (ii=0; ii<6; ii++)
        {
            servoAngle[ii*3] = angle[ii*3]*Constants.RADTODEG + 150.;
            servoAngle[ii*3+1] = angle[ii*3+1]*Constants.RADTODEG + 150.;
            servoAngle[ii*3+2] = angle[ii*3+2]*Constants.RADTODEG + 150.;
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
        targetz = target[2].getPos() - legPos[leg][2];
        targetr = Math.sqrt(Math.pow(target[0].getPos()- legPos[leg][0],2) + Math.pow(target[1].getPos()- legPos[leg][1],2));
        targetang = Math.atan2(target[1].getPos()- legPos[leg][1],target[0].getPos()- legPos[leg][0]) - legAng[leg]; // atan2 [-pi:pi]

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
        posz = fkpos[2].getPos() - legPos[leg][2];
        posr = Math.sqrt(Math.pow(fkpos[0].getPos()- legPos[leg][0],2) + Math.pow(fkpos[1].getPos()- legPos[leg][1],2));

        diff = Math.sqrt(Math.pow(targetr-posr,2) + Math.pow(targetz-posz,2));
        // ITERATE
        converged = false;
        for (iter=0; iter<Constants.MAXITER && !converged; iter++)
        {
            // compute jacobian
            p[0] = targetr - posr;
            p[1] = targetz - posz;
            ang1 = fkangles[1].getPos()- femurAngle;
            ang2 = fkangles[2].getPos()- tibiaAngle;
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
            posz = fkpos[2].getPos() - legPos[leg][2];
            posr = Math.sqrt(Math.pow(fkpos[0].getPos()- legPos[leg][0],2) + Math.pow(fkpos[1].getPos()- legPos[leg][1],2));
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

        ang0 = angles[0].getPos()+ legAng[leg];
        ang1 = angles[1].getPos()- femurAngle;
        ang2 = angles[2].getPos()- tibiaAngle;

        r = length[0] + length[1]*Math.cos(ang1) + length[2]*Math.cos(ang1+ang2);
        pos[0].setPos(legPos[leg][0] + r*Math.cos(ang0));
        pos[1].setPos(legPos[leg][1] + r*Math.sin(ang0));
        pos[2].setPos(legPos[leg][2] + length[1]*Math.sin(ang1) + length[2]*Math.sin(ang1+ang2));
    }

    public void stand ()
    {
        int ii;
        Position[] target = new Position[3];
        for (ii=0; ii<6; ii++)
        {
            target[0].setPos(legPos[ii][0] + 10.0*Math.cos(legAng[ii]));
            target[1].setPos(legPos[ii][1] + 10.0*Math.sin(legAng[ii]));
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
            target[0].setPos(legPos[ii][0] + 10.0*Math.cos(legAng[ii]));
            target[1].setPos(legPos[ii][1] + 10.0*Math.sin(legAng[ii]));
            target[2].setPos(-5.0);
            IKSolve(ii,target);
        }
        setServoAngles();
    }

    public double getDrXpos() {
        return drXpos;
    }

    public void setDrXpos(double drXpos) {
        this.drXpos = drXpos;
    }

    public double[] getServoAngle() {
        return servoAngle;
    }

    public void setServoAngle(double[] servoAngle) {
        this.servoAngle = servoAngle;
    }

    public double[] getAngle() {
        return angle;
    }

    public void setAngle(double[] angle) {
        this.angle = angle;
    }

    public double[] getLength() {
        return length;
    }

    public void setLength(double[] length) {
        this.length = length;
    }

    public double getFemurAngle() {
        return femurAngle;
    }

    public void setFemurAngle(double femurAngle) {
        this.femurAngle = femurAngle;
    }

    public double getTibiaAngle() {
        return tibiaAngle;
    }

    public void setTibiaAngle(double tibiaAngle) {
        this.tibiaAngle = tibiaAngle;
    }

    public double[] getAngleub() {
        return angleub;
    }

    public void setAngleub(double[] angleub) {
        this.angleub = angleub;
    }

    public double[] getAnglelb() {
        return anglelb;
    }

    public void setAnglelb(double[] anglelb) {
        this.anglelb = anglelb;
    }

    public double[][] getLegPos() {
        return legPos;
    }

    public void setLegPos(double[][] legPos) {
        this.legPos = legPos;
    }

    public double[][] getLegPos1() {
        return legPos1;
    }

    public void setLegPos1(double[][] legPos1) {
        this.legPos1 = legPos1;
    }

    public double[] getLegAng() {
        return legAng;
    }

    public void setLegAng(double[] legAng) {
        this.legAng = legAng;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSmoothSpeed() {
        return smoothSpeed;
    }

    public void setSmoothSpeed(double smoothSpeed) {
        this.smoothSpeed = smoothSpeed;
    }

    public double getFdf() {
        return fdf;
    }

    public void setFdf(double fdf) {
        this.fdf = fdf;
    }

    public double getTurning() {
        return turning;
    }

    public void setTurning(double turning) {
        this.turning = turning;
    }

    public double getSmoothTurning() {
        return smoothTurning;
    }

    public void setSmoothTurning(double smoothTurning) {
        this.smoothTurning = smoothTurning;
    }

    public double getStandHeight() {
        return standHeight;
    }

    public void setStandHeight(double standHeight) {
        this.standHeight = standHeight;
    }

    public double getSweepModifier() {
        return sweepModifier;
    }

    public void setSweepModifier(double sweepModifier) {
        this.sweepModifier = sweepModifier;
    }

    public double getSpeedModifier() {
        return speedModifier;
    }

    public void setSpeedModifier(double speedModifier) {
        this.speedModifier = speedModifier;
    }

    public double getMaxSweep() {
        return maxSweep;
    }

    public void setMaxSweep(double maxSweep) {
        this.maxSweep = maxSweep;
    }

    public double getSsTime() {
        return ssTime;
    }

    public void setSsTime(double ssTime) {
        this.ssTime = ssTime;
    }

    public boolean isSsRunning() {
        return ssRunning;
    }

    public void setSsRunning(boolean ssRunning) {
        this.ssRunning = ssRunning;
    }

    public double[] getSsx0() {
        return ssx0;
    }

    public void setSsx0(double[] ssx0) {
        this.ssx0 = ssx0;
    }

    public double[] getSsy0() {
        return ssy0;
    }

    public void setSsy0(double[] ssy0) {
        this.ssy0 = ssy0;
    }

    public double[] getSsz0() {
        return ssz0;
    }

    public void setSsz0(double[] ssz0) {
        this.ssz0 = ssz0;
    }

    public double getDrYpos() {
        return drYpos;
    }

    public void setDrYpos(double drYpos) {
        this.drYpos = drYpos;
    }

    public double getDrAng() {
        return drAng;
    }

    public void setDrAng(double drAng) {
        this.drAng = drAng;
    }

    public boolean isDebug() {
        return debug;
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public Bezier2D getBezierWalkUp() {
        return bezierWalkUp;
    }

    public void setBezierWalkUp(Bezier2D bezierWalkUp) {
        this.bezierWalkUp = bezierWalkUp;
    }

    public Bezier2D getBezierWalkDOwn() {
        return bezierWalkDOwn;
    }

    public void setBezierWalkDOwn(Bezier2D bezierWalkDOwn) {
        this.bezierWalkDOwn = bezierWalkDOwn;
    }
}
