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


import java.util.Arrays;

public class Hexapod {


    // both sets of angles are relative to leg root and leg angle
    private float[] servoAngle = new float[18]; // in degrees, 0 to 300 or so (subject to more constraint)
    private float[] angle = new float[18]; // in radians, direction and offset corrected
     // Hexapod body
    private float[] length = new float[3]; // length of coxa, femur, tibia
    private float femurAngle;
    private float tibiaAngle;
    private float[] angleub = new float[3];
    private float[] anglelb = new float[3]; // angle bounds
    private float[][] legPos = new float[6][3]; // root of leg in xyz
    private float[][] legPos1 = new float[6][3]; // default resting position of leg
    private float[] legAng = new float[6]; // root angle of leg
     // for walking
    private float time, speed;
    private float smoothSpeed, fdf;
    private float turning, smoothTurning;
    private float standHeight;
    private float sweepModifier, speedModifier, maxSweep;
     // for safestand
    private float ssTime;
    private boolean ssRunning;
    private float[] ssx0 = new float[6]; // initial positions
    private float[] ssy0 = new float[6]; // initial positions
    private float[] ssz0 = new float[6]; // initial positions
     // dead-reckoning
    private float drXpos;
    private float drYpos;
    private float drAng;
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
        length[0] = 5.26f; // in cm
        length[1] = 6.53f;
        length[2] = 13.2f;
        femurAngle = 12.3f*Constants.DEGTORAD; // old: 11.5, 9.53
        tibiaAngle = 40.0f*Constants.DEGTORAD; // old_47.3, 45.0

        anglelb[0] = 75.0f*Constants.DEGTORAD;
        angleub[0] = 245.f*Constants.DEGTORAD;
        anglelb[1] = 55.0f*Constants.DEGTORAD;
        angleub[1] = 250.f*Constants.DEGTORAD;
        anglelb[2] = 40.0f*Constants.DEGTORAD;
        angleub[2] = 215.f*Constants.DEGTORAD;
        for (ii=0; ii<3; ii++)
        {
            anglelb[ii] -= 150.*Constants.DEGTORAD; // because servos are straight at 150 degrees
            angleub[ii] -= 150.*Constants.DEGTORAD;
            //anglelb[ii] *= 0.75;
            //angleub[ii] *= 0.75;
        }

        legPos[0][0] = 12.50f;
        legPos[0][1] = 5.95f;
        legPos[0][2] = 1.82f;
        legPos[1][0] = 0.0f;
        legPos[1][1] = 9.957f;
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
        legAng[0] = 45.0f*Constants.DEGTORAD;
        legAng[1] = 90.0f*Constants.DEGTORAD;
        legAng[2] = 135.f*Constants.DEGTORAD;
        legAng[3] =-45.f*Constants.DEGTORAD;
        legAng[4] =-90.0f*Constants.DEGTORAD;
        legAng[5] =-135.0f*Constants.DEGTORAD;

        // default target for each leg
        for (ii=0; ii<6; ii++)
        {
            legPos1[ii][0] = (float) (legPos[ii][0] + 12.0f * Math.cos(legAng[ii]));
            legPos1[ii][1] = (float) (legPos[ii][1] + 12.0f * Math.sin(legAng[ii]));
            legPos1[ii][2] = -10.0f;
        }

        // initialize bezier curve gait
        // goes from +-1 in x and 0 to 1 in z
        bezierWalkUp.addPoint(-0.83775f,0);
        bezierWalkUp.addPoint(-1.11701f,0);
        bezierWalkUp.addPoint(-1.39626f,0);
        bezierWalkUp.addPoint(0,3.2f);
        bezierWalkUp.addPoint(1.39626f,0);
        bezierWalkUp.addPoint(1.11701f,0);
        bezierWalkUp.addPoint(0.83775f,0);

        bezierWalkDOwn.addPoint(0.83775f,0);
        bezierWalkDOwn.addPoint(-0.83775f,0);

//        hexlock.lock();
        speed = 0.0f;
        turning = 0.0f;
        standHeight = 2.0f;
//        hexlock.unlock()f;
        smoothSpeed = 0.0f;
        time = 0.0f;
        fdf = 0.50f;

        drXpos = 0.0f;
        drYpos = 0.0f;
        drAng = 0.0f;
    }

    public synchronized void step (float dt)
    {
        int ii, modt;
        float absspeed, speedsgn, ssfrac, ssfrac2;
        float cycletime, xpos, ypos;
        Position[] target = new Position[3];

        for (int i =0; i < target.length; i++){
            target[i] = new Position();
        }

        Position zpos = new Position();
        float legraise;
        float turn_dist = 0, rpos, maxdist;
        Position thtpos = new Position();
        float dist, tht0, turn_step, speed_step;

        // clamp speed and turning, just in case
//        hexlock.lock();
        if (speed > Constants.MAX_SPEED) speed = Constants.MAX_SPEED;
        if (speed < -Constants.MAX_SPEED) speed = -Constants.MAX_SPEED;
        if (turning > 1.0) turning = 1.0f;
        if (turning < -1.0) turning = -1.0f;
        if (standHeight < -2.0) standHeight = -2.0f;
        if (standHeight > 2.0) standHeight = 2.0f;

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
        speedsgn = 1.0f;
        if (smoothSpeed < 0.0) speedsgn = -1.0f;

        // walking speed is influenced by leg sweep and movement speed
        legraise = 1.0f;
        if (absspeed < 0.05)
            legraise = absspeed/0.05f;
        if (absspeed < 0.2)
        {
            sweepModifier = absspeed*0.8f/0.2f;
            speedModifier = 0.25f;
        } else if (absspeed < 0.8) {
            sweepModifier = 0.8f;
            speedModifier = absspeed/ sweepModifier;
        } else if (absspeed < 1.0) {
            sweepModifier = absspeed;
            speedModifier = 1.0f;
        } else {
            sweepModifier = 1.0f;
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
                target[0].setPos(legPos[ii][0]*1.5f);
                if (ii == 0 || ii == 2) target[1].setPos(14.0f);
                if (ii == 1) target[1].setPos(18.0f);
                if (ii == 3 || ii == 5) target[1].setPos(-14.0f);
                if (ii == 4) target[1].setPos(-18.0f);
                target[0].setPos(legPos1[ii][0]);
                target[1].setPos(legPos1[ii][1]);
                target[2].setPos(-10.f + standHeight);
                // given final target, turn into current target
                if (ssfrac < 0.5)
                {
                    ssfrac2 = ssfrac*2.0f;
                    if (ii % 2 == 0)
                    {
                        target[0].setPos(ssx0[ii] + ssfrac2*(target[0].getPos()-ssx0[ii]));
                        target[1].setPos(ssy0[ii] + ssfrac2*(target[1].getPos()-ssy0[ii]));
                        target[2].setPos((float) (ssz0[ii] + ssfrac2*(target[2].getPos()-ssz0[ii]) +
                                                        2.0*Math.pow(Math.sin(3.1416*ssfrac2),2)));
                    } else {
                        target[0].setPos(ssx0[ii]);
                        target[1].setPos(ssy0[ii]);
                        target[2].setPos(ssz0[ii]);
                    }
                } else {
                    ssfrac2 = (ssfrac-0.5f)*2.0f;
                    if (ii % 2 == 0)
                    {
                        // don't modify targets
                    } else {
                        target[0].setPos(ssx0[ii] + ssfrac2*(target[0].getPos()-ssx0[ii]));
                        target[1].setPos(ssy0[ii] + ssfrac2*(target[1].getPos()-ssy0[ii]));
                        target[2].setPos((float) (ssz0[ii] + ssfrac2*(target[2].getPos()-ssz0[ii]) +
                                                        2.0*Math.pow(Math.sin(3.1416*ssfrac2),2)));
                    }
                }
                IKSolve(ii,target);
            }
            if (ssTime > Constants.SS_DURATION)
            {
                ssRunning = false;
                ssTime = 0.0f;
            }
        } else {


            // based on current turning, compute turning math
            if (Math.abs(smoothTurning) <= Constants.TURN_TOL)
                turn_dist = (float) (Math.tan((1.0-Constants.TURN_TOL)*3.1416/2.0)*50.);
            else if (Math.abs(smoothTurning) > Constants.TURN_TOL)
                turn_dist = (float) (Math.tan((1.0- smoothTurning)*3.1416/2.0)*50.);
            // compute dist between turn_dist and farthest leg
            maxdist = 0.0f;
            for (ii=0; ii<6; ii++)
            {
                dist = (float) Math.sqrt(Math.pow(legPos1[ii][0],2) + Math.pow(legPos1[ii][1]-turn_dist,2));
                if (dist > maxdist) maxdist = dist;
            }
            // each leg can only sweep so much, so use this farthest leg
            // to determine the angle of sweep that every leg must do
            maxSweep = 8.f* sweepModifier /maxdist;
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
                cycletime = (float) Math.IEEEremainder(time + 0.5*ii + (legPos[ii][0]- legPos[0][0])*0.0125, 1.0);

                // use bezier curve to either be up or down
                // bezierWalkDOwn goes between +/- 0.83775
                if (cycletime < fdf) bezierWalkDOwn.getPos(cycletime/fdf, thtpos, zpos);
                else bezierWalkUp.getPos((float) ((cycletime-fdf)/(1.-fdf)), thtpos, zpos);
                // convert thtpos into angle?
                thtpos.setPos(thtpos.getPos()* maxSweep);

                // convert rpos to xpos,ypos
                dist = (float) Math.sqrt(Math.pow(legPos1[ii][0],2) + Math.pow(legPos1[ii][1]-turn_dist,2));
                tht0 = (float) Math.atan2(legPos1[ii][1]-turn_dist, legPos1[ii][0]);
                xpos = (float) (dist*Math.cos(thtpos.getPos()+tht0));
                ypos = (float) (turn_dist + dist*Math.sin(thtpos.getPos()+tht0));

                // set up the IK target
                target[0].setPos(xpos);
                target[1].setPos(ypos);
                target[2].setPos((float) (-10.0 + zpos.getPos()*3.0*legraise + standHeight));
                if (standHeight < 0.0) target[2].setPos((float) (target[2].getPos()-1.7*zpos.getPos()* standHeight));


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
        for (int i =0; i < fkangles.length; i++){
            fkangles[i] = new Position();
        }
        for (int i =0; i < fkpos.length; i++){
            fkpos[i] = new Position();
        }
        // initialize
        ssTime = 0.0f;
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
            angle[ii*3] = (float) ((servoAngle[ii*3]-150.)*Constants.DEGTORAD);
            angle[ii*3+1] = (float) ((servoAngle[ii*3+1]-150.)*Constants.DEGTORAD);
            angle[ii*3+2] = (float) ((servoAngle[ii*3+2]-150.)*Constants.DEGTORAD);
        }
    }

    private void setServoAngles()
    {
        int ii;
        for (ii=0; ii<6; ii++)
        {
            servoAngle[ii*3] = (float) (angle[ii*3]*Constants.RADTODEG + 150.);
            servoAngle[ii*3+1] = (float) (angle[ii*3+1]*Constants.RADTODEG + 150.);
            servoAngle[ii*3+2] = (float) (angle[ii*3+2]*Constants.RADTODEG + 150.);
        }
    }

    // target is a float[3] giving absolute position
// return whether or not the solver was successful
    private boolean IKSolve(int leg, Position[] target)
    {
        int iter;
        boolean converged;
        float diff;
        float targetr, targetz, targetang;
        Position[] fkpos = new Position[3];
        Position[] fkangles = new Position[3];
        for (int i =0; i < fkpos.length; i++){
            fkpos[i] = new Position();
        }
        for (int i =0; i < fkangles.length; i++){
            fkangles[i] = new Position();
        }
        float[][] J = new float[2][2];
        float[][] inv = new float[2][2];
        float[] delta = new float[2];
        float[] p = new float[2];
        float posr, posz, ang1, ang2, det;

        // convert absolute position to polar around leg root
        targetz = target[2].getPos() - legPos[leg][2];
        targetr = (float) Math.sqrt(Math.pow(target[0].getPos()- legPos[leg][0],2) + Math.pow(target[1].getPos()- legPos[leg][1],2));
        targetang = (float) (Math.atan2(target[1].getPos()- legPos[leg][1],target[0].getPos()- legPos[leg][0]) - legAng[leg]); // atan2 [-pi:pi]

        // easy part: can the coxa servo get to the right angle?
        if (targetang > angleub[0] || targetang < anglelb[0]) return false;
        // else, go ahead and set coxa servo. One out of three angles done!
        angle[leg*3] = targetang;

        // begin 2-joint IK solver using jacobian pseudo-inverse
        // whenever we call FKSolve, need to convert to polar coords around leg root
        fkangles[0].setPos(angle[leg*3]); // already solved for
        // starting point is influenced by actual current point
        // but this makes it safer in case the leg has somehow gone out of bounds
        fkangles[1].setPos((float) (angle[leg*3+1]*0.5));
        fkangles[2].setPos((float) (angle[leg*3+2]*0.5));
        FKSolve(leg, fkangles, fkpos);
        posz = fkpos[2].getPos() - legPos[leg][2];
        posr = (float) Math.sqrt(Math.pow(fkpos[0].getPos()- legPos[leg][0],2) + Math.pow(fkpos[1].getPos()- legPos[leg][1],2));

        diff = (float) Math.sqrt(Math.pow(targetr-posr,2) + Math.pow(targetz-posz,2));
        // ITERATE
        converged = false;
        for (iter=0; iter<Constants.MAXITER && !converged; iter++)
        {
            // compute jacobian
            p[0] = targetr - posr;
            p[1] = targetz - posz;
            ang1 = fkangles[1].getPos()- femurAngle;
            ang2 = fkangles[2].getPos()- tibiaAngle;
            J[0][0] = (float) (-length[1]*Math.sin(ang1) - length[2]*Math.sin(ang1+ang2)); // dr/dang1
            J[1][0] = (float) (-length[2]*Math.sin(ang1+ang2)); // dr/dang2
            J[0][1] = (float) (length[1]*Math.cos(ang1) + length[2]*Math.cos(ang1+ang2)); // dz/dang2
            J[1][1] = (float) (length[2]*Math.cos(ang1+ang2)); // dz/dang2
            // compute inverse
            det = (float) (1.0/(J[0][0]*J[1][1]-J[0][1]*J[1][0]));
            inv[0][0] = J[1][1]*det;
            inv[1][0] = -J[1][0]*det;
            inv[0][1] = -J[0][1]*det;
            inv[1][1] = J[0][0]*det;
            delta[0] = p[0]*inv[0][0] + p[1]*inv[0][1];
            delta[1] = p[0]*inv[1][0] + p[1]*inv[1][1];
            fkangles[1].setPos((float) (fkangles[1].getPos()+delta[0]*0.5));
            fkangles[2].setPos((float) (fkangles[2].getPos()+delta[1]*0.5));
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
            posr = (float) Math.sqrt(Math.pow(fkpos[0].getPos()- legPos[leg][0],2) + Math.pow(fkpos[1].getPos()- legPos[leg][1],2));
            // convergence criteria
            diff = (float) Math.sqrt(Math.pow(targetr-posr,2) + Math.pow(targetz-posz,2));
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
// given a flat[3] angles, compute position and put in float[3] pos
// input angles are in radians and are offset properly
    private void FKSolve(int leg, Position[] angles, Position[] pos)
    {
        float r, ang0, ang1, ang2;

        ang0 = angles[0].getPos()+ legAng[leg];
        ang1 = angles[1].getPos()- femurAngle;
        ang2 = angles[2].getPos()- tibiaAngle;

        r = (float) (length[0] + length[1]*Math.cos(ang1) + length[2]*Math.cos(ang1+ang2));
        pos[0].setPos((float) (legPos[leg][0] + r*Math.cos(ang0)));
        pos[1].setPos((float) (legPos[leg][1] + r*Math.sin(ang0)));
        pos[2].setPos((float) (legPos[leg][2] + length[1]*Math.sin(ang1) + length[2]*Math.sin(ang1+ang2)));
    }

    public void stand ()
    {
        int ii;
        Position[] target = new Position[3];
        for (ii=0; ii<6; ii++)
        {
            target[0].setPos((float) (legPos[ii][0] + 10.0*Math.cos(legAng[ii])));
            target[1].setPos((float) (legPos[ii][1] + 10.0*Math.sin(legAng[ii])));
            target[2].setPos(-10.0f);
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
            target[0].setPos((float) (legPos[ii][0] + 10.0*Math.cos(legAng[ii])));
            target[1].setPos((float) (legPos[ii][1] + 10.0*Math.sin(legAng[ii])));
            target[2].setPos(-5.0f);
            IKSolve(ii,target);
        }
        setServoAngles();
    }

    public float[] getServoAngle() {
        return servoAngle;
    }

    public Hexapod setServoAngle(float[] servoAngle) {
        this.servoAngle = servoAngle;
        return this;
    }

    public float[] getAngle() {
        return angle;
    }

    public Hexapod setAngle(float[] angle) {
        this.angle = angle;
        return this;
    }

    public float[] getLength() {
        return length;
    }

    public Hexapod setLength(float[] length) {
        this.length = length;
        return this;
    }

    public float getFemurAngle() {
        return femurAngle;
    }

    public Hexapod setFemurAngle(float femurAngle) {
        this.femurAngle = femurAngle;
        return this;
    }

    public float getTibiaAngle() {
        return tibiaAngle;
    }

    public Hexapod setTibiaAngle(float tibiaAngle) {
        this.tibiaAngle = tibiaAngle;
        return this;
    }

    public float[] getAngleub() {
        return angleub;
    }

    public Hexapod setAngleub(float[] angleub) {
        this.angleub = angleub;
        return this;
    }

    public float[] getAnglelb() {
        return anglelb;
    }

    public Hexapod setAnglelb(float[] anglelb) {
        this.anglelb = anglelb;
        return this;
    }

    public float[][] getLegPos() {
        return legPos;
    }

    public Hexapod setLegPos(float[][] legPos) {
        this.legPos = legPos;
        return this;
    }

    public float[][] getLegPos1() {
        return legPos1;
    }

    public Hexapod setLegPos1(float[][] legPos1) {
        this.legPos1 = legPos1;
        return this;
    }

    public float[] getLegAng() {
        return legAng;
    }

    public Hexapod setLegAng(float[] legAng) {
        this.legAng = legAng;
        return this;
    }

    public float getTime() {
        return time;
    }

    public Hexapod setTime(float time) {
        this.time = time;
        return this;
    }

    public float getSpeed() {
        return speed;
    }

    public Hexapod setSpeed(float speed) {
        this.speed = speed;
        return this;
    }

    public float getSmoothSpeed() {
        return smoothSpeed;
    }

    public Hexapod setSmoothSpeed(float smoothSpeed) {
        this.smoothSpeed = smoothSpeed;
        return this;
    }

    public float getFdf() {
        return fdf;
    }

    public Hexapod setFdf(float fdf) {
        this.fdf = fdf;
        return this;
    }

    public float getTurning() {
        return turning;
    }

    public Hexapod setTurning(float turning) {
        this.turning = turning;
        return this;
    }

    public float getSmoothTurning() {
        return smoothTurning;
    }

    public Hexapod setSmoothTurning(float smoothTurning) {
        this.smoothTurning = smoothTurning;
        return this;
    }

    public float getStandHeight() {
        return standHeight;
    }

    public Hexapod setStandHeight(float standHeight) {
        this.standHeight = standHeight;
        return this;
    }

    public float getSweepModifier() {
        return sweepModifier;
    }

    public Hexapod setSweepModifier(float sweepModifier) {
        this.sweepModifier = sweepModifier;
        return this;
    }

    public float getSpeedModifier() {
        return speedModifier;
    }

    public Hexapod setSpeedModifier(float speedModifier) {
        this.speedModifier = speedModifier;
        return this;
    }

    public float getMaxSweep() {
        return maxSweep;
    }

    public Hexapod setMaxSweep(float maxSweep) {
        this.maxSweep = maxSweep;
        return this;
    }

    public float getSsTime() {
        return ssTime;
    }

    public Hexapod setSsTime(float ssTime) {
        this.ssTime = ssTime;
        return this;
    }

    public boolean isSsRunning() {
        return ssRunning;
    }

    public Hexapod setSsRunning(boolean ssRunning) {
        this.ssRunning = ssRunning;
        return this;
    }

    public float[] getSsx0() {
        return ssx0;
    }

    public Hexapod setSsx0(float[] ssx0) {
        this.ssx0 = ssx0;
        return this;
    }

    public float[] getSsy0() {
        return ssy0;
    }

    public Hexapod setSsy0(float[] ssy0) {
        this.ssy0 = ssy0;
        return this;
    }

    public float[] getSsz0() {
        return ssz0;
    }

    public Hexapod setSsz0(float[] ssz0) {
        this.ssz0 = ssz0;
        return this;
    }

    public float getDrXpos() {
        return drXpos;
    }

    public Hexapod setDrXpos(float drXpos) {
        this.drXpos = drXpos;
        return this;
    }

    public float getDrYpos() {
        return drYpos;
    }

    public Hexapod setDrYpos(float drYpos) {
        this.drYpos = drYpos;
        return this;
    }

    public float getDrAng() {
        return drAng;
    }

    public Hexapod setDrAng(float drAng) {
        this.drAng = drAng;
        return this;
    }

    public boolean isDebug() {
        return debug;
    }

    public Hexapod setDebug(boolean debug) {
        this.debug = debug;
        return this;
    }

    public Bezier2D getBezierWalkUp() {
        return bezierWalkUp;
    }

    public Hexapod setBezierWalkUp(Bezier2D bezierWalkUp) {
        this.bezierWalkUp = bezierWalkUp;
        return this;
    }

    public Bezier2D getBezierWalkDOwn() {
        return bezierWalkDOwn;
    }

    public Hexapod setBezierWalkDOwn(Bezier2D bezierWalkDOwn) {
        this.bezierWalkDOwn = bezierWalkDOwn;
        return this;
    }

    @Override
    public String toString() {
        return "Hexapod{" +
                "servoAngle=" + Arrays.toString(servoAngle) +
                '}';
    }
}
