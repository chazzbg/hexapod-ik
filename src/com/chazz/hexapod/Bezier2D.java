package com.chazz.hexapod;


import java.util.Vector;

class Bezier2D {

    private Vector<Float> xpos = new Vector<Float>();
    private Vector<Float> ypos = new Vector<Float>();

    Bezier2D () {}

    void addPoint (float x, float y)
    {
        xpos.add(x);
        ypos.add(y);
    }

    // t must be [0:1]
    void getPos (float t, Position xret, Position yret)
    {
        int ii, ij, npoints;
        float[] x, y;
        npoints = xpos.size();
        x = new float [npoints];
        y = new float [npoints];
        // load with current points
        for (ii=0; ii<npoints; ii++)
        {
            x[ii] = xpos.get(ii);
            y[ii] = ypos.get(ii);
        }

        // iterate over levels
        for (ii=0; ii<npoints-1; ii++)
        {
            for (ij=0; ij<npoints-ii-1; ij++)
            {
                x[ij] = (1.0f-t)*x[ij] + t*x[ij+1];
                y[ij] = (1.0f-t)*y[ij] + t*y[ij+1];
            }
        }


        xret.setPos(x[0]);
        yret.setPos(y[0]);
    }
}
