package com.chazz.hexapod;


import java.util.Vector;

class Bezier2D {

    private Vector<Double> xpos = new Vector<Double>();
    private Vector<Double> ypos = new Vector<Double>();

    Bezier2D () {}

    void addPoint (double x, double y)
    {
        xpos.add(x);
        ypos.add(y);
    }

    // t must be [0:1]
    void getPos (double t, Position xret, Position yret)
    {
        int ii, ij, npoints;
        double[] x, y;
        npoints = xpos.size();
        x = new double [npoints];
        y = new double [npoints];
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
                x[ij] = (1.0-t)*x[ij] + t*x[ij+1];
                y[ij] = (1.0-t)*y[ij] + t*y[ij+1];
            }
        }


        xret.setPos(x[0]);
        yret.setPos(y[0]);
    }
}
