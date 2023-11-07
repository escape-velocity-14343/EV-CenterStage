package org.firstinspires.ftc.teamcode.subsystems.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Vector;

import java.util.ArrayList;

public class CubicBezier extends Spline {
    public ArrayList<Vector> controlpoints;

    public CubicBezier(ArrayList<Vector> controls) {
        assert controls.size() == 4; 
        controlpoints = controls;
        length = 1.0;
    }


    public void initArclen(int acc) {
        arclength = 0;
        double accval = (double) acc;
        //System.out.println(length);
        for(double i = (1.0/accval)*length; i <= length; i += (1.0/accval)*length) {
            Vector prev = getValue(i-(1.0/accval)*length);
            Vector now = getValue(i);
            arclength += Vector.dist(prev, now);
            //System.out.println(t);
        }
        //System.out.println(arclength);
        length = arclength;
    }

    public Vector getValue(double t) {
        t = constrain(t);
        return Vector.add(Vector.add(controlpoints.get(0).scale(Math.pow(1-t, 3)), controlpoints.get(1).scale(3*Math.pow(1-t, 2)*t)), 
                        Vector.add(controlpoints.get(2).scale(3*Math.pow(t, 2)*(1-t)), controlpoints.get(3).scale(Math.pow(t, 3))));
    }

    public Vector getVelocity(double t) {
        t = constrain(t);
        return Vector.add(Vector.add(Vector.add(controlpoints.get(0).scale(-1), controlpoints.get(1)).scale(3*Math.pow(1-t, 2)), 
                        Vector.add(controlpoints.get(1).scale(-1), controlpoints.get(2)).scale(6*t*(1-t))), Vector.add(controlpoints.get(2).scale(-1), controlpoints.get(3)).scale(3*Math.pow(t, 2)));
    }

    public Vector getAcceleration(double t) {
        t = constrain(t);
        return Vector.add(Vector.add(Vector.add(controlpoints.get(2), controlpoints.get(1).scale(-2)), controlpoints.get(0)).scale(6*(1-t)),
                        Vector.add(Vector.add(controlpoints.get(3), controlpoints.get(2).scale(-2)), controlpoints.get(1)).scale(6*(t)));

    }
}