package org.firstinspires.ftc.teamcode.subsystems.Trajectory;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Vector;

public class Lerp extends Spline {
    public ArrayList<Vector> controlpoints;

    public Lerp(ArrayList<Vector> controls) {
        assert controls.size() == 2;
        controlpoints = controls;
        length = 1;
    }

    public Vector getValue(double t) {
        t = constrain(t);
        return Vector.add(controlpoints.get(0).scale(1-t), controlpoints.get(1).scale(t));
    }

    public Vector getVelocity(double t) {
        t = constrain(t);
        return Vector.add(controlpoints.get(0).scale(-1), controlpoints.get(1));
    }

    public Vector getAcceleration(double t) {
        t = constrain(t);
        return Vector.zeroVector(controlpoints.get(0).size);
    }

    public void initArclen(int acc) {
        arclength = Vector.dist(controlpoints.get(0), controlpoints.get(1));
        length = arclength;
    }
}
