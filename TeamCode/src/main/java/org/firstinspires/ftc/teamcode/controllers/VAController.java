package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.pathutils.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.pathutils.VAPair;

public class VAController {
    private double kV;
    private double kA;
    private AsymmetricMotionProfile sp;
    public VAController(double kV, double kA) {
        this.kV = kV;
        this.kA = kA;
    }

    public void setVA(double kV, double kA) {
        this.kV = kV;
        this.kA = kA;
    }

    private double calculate(VAPair pv) {
        return pv.getVelocity() * this.kV + pv.getAcceleration() * this.kA;
    }

    public double calculate(double distance, double velocity) {
        return this.calculate(sp.get(distance, velocity));
    }

    public double calculate(double distance, double velocity, AsymmetricMotionProfile sp) {
        this.setMotionProfile(sp);
        return this.calculate(distance, velocity);
    }

    public void setMotionProfile(AsymmetricMotionProfile sp) {
        this.sp = sp;
    }


}
