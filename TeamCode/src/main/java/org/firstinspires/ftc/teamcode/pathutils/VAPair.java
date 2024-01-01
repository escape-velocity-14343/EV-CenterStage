package org.firstinspires.ftc.teamcode.pathutils;

public class VAPair {
    private double velocity;
    private double acceleration;
    public VAPair(double velocity, double acceleration) {
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double getVelocity() {
        return this.velocity;
    }

    public double getAcceleration() {
        return this.acceleration;
    }
}
