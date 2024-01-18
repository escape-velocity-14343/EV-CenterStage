package org.firstinspires.ftc.teamcode.controllers;

/**
 * Convenience class that has useful default coefficients for motors. Extends SquIDF.
 */
public class MotorSquIDF extends SquIDF {

    public MotorSquIDF(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0.01, 0.01);
    }

    public MotorSquIDF(double kP, double kI, double kD, double kF, double increment, double decrement) {
        super(kP, kI, kD, kF, increment, decrement);
        setFeedforwardBounds(0, 0.2);
        setFeedforwardTolerance(0.01);
    }



}
