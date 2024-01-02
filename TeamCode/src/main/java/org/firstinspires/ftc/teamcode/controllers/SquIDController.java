package org.firstinspires.ftc.teamcode.controllers;

import com.arcrobotics.ftclib.controller.PIDController;

// goofy asf sqrt pid control
// #SquIDgame
public class SquIDController extends BetterPIDFController {

    public SquIDController(double kP, double kI, double kD) {
        super(kP, kI, kD, 0);
    }

    @Override
    public double calculate(double pv) {
        double out = super.calculate(pv);
        return Math.signum(out)*Math.sqrt(Math.abs(out));
    }

    public double calculateSQRT(double pv) {
        double error = getSetPoint() - pv;
        return super.calculate(Math.signum(error)*Math.sqrt(Math.abs(error)), 0);
    }




}
