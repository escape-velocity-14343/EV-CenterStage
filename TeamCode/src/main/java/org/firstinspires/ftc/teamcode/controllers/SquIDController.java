package org.firstinspires.ftc.teamcode.controllers;

import com.arcrobotics.ftclib.controller.PIDController;

// goofy asf sqrt pid control
// #SquIDgame
public class SquIDController extends BetterPIDFController {

    private boolean resetIntegral = false;
    private double integralDecayFactor = 1;
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

    public void resetIntegralOnSetPointChange(boolean yes) {
        resetIntegral = yes;
    }

    /**
     * @param factor Used as a per-second multiplier (i.e. actual error = uncorrected error * factor ^ t)
     */
    public void setIntegralDecay(double factor) {
        this.integralDecayFactor = factor;
    }

    public void setMinIntegrationError(double error) {

    }

    @Override
    public void setSetPoint(double sp) {
        if (resetIntegral && sp != getSetPoint()) {
            clearTotalError();
        }
        super.setSetPoint(sp);
    }




}
