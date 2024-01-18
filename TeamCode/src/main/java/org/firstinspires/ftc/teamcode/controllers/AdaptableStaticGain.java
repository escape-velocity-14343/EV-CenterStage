package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.Range;

/**
 * A self-adapting static gain. Works as follows:
 * If the system is not moving and also not done, the static gain will be incremented.
 * If the system is overshooting (the sign of the error changes), the static gain will be decremented.
 * This ensures automatic and robust tuning, regardless of field condition.
 */
public class AdaptableStaticGain {
    /**
     * The static gain.
     */
    private double kStatic;

    /**
     * The amount the gain will be incremented by when there is no change in error.
     */
    private double increment;
    /**
     * The amount the gain will be decremented by when the controller overshoots. By default this is the same as increment.
     */
    private double decrement;

    /**
     * The maximum possible change in error where the system is still not moving.
     */
    private double incrementTolerance = 0.01;

    private double maxGain = Double.POSITIVE_INFINITY;
    private double minGain = Double.NEGATIVE_INFINITY;

    private double lastError = Double.NaN;

    public AdaptableStaticGain(double kStatic, double increment) {
        this.kStatic = kStatic;
        this.increment = increment;
        this.decrement = increment;
    }

    public AdaptableStaticGain(double kStatic, double increment, double decrement) {
        this(kStatic, increment);
        this.decrement = decrement;
    }

    /**
     * Cap the possible values for the gain. Use {@code Double.POSITIVE_INFINITY} or {@code Double.NEGATIVE_INFINITY} to uncap bounds.
     */
    public void setBounds(double min, double max) {
        this.minGain = min;
        this.maxGain = max;
    }

    /**
     * @param tolerance The maximum possible change in error where the system is still not moving.
     */
    public void setIncrementTolerance(double tolerance) {
        this.incrementTolerance = tolerance;
    }

    public void update(double error) {
        if (Math.abs(error - lastError) < incrementTolerance) {
            kStatic += increment;
        } else if (Math.signum(error) != Math.signum(lastError)) {
            kStatic -= decrement;
        }
        kStatic = Range.clip(kStatic, minGain, maxGain);
        lastError = error;
    }

    public void update(double pv, double sp, double tolerance) {
        if (Math.abs(sp-pv) > tolerance) {
            this.update(pv, sp);
        }
    }

    public void update(double pv, double sp) {
        this.update(sp-pv);
    }

    public double calculate() {
        return kStatic;
    }
}
