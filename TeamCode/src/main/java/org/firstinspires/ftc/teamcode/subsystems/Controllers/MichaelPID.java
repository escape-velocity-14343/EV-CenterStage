package org.firstinspires.ftc.teamcode.subsystems.Controllers;

public class MichaelPID {
    double kP;
    double kI;
    double kD;

    double isum;
    double lasterror;

    /**
     * Standard PID controller.
     * @param kP Proportional correction term.
     * @param kD javadocs coming 2025
     */
    public MichaelPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.isum = 0;
        this.lasterror = 0;
    }

    /**
     * Ensure {@code timeelapsed} is of constant unit.
     */
    public double get(double error, double timeelapsed) {
        isum += (error+lasterror)*timeelapsed/2;
        double deriv = (error-lasterror)/timeelapsed;
        lasterror = error;
        return (error * kP) + (isum * kI) + (deriv * kD);
    }
}
