package org.firstinspires.ftc.teamcode.wrappers;

import static org.firstinspires.ftc.teamcode.wrappers.constants.ACCEPTABLE_MOTOR_POWER_DELTA;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class CachedMotor extends Motor {

    private double lastPower = 0;
    @Override
    /**
     * Common method for setting the speed of a motor.
     *
     * @param output The percentage of power to set. Value should be between -1.0 and 1.0.
     */
    public void set(double output) {
        double power = lastPower;
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            power = velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND;
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            power = output * error;
        } else {
            power = output;
        }
        if (Math.abs(lastPower-power) > ACCEPTABLE_MOTOR_POWER_DELTA) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public void stopMotor() {
        if (Math.abs(lastPower) > ACCEPTABLE_MOTOR_POWER_DELTA) {
            motor.setPower(0);
            lastPower = 0;
        }
    }

}
