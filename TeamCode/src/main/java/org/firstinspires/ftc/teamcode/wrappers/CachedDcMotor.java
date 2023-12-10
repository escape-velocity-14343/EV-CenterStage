package org.firstinspires.ftc.teamcode.wrappers;

import static org.firstinspires.ftc.teamcode.wrappers.constants.ACCEPTABLE_MOTOR_POWER_DELTA;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

public class CachedDcMotor extends DcMotorImpl implements DcMotor {
    public double lastPower = 0;

    /**
     * Constructor
     *
     * @param controller DC motor controller this motor is attached to
     * @param portNumber portNumber position on the controller
     */
    public CachedDcMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber, Direction.FORWARD);
    }

    /**
     * Constructor
     *
     * @param controller DC motor controller this motor is attached to
     * @param portNumber portNumber port number on the controller
     * @param direction direction this motor should spin
     */
    public CachedDcMotor(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction, MotorConfigurationType.getUnspecifiedMotorType());
    }

    /**
     * Constructor
     *
     * @param controller DC motor controller this motor is attached to
     * @param portNumber portNumber port number on the controller
     * @param direction direction this motor should spin
     * @param motorType the type we know this motor to be
     */
    public CachedDcMotor(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }

    @Override
    /**
     * Set the current motor power
     *
     * @param power from -1.0 to 1.0
     */
    synchronized public void setPower(double power) {
        // Power must be positive when in RUN_TO_POSITION mode : in that mode, the
        // *direction* of rotation is controlled instead by the relative positioning
        // of the current and target positions.
        if (getMode() == RunMode.RUN_TO_POSITION) {
            power = Math.abs(power);
        } else {
            power = adjustPower(power);
        }
        if (Math.abs(power-lastPower) > ACCEPTABLE_MOTOR_POWER_DELTA) {
            internalSetPower(power);
            lastPower = power;
        }
    }
}
