package org.firstinspires.ftc.teamcode.drivers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class AS5600 {
    AnalogInput analog;
    boolean invert = false;


    public double offset = 0;

    /**
     * Constructs the instance AS5600 for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */

    public AS5600(HardwareMap hMap, String id) {
        analog = hMap.get(AnalogInput.class, id);



    }
    public double getDegrees() {
        double v = analog.getVoltage();
        double angle = AngleUnit.normalizeDegrees(v*360/3.3+offset);
        if (invert) {
            angle = AngleUnit.normalizeDegrees(-angle);
        }

        return angle;
    }
    public double getRadians() {
        return Math.toRadians(getDegrees());
    }
    public void setOffset(double offset) {
        this.offset=offset;
    }
    public void setInverted(boolean inverted) {
        this.invert = inverted;
    }

}
