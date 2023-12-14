package org.firstinspires.ftc.teamcode.drivers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



public class AnalogEncoder {
    AnalogInput analog;
    InterpLUT lut = new InterpLUT();

    double offset = 0.0;



    /**
     * Constructs the instance AS5600 for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */

    public AnalogEncoder(HardwareMap hMap, String id) {
        analog = hMap.get(AnalogInput.class, id);

    }
    public double getDegrees() {
        double v = analog.getVoltage();

        return v*360.0/3.3+offset;
    }
    public double getRawVoltage() {
        return analog.getVoltage();
    }

    public double getRadians() {
        return Math.toRadians(getDegrees());
    }
    public void setOffset(double of) {
        offset = of;
    }

}
