package org.firstinspires.ftc.teamcode.drivers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public class AnalogEncoder {
    AnalogInput analog;
    InterpLUT lut = new InterpLUT();

    public static double offset = 0;

    /**
     * Constructs the instance AS5600 for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */

    public AnalogEncoder(HardwareMap hMap, String id) {
        analog = hMap.get(AnalogInput.class, id);
        lut.add(0,0);
        lut.add(12, 17);
        lut.add(30.5, 45);
        lut.add(44.6, 65);
        lut.add(58.6, 82.5);
        lut.add(74.2, 109);
        lut.add(88.6,129);
        lut.add(103.9, 152);
        lut.add(120,178);
        lut.add(134, 199);
        lut.add(149.5, 224.5);
        lut.add(175.5,265);
        lut.add(189.5,280);
        lut.add(194.9, 289);
        lut.add(209.6, 305);
        //lut.add()



    }
    public double getDegrees() {
        double v = analog.getVoltage();
        double voltage = 0;
        if (v!=0) {
            voltage = 3.3-(3.3*(3.3+2*v)-3.3*Math.sqrt(3.3*3.3-4*3.3*v+12*v*v))/(4*v);
        }

        return AngleUnit.normalizeDegrees(v*360/3.3+offset);
    }
    public double getRadians() {
        return Math.toRadians(getDegrees());
    }

}
