package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class UltrasonicSensor {
    AnalogInput goober;
    public UltrasonicSensor(HardwareMap hMap, String key) {
        goober = hMap.analogInput.get(key);
    }
    /**
     * @return the distance in CM
     * */
    public double getDistance() {
        return goober.getVoltage()/3.3*500.0;
    }
}
