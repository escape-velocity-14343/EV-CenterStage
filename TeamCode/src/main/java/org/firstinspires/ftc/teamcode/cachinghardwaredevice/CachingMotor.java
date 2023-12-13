package org.firstinspires.ftc.teamcode.cachinghardwaredevice;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CachingMotor extends Motor {
    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public CachingMotor(@NonNull HardwareMap hMap, String id, @NonNull Motor.GoBILDA gobildaType) {
        motor = new CachingDcMotor(hMap.get(DcMotor.class, id));
        encoder = new Encoder(motor::getCurrentPosition);

        runmode = RunMode.RawPower;
        type = gobildaType;

        ACHIEVABLE_MAX_TICKS_PER_SECOND = gobildaType.getAchievableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public CachingMotor(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motor.getMotorType().getAchieveableMaxTicksPerSecond();
    }
}
