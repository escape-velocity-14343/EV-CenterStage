package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;

public abstract class Robot extends LinearOpMode {
    //Drive
    public MotorEx leftTop;
    public MotorEx leftBottom;
    public MotorEx rightTop;
    public MotorEx rightBottom;
    public AnalogEncoder leftHeading;
    public AnalogEncoder rightHeading;
    public Motor.Encoder forwardEnc;
    public Motor.Encoder leftEnc;

    //Subsystems
    public Bucket bucket;
    public Intake intake;

    public void initialize() {
        bucket = new Bucket(hardwareMap);
        intake = new Intake(hardwareMap);
    }
}
