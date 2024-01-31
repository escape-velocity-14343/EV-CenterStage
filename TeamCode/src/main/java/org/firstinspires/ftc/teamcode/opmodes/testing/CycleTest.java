package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous
public class CycleTest extends Robot {
    public static double tiltAngle = 11;
    public static double ultrasonicDistance = 10;
    public static double lifterHeight = 2.2;
    public static double maxTiltVelo = 0.0001;
    public static double maxArmVelo = 0.01;
    public static double ultrasonicTolerance = 0.5;
    public static double tiltTolerance = 0.5;
    public static double extendInches = 40;
    enum cycle {
        RETRACTED,
        TILTING,
        EXTENDING,
        CORRECTING,
        INTAKING,
        INTAKING2,
        RETRACTING,
        DONE
    }

    cycle cycleEnum = cycle.RETRACTED;

    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
            //arm.setLifterHeight(0);
            bucket.unlatch();
            bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
        }
        setFSMtoAuto();
        swerve.setAuton();
        while (opModeIsActive()) {
            update();
            switch (cycleEnum) {
                case RETRACTED:
                    cycleEnum = cycle.TILTING;
                    timer.reset();
                    break;
                case TILTING:
                    ArmIVK.calcIntakeIVK(0, lifterHeight, Math.toRadians(arm.getTilt()));
                    bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                    if (arm.getTilt() > 1) {
                        arm.tiltArm(0);
                        timer.reset();
                    } else {
                        arm.tiltArm(-0.5);
                        if (arm.getTiltVelocity() < maxTiltVelo || timer.seconds() > 0.5) {
                            cycleEnum = cycle.EXTENDING;
                        }
                    }
                    break;
                case EXTENDING:
                    ArmIVK.calcIntakeIVK(0, lifterHeight, Math.toRadians(arm.getTilt()));
                    bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt()+0.5);
                    //arm.ultrasonicExtend(ultrasonicDistance+5);
                    arm.extendInches(extendInches-5);
                    if (arm.isDone(30)) {
                        cycleEnum = cycle.CORRECTING;
                    }
                    break;
                case CORRECTING:
                    // TODO: add this
                    cycleEnum = cycle.INTAKING;
                    break;
                case INTAKING:
                    arm.extendInches(extendInches);
                    if (arm.isDone(10)) {
                        bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                        if (timer.seconds() > 1) {
                            cycleEnum = cycle.INTAKING2;
                            timer.reset();
                        }
                    } else {
                        timer.reset();
                    }
                    break;
                case INTAKING2:
                    bucket.latch();
                    if (timer.seconds() > 2) {
                        cycleEnum = cycle.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    arm.moveSlides(-0.5);
                    if (timer.seconds() > 2 || arm.isDone(10)) {
                        cycleEnum = cycle.DONE;
                    }
                    break;

            }
            telemetry.addData("tilt velo", arm.getTiltVelocity());
        }
    }

}
