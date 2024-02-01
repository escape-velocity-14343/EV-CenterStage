package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;

@Config
@Autonomous
public class CycleTest extends AutoBase {
    public static double tiltAngle = 11;
    public static double ultrasonicDistance = 10;
    public static double lifterHeight = 2.2;
    public static double maxTiltVelo = 0.0001;
    public static double maxArmVelo = 0.01;
    public static double ultrasonicTolerance = 0.5;
    public static double tiltTolerance = 0.5;
    public static double extendInches = 40;
    enum cyclestate {
        RETRACTED,
        TILTING,
        EXTENDING,
        CORRECTING,
        INTAKING,
        INTAKING2,
        RETRACTING,
        DONE
    }

    cyclestate cycleEnum = cyclestate.RETRACTED;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
            timer.reset();
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
                    cycleEnum = cyclestate.TILTING;

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
                            cycleEnum = cyclestate.EXTENDING;
                        }
                    }
                    break;
                case EXTENDING:
                    ArmIVK.calcIntakeIVK(0, lifterHeight, Math.toRadians(arm.getTilt()));
                    bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                    //arm.ultrasonicExtend(ultrasonicDistance+5);
                    arm.extendInches(extendInches-5);
                    if (arm.isDone(30)) {
                        cycleEnum = cyclestate.CORRECTING;
                    }
                    break;
                case CORRECTING:
                    swerve.driveRobotCentric(0,0, stackProc.getX());
                    arm.ultrasonicExtend(ultrasonicDistance);
                    if (SwerveModule.compare(stackProc.getX(),0,20)&&SwerveModule.compare(arm.getUltrasonicInches(),ultrasonicDistance,ultrasonicTolerance)) {
                        cycleEnum = cyclestate.INTAKING;
                    }
                    // TODO: add this
                    //cycleEnum = cyclestate.INTAKING;
                    break;
                case INTAKING:
                    arm.ultrasonicExtend(extendInches-5);
                    if (SwerveModule.compare(arm.getUltrasonicInches(),ultrasonicDistance,ultrasonicTolerance)||bucket.getNumPixels()>0) {
                        bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                        if (timer.seconds() > 1) {
                            cycleEnum = cyclestate.INTAKING2;
                            timer.reset();
                        }
                    } else {
                        timer.reset();
                    }
                    break;
                case INTAKING2:
                    bucket.latch();
                    if (timer.seconds() > 2) {
                        cycleEnum = cyclestate.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    bucket.tilt(ArmIVK.getBarTilt(),ArmIVK.getBucketTilt()+0.5);
                    arm.moveSlides(-0.5);
                    if (timer.seconds() > 2 || arm.isDone(10)) {
                        cycleEnum = cyclestate.DONE;
                    }
                    break;

            }
            telemetry.addData("tilt velo", arm.getTiltVelocity());
        }
    }

}
