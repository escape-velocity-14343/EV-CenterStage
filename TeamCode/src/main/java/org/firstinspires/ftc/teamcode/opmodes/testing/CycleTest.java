package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;

@Config
@Autonomous
public class CycleTest extends AutoBase {
    public static double tiltAngle = 11;
    public static double ultrasonicDistance = 3.5;
    public static double lifterHeight = 2.1;
    public static double maxTiltVelo = 0.0001;
    public static double maxArmVelo = 0.01;
    public static double ultrasonicTolerance = 0.5;
    public static double tiltTolerance = 0.5;
    public static double extendInches = 40;
    public static double retractDist = 40;
    private int retractPos = 0;
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
        //initStackPortal();
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
                            timer.reset();
                        }
                    }
                    break;
                case EXTENDING:
                    ArmIVK.calcIntakeIVK(0, lifterHeight+0.2, Math.toRadians(arm.getTilt()));
                    bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                    arm.ultrasonicExtend(ultrasonicDistance+5);
                    //arm.extendInches(extendInches-5);
                    if (arm.isDone(30)) {
                        cycleEnum = cyclestate.CORRECTING;
                    }
                    break;
                case CORRECTING:
                    //swerve.driveRobotCentric(0,0, stackProc.getMiddle());
                    /*arm.ultrasonicExtend(ultrasonicDistance);
                    if (SwerveModule.compare(stackProc.getMiddle(),0,20)||SwerveModule.compare(arm.getUltrasonicInches(),ultrasonicDistance,ultrasonicTolerance)) {
                        cycleEnum = cyclestate.INTAKING;
                        timer.reset();
                    }*/
                    // TODO: add this
                    cycleEnum = cyclestate.INTAKING;
                    break;
                case INTAKING:
                    arm.moveSlides(0.5);
                    if (bucket.getNumPixels()>0) {
                        ArmIVK.calcIntakeIVK(0, lifterHeight-0.2, Math.toRadians(arm.getTilt()));
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
                        retractPos = arm.getPosition();
                        timer.reset();
                    }
                    break;
                case RETRACTING:
                    arm.extend(retractPos - retractDist);
                    if (timer.seconds() > 1) {
                        cycleEnum = cyclestate.DONE;
                        timer.reset();
                    }
                    break;
                case DONE:
                    bucket.tilt(ArmIVK.getBarTilt(),ArmIVK.getBucketTilt()+0.5);
                    if (timer.seconds()>0.5) {
                        arm.extendInches(2);
                    }
                    break;


            }
            telemetry.addData("tilt velo", arm.getTiltVelocity());
        }
    }

}
