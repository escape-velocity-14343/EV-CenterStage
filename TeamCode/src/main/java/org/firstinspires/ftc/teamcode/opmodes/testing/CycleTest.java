package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous
public class CycleTest extends Robot {
    public static double tiltAngle = 11;
    public static double ultrasonicDistance = 4;
    public static double lifterHeight = 1.6;
    public static double maxTiltVelo = 0.0001;
    public static double maxArmVelo = 0.01;
    public static double ultrasonicTolerance = 0.5;
    public static double tiltTolerance = 0.5;
    enum cycle {
        RETRACTED,
        TILTED,
        LIFTER,
        EXTENDED,
        DROPPED,
        INTAKE,
        BUCKETFOLDED,
        DONE,
        FAILED
    }

    cycle cycleEnum = cycle.RETRACTED;

    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
            arm.setLifterHeight(0);
            bucket.unlatch();
            bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
        }
        setFSMtoAuto();
        swerve.setAuton();
        while (opModeIsActive()) {
            update();
            switch (cycleEnum) {
                case RETRACTED:
                    arm.tiltArm(tiltAngle);
                    if (arm.isTilted(tiltTolerance) && Math.abs(arm.getTiltVelocity()) < maxTiltVelo) {
                        cycleEnum = cycle.TILTED;
                        arm.moveTilt(0);
                        timer.reset();
                    }
                    break;
                case TILTED:
                    arm.setLifterHeight(lifterHeight);
                    if (timer.seconds() > 1) {
                        cycleEnum = cycle.LIFTER;
                    }
                    break;
                case LIFTER:
                    arm.ultrasonicExtend(ultrasonicDistance);
                    bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                    if (Math.abs(arm.getUltrasonicInches() - ultrasonicDistance) < ultrasonicTolerance && Math.abs(arm.getVelocity()) < maxArmVelo) {
                        arm.moveSlides(0);
                        cycleEnum = cycle.EXTENDED;
                        timer.reset();
                    }
                    break;
                case EXTENDED:
                    arm.setLifterHeight(0);
                    if (timer.seconds() > 1) {
                        timer.reset();
                        cycleEnum = cycle.DROPPED;
                    }
                    break;
                case DROPPED:
                    bucket.latch();
                    if (timer.seconds() > 1) {
                        timer.reset();
                        cycleEnum = cycle.INTAKE;
                    }
                    break;
                case INTAKE:
                    bucket.tilt(1);
                    if (timer.seconds() > 1) {
                        cycleEnum = cycle.BUCKETFOLDED;
                        timer.reset();
                    }
                    break;
                case BUCKETFOLDED:
                    arm.extend(0);
                    if (timer.seconds() > 1 && (arm.isDone(20) || arm.getVelocity() < maxArmVelo)) {
                        arm.moveSlides(0);
                        cycleEnum = cycle.DONE;
                    }
                    break;

            }
            telemetry.addData("tilt velo", arm.getTiltVelocity());
        }
    }

}
