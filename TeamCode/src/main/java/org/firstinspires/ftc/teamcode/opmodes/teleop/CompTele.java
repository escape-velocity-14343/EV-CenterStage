package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


@TeleOp(name="Comp TeleOp")
public class CompTele extends Robot {

    Gamepad lastgamepad1c = new Gamepad();
    Gamepad lastgamepad2c = new Gamepad();

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            update();

            swerve.driveFieldCentric(-gamepad1c.left_stick_x, -gamepad1c.left_stick_y, gamepad1c.right_stick_x);

            // outtake
            if (inOuttake() && isDone()) {
                if (gamepad1c.left_trigger > 0) {
                    setOuttake(getArmDistance(), getArmHeight() - gamepad1c.left_trigger * (loopNanos / 1e8));
                } else if (gamepad1c.right_trigger > 0) {
                    setOuttake(getArmDistance(), getArmHeight() + gamepad1c.right_trigger * (loopNanos / 1e8));
                }

                if (gamepad1c.square) {
                    bucket.setLeftLatch(false);
                }
                if (gamepad1c.circle) {
                    bucket.setRightLatch(false);
                }
                if (gamepad2c.square) {
                    bucket.setLeftLatch(true);
                }
                if (gamepad2c.circle) {
                    bucket.setRightLatch(true);
                }
                if (gamepad2c.left_trigger > 0) {
                    setOuttake(getArmDistance() - gamepad2c.left_trigger * (loopNanos / 1e8), getArmHeight());
                } else if (gamepad2c.right_trigger > 0) {
                    setOuttake(getArmDistance() + gamepad2c.right_trigger * (loopNanos / 1e8), getArmHeight());
                }
                if (gamepad2c.triangle) {
                    bucket.dropFromStack();
                }

            }
            // intake
            if (inIntake() && isDone()) {
                if (gamepad1c.left_trigger > 0) {
                    arm.moveSlides(-gamepad1c.left_trigger);
                } else if (gamepad1c.right_trigger > 0) {
                    arm.moveSlides(gamepad1c.right_trigger);
                } else {
                    arm.moveSlides(0);
                }

                if (gamepad1c.square) {
                    bucket.setLeftLatch(!bucket.leftIsLatched());
                }
                if (gamepad1c.circle) {
                    bucket.setRightLatch(!bucket.rightIsLatched());
                }
            }

            // fsm transitions
            if (gamepad1c.triangle) {
                outtake();
                setOuttake(10, getArmHeight());
            }
            if (gamepad1c.cross) {
                outtake();
                setOuttake(24, getArmHeight());
            }
            if (gamepad1c.right_bumper) {
                intake();
                setIntake(0);
            }
            if (gamepad1c.left_bumper) {
                intake();
                setIntake(0);
            }
            if (gamepad1c.touchpad) {
                foldArm();
            }

            // turn off heading lock if arm is extended
            if (arm.getPosition() > 50) {
                swerve.setAuton();
            } else {
                swerve.setNormal();
            }
            telemetry.addData("arm enc pos", arm.getPosition());

            // overrides
            /*if (gamepad2c.left_stick_button && !lastgamepad2c.left_stick_button) {
                setAutoLatch(false);
            }
            if (gamepad2c.right_stick_button && !lastgamepad2c.right_stick_button) {
                setAutoRetract(false);
            }
            if (gamepad2c.options && !lastgamepad2c.options) {
                setFSMtoAuto();
            }
            if (gamepad2c.share && !lastgamepad2c.share) {
                flipHeadingLock();
            }
            if (inAuto()) {
                if (gamepad2c.left_trigger > 0.7) {
                    arm.moveSlides(gamepad2c.left_trigger);
                } else if (gamepad2c.right_trigger > 0.7) {
                    arm.moveSlides(-gamepad2c.right_trigger);
                }
                if (gamepad2c.left_bumper) {
                    arm.moveTilt(1);
                } else if (gamepad2c.right_bumper) {
                    arm.moveTilt(-1);
                }
                if (gamepad2c.dpad_left) {
                    arm.resetSlides();
                } else if (gamepad2c.dpad_right) {
                    arm.resetTilt();
                } else if (gamepad2c.dpad_up) {
                    arm.reset();
                }
                if (gamepad2c.square) {
                    swerve.setAuton();
                } else if (gamepad2c.cross) {
                    swerve.headingLock(true, getHeading());
                    swerve.headingLock(false, getHeading());
                }
            }*/



            if (gamepad1c.options && !lastgamepad1c.options) {
                imu.resetYaw();
            }

            lastgamepad1c.copy(gamepad1c);
            lastgamepad2c.copy(gamepad2c);
        }
    }
}
