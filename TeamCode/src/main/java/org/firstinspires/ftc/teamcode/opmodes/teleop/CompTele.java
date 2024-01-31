package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


@Config
@TeleOp(name="Comp TeleOp")
public class CompTele extends Robot {

    Gamepad lastgamepad1c = new Gamepad();
    Gamepad lastgamepad2c = new Gamepad();
    public static double intakeTilt = 0;

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
                    setOuttake((getArmDistance() < 14 ? 14 : getArmDistance() - gamepad1c.left_trigger * (loopNanos / 4e7)), getArmHeight() - gamepad1c.left_trigger * (loopNanos / 2e7));
                } else if (gamepad1c.right_trigger > 0) {
                    setOuttake(getArmDistance() + gamepad1c.right_trigger * (loopNanos / 4e7), getArmHeight() + gamepad1c.right_trigger * (loopNanos / 2e7));
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
                    setOuttake((getArmDistance() < 14 ? 14 : getArmDistance() - gamepad2c.left_trigger * (loopNanos / 1e8)), getArmHeight());
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
                } else if (gamepad1c.right_trigger > 0  && arm.getPosition() < 1700) {
                    arm.moveSlides(gamepad1c.right_trigger);
                } else {
                    arm.moveSlides(0);
                }

                if (gamepad1c.square&&!lastgamepad1c.square) {
                    bucket.setLeftLatch(!bucket.leftIsLatched());
                }
                if (gamepad1c.circle&&!lastgamepad1c.circle) {
                    bucket.setRightLatch(!bucket.rightIsLatched());
                }
                if (gamepad2c.left_trigger > 0) {
                    arm.moveTilt(-gamepad2c.left_trigger);
                } else if (gamepad2c.right_trigger > 0) {
                    arm.moveTilt(gamepad2c.right_trigger);
                }
            }

            if (inHang()) {
                if (gamepad1c.left_trigger > 0) {
                    hang.setPower(gamepad1c.left_trigger);
                } else if (gamepad1c.right_trigger > 0) {
                    // prevent overextending
                    hang.setPower(-Range.clip(gamepad1c.right_trigger, 0, 0.7));
                } else {
                    hang.setPower(0);
                }

                if (gamepad1c.dpad_down&&!lastgamepad1c.dpad_down) {
                    incrementHangState();
                }

                if (gamepad2c.left_trigger > 0) {
                    arm.moveSlides(-gamepad2.left_trigger);
                } else if (gamepad2c.right_trigger > 0  && arm.getPosition() < 1700) {
                    arm.moveSlides(gamepad2c.right_trigger);
                } else {
                    arm.moveSlides(0);
                }
            }

            if (inDrone()) {
                if (gamepad1c.dpad_right) {
                    drone.setPosition(0);
                } else if (gamepad1c.dpad_left) {
                    drone.setPosition(1);
                }

                if (gamepad2c.left_trigger > 0) {
                    arm.moveSlides(-gamepad1c.left_trigger);
                } else if (gamepad2c.right_trigger > 0  && arm.getPosition() < 1700) {
                    arm.moveSlides(gamepad1c.right_trigger);
                } else {
                    arm.moveSlides(0);
                }
            }

            // fsm transitions
            if (gamepad1c.triangle) {
                outtake();
                setOuttake(14+getArmHeight()/2, getArmHeight());
            }
            if (gamepad1c.cross) {
                outtake();
                setOuttake(24+getArmHeight()/2, getArmHeight());
            }
            if (gamepad1c.right_bumper) {
                intake();
                setIntake(intakeTilt);
            }
            if (gamepad1c.left_bumper&&!lastgamepad1c.left_bumper) {
                if (bucket.leftIsLatched() != bucket.rightIsLatched()) {
                    bucket.latch();
                } else {
                    bucket.setLeftLatch(!bucket.leftIsLatched());
                    bucket.setRightLatch(!bucket.rightIsLatched());
                }
            }
            if (gamepad1c.touchpad) {
                if (inIntake()) {
                    iFoldArm();
                } else if (!lastgamepad1c.touchpad){
                    foldArm();
                }
            }
            if (gamepad1c.dpad_down && !inHang()) {
                hang();
            }

            if (gamepad1c.dpad_up) {
                drone();
            }

            // turn off heading lock if arm is extended
            if (arm.getPosition() > 200) {
                swerve.setAuton();
            } else {
                swerve.setNormal();
            }
            telemetry.addData("arm enc pos", arm.getPosition());
            telemetry.addData("left prox", bucket.getLeftDist());
            telemetry.addData("right prox", bucket.getRightDist());

            //
            if (gamepad2c.left_stick_button && !lastgamepad2c.left_stick_button) {
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
            if (gamepad1c.options && !lastgamepad1c.options) {
                odometry.resetYaw();
            }
            if (inAuto()) {
                if (gamepad2c.left_trigger > 0) {
                    arm.moveSlides(-gamepad2c.left_trigger);
                } else if (gamepad2c.right_trigger > 0) {
                    arm.moveSlides(gamepad2c.right_trigger);
                } else {
                    arm.moveSlides(0);
                }
                if (gamepad2c.left_bumper) {
                    arm.moveTilt(0.5);
                } else if (gamepad2c.right_bumper) {
                    arm.moveTilt(-0.5);
                } else {
                    arm.moveTilt(0);
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
            }
            /*if (gamepad2c.left_trigger>0) {
                hang.setPower(-gamepad2c.left_trigger);
            }
            else if (gamepad2c.right_trigger>0) {
                hang.setPower(gamepad2c.right_trigger);
            }
            else {
                hang.setPower(0);
            }*/



            if (gamepad1c.options && !lastgamepad1c.options) {
                odometry.resetYaw();
            }

            lastgamepad1c.copy(gamepad1c);
            lastgamepad2c.copy(gamepad2c);
        }
    }
}
