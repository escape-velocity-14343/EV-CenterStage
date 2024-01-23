package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class Reset extends Robot {
    @Override
    public void runOpMode() {
        initialize();
        arm.reset();
        setFSMtoAuto();
        swerve.setAuton();
        odometry.resetYaw();
        while (!isStopRequested()) {
            update();
            telemetry.addData("armpositionbefore", arm.getPosition());
            arm.reset();
            telemetry.addData("armpositionafter", arm.getPosition());
        }
    }
}
