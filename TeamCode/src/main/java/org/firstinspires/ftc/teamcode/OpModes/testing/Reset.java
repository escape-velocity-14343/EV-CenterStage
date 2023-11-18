package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class Reset extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while(!isStopRequested()) {
            slides.reset();
            odometry.reset();
            resetIMU();
        }

    }
}
