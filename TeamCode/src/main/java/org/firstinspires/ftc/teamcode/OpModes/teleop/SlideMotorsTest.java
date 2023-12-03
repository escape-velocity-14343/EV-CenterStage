package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


@TeleOp
public class SlideMotorsTest extends Robot {

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        slides = new Slides(hardwareMap);
        slides.reset();
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            if (timer.seconds() < 1) {
                slides.tilt(1);
            } else if (timer.seconds() < 3) {
                slides.moveSlides(1);
            } else {
                slides.moveSlides(0);
            }

        }
    }
}
