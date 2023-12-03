package org.firstinspires.ftc.teamcode.OpModes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.auton.AutonBase;

@Autonomous(name="Blue Backstage")
public class BlueBackstage extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initAuton(RunPos.BLUE_BACKSTAGE, ScoreAmount.PRELOADS);
        setAlliance(false);
        setSide(true);
        run();
    }
}
