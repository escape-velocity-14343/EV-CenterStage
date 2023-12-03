package org.firstinspires.ftc.teamcode.OpModes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Backstage")
public class RedBackstage extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initAuton(RunPos.RED_BACKSTAGE, ScoreAmount.PRELOADS);
        setAlliance(true);
        setSide(true);
        run();
    }
}
