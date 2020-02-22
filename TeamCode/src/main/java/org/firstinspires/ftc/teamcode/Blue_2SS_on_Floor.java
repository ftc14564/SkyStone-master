package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue_2SS_on_Floor")
public class Blue_2SS_on_Floor extends Autonomous2020 {

    @Override
    public void runOpMode() {

        runAutonomousDS(true, false, true);
    }

}