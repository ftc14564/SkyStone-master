package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE_move_foundation_only")
public class SkyStone_Blue_Tray extends Autonomous2020 {

    @Override
    public void runOpMode() {

        runAutonomousTrayDS(true);
    }

}