package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED_move_foundation_only")
public class SkyStone_Red_Tray extends Autonomous2020 {

    @Override
    public void runOpMode() {

        runAutonomousTrayDS(false);
    }

}