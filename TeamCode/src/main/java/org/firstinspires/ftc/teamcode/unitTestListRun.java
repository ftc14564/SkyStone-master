package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "unitTestListRun")
public class unitTestListRun extends Autonomous2020 {


    @Override
    public void runOpMode() {


        initFn();


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            EncoderStraight(12);
            EncoderStraight(-12);
            EncoderStrafe(12);
            EncoderStrafe(-12);
            grab_front.setPosition(0.8);
            grab_back.setPosition(0.5);


        }

    }
}