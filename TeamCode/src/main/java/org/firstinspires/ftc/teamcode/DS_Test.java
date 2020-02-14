package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "DS_Test")
public class DS_Test extends Autonomous2020 {

    @Override
    public void runOpMode() {

        initFn();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            idle();

            telemetry.addData(" bbr", distanceSensor_bbr.getDistance(DistanceUnit.CM));
            telemetry.addData(" ffl", distanceSensor_ffl.getDistance(DistanceUnit.CM));
            telemetry.addData(" ffr", distanceSensor_ffr.getDistance(DistanceUnit.CM));

            telemetry.addData(" rf", distanceSensor_rf.getDistance(DistanceUnit.CM));
            telemetry.addData(" rb", distanceSensor_rb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
            if (DEBUG) System.out.println("14564dbg 2m rf:" + distanceSensor_rf.getDistance(DistanceUnit.CM));
            if (DEBUG) System.out.println("14564dbg 2m rb:" + distanceSensor_rb.getDistance(DistanceUnit.CM));
            if (DEBUG) System.out.println("14564dbg 2m lf:" + distanceSensor_lf.getDistance(DistanceUnit.CM));
            if (DEBUG) System.out.println("14564dbg 2m lb:" + distanceSensor_lb.getDistance(DistanceUnit.CM));
            if (DEBUG) System.out.println("14564dbg 2m bbr:" + distanceSensor_bbr.getDistance(DistanceUnit.CM));

            if (DEBUG) System.out.println("14564dbg 2m ffl:" + distanceSensor_ffl.getDistance(DistanceUnit.CM));
            if (DEBUG) System.out.println("14564dbg 2m ffr:" + distanceSensor_ffr.getDistance(DistanceUnit.CM));


            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (DEBUG) System.out.println(" 14564dbg Angle " + angles);

            sleep(10);

            telemetry.update();
        }
    }

}