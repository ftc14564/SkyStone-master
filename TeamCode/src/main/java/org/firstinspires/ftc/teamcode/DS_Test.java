package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@Autonomous(name = "DS_Test")
public class DS_Test extends Autonomous2020 {

    @Override
    public void runOpMode() {

        initFn();

        isBlueSide = false;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            idle();

            telemetry.addData(" bbr", distanceSensor_bbr.getDistance(DistanceUnit.INCH));
            telemetry.addData(" ffl", distanceSensor_ffl.getDistance(DistanceUnit.INCH));
            telemetry.addData(" ffr", distanceSensor_ffr.getDistance(DistanceUnit.INCH));

            telemetry.addData(" rf", distanceSensor_rf.getDistance(DistanceUnit.INCH));
            telemetry.addData(" rb", distanceSensor_rb.getDistance(DistanceUnit.INCH));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.INCH));
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.INCH));
            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle);
            telemetry.addData("Expected X Position", expectedPositionX(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle));
            telemetry.addData("Expected Y Position", expectedPositionY((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle)));
            if (DEBUG) System.out.println("14564dbg 2m rf:" + distanceSensor_rf.getDistance(DistanceUnit.INCH));
            if (DEBUG) System.out.println("14564dbg 2m rb:" + distanceSensor_rb.getDistance(DistanceUnit.INCH));
            if (DEBUG) System.out.println("14564dbg 2m lf:" + distanceSensor_lf.getDistance(DistanceUnit.INCH));
            if (DEBUG) System.out.println("14564dbg 2m lb:" + distanceSensor_lb.getDistance(DistanceUnit.INCH));
            if (DEBUG) System.out.println("14564dbg 2m bbr:" + distanceSensor_bbr.getDistance(DistanceUnit.INCH));

            if (DEBUG) System.out.println("14564dbg 2m ffl:" + distanceSensor_ffl.getDistance(DistanceUnit.INCH));
            if (DEBUG) System.out.println("14564dbg 2m ffr:" + distanceSensor_ffr.getDistance(DistanceUnit.INCH));


            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (DEBUG) System.out.println(" 14564dbg Angle " + angles);

            sleep(10);

            telemetry.update();
        }


    }
    public double expectedPositionX(double heading){
        double sensorValue=0;
        double sensorAngle=0;
        double sensorAngleToCenter=0;
        double sensorToCenter=0;
        if((!isBlueSide)&&((heading>-90)&&(heading<90))) {
            sensorValue = distanceSensor_rb.getDistance(DistanceUnit.INCH);
            sensorAngle = -90;
            sensorAngleToCenter = -135;
            sensorToCenter = 10;
        }
        //-90 is the wall angle
        double sensorToWallX = Math.cos((-90-(heading+sensorAngle))*(Math.PI/180))*sensorValue;
        telemetry.addData("wall", sensorToWallX);
        //Calculates the X distance from the distance sensor to the wall
        double sensorToCenterX = Math.cos((-90-(heading+sensorAngleToCenter))*(Math.PI/180))*sensorToCenter;
        telemetry.addData("center", sensorToCenterX);
        //Calculates the X distance from the distance sensor to the center of the robot
        double xPosition = sensorToWallX + sensorToCenterX;
        //Adds the two values to get the X distance from the center of the robot to the wall
        return xPosition;
    }
    public double expectedPositionY(double heading){
        double sensorValue=0;
        double sensorAngle=0;
        double sensorAngleToCenter=0;
        double sensorToCenter=0;
        if((!isBlueSide)&&((heading>-90)&&(heading<90))) {
            sensorValue = distanceSensor_bbr.getDistance(DistanceUnit.INCH);
            sensorAngle = 180;
            sensorAngleToCenter = -150;
            sensorToCenter = 8.5;
        }
        //-90 is the wall angle
        double sensorToWallY = Math.cos((180-(heading+sensorAngle))*(Math.PI/180))*sensorValue;
        telemetry.addData("wall", sensorToWallY);
        //Calculates the Y distance from the distance sensor to the wall
        double sensorToCenterY = Math.cos((180-(heading+sensorAngleToCenter))*(Math.PI/180))*sensorToCenter;
        telemetry.addData("center", sensorToCenterY);
        //Calculates the Y distance from the distance sensor to the center of the robot
        double yPosition = sensorToWallY + sensorToCenterY;
        //Adds the two values to get the Y distance from the center of the robot to the wall
        return yPosition;
    }
    public double expectedPosition(double heading, boolean isX){
        double sensorValue=0;
        double sensorAngle=0;
        double sensorAngleToCenter=0;
        double sensorToCenter=0;
        if((!isBlueSide)&&((heading>-45)&&(heading<45))) {
            if(isX) {
                sensorValue = distanceSensor_rb.getDistance(DistanceUnit.INCH);
                sensorAngle = -90;
            } else {
                sensorValue = distanceSensor_bbr.getDistance(DistanceUnit.INCH);
                sensorAngle = 180;
                sensorAngleToCenter = -150;
            }
            sensorToCenter = 8.5;
        }
        //-90 is the wall angle
        double sensorToWallY = Math.cos((180-(heading+sensorAngle))*(Math.PI/180))*sensorValue;
        telemetry.addData("wall", sensorToWallY);
        //Calculates the Y distance from the distance sensor to the wall
        double sensorToCenterY = Math.cos((180-(heading+sensorAngleToCenter))*(Math.PI/180))*sensorToCenter;
        telemetry.addData("center", sensorToCenterY);
        //Calculates the Y distance from the distance sensor to the center of the robot
        double yPosition = sensorToWallY + sensorToCenterY;
        //Adds the two values to get the Y distance from the center of the robot to the wall
        return yPosition;
    }
}