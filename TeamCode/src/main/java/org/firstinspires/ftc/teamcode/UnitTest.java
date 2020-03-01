package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.view.SurfaceView;


@TeleOp (name = "UnitTest")
public class UnitTest extends Autonomous2020 {
//final double TICKS_PER_INCH_STRAIGHT = 89.1;
//final double TICKS_PER_INCH_STRAFE = 115.00;
//orignal ticks per inch strafe was 126


    @Override
    public void runOpMode() {


        initFn();

        isBlueSide = false;

        //int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);




        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {


            telemetry.addData("rb:", motorRightBack.getCurrentPosition());
            telemetry.addData("rf:", motorRightFront.getCurrentPosition());
            telemetry.addData("lb:", motorLeftBack.getCurrentPosition());
            telemetry.addData("lf:", motorLeftFront.getCurrentPosition());
            telemetry.update();

            idle();
            powerReductionFactor = 1;
            isBlueSide = false;
            where_head = FldDirection.Face_Fld_Foundation;



            boolean makeparalleltest = false;
            boolean gyroDirectionTest = false;
            boolean foundationTest = false;
            boolean dsMoverTest = false;
            boolean encoderMove_test = false;
            boolean encoderMoveStrafe_test = false;
            boolean avgTest = false;
            boolean encoderRead = false;
            boolean triagleStright = false;





            if(encoderMoveStrafe_test) {

                isBlueSide = false;
                where_head = FldDirection.Face_Fld_Foundation;

//                sideArmSetStateLeft(SideArmState.GRAB);
//                sleep(2000);
//                sideArmSetStateLeft(SideArmState.GRAB_HOLD_HIGH);
//                sleep(2000);
                if(DEBUG) System.out.println(" Startig 5 fwd !!!!");
                EncoderStrafeGyro(5);
                if(DEBUG) System.out.println(" Startig 5 bwd !!!!");


                EncoderStrafeGyro(-5);


                sleep(1000);
                if(DEBUG) System.out.println(" Startig 10 fwd !!!!");

                EncoderStrafeGyro(10);
                if(DEBUG) System.out.println(" Startig 10 bwd !!!!");

                EncoderStrafeGyro(-10);
                sleep(1000);

                if(DEBUG) System.out.println(" Startig 25 fwd !!!!");

                EncoderStrafeGyro(25);
                if(DEBUG) System.out.println(" Startig 25 bwd !!!!");

                EncoderStrafeGyro(-25);
                sleep(1000);

                if(DEBUG) System.out.println(" Startig 75 fwd !!!!");
                EncoderStrafeGyro(75);
                if(DEBUG) System.out.println(" Startig 75 bwd !!!!");

                EncoderStrafeGyro(-75);

                break;
            }

            if(triagleStright)
            {

                EncoderMoveDist(1,75,false, false, 1);
                sleep(5000);
                break;
            }

            if(encoderMove_test) {

                isBlueSide = false;
                where_head = FldDirection.Face_Fld_Foundation;


                telemetry.update();

                //EncoderMoveDist(1, -25,false, true, 0.35);


//
//                if(DEBUG) System.out.println(" Startig 5 fwd !!!!");
//                EncoderMoveDist(1,4,false, false, 0);
//               // EncoderStraight(4);
//                if(DEBUG) System.out.println(" Startig 5 bwd !!!!");
//                EncoderMoveDist(1,-4, false, false, 0);
//
//                gyroTurnDirection(FldDirection.Face_Fld_Center);
//
//                if(DEBUG) System.out.println(" Startig 5 fwd !!!!");
//                EncoderMoveDist(1,4,false, false, 0);
//                // EncoderStraight(4);
//                if(DEBUG) System.out.println(" Startig 5 bwd !!!!");
//                EncoderMoveDist(1,-4, false, false, 0);
//                //EncoderStraight(-4);




                sleep(1000);
                if(DEBUG) System.out.println(" 14564dbg Startig 5 fwd !!!!");
                EncoderStraightGyro(5);
                if(DEBUG) System.out.println(" 14564dbg Startig 5 bwd !!!!");
                EncoderStraightGyro(-5);


                sleep(1000);
                if(DEBUG) System.out.println(" 14564dbg Startig 10 fwd !!!!");
                EncoderStraightGyro(10);
                if(DEBUG) System.out.println(" 14564dbg Startig 10 bwd !!!!");
                EncoderStraightGyro(-10);

                sleep(1000);
                if(DEBUG) System.out.println(" 14564dbg Startig 25 fwd !!!!");
                EncoderStraightGyro(25);
                if(DEBUG) System.out.println(" 14564dbg Startig 25 bwd !!!!");
                EncoderStraightGyro(-25);

                sleep(1000);
                if(DEBUG) System.out.println(" 14564dbg Startig 75 fwd !!!!");
                EncoderStraightGyro(75);
                if(DEBUG) System.out.println(" 14564dbg Startig 75 bwd !!!!");
                EncoderStraightGyro(-75);

                break;
            }


            if(gyroDirectionTest){
//                isBlueSide = false;
//
//                gyroTurnDirection(FldDirection.Face_Fld_Foundation);
//                gyroTurnDirection(FldDirection.Face_Fld_Center);
//                gyroTurnDirection(FldDirection.Face_Fld_Audience);
//                gyroTurnDirection(FldDirection.Face_Fld_Drivers);
//
//                sleep(2000);
//
//                gyroTurnDirection(FldDirection.Face_Fld_Audience);
//
//                sleep(2000);

                isBlueSide = true;

                gyroTurnDirection(FldDirection.Face_Fld_Center);
                gyroTurnDirection(FldDirection.Face_Fld_Foundation);
                gyroTurnDirection(FldDirection.Face_Fld_Drivers);
                gyroTurnDirection(FldDirection.Face_Fld_Audience);

                break;
            }

            if (makeparalleltest){

              // makeParallelRight(20);
//                makeParallelFront(10);
                makeParallelLeft(20);
//                makeParallelFront(10);
               // sleep(2000);
               //     break;

            }

            if (avgTest){
                double rf = DSRead(distanceSensor_rf);

            }

            if (dsMoverTest){


//                makeParallelRight(28);
                DSMove(0.5, 20, 20, false,false, true, 0, false);
//                System.out.println("BBR " + getMovingAverage(distanceSensor_bbr));
//                System.out.println("FFR " + getMovingAverage(distanceSensor_ffr));
//                System.out.println("FFl " + getMovingAverage(distanceSensor_ffl));
//                System.out.println("LB " + getMovingAverage(distanceSensor_lb));
//                System.out.println("LF " + getMovingAverage(distanceSensor_lf));
//                System.out.println("RB " + getMovingAverage(distanceSensor_rb));
//                System.out.println("RF " + getMovingAverage(distanceSensor_rf));
                DSMove(0.5, 20, 20, false,true, true, 0, false);
//                System.out.println("BBR " + getMovingAverage(distanceSensor_bbr));
//                System.out.println("FFR " + getMovingAverage(distanceSensor_ffr));
//                System.out.println("FFl " + getMovingAverage(distanceSensor_ffl));
//                System.out.println("LB " + getMovingAverage(distanceSensor_lb));
//                System.out.println("LF " + getMovingAverage(distanceSensor_lf));
//                System.out.println("RB " + getMovingAverage(distanceSensor_rb));
//                System.out.println("RF " + getMovingAverage(distanceSensor_rf));//                break;
//                DSMove(1, 20, 20, false,true, true, 0, false);



                //gyroTurnDirection(FldDirection.Face_Fld_Foundation);
               // DSMove(1, 50, 24, false, false, false);
                //gyroTurnDirection(FldDirection.Face_Fld_Audience);
                //DSMove(1, 20, 10, true,false, true);
 //               sideArmSetState(SideArmState.PRE_GRAB);


//                DSMove(1, 21, 32, false,true, true, 0, false);

//                sideArmSetState(SideArmState.GRAB);
//                sleep(600);
//                sideArmSetState(SideArmState.GRAB_HOLD_HIGH);
//                sleep(600);
//
//                DSMove(1, 30, 15, false,false, false,0, false );
//                DSMove(1, 24, 32, false,false, true, 0, false);
//                sideArmSetState(SideArmState.THROW);
//                sleep(300);
//
//
//                DSMove(1, 30, 15, false,true, false, 0, false);
//                sideArmSetState(SideArmState.PRE_GRAB);
//                DSMove(1, 13, 32, false,true, true, 0, false);
//                sleep(1000);
//                DSMove(1, 30, 15, false,false, false, 0.2);
//                DSMove(1, 24, 32, false,false, true, -0.2);
//                sleep(1000);
//                DSMove(1, 30, 15, false,true, false, -0.2);
//                DSMove(1, 5, 32, false,true, true, 0.2);
//                sleep(1000);
//                DSMo ve(1, 30, 15, false,false, false, 0.2);
//                DSMove(1, 24, 32, false,false, true, -0.2);
                //break;
            }

            if (foundationTest){
                isBlueSide = false;

                DS_MoveFoundation();
                break;

            }

//            telemetry.addData(" ffr", distanceSensor_ffr.getDistance(DistanceUnit.CM));
//            telemetry.addData(" ffl", distanceSensor_ffl.getDistance(DistanceUnit.CM));
//            telemetry.addData(" rf", distanceSensor_rf.getDistance(DistanceUnit.CM));
//            telemetry.addData(" rb", distanceSensor_rb.getDistance(DistanceUnit.CM));
//            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
//            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
//            if(DEBUG) System.out.println( "2m rf" + distanceSensor_rf.getDistance(DistanceUnit.CM));
//            if(DEBUG) System.out.println("2m rb" + distanceSensor_rb.getDistance(DistanceUnit.CM));
//            if(DEBUG) System.out.println("2m lf" + distanceSensor_lf.getDistance(DistanceUnit.CM));
//            if(DEBUG) System.out.println("2m lb" +  distanceSensor_lb.getDistance(DistanceUnit.CM));
//            if(DEBUG) System.out.println("2m ffr" + distanceSensor_ffr.getDistance(DistanceUnit.CM));
//            if(DEBUG) System.out.println("2m ffl" +  distanceSensor_ffl.getDistance(DistanceUnit.CM));

//            telemetry.addData("rf:", motorRightFront.getCurrentPosition());
//            telemetry.addData("lf:", motorLeftFront.getCurrentPosition());
//            telemetry.addData("rb:", motorRightBack.getCurrentPosition());
//            telemetry.addData("lb:", motorLeftBack.getCurrentPosition());

            telemetry.update();


            if (gamepad2.x) {
//                armExtended(10);
                grabCollection();
                straight_inch(1, 1, 10);
                closeGrabber();
            }

            if (gamepad1.right_bumper){
//                armExtended(5);
            }

            if (gamepad1.x) {
                liftInch(5.5);
            }
            if (gamepad1.y) {
                liftInch(11);
            }

            if(gamepad2.right_bumper){
                gyroTurnREV(1, 270);
            }
            if(gamepad2.left_bumper){
                gyroTurnREV(1, -45);
            }
            if(gamepad2.dpad_up){
                where_cam_x =0;
                where_cam_y=0;
                isBlueSide = true;
                EncoderGoto(5,5, 1);
                EncoderGoto(0,0, 1);
                gyroTurnDirection(FldDirection.Face_Fld_Foundation);
                EncoderGoto(-5,-5, 1);

            }
            if(gamepad2.dpad_down){
                where_cam_x =0;
                where_cam_y=0;
                isBlueSide = false;
                EncoderGoto(5,5, 1);
                EncoderGoto(0,0, 1);
                gyroTurnDirection(FldDirection.Face_Fld_Foundation);
                EncoderGoto(-5,-5, 1);
            }

            if(gamepad2.dpad_right){
                makeParallelRight(4);
            }
            if(gamepad2.dpad_left) {
                makeParallelLeft(27);
            }

            if(gamepad1.dpad_up){
                EncoderMoveDist(1,10,false, false, 0);
            }
            if(gamepad1.dpad_down){
                EncoderMoveDist(1,-10,false, false, 0);
            }
            if(gamepad1.dpad_right){
                EncoderMoveDist(1,5,true, false, 0);
            }
            if(gamepad1.dpad_left){
                EncoderMoveDist(1,-5,true, false, 0);
            }

            
//            if(gamepad1.dpad_down){
//                strafe_inch(0.8,1,12);
//                telemetry.addData("Encoder value LB", motorLeftBack.getCurrentPosition());
//                telemetry.addData("Encoder value RB", motorRightBack.getCurrentPosition());
//                telemetry.addData("Encoder value LF", motorLeftFront.getCurrentPosition());
//                telemetry.addData("Encoder value RF", motorRightFront.getCurrentPosition());
//                System.out.println( "LEFT BACK" + motorLeftBack.getCurrentPosition());
//                System.out.println("RIGHT BACK" + motorRightBack.getCurrentPosition());
//                System.out.println("RIGHT FRONT" + motorRightFront.getCurrentPosition());
//                System.out.println("LEFT FRONT" + motorLeftFront.getCurrentPosition());
//            }
//            if(gamepad1.dpad_up){
//                strafe_inch(0.8,1,24);
//                telemetry.addData("Encoder value LB", motorLeftBack.getCurrentPosition());
//                telemetry.addData("Encoder value RB", motorRightBack.getCurrentPosition());
//                telemetry.addData("Encoder value LF", motorLeftFront.getCurrentPosition());
//                telemetry.addData("Encoder value RF", motorRightFront.getCurrentPosition());
//                System.out.println( "LEFT BACK" + motorLeftBack.getCurrentPosition());
//                System.out.println("RIGHT BACK" + motorRightBack.getCurrentPosition());
//                System.out.println("RIGHT FRONT" + motorRightFront.getCurrentPosition());
//                System.out.println("LEFT FRONT" + motorLeftFront.getCurrentPosition());
//            }
//            if(gamepad1.dpad_right){
//                strafe_inch(0.8,-1,12);
//                telemetry.addData("Encoder value LB", motorLeftBack.getCurrentPosition());
//                telemetry.addData("Encoder value RB", motorRightBack.getCurrentPosition());
//                telemetry.addData("Encoder value LF", motorLeftFront.getCurrentPosition());
//                telemetry.addData("Encoder value RF", motorRightFront.getCurrentPosition());
//                System.out.println( "LEFT BACK" + motorLeftBack.getCurrentPosition());
//                System.out.println("RIGHT BACK" + motorRightBack.getCurrentPosition());
//                System.out.println("RIGHT FRONT" + motorRightFront.getCurrentPosition());
//                System.out.println("LEFT FRONT" + motorLeftFront.getCurrentPosition());
//            }
//            if(gamepad1.dpad_left){
//                strafe_inch(0.8,-1,24);
//                telemetry.addData("Encoder value LB", motorLeftBack.getCurrentPosition());
//                telemetry.addData("Encoder value RB", motorRightBack.getCurrentPosition());
//                telemetry.addData("Encoder value LF", motorLeftFront.getCurrentPosition());
//                telemetry.addData("Encoder value RF", motorRightFront.getCurrentPosition());
//                System.out.println( "LEFT BACK" + motorLeftBack.getCurrentPosition());
//                System.out.println("RIGHT BACK" + motorRightBack.getCurrentPosition());
//                System.out.println("RIGHT FRONT" + motorRightFront.getCurrentPosition());
//                System.out.println("LEFT FRONT" + motorLeftFront.getCurrentPosition());
//            }

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
//            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                    (int) (sensorColor.green() * SCALE_FACTOR),
//                    (int) (sensorColor.blue() * SCALE_FACTOR),
//                    hsvValues);
//
//            // send the info back to driver station using telemetry function.
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", sensorColor.alpha());
//            telemetry.addData("Red  ", sensorColor.red());
//            telemetry.addData("Green", sensorColor.green());
//            telemetry.addData("Blue ", sensorColor.blue());
//            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//                }
//            });
//
//            telemetry.update();
        }

        if(DEBUG) System.out.println("14564dbg Stopped ");


    }
}