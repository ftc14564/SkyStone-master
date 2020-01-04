package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData(" rf", distanceSensor_rf.getDistance(DistanceUnit.CM));
            telemetry.addData(" rb", distanceSensor_rb.getDistance(DistanceUnit.CM));
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
            straight_inch(0.4,1,6);
            }
            if(gamepad2.left_bumper){
                straight_inch(0.4,-1,6);
            }
            if(gamepad2.dpad_up){
                straight_inch(0.6,1,12);
            }
            if(gamepad2.dpad_down){
                straight_inch(0.6,-1,12);
            }
            if(gamepad2.dpad_right){
                straight_inch(0.6,1,24);
            }
            if(gamepad2.dpad_left){
                straight_inch(0.6,-1,24);
            }

            if(gamepad1.dpad_up){
                EncoderMoveDist(1,10,false);
            }
            if(gamepad1.dpad_down){
                EncoderMoveDist(1,-10,false);
            }
            if(gamepad1.dpad_right){
                EncoderMoveDist(1,10,true);
            }
            if(gamepad1.dpad_left){
                EncoderMoveDist(1,-10,true);
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
        }

    }
}