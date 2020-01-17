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

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

//            telemetry.addData(" rf", distanceSensor_rf.getDistance(DistanceUnit.CM));
//            telemetry.addData(" rb", distanceSensor_rb.getDistance(DistanceUnit.CM));
            telemetry.addData("rf:", motorRightFront.getCurrentPosition());
            telemetry.addData("lf:", motorLeftFront.getCurrentPosition());
            telemetry.addData("rb:", motorRightBack.getCurrentPosition());
            telemetry.addData("lb:", motorLeftBack.getCurrentPosition());

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
                gyroTurnREV(1, 45);
            }
            if(gamepad2.left_bumper){
                gyroTurnREV(1, -45);
            }
            if(gamepad2.dpad_up){
                gyroTurnREV(1, 90);
            }
            if(gamepad2.dpad_down){
                gyroTurnREV(1, -90);
            }
            if(gamepad2.dpad_right){
                makeParallelRight(4);
            }
            if(gamepad2.dpad_left) {
                makeParallelLeft(27);
            }

            if(gamepad1.dpad_up){
                EncoderMoveDist(1,10,false);
            }
            if(gamepad1.dpad_down){
                EncoderMoveDist(1,-10,false);
            }
            if(gamepad1.dpad_right){
                EncoderMoveDist(1,20,true);
            }
            if(gamepad1.dpad_left){
                EncoderMoveDist(1,-20,true);
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
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

    }
}