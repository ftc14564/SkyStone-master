package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Autonomous2020")
public class Autonomous2020 extends Teleop2020  {


    public void initFn() {
        teleopInitFn();
        telemetry.addData("Init: Done ", "");
        telemetry.update();


    }


    public void stopWheels() {
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setPower(0);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setPower(0);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setPower(0);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setPower(0);
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double P_TURN_COEFF = 0.05;
    double TURN_THRESHOLD = 1;

    public void gyroTurnREV(double speed, double angle) {

        telemetry.addData("starting gyro turn", "-----");
        telemetry.update();

        //Reverse direction needed for Tetrix motors
        angle = angle*-1;

        Orientation prev_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        int stall_counter = 0;
        while (opModeIsActive() && !isStopRequested() && !onTargetAngleREV(speed, angle, P_TURN_COEFF, 5)) {

            Orientation new_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if( prev_angles.firstAngle == new_angles.firstAngle) {
                stall_counter++;
            }
            else {
                stall_counter = 0;
                prev_angles = new_angles;
            }

            if (stall_counter > 10)
                break;

            telemetry.update();
            idle();
            telemetry.addData("-->", "inside while loop :-(");
            telemetry.update();
        }
        //sleep(100);
        while (opModeIsActive() && !isStopRequested() && !onTargetAngleREV(speed, angle, P_TURN_COEFF / 5, 1)) {

            Orientation new_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if( prev_angles.firstAngle == new_angles.firstAngle) {
                stall_counter++;
            }
            else {
                stall_counter = 0;
                prev_angles = new_angles;
            }

            if (stall_counter > 10)
                break;

            telemetry.update();
            idle();
            telemetry.addData("-->", "inside while loop :-(");
            telemetry.update();
        }

        telemetry.addData("done with gyro turn", "-----");
        telemetry.update();
    }

    boolean onTargetAngleREV(double speed, double angle, double PCoeff, double turnThreshold) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        //determine turm power based on error
        error = getErrorREV(angle);

        if (Math.abs(error) <= turnThreshold) {

            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            stopWheels();
        } else {

            steer = getSteerREV(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            //leftSpeed = -5;
        }
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double weightConstant = 0.8;//this constant will depend on the robot. you need to test experimentally to see which is best

        while (Math.abs(weightConstant * leftSpeed) < 0.25)
            weightConstant *= 1.5;

        motorLeftFront.setPower(weightConstant * leftSpeed);
        motorRightFront.setPower(weightConstant * rightSpeed);
        motorLeftBack.setPower(weightConstant * leftSpeed);
        motorRightBack.setPower(weightConstant * rightSpeed);

        telemetry.addData("Target angle", "%5.2f", angle);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f/%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getErrorREV(double targetAngle) {

        double robotError;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotError = targetAngle - angles.firstAngle;

        while (robotError > 180) robotError -= 360;

        while (robotError <= -180) robotError += 360;

        telemetry.addData("Robot Error", "%5.2f", robotError);
        telemetry.update();

        return robotError;

    }

    public double getSteerREV(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void Rotate(double power, int direction, double angle) {

        //angle -=angle*.35;
        power /= 1.5;

        imu.initialize(parameters);
        if (direction == -1.0) {
            // LEFT
            //Clockwise
            motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            // RIGHT
            //Counter Clockwise
            motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);

        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Robot turning", "Yay!");
        telemetry.update();
        //sleep(150);

        int counter = 0;

        if (direction == 1) {

            // RIGHT
            telemetry.addData("Robot turning right: ", angle);
            telemetry.update();
            //sleep(150);
            while (opModeIsActive() && !isStopRequested() && (Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))) < angle)) {
                telemetry.update();
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();


                double _power = 1.5 * power * ((angle - Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))) / angle);

                if (_power < 0.3) _power = 0.3;

                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            stopWheels();
        } else {

            // LEFT
            telemetry.addData("Robot turning left: ", angle);
            telemetry.update();
            //sleep(150);

            while (opModeIsActive() && !isStopRequested() && (((Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))) < angle)) {
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();

                double _power = 1.5 * power * ((angle - Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))) / angle);
                if (_power < 0.3) _power = 0.3;
                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            stopWheels();

        }
        //stopRobot and change modes back to normal
        telemetry.addData("turned (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void dummyRotate(double power, int direction, double angle){
        telemetry.addData("Rotating :D - Neel HIGH IQ", "ROTATING");
    }
    public void dummyStrafeRight(double power, int direction, float value){
        telemetry.addData("strafing right :D - Neel high iq", "Move right :(");
    }
    public void dummyStrafeLeft(double power, int direction, float value){
        telemetry.addData("strafing left :D - Neel high iq", "Move Left :-(");
    }
    public void dummyStraight(double power, int direction, float value){
        telemetry.addData("Moving straight upwards :D - Neel high iq","MMMMMOVE up por favor");
    }
    public void center (float x, float y, double angle){
        dummyRotate(1,-1,angle);
        if (x<0){
            dummyStrafeLeft(0.5, -1, Math.abs(x));
        }
        if(x>0){
            dummyStrafeRight(0.5,1,Math.abs(x));
        }
        if(y>0){
            dummyStraight(0.75,1,Math.abs(y)-6);
        }
    }

//    public void dummyState1Grab(){
//        testFront = 90;
//        testBack = 90;
//        testTop = 0;
//        //testFrontt.SetPosition
//    }
//    public void dummyState2Open(){
//        testFront = 180;
//        testBack = 90;
//        testTop = 0;
//    }
//    public void dummyState3TurnedGrab(){
//        testFront = 90;
//        testBack = 90;
//        testTop = 90;
//    }
//    public void dummyState4Folded(){
//        testFront = 10;
//        testBack = 10;
//        testTop = 0;
//    }
    public void SlowerRotate(double power, int direction, double angle) {

        //angle -=angle*.35;
        power /= 3;

        imu.initialize(parameters);
        if (direction == -1.0) {
            // LEFT
            //Clockwise
            motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            // RIGHT
            //Counter Clockwise
            motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);

        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Robot turning", "Yay!");
        telemetry.update();
        //sleep(150);

        int counter = 0;

        if (direction == 1) {

            // RIGHT
            telemetry.addData("Robot turning right: ", angle);
            telemetry.update();
            //sleep(150);
            while (opModeIsActive() && !isStopRequested() && (Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))) < angle) &&
                    counter++ < 50) {
                //counter++;
                /*if(System.currentTimeMillis()-startTime > 29500 ){
                    break;
                }*/
                telemetry.update();
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();


                double _power = 1.5 * power * ((angle - Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))) / angle);

                // if(_power < 0.3) _power = 0.3;

                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
            stopWheels();
        } else {

            // LEFT
            telemetry.addData("Robot turning left: ", angle);
            telemetry.update();
            //sleep(150);

            while (opModeIsActive() && !isStopRequested() && (((Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle))))) < angle) &&
                    counter++ < 50) {
                telemetry.addData("turning (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.update();

                double _power = 1.5 * power * ((angle - Math.abs(((Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))))) / angle);
                //  if(_power < 0.3) _power = 0.3;
                motorLeftFront.setPower(_power);
                motorRightBack.setPower(_power);
                motorRightFront.setPower(_power);
                motorLeftBack.setPower(_power);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            stopWheels();

        }
        //stopRobot and change modes back to normal
        telemetry.addData("turned (imu degrees)", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void EncoderStraight(double dist){
        EncoderMoveDist(1, dist,false);
    }

    public void EncoderStrafe(double dist){
        EncoderMoveDist(1, dist,true);
    }

    double P_FWD_COEFF = -0.005;
    double FWD_THRESHOLD = 1;

    public void EncoderMoveDist(double speed, double distance, Boolean strafe) {

        if(!strafe)
            where_x += distance;
        else
            where_y += distance;

        distance *= -1;

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);

        double target_encoder = 0;

        if(!strafe) {
            target_encoder = (distance * TICKS_PER_INCH_STRAIGHT);
            speed *= 0.9;
        }
        else {
            target_encoder = (distance * TICKS_PER_INCH_STRAFE);
        }

        double prev_pos = motorLeftFront.getCurrentPosition();
        int stall_counter = 0;
        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed, target_encoder, P_FWD_COEFF, TICKS_PER_INCH_STRAIGHT, strafe)) {
            if (prev_pos == motorLeftFront.getCurrentPosition()) {
                stall_counter++;
            }
            else {
                stall_counter = 0;
                prev_pos = motorLeftFront.getCurrentPosition();
            }
            if(stall_counter > 10)
                break;

            idle();
        }
        //sleep(100);
//        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed/2, target_encoder, P_FWD_COEFF / 2, TICKS_PER_INCH_STRAIGHT/5, strafe)) {
//            idle();
//        }

    }

    boolean onTargetDist(double speed, double distance, double PCoeff, double distThreshold, Boolean strafe) {
        double error;
        double steer;
        boolean onTarget = false;
        double power;

        //determine  power based on error
        error = getErrorDist(distance, strafe, distThreshold);

        if (Math.abs(error) <= distThreshold) {

            steer = 0.0;
            power = 0.0;
            onTarget = true;
            stopWheels();
        } else {

            steer = getSteerDist(error, PCoeff);
            power = speed * steer;
        }

        double weightConstant = 0.8;//this constant will depend on the robot. you need to test experimentally to see which is best

        while (Math.abs(weightConstant * power) < 0.25)
            weightConstant *= 1.2;

        if (!strafe)
            simpleStraight(weightConstant*power);
        else
            simpleStrafe(weightConstant*power*1.5);


        telemetry.addData("Target dist", "%5.2f", distance);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f", speed);

        return onTarget;
    }

    public double getErrorDist(double targetDist, Boolean strafe, double distThr) {

        double robotError;
        double robotError2;
        double err;
        double curr_encoder = 0;
        if(!strafe)
             curr_encoder = motorLeftFront.getCurrentPosition();
        else
            curr_encoder = 1 * motorLeftFront.getCurrentPosition();

        robotError = targetDist - curr_encoder;

        if (targetDist <0) {
            robotError2 = Math.min(curr_encoder, -1*(distThr*5));
            err = Math.max(robotError, robotError2);
        }
         else {
            robotError2 = Math.max(curr_encoder, (distThr*5));
            err = Math.min(robotError, robotError2);
        }
        telemetry.addData("Robot Error", "%5.2f", robotError);
        telemetry.update();

        return err;

    }

    public double getSteerDist(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void simpleStraight(double power) {
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorRightFront.setPower(power);
        motorLeftBack.setPower(power);

    }

    public void simpleStrafe(double power) {
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorRightFront.setPower(power);
        motorLeftBack.setPower(power);

    }

    public void straight(double power, int direction, double distance) {

//        distance /= 2.25;
//        power /= 1;

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);

/*
        while(motorLeftFront.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }
        */

        telemetry.addData("Straight", "In straight()");
        telemetry.update();


        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);


        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);




        telemetry.addData("Straight", "Still in straight()");
        telemetry.update();

        double avg_encoder = (motorLeftFront.getCurrentPosition() + motorLeftBack.getCurrentPosition() + motorRightBack.getCurrentPosition() + motorRightFront.getCurrentPosition()) / 4;


        while (opModeIsActive() && !isStopRequested() && (avg_encoder < Math.abs(distance))){

            /*if(System.currentTimeMillis()-startTime > 29500 ){
                break;
            }*/
            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.update();
            if (Math.abs(motorRightFront.getCurrentPosition()) < .01 * Math.abs(distance)) {

                motorLeftFront.setPower(direction * .4 * power);
                motorRightBack.setPower(direction * .4 * power);
                motorRightFront.setPower(direction * .4 * power);
                motorLeftBack.setPower(direction * .4 * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .2 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .9 * power);
                motorRightBack.setPower(direction * .9 * power);
                motorRightFront.setPower(direction * .9 * power);
                motorLeftBack.setPower(direction * .9 * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .7 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .8 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .8 * power);
                motorRightBack.setPower(direction * .8 * power);
                motorRightFront.setPower(direction * .8 * power);
                motorLeftBack.setPower(direction * .8 * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .9 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .6 * power);
                motorRightBack.setPower(direction * .6 * power);
                motorRightFront.setPower(direction * .6 * power);
                motorLeftBack.setPower(direction * .6 * power);
            } else {
                motorLeftFront.setPower(direction * .4 * power);
                motorRightBack.setPower(direction * .4 * power);
                motorRightFront.setPower(direction * .4 * power);
                motorLeftBack.setPower(direction * .4 * power);
            }
            avg_encoder = (motorLeftFront.getCurrentPosition() + motorLeftBack.getCurrentPosition() + motorRightBack.getCurrentPosition() + motorRightFront.getCurrentPosition()) / 4;
            sleep(10);
        }
        stopWheels();

        //stopRobot and change modes back to normal
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        //while (motorLeftFront.getCurrentPosition() != 0) {
        //waitOneFullHardwareCycle();
        //}
//        sleep(200);


    }
    public void straight_inch(double power, int direction, double distance){

        double ticks = distance* TICKS_PER_INCH_STRAIGHT;
        straight(power, direction, ticks);
    }

    public void strafe_inch(double power, int direction, double distance){
       double ticks = distance* TICKS_PER_INCH_STRAFE;
        strafe(power, direction, ticks);
    }

    /* direction : +1 is right , -1 is left
       distance: in ticks
     */
    public void strafe(double power, int direction, double distance) {

//        distance /= 2.25;
//        power /= 1.5;

        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);


        telemetry.addData("Strafe", "strafe");
        telemetry.update();


        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(RUN_WITHOUT_ENCODER);


        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        double avg_encoder = (Math.abs(motorLeftFront.getCurrentPosition()) + Math.abs(motorLeftBack.getCurrentPosition()) + Math.abs(motorRightBack.getCurrentPosition()) + Math.abs(motorRightFront.getCurrentPosition())) / 4;


        while (opModeIsActive() && !isStopRequested() &&  avg_encoder < Math.abs(distance)){
//                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
//                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
//                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance))) {


            /*if(System.currentTimeMillis()-startTime > 29500 ){
                break;
            }*/
            sleep(10);
            System.out.println( "Strafe LEFT BACK " + motorLeftBack.getCurrentPosition());
            System.out.println( "Strafe RIGHT BACK " + motorRightBack.getCurrentPosition());
            System.out.println( "Strafe LEFT FRONT " + motorLeftFront.getCurrentPosition());
            System.out.println( "Strafe RIGHT FRONT " + motorRightFront.getCurrentPosition());
            System.out.println( "Strafe Avg Encoder" + avg_encoder);

            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.update();
            if (Math.abs(motorRightFront.getCurrentPosition()) < .1 * Math.abs(distance)) {

                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .8 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .85 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .8 * power);
                motorRightBack.setPower(direction * .8 * power);
                motorRightFront.setPower(direction * .8 * power);
                motorLeftBack.setPower(direction * .8 * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .9 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .7 * power);
                motorRightBack.setPower(direction * .7 * power);
                motorRightFront.setPower(direction * .7 * power);
                motorLeftBack.setPower(direction * .7 * power);
            } else if (Math.abs(motorRightFront.getCurrentPosition()) < .95 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .6 * power);
                motorRightBack.setPower(direction * .6 * power);
                motorRightFront.setPower(direction * .6 * power);
                motorLeftBack.setPower(direction * .6 * power);
            } else {
                motorLeftFront.setPower(direction * .5 * power);
                motorRightBack.setPower(direction * .5 * power);
                motorRightFront.setPower(direction * .5 * power);
                motorLeftBack.setPower(direction * .5 * power);
            }
            avg_encoder = (Math.abs(motorLeftFront.getCurrentPosition()) + Math.abs(motorLeftBack.getCurrentPosition()) + Math.abs(motorRightBack.getCurrentPosition()) + Math.abs(motorRightFront.getCurrentPosition())) / 4;

        }
        stopWheels();

        //stopRobot and change modes back to normal
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        //while (motorLeftFront.getCurrentPosition() != 0) {
        //waitOneFullHardwareCycle();
        //}

        //back to non strafing convention
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // sleep(200);

    }


    public void makeParallelLeft(double distance_from_wall) {
        double sensor_gap = 32.5;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (distanceSensor_lb.getDistance(DistanceUnit.CM) < (distanceSensor_lf.getDistance(DistanceUnit.CM))) {
            double theta;
            //telemetry.addData(" test ", 1);

            double diff1 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_lf.getDistance(DistanceUnit.CM)) - (distanceSensor_lb.getDistance(DistanceUnit.CM));
            double diff = (diff1 + diff2 + diff3) / 3;
            double temp = diff / sensor_gap;
            //telemetry.addData(" test ", 2);
            //telemetry.update();
            theta = Math.asin(temp) * 180 / 3.141592;
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta", theta);
            telemetry.update();
            if (theta > 1) {
                gyroTurnREV(1, angles.firstAngle + theta);
            }

        } else {
            double teta;
            double diff1 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_lb.getDistance(DistanceUnit.CM)) - (distanceSensor_lf.getDistance(DistanceUnit.CM));
            double diff = (diff1 + diff2 + diff3) / 3;
            double temp = diff / sensor_gap;
            teta = Math.asin(temp) * 180 / 3.141592;
            if (teta > 1) {
                gyroTurnREV(1, angles.firstAngle - teta);
            }
            telemetry.addData(" lb_", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf_", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta_", teta);
            telemetry.update();
        }

        double dis = distanceSensor_lb.getDistance(DistanceUnit.CM) / 2.54;
        if (dis < distance_from_wall) {
            EncoderMoveDist(1, (distance_from_wall-dis),true);
        } else if (dis > distance_from_wall) {
            EncoderMoveDist(1,-1*(dis-distance_from_wall),true);
        }

    }

    public void makeParallelRight(double distance_from_wall) {

        double sensor_gap = 32.5;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (distanceSensor_rb.getDistance(DistanceUnit.CM) < (distanceSensor_rf.getDistance(DistanceUnit.CM))) {
            double theta;
            //telemetry.addData(" test ", 1);

            double diff1 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff4 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff5 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff6 = (distanceSensor_rf.getDistance(DistanceUnit.CM)) - (distanceSensor_rb.getDistance(DistanceUnit.CM));
            double diff = (diff1 + diff2 + diff3 + diff4 + diff5 + diff6) / 6;
            double temp = diff / sensor_gap;
            //telemetry.addData(" test ", 2);
            //telemetry.update();
            theta = Math.asin(temp) * 180 / 3.141592;
            telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
            telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta", theta);
            telemetry.update();
            if (theta > 1) {

                gyroTurnREV(1, angles.firstAngle - theta);
            }

        } else {
            double teta;
            double diff1 = (distanceSensor_rb.getDistance(DistanceUnit.CM)) - (distanceSensor_rf.getDistance(DistanceUnit.CM));
            double diff2 = (distanceSensor_rb.getDistance(DistanceUnit.CM)) - (distanceSensor_rf.getDistance(DistanceUnit.CM));
            double diff3 = (distanceSensor_rb.getDistance(DistanceUnit.CM)) - (distanceSensor_rf.getDistance(DistanceUnit.CM));
            double diff = (diff1 + diff2 + diff3) / 3;
            double temp = diff / sensor_gap;
            teta = Math.asin(temp) * 180 / 3.141592;
            if (teta > 1) {
                gyroTurnREV(1, angles.firstAngle + teta);
            }
            telemetry.addData(" rb_", distanceSensor_rb.getDistance(DistanceUnit.CM));
            telemetry.addData(" rf_", distanceSensor_rf.getDistance(DistanceUnit.CM));
            telemetry.addData(" theta_", teta);
            telemetry.update();
        }

        double dis = distanceSensor_rb.getDistance(DistanceUnit.CM) / 2.54;
        if (dis < distance_from_wall) {
            EncoderMoveDist(1,-1*(distance_from_wall-dis),true);
        } else if (dis > distance_from_wall) {
            EncoderMoveDist(1,dis-distance_from_wall,true);
        }
    }
    class extendThread implements Runnable {
        @Override
        public void run() {
            idle();
            sleep(9);
            armExtended(4.675);
            //armExtended(0.7625);


        }
    }


    @Override
    public void runOpMode() {
        runAutonomous(true, false);
    }

    Boolean vuFindBlock(Boolean isBlueSide) {
        boolean blockSeen = false;

        int i = 0;
        int strafeCount = 0;
        int retryCount = 0;
        while (!isStopRequested()) {
            //START
            i++;
            //telemetry.addData("first step called %d times", i);

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }


            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",

                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.update();

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                y = (translation.get(1))/mmPerInch;
                x = (translation.get(0))/mmPerInch;
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                center(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle);

                blockSeen = true;
                telemetry.addData("reached here", "yes");
                break;

            }
            else if (retryCount < 10) {
                retryCount++;
                continue;
            }
            else {
                strafeCount++;
                telemetry.addData("Visible Target", "none");
                if(isBlueSide){
                    EncoderStrafe(8);
                }else{
                    EncoderStrafe(-8);
                }

                retryCount=0;
                if(strafeCount > 1)
                    break;
                //sleep(1000);
            }

            telemetry.update();

        }

        return blockSeen;
    }
    public void grabAndDropBlock_Arm(Boolean isBlueSide, double fwd) {

        grabCollection();
        EncoderStraight(fwd);
        closeGrabber();
        sleep(1200);
        liftInch(0.3);

        //step back
        EncoderStraight(-22);
        gyroTurnREV(1, 90);

        //we always start from camera aligned with end of 5th block (i.e. 5*8 = 40 inch)
        //take to drop zone ( 2 tiles away towards the bridge )
        double dropDist = 0;
        if(isBlueSide){
            dropDist = -1*(where_y) - 35;
        }
        else {
            dropDist = -1* (where_y) + 48;
        }
        EncoderStraight(dropDist);
//        gyroTurnREV(1,0);
//        EncoderStraight(6);

        grabCollection();

//        EncoderStraight(-6);
//        gyroTurnREV(1, 180);
//        foundation.setPosition(0);
//        EncoderMoveDist(0.5, -12,false);
//        foundation.setPosition(1);
//        sleep(500);
//        gyroTurnREV(1, 270);



    }
    public void grabAndDropBlock_Hook(Boolean isBlueSide) {

        EncoderStraight(18);
        sleep(700);

        foundation.setPosition(1);
        sleep(600);
//        if(isBlueSide) {
//            tray_right.setPosition(0);
//            sleep(600);
//        }
//        else{
//            tray_left.setPosition(0);
//            sleep(600);
//        }
        //step back
        EncoderMoveDist(0.6, -18,false);

        //we always start from camera aligned with end of 5th block (i.e. 5*8 = 40 inch)
        //take to drop zone ( 2 tiles away towards the bridge )
        double dropDist = 0;
        if(isBlueSide){
            dropDist = -1*(where_y) - 48;
        }
        else {
            dropDist = -1* (where_y) + 48;
        }
        EncoderStrafe(dropDist);

        //drop the block
//        tray_left.setPosition(0.8);
    }

    public void runAutonomousTray(Boolean isBlueSide) {

        initFn();

        waitForStart();

        //assume that robot is aligned such that it's
        //exactly two tiles away from the bridge towards the tray
        if(isBlueSide)
            EncoderStrafe(6);
        else
            EncoderStrafe(-6);

        foundation.setPosition(0);
        EncoderMoveDist(0.5, -40,false);
        foundation.setPosition(1);
        sleep(500);
        gyroTurnREV(1, 180);
        EncoderMoveDist(0.5, -25,false);
        foundation.setPosition(0);
        EncoderStrafe(-25);
        EncoderStraight(-18);
        EncoderStrafe(-24);
        //park after 12 seconds
//        sleep(12000);


        double parkDist = 0;
        if(isBlueSide){
            parkDist = -46;  //48 + some extra to be over the line
        }
        else {
            parkDist = 46;
        }
        EncoderStrafe(parkDist);


    }


    public void runAutonomous(Boolean isBlueSide, Boolean useArm) {


        initFn();

        waitForStart();

        new Thread(new extendThread()).start();

        //sleep(9000);

        EncoderStraight(12);
        sleep(2000);

        Boolean blockSeen = vuFindBlock(isBlueSide);

        if (blockSeen) {

            double blockDist=0;

            if(!useArm)
                blockDist = y + 11;
            else
                blockDist = y + 4.5;

            EncoderStrafe (blockDist);
        }

        //grab a block (even if it's a random one)
        if(!useArm)
            grabAndDropBlock_Hook(isBlueSide);
         else
            grabAndDropBlock_Arm(isBlueSide, 18);


        //return for second
//        double returnDist = 0;
//        if(isBlueSide){
//            returnDist = 24 + 27;
//        }
//        else {
//            returnDist = -24 - 40;
//        }
//        EncoderStrafe(returnDist);
//
//        if (isBlueSide){
//            makeParallelRight();
//        }
//        else {
//            makeParallelLeft();
//        }
//
//        //see second one ?
//        sleep(300);
//        blockSeen = vuFindBlock(isBlueSide);
//
//        if (blockSeen) {
//            double blockDist=0;
//
//            if(Math.abs(y)>10){
//                if(y>0){
//                    y = y-10;
//                }
//                else {
//                    y = y+10;
//                }
//            }
//            if(!useArm)
//                blockDist = y -3;
//            else
//                blockDist = y + 4.5;
//
//            EncoderStrafe (blockDist);
//        }
//
//        //grab a block (even if it's a random one)
//        if(!useArm)
//            grabAndDropBlock_Hook(isBlueSide);
//        else
//            grabAndDropBlock_Arm(isBlueSide, 18);

        //park (1 tile back towards the bridge)
        double parkDist = 0;
        if(isBlueSide){
            parkDist = -20;
        }
        else {
            parkDist = -28;
        }
        EncoderStraight(parkDist);

    }




}






