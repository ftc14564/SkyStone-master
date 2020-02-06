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
import java.util.Base64;
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

        if(DEBUG)  System.out.println("14564dbg gyroTurnREV: " + angle);

        Orientation prev_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        int stall_counter = 0;
        if(Math.abs(prev_angles.firstAngle - angle) > 10) {
            while (opModeIsActive() && !isStopRequested() && !onTargetAngleREV(speed, angle, P_TURN_COEFF, 5)) {

                Orientation new_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (prev_angles.firstAngle == new_angles.firstAngle) {
                    stall_counter++;
                } else {
                    stall_counter = 0;
                    prev_angles = new_angles;
                }

                if (stall_counter > 10)
                    break;

                telemetry.addData("-->", "inside while loop :-(");
                telemetry.update();
                idle();
            }
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

            telemetry.addData("-->", "inside while loop :-(");
            telemetry.update();
            idle();
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
        if(DEBUG) System.out.println("14564dbg gyroTurnREV power: " + (weightConstant * leftSpeed));

        telemetry.addData("Target angle", "%5.2f", angle);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f/%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getErrorREV(double targetAngle) {

        double robotError;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotError = targetAngle - angles.firstAngle;

        while (robotError > 180) robotError -= 360;

        while (robotError <= -180) robotError += 360;

//        telemetry.addData("Robot Error", "%5.2f", robotError);
//        telemetry.update();
        if(DEBUG)  System.out.println("14564dbg gyroTurnREV: Error " + robotError + " angles " + angles) ;


        return robotError;

    }

    public double getSteerREV(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroTurnDirection(FldDirection dir) {
        where_head = dir;


        if(isBlueSide) {
            if(dir == FldDirection.Face_Fld_Center)
                gyroTurnREV(1,0);
            if(dir == FldDirection.Face_Fld_Foundation)
                gyroTurnREV(1,90);
            if(dir == FldDirection.Face_Fld_Audience)
                gyroTurnREV(1,270);
            if(dir == FldDirection.Face_Fld_Drivers)
                gyroTurnREV(1,180);
            if(dir == FldDirection.Face_Fld_Driver_Diag)
                gyroTurnREV(1,225);
        }
        else {
            if(dir == FldDirection.Face_Fld_Center)
                gyroTurnREV(1,0);
            if(dir == FldDirection.Face_Fld_Foundation)
                gyroTurnREV(1,270);
            if(dir == FldDirection.Face_Fld_Audience)
                gyroTurnREV(1,90);
            if(dir == FldDirection.Face_Fld_Drivers)
                gyroTurnREV(1,180);
            if(dir == FldDirection.Face_Fld_Driver_Diag)
                gyroTurnREV(1,135);
        }

    }

    public void EncoderStraight(double dist){
        EncoderMoveDist(1, dist,false);
    }

    public void EncoderGoto(double x, double y, double power) {
        if (!isBlueSide) {
            if (where_head == FldDirection.Face_Fld_Center) {
                EncoderMoveDist(power, y - where_y, false);
                EncoderMoveDist(power, x - where_x, true);
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                EncoderMoveDist(power, x - where_x, false);
                EncoderMoveDist(power, -1 * (y - where_y), true);
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                EncoderMoveDist(power, -1 * (y - where_y), false);
                EncoderMoveDist(power, -1 * (x - where_x), true);
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                EncoderMoveDist(power, -1 * (x - where_x), false);
                EncoderMoveDist(power, y - where_y, true);
            }
        }
        if (isBlueSide) {
            if (where_head == FldDirection.Face_Fld_Center) {
                EncoderMoveDist(power, y - where_y, false);
                EncoderMoveDist(power, -1*(x - where_x), true);
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                EncoderMoveDist(power, -1*(x - where_x), false);
                EncoderMoveDist(power, -1 * (y - where_y), true);
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                EncoderMoveDist(power, -1 * (y - where_y), false);
                EncoderMoveDist(power, x - where_x, true);
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                EncoderMoveDist(power, x - where_x, false);
                EncoderMoveDist(power, y - where_y, true);
            }
        }
    }

    public void EncoderStrafe(double dist){
        EncoderMoveDist(1, dist,true);
    }

    double P_FWD_COEFF = -0.005;
    double FWD_THRESHOLD = 1;

    public void EncoderMoveDist(double speed, double distance, Boolean strafe) {

        if(DEBUG) System.out.println("14564dbg EncoderMoveDist: " + distance + "Starfe: " + strafe);

        if (!isBlueSide) {

            if (where_head == FldDirection.Face_Fld_Center) {
                if (strafe)
                    where_x += distance;
                else
                    where_y += distance;
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                if (strafe)
                    where_y -= distance;
                else
                    where_x += distance;
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                if (strafe)
                    where_x -= distance;
                else
                    where_y -= distance;
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                if (strafe)
                    where_y += distance;
                else
                    where_x -= distance;
            }
        }
        else {

            if (where_head == FldDirection.Face_Fld_Center) {
                if (strafe)
                    where_x -= distance;
                else
                    where_y += distance;
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                if (strafe)
                    where_y -= distance;
                else
                    where_x -= distance;
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                if (strafe)
                    where_x += distance;
                else
                    where_y -= distance;
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                if (strafe)
                    where_y += distance;
                else
                    where_x += distance;
            }
        }
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

        if(DEBUG) System.out.println("14564dbg simpleStraight power: " + power);

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

        if(DEBUG) System.out.println("14564dbg simpleStrafe power: " + power);


    }

    //Distance Sensor Move
    double P_DS_COEFF = 0.1;
    double P_DS_TURN_COEFF = 0.1;
    double P_DS_ERR_MARGIN = 2;

    public void DSMove(double speed, double fwd_dist, double side_dist, Boolean useLeftSide, Boolean driveReverse, Boolean brakeStop, double turnDelta) {

        if (DEBUG) System.out.println("14564dbg DSMove: fwd " + fwd_dist + " side " + side_dist +
                                      " useLeftSide " + useLeftSide + " reverse " + driveReverse + " brake " + brakeStop);

        Rev2mDistanceSensor ds1;
        Rev2mDistanceSensor ds2;
        Rev2mDistanceSensor ff1;

        //if d1 is larger than d2 we want to go clockwise
        if (useLeftSide) {
            ds1 = distanceSensor_lb;
            ds2 = distanceSensor_lf;
        } else {
            ds1 = distanceSensor_rf;
            ds2 = distanceSensor_rb;
        }

        if(driveReverse) {
            ff1 = distanceSensor_bbr;
        }
        else {
            ff1 = distanceSensor_ffl;
        }

        double fd1 = DSRead(ff1);
        double sd1 = DSRead(ds1);
        double sd2 = DSRead(ds2);


        double counter = 0;

        int stall_counter = 0;
        double prev_pos = motorLeftFront.getCurrentPosition();


        while (opModeIsActive() && !isStopRequested()) {

            idle();

            counter++;

            double fwd_error = fd1 - fwd_dist ;

            if((!brakeStop) && (fwd_error < 0))
                break;

            double side_error;
            if (useLeftSide)
                side_error = -1 * (sd1 - side_dist);
            else
                side_error = sd1 - side_dist;

            if ((Math.abs(fwd_error) < P_DS_ERR_MARGIN) && (Math.abs(side_error) < P_DS_ERR_MARGIN)) {
                if(brakeStop) {
                    stopWheels();
                }
                break;
            }

            double fwd_pwr = Range.clip(fwd_error * P_DS_COEFF*speed, -1, 1);
            double side_pwr = Range.clip(side_error * P_DS_COEFF*speed, -1, 1);

            double turn_pwr = (sd1-sd2) * P_DS_TURN_COEFF + turnDelta;

            if(driveReverse) {
                fwd_pwr = -1 * fwd_pwr;
            }

            while(Math.abs(fwd_pwr) < 0.2) {
                fwd_pwr *= 1.2;
            }

            if (DEBUG) System.out.println("14564dbg DSMove: fwd_err " + fwd_error + " side_err " + side_error + " turn " + turn_pwr);

            vectorCombine(side_pwr, fwd_pwr, turn_pwr);

            fd1 = DSRead(ff1);

            if (DEBUG) System.out.println("14564dbg DSMove: fd1  " + fd1);

            sd1 = DSRead(ds1);
            if((counter%3) == 1)
                sd2 = DSRead(ds2);

            double curr_pos = motorLeftFront.getCurrentPosition();
            if (prev_pos == curr_pos) {
                stall_counter++;
            } else {
                stall_counter = 0;
                prev_pos = curr_pos;
            }

            if (stall_counter > 10) {
                if (DEBUG) System.out.println("14564dbg DSMove: Stall detected");
                break;
            }
        }
    }

    public void DS_MoveFoundation(){
        Rev2mDistanceSensor bb1;



        foundation.setPosition(FOUNDATION_UP);
        gyroTurnDirection(FldDirection.Face_Fld_Drivers);
        Boolean useLeftSide = true;
        if(isBlueSide) {
            useLeftSide = false;
        }

        DSMove(1, 24, 20, useLeftSide,false, true, 0);

        bb1 = distanceSensor_bbr;

        double distToFoundation = DSRead(bb1);
        if (distToFoundation > 20){
            distToFoundation = 20;
        }

        EncoderMoveDist(0.5, -(distToFoundation+5), false);
        //grabbing foundation
        foundation.setPosition(FOUNDATION_DOWN);
        sleep(600);
        //TO DO : CALL AGAIN IF MISSED

        EncoderMoveDist(0.8, 15,false);
        gyroTurnDirection(FldDirection.Face_Fld_Driver_Diag);
        EncoderStraight(20);

        gyroTurnDirection(FldDirection.Face_Fld_Audience);
        foundation.setPosition(FOUNDATION_UP);
        EncoderMoveDist(0.75, -25,false);

        //DSMove(0.75, 10, 26, useLeftSide, true, true);

        EncoderMoveDist(1, 25,false);
        if(isBlueSide)
            makeParallelRight(26);
        else
            makeParallelLeft(26);

        EncoderMoveDist(0.75, 20,false);


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
            if(DEBUG) {
                System.out.println("Strafe LEFT BACK " + motorLeftBack.getCurrentPosition());
                System.out.println("Strafe RIGHT BACK " + motorRightBack.getCurrentPosition());
                System.out.println("Strafe LEFT FRONT " + motorLeftFront.getCurrentPosition());
                System.out.println("Strafe RIGHT FRONT " + motorRightFront.getCurrentPosition());
                System.out.println("Strafe Avg Encoder" + avg_encoder);
            }
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

    double DSRead(Rev2mDistanceSensor ds) {

        if(opModeIsActive() && !isStopRequested()) {
           return ds.getDistance(DistanceUnit.INCH);
        } else
            return 0 ;
    }

    public void makeParallelLeft(double distance_from_wall) {
        double sensor_gap = 16.5;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double dis1 = DSRead(distanceSensor_lb);
        double dis2 = DSRead(distanceSensor_lf);
        double dist = (dis1 + dis2)/2;

        if(DEBUG) System.out.println("14564dbg MP L: Angle " + angles + " lb " + dis1 + " lf " + dis2);

        if((dist -  distance_from_wall) < 75) {
            if (DSRead(distanceSensor_lb) < DSRead(distanceSensor_lf)) {
                double theta;
                //telemetry.addData(" test ", 1);

                double diff1 = DSRead(distanceSensor_lf) - DSRead(distanceSensor_lb);
                double diff2 = DSRead(distanceSensor_lf) - DSRead(distanceSensor_lb);
                double diff3 = DSRead(distanceSensor_lf) - DSRead(distanceSensor_lb);
                double diff = (diff1 + diff2 + diff3) / 3;
                double temp = diff / sensor_gap;
                //telemetry.addData(" test ", 2);
                //telemetry.update();
                theta = Math.asin(temp) * 180 / 3.141592;
//                telemetry.addData(" lb", distanceSensor_lb.getDistance(DistanceUnit.CM));
//                telemetry.addData(" lf", distanceSensor_lf.getDistance(DistanceUnit.CM));
//                telemetry.addData(" theta", theta);
//                telemetry.update();
                if(DEBUG) System.out.println("14564dbg MP L: theta" + theta);

                if (theta > 1) {
                    gyroTurnREV(1, angles.firstAngle + theta);
                }

            } else {
                double teta;
                double diff1 = DSRead(distanceSensor_lb) - DSRead(distanceSensor_lf);
                double diff2 = DSRead(distanceSensor_lb) - DSRead(distanceSensor_lf);
                double diff3 = DSRead(distanceSensor_lb) - DSRead(distanceSensor_lf);
                double diff = (diff1 + diff2 + diff3) / 3;
                double temp = diff / sensor_gap;
                teta = Math.asin(temp) * 180 / 3.141592;
                if(DEBUG) System.out.println("14564dbg MP L: theta" + teta);

                if (teta > 1) {
                    gyroTurnREV(1, angles.firstAngle - teta);
                }
//                telemetry.addData(" lb_", distanceSensor_lb.getDistance(DistanceUnit.CM));
//                telemetry.addData(" lf_", distanceSensor_lf.getDistance(DistanceUnit.CM));
//                telemetry.addData(" theta_", teta);
//                telemetry.update();
            }
            dis1 = DSRead(distanceSensor_lb);
            dis2 = DSRead(distanceSensor_lf);
            dist = (dis1 + dis2)/2;
        }
        if(distance_from_wall > 0) {
            if (dist < distance_from_wall) {
                EncoderMoveDist(0.8, (distance_from_wall - dist), true);
            } else if (dist > distance_from_wall) {
                if ((dist - distance_from_wall) > 24) {
                    simpleStrafe(-0.8);
                    makeParallelLeft(distance_from_wall);
                } else
                    EncoderMoveDist(0.8, -1 * (dist - distance_from_wall), true);
            }
        }
    }

    public void makeParallelRight(double distance_from_wall) {

        double sensor_gap = 13.26;
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double dis1 = DSRead(distanceSensor_rb);
        double dis2 = DSRead(distanceSensor_rf);
        double dist = (dis1 + dis2)/2;

        if(DEBUG) System.out.println("14564dbg MP R: Angle " + angles + " rb " + dis1 + " rf " + dis2);

        if((dist -  distance_from_wall) < 75) {
            if (DSRead(distanceSensor_rb) < DSRead((distanceSensor_rf))) {
                double theta;
                //telemetry.addData(" test ", 1);

                double diff1 = DSRead(distanceSensor_rf) - DSRead(distanceSensor_rb);
                double diff2 = DSRead(distanceSensor_rf) - DSRead(distanceSensor_rb);
                double diff3 = DSRead(distanceSensor_rf) - DSRead(distanceSensor_rb);

                double diff = (diff1 + diff2 + diff3 ) / 3;
                double temp = diff / sensor_gap;
                //telemetry.addData(" test ", 2);
                //telemetry.update();
                theta = Math.asin(temp) * 180 / 3.141592;
//                telemetry.addData(" lb", distanceSensor_rb.getDistance(DistanceUnit.CM));
//                telemetry.addData(" lf", distanceSensor_rf.getDistance(DistanceUnit.CM));
//                telemetry.addData(" theta", theta);
//                telemetry.update();
                if (theta > 1) {

                    gyroTurnREV(1, angles.firstAngle - theta);
                }

            } else {
                double teta;
                double diff1 = DSRead(distanceSensor_rb) - DSRead(distanceSensor_rf);
                double diff2 = DSRead(distanceSensor_rb) - DSRead(distanceSensor_rf);
                double diff3 = DSRead(distanceSensor_rb) - DSRead(distanceSensor_rf);
                double diff = (diff1 + diff2 + diff3) / 3;
                double temp = diff / sensor_gap;
                teta = Math.asin(temp) * 180 / 3.141592;
                if (teta > 1) {
                    gyroTurnREV(1, angles.firstAngle + teta);
                }
//                telemetry.addData(" rb_", distanceSensor_rb.getDistance(DistanceUnit.CM));
//                telemetry.addData(" rf_", distanceSensor_rf.getDistance(DistanceUnit.CM));
//                telemetry.addData(" theta_", teta);
//                telemetry.update();
            }
            dis1 = DSRead(distanceSensor_rb);
            dis2 = DSRead(distanceSensor_rf);
            dist = (dis1 + dis2)/2;
        }
        if(distance_from_wall > 0) {
            if (dist < distance_from_wall) {
                EncoderMoveDist(0.8, -1 * (distance_from_wall - dist), true);
            } else if (dist > distance_from_wall) {
                if ((dist - distance_from_wall) > 24) {
                    simpleStrafe(0.8);
                    makeParallelRight(distance_from_wall);
                } else
                    EncoderMoveDist(0.8, dist - distance_from_wall, true);
            }
        }
    }

//    public void makeParallelFront(double distance_from_wall) {
//
//        double sensor_gap = 13.97;
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double dis1 = DSRead(distanceSensor_ffl);
//        double dis2 = DSRead(distanceSensor_ffr);
//        double dist = (dis1 + dis2)/2;
//        if((dist -  distance_from_wall) < 75) {
//            if (DSRead(distanceSensor_ffl) < DSRead(distanceSensor_ffr)) {
//                double theta;
//                //telemetry.addData(" test ", 1);
//
//                double diff1 = DSRead(distanceSensor_ffr) - DSRead(distanceSensor_ffl);
//                double diff2 = DSRead(distanceSensor_ffr) - DSRead(distanceSensor_ffl);
//                double diff3 = DSRead(distanceSensor_ffr) - DSRead(distanceSensor_ffl);
//
//                double diff = (diff1 + diff2 + diff3 ) / 3;
//                double temp = diff / sensor_gap;
//                //telemetry.addData(" test ", 2);
//                //telemetry.update();
//                theta = Math.asin(temp) * 180 / 3.141592;
////                telemetry.addData(" ffl", distanceSensor_ffl.getDistance(DistanceUnit.CM));
////                telemetry.addData(" ffr", distanceSensor_ffr.getDistance(DistanceUnit.CM));
////                telemetry.addData(" theta", theta);
////                telemetry.update();
//                if (theta > 1) {
//
//                    gyroTurnREV(1, angles.firstAngle + theta);
//                }
//
//            } else {
//                double teta;
//                double diff1 = DSRead(distanceSensor_ffl) - DSRead(distanceSensor_ffr);
//                double diff2 = DSRead(distanceSensor_ffl) - DSRead(distanceSensor_ffr);
//                double diff3 = DSRead(distanceSensor_ffl) - DSRead(distanceSensor_ffr);
//                double diff = (diff1 + diff2 + diff3) / 3;
//                double temp = diff / sensor_gap;
//                teta = Math.asin(temp) * 180 / 3.141592;
//                if (teta > 1) {
//                    gyroTurnREV(1, angles.firstAngle - teta);
//                }
////                telemetry.addData(" ffl_", distanceSensor_ffl.getDistance(DistanceUnit.CM));
////                telemetry.addData(" ffr_", distanceSensor_ffr.getDistance(DistanceUnit.CM));
////                telemetry.addData(" theta_", teta);
////                telemetry.update();
//            }
//            dis1 = DSRead(distanceSensor_ffl);
//            dis2 = DSRead(distanceSensor_ffr);
//            dist = (dis1 + dis2)/2;
//        }
//        if(distance_from_wall > 0) {
//            if (dist < distance_from_wall) {
//                EncoderMoveDist(0.8, -1 * (distance_from_wall - dist), false);
//            } else if (dist > distance_from_wall) {
//                if ((dist - distance_from_wall) > 24) {
//                    simpleStrafe(0.8);
//                    makeParallelFront(distance_from_wall);
//                } else
//                    EncoderMoveDist(0.8, dist - distance_from_wall, false);
//            }
//        }
//    }

    class extendThread implements Runnable {
        @Override
        public void run() {
            idle();
            sleep(9);
            armExtended(6.5);
            grabCollection();
            armExtended(3);

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
                vu_x = (translation.get(1))/mmPerInch;
                vu_y = (translation.get(0))/mmPerInch;
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                blockSeen = true;
                telemetry.addData("reached here", "yes");
                if(DEBUG) System.out.println("14564dbg seen retry count: " + retryCount);

                break;

            }
            else if (retryCount < 5) {
                retryCount++;
                if(DEBUG)  System.out.println("14564dbg retry count: " + retryCount);
                sleep(200);
                continue;
            }
            else {
                strafeCount++;
                telemetry.addData("No Visible Target", "none");
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
    public void grabAndDropBlock_Arm(Boolean doFoundation) {


        EncoderGoto(where_x, 56, 0.8);
        closeGrabber();
        sleep(800);
        setLiftPosition(0.3);

        //step back
        EncoderGoto(where_x, 38, 0.8);

        //turn
        gyroTurnDirection(FldDirection.Face_Fld_Foundation);

        if(doFoundation) {
            //go  to foundation
            EncoderGoto(122, where_y, 1);



            setLiftPosition(5*REV_CORE_HEX_TICKS_PER_INCH);
            gyroTurnDirection(FldDirection.Face_Fld_Center);
            EncoderGoto(where_x, 52, 0.8);

            //drop block
            grabCollection();

            sleep(200);
            EncoderGoto(where_x, 46, 0.8);
            gyroTurnDirection(FldDirection.Face_Fld_Drivers);
            foundation.setPosition(0);
            EncoderGoto(where_x, 54, 0.4);  //slow back into tray
            foundation.setPosition(0.7);
            sleep(200);
            EncoderGoto(where_x, 20, 0.7);
            gyroTurnDirection(FldDirection.Face_Fld_Audience);
            EncoderGoto(where_x+6, where_y, 1);

            foundation.setPosition(0);

            setLiftPosition(0);

            if (isBlueSide) {
                makeParallelRight(30);
            } else {
                makeParallelLeft(30);

            }
        }
        else {
            EncoderGoto(96, where_y, 1);
            //drop block
            grabCollection();
        }


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
            dropDist = -1*(where_x) - 48;
        }
        else {
            dropDist = -1* (where_x) + 48;
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
        foundation.setPosition(0.7);
        sleep(500);
        gyroTurnREV(1, 180);
        EncoderMoveDist(0.5, -25,false);
        foundation.setPosition(0);
        EncoderStrafe(-25);
        EncoderStraight(-18);
//        EncoderStrafe(-24);
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


    public void runAutonomous(Boolean isBlue, Boolean doFoundation) {


        USE_VUFORIA = true;

        isBlueSide = isBlue;

        if(!isBlueSide){
            where_y = 18;
            where_x = 36;
        }
        if(isBlueSide){
            where_y = 18;
            where_x = 44;
        }



        Boolean returnForSecond = true;
        Boolean doParking = true;
        Boolean testVuOnly = false;

        initFn();

        waitForStart();

        if(!testVuOnly) {
            new Thread(new extendThread()).start();
        }

        EncoderGoto(where_x, 30, 1);


        Boolean blockSeen = vuFindBlock(isBlueSide);


        if (blockSeen) {

            double blockDist=0;


            blockDist = vu_x + CAM_OFFSET;

            EncoderStrafe (blockDist);
        }

        double firstSkyStone_X = where_x;

        if(!testVuOnly) {
            //grab a block (even if it's a random one)

            grabAndDropBlock_Arm(doFoundation);
        }




        if (!doFoundation && returnForSecond) {

            EncoderGoto(60, where_y, 1);

            gyroTurnDirection(FldDirection.Face_Fld_Foundation);


//            if (isBlueSide) {
//                makeParallelLeft(20);
//            } else {
//                makeParallelRight(20);
//            }

            if(firstSkyStone_X < 32) {
                EncoderGoto(20, where_y, 1);
                gyroTurnDirection(FldDirection.Face_Fld_Center);
            }
            else {
                EncoderGoto(firstSkyStone_X - 18, where_y, 1);

                gyroTurnDirection(FldDirection.Face_Fld_Center);
                blockSeen = vuFindBlock(isBlueSide);

                if (blockSeen) {
                    double blockDist=0;
                    blockDist = vu_x + CAM_OFFSET;
                    EncoderStrafe (blockDist);
                }
            }

            EncoderGoto(where_x, where_y + 14, 0.8);
            closeGrabber();
            sleep(800);
            setLiftPosition(0.3);
            EncoderGoto(where_x, where_y - 10, 0.8);

            gyroTurnDirection(FldDirection.Face_Fld_Foundation);

            EncoderGoto(96, where_y, 1);
            //drop block
            grabCollection();
        }


        //park (1 tile back towards the bridge)
        if(doParking) {
            EncoderGoto(72, 40, 1);
        }
    }




}






