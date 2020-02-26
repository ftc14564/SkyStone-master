package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


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

                if (stall_counter > 3)
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

            if (stall_counter > 3)
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


    double get_where_angle(FldDirection head)
    {
        double angle = 0;
        if(isBlueSide) {
            if(head == FldDirection.Face_Fld_Audience)
                angle = 0;
            if(head == FldDirection.Face_Fld_Center)
                angle = 90;
            if(head == FldDirection.Face_Fld_Drivers)
                angle = 270;
            if(head == FldDirection.Face_Fld_Foundation)
                angle = 180;
            if(head == FldDirection.Face_Fld_Center_Foundation)
                angle = 135;
        }
        else {
            if(head == FldDirection.Face_Fld_Foundation)
                angle = 0;
            if(head == FldDirection.Face_Fld_Drivers)
                angle = 270;
            if(head == FldDirection.Face_Fld_Center)
                angle = 90;
            if(head == FldDirection.Face_Fld_Audience)
                angle = 180;
            if(head == FldDirection.Face_Fld_Center_Foundation)
                angle = 45;
        }

        return angle;
    }

    public void gyroTurnDirection(FldDirection dir) {
        where_head = dir;

        double angle = get_where_angle(dir);

                gyroTurnREV(1,angle);
    }

    public void EncoderStraight(double dist){
        EncoderMoveDist(1, dist,false, false, 0);
    }

    public void EncoderStraightGyro(double dist){
        EncoderMoveDist(1, dist,false, true, 0);
    }

    public void EncoderGoto(double x, double y, double power) {
        if (!isBlueSide) {
            if (where_head == FldDirection.Face_Fld_Center) {
                EncoderMoveDist(power, y - where_cam_y, false, false, 0);
                EncoderMoveDist(power, x - where_cam_x, true, false, 0);
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                EncoderMoveDist(power, x - where_cam_x, false, false, 0);
                EncoderMoveDist(power, -1 * (y - where_cam_y), true, false, 0);
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                EncoderMoveDist(power, -1 * (y - where_cam_y), false, false, 0);
                EncoderMoveDist(power, -1 * (x - where_cam_x), true, false, 0);
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                EncoderMoveDist(power, -1 * (x - where_cam_x), false, false, 0);
                EncoderMoveDist(power, y - where_cam_y, true, false, 0);
            }
        }
        if (isBlueSide) {
            if (where_head == FldDirection.Face_Fld_Center) {
                EncoderMoveDist(power, y - where_cam_y, false, false, 0);
                EncoderMoveDist(power, -1*(x - where_cam_x), true, false, 0);
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                EncoderMoveDist(power, -1*(x - where_cam_x), false, false, 0);
                EncoderMoveDist(power, -1 * (y - where_cam_y), true, false, 0);
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                EncoderMoveDist(power, -1 * (y - where_cam_y), false, false, 0);
                EncoderMoveDist(power, x - where_cam_x, true, false, 0);
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                EncoderMoveDist(power, x - where_cam_x, false, false, 0);
                EncoderMoveDist(power, y - where_cam_y, true, false, 0);
            }
        }
    }

    public void EncoderStrafe(double dist){
        EncoderMoveDist(1, dist,true, false, 0);
    }

    public void EncoderStrafeGyro(double dist){
        EncoderMoveDist(1, dist,true, true, 0);
    }

    double P_FWD_COEFF = -0.003;
    double FWD_THRESHOLD = 1;

    public void EncoderMoveDist(double speed, double distance, Boolean strafe, Boolean gyroCorrection, double sideWays) {

        if(DEBUG) System.out.println("14564dbg EncoderMoveDist: " + distance + "Starfe: " + strafe);

        if (!isBlueSide) {

            if (where_head == FldDirection.Face_Fld_Center) {
                if (strafe)
                    where_cam_x += distance;
                else
                    where_cam_y += distance;
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                if (strafe)
                    where_cam_y -= distance;
                else
                    where_cam_x += distance;
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                if (strafe)
                    where_cam_x -= distance;
                else
                    where_cam_y -= distance;
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                if (strafe)
                    where_cam_y += distance;
                else
                    where_cam_x -= distance;
            }
        }
        else {

            if (where_head == FldDirection.Face_Fld_Center) {
                if (strafe)
                    where_cam_x -= distance;
                else
                    where_cam_y += distance;
            } else if (where_head == FldDirection.Face_Fld_Audience) {
                if (strafe)
                    where_cam_y -= distance;
                else
                    where_cam_x -= distance;
            } else if (where_head == FldDirection.Face_Fld_Drivers) {
                if (strafe)
                    where_cam_x += distance;
                else
                    where_cam_y -= distance;
            } else if (where_head == FldDirection.Face_Fld_Foundation) {
                if (strafe)
                    where_cam_y += distance;
                else
                    where_cam_x += distance;
            }
        }
        distance *= -1;

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

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
            //speed *= 0.9;
        }
        else {
            target_encoder = (distance * TICKS_PER_INCH_STRAFE);
        }

        double prev_pos = motorRightBack.getCurrentPosition();

        //first pass at high speed (if going large dist)
        int stall_counter = 0;
        double sidePwr = sideWays;
//        if((Math.abs(target_encoder) > TICKS_PER_INCH_STRAIGHT*10) || (strafe)){
            while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed, target_encoder, P_FWD_COEFF, TICKS_PER_INCH_STRAIGHT, strafe, gyroCorrection, sidePwr)) {
                double currPosition = motorRightBack.getCurrentPosition();
                if (prev_pos == currPosition) {
                    stall_counter++;
                } else {
                    stall_counter = 0;
                    prev_pos = currPosition;
                }
                if (stall_counter > 10)
                    break;

                if(Math.abs(currPosition) > Math.abs(target_encoder*.3))
                    sidePwr = sideWays*-1;
                idle();
            }
        //}


        //second pass at low speed for fine granined distance
//        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed, target_encoder, P_FWD_COEFF , TICKS_PER_INCH_STRAIGHT/4, strafe, gyroCorrection, turnFactor)) {
//            if (prev_pos == motorRightBack.getCurrentPosition()) {
//                stall_counter++;
//            }
//            else {
//                stall_counter = 0;
//                prev_pos = motorRightBack.getCurrentPosition();
//            }
//            if(stall_counter > 10)
//                break;
//
//            idle();
//        }

    }


    boolean onTargetDist(double speed, double distance, double PCoeff, double distThreshold, Boolean strafe, Boolean gyroCorrection, double sideWays) {
        double error;
        double steer;
        boolean onTarget = false;
        double power;

        //determine  power based on error
        error = getErrorDist(distance, strafe, distThreshold);

        if (Math.abs(error) <= distThreshold) {

            steer = 0.0;
            power = 0.0;
            sideWays = 0.0;
            onTarget = true;
            stopWheels();
        } else {

            steer = getSteerDist(error, PCoeff);
            power = speed * steer;
        }

        double weightConstant = 0.98;//this constant will depend on the robot. you need to test experimentally to see which is best


        double turn_pwr = 0;
        if(gyroCorrection) {
            if ((running_counter++ % 3) == 0) {
                double curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
                if(curr_angle < -90)
                    curr_angle += 360;
                turn_pwr = (get_where_angle(where_head) - curr_angle) * 0.1;

                if (DEBUG)
                    System.out.println("14564dbg where_head: " + where_head + " " + get_where_angle(where_head));
                if (DEBUG) System.out.println("14564dbg curr_angle: " + curr_angle);

                turn_pwr = turn_pwr * -1;

                if (DEBUG) System.out.println("14564dbg turn_pwr: " + turn_pwr);

            }
        }

        if(strafe) {
            power = power*2;
            while (Math.abs(weightConstant * power) < 0.6)
                weightConstant *= 1.2;
        }
        else {
            while (Math.abs(weightConstant * power) < 0.2)
                weightConstant *= 1.2;
        }


        if (!strafe) {
            double sidePwr = sideWays*weightConstant * power;
            if(sidePwr == weightConstant * power)
                sidePwr = sideWays+0.001;  //just to avoid divide by 0

            vectorCombineSimple(-sidePwr, weightConstant * power, turn_pwr);
        }
        else {
            vectorCombineSimple( weightConstant * power, 0, turn_pwr);
        }

        if(DEBUG) System.out.println("14564dbg " + " steer " + steer + " error " + error + " power " + power*weightConstant);


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
             curr_encoder = motorRightBack.getCurrentPosition();
        else
            curr_encoder = 1 * motorRightBack.getCurrentPosition();

        robotError = targetDist - curr_encoder;

        if (targetDist <0) {
            robotError2 = Math.min(curr_encoder, -1*(distThr*5));
            err = Math.max(robotError, robotError2);
        }
         else {
            robotError2 = Math.max(curr_encoder, (distThr*5));
            err = Math.min(robotError, robotError2);
        }

         if(DEBUG) System.out.println("14564dbg getError curr_enc= " + curr_encoder + " err= " + err + " distThr= " + distThr);
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
    double P_DS_COEFF = 0.01;
    double P_DS_TURN_COEFF = 0.075;
    double P_DS_ERR_MARGIN = 1.5; //ORIGINALLY HAD A VALUE OF 2

    public void DSMove(double speed, double fwd_dist, double side_dist, Boolean useLeftSide, Boolean driveReverse, Boolean brakeStop, double turnDelta, Boolean useFwdEncode) {

        if (DEBUG) System.out.println("14564dbg DSMove: fwd " + fwd_dist + " side " + side_dist +
                                      " useLeftSide " + useLeftSide + " reverse " + driveReverse + " brake " + brakeStop + " turnDelta " + turnDelta + " useFdEncode " + useFwdEncode);

        Rev2mDistanceSensor ds1;
        Rev2mDistanceSensor ds2;
        Rev2mDistanceSensor ff1;

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);


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
            ff1 = distanceSensor_ffr;
        }

        double fd1 = DSRead(ff1);
        double sd1 = DSReadAvg(ds1);
        double sd2 = DSReadAvg(ds2);


        double counter = 0;

        int stall_counter = 0;
        double prev_pos = motorRightBack.getCurrentPosition() / TICKS_PER_INCH_STRAIGHT;
        double curr_pos = prev_pos;
        double target_pos = curr_pos - (fwd_dist);


        double fwd_error = 0;

        while (opModeIsActive() && !isStopRequested()) {

            idle();

            counter++;

            if(useFwdEncode) {

                fwd_error = curr_pos - target_pos;

            }
            else {
                fwd_error = fd1 - fwd_dist;
            }

            if((!brakeStop) && (fwd_error < 0))
                break;

            double side_error;
            if (useLeftSide)
                side_error = -1 * (sd1 - side_dist);
            else
                side_error = sd1 - side_dist;


            if(side_error > 32)
                side_error = 32;


            if ((Math.abs(fwd_error) < P_DS_ERR_MARGIN) && (Math.abs(side_error) < P_DS_ERR_MARGIN)) {
                if(brakeStop) {
                    stopWheels();
                }
                break;
            }

            double fwd_pwr = Range.clip(fwd_error * Math.abs(fwd_error)* P_DS_COEFF*speed, -1, 1);
            double side_pwr = Range.clip(side_error *Math.abs(side_error)* P_DS_COEFF*speed*3, -1, 1);

            double turn_pwr = (sd1-sd2) * P_DS_TURN_COEFF + turnDelta;

            if((sd1-sd2) > 15)
                turn_pwr = 0;


            if(driveReverse) {
                fwd_pwr = -1 * fwd_pwr;
            }

            if(fwd_pwr < 0.2 && fwd_pwr > 0) {
                fwd_pwr = 0.2;
            }
            else if (fwd_pwr > -0.2 && fwd_pwr < 0) {
                fwd_pwr = -0.2;
            }

            if(side_pwr < 0.3 && side_pwr > 0) {
                side_pwr = 0.3;
            }
            else if (side_pwr > -0.3 && side_pwr < 0) {
                side_pwr = -0.3;
            }

            if (DEBUG) System.out.println("14564dbg DSMove: fwd_err " + fwd_error + " side_err " + side_error + " turn " + turn_pwr);
            if (DEBUG) System.out.println("14564dbg DSMove: fwd_pwr " + fwd_pwr + "side_pwr" + side_pwr);


            vectorCombineSimple(side_pwr, fwd_pwr, turn_pwr);

            fd1 = DSRead(ff1);

            if (DEBUG) System.out.println("14564dbg DSMove: fd1  " + fd1);

            sd1 = DSReadAvg(ds1);
            if((counter%3) == 1)
                sd2 = DSReadAvg(ds2);

            curr_pos = motorRightBack.getCurrentPosition() / TICKS_PER_INCH_STRAIGHT;
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
        Rev2mDistanceSensor ff1;



        foundation.setPosition(FOUNDATION_UP);
        gyroTurnDirection(FldDirection.Face_Fld_Center);
        Boolean useLeftSide = false;
        if(isBlueSide) {
            useLeftSide = true;
        }

        DSMove(0.8, 28, 18, useLeftSide,true, true, 0, false);

        ff1 = distanceSensor_ffl;

        double distToFoundation = DSRead(ff1);
        if (distToFoundation > 10){
            distToFoundation = 10;
        }

        if(DEBUG) System.out.println("14564dbg Foundation Dist " + distToFoundation );

        EncoderStraight( distToFoundation+1);
        //grabbing foundation
        foundation.setPosition(FOUNDATION_DOWN);
        sleep(600);
        //TO DO : CALL AGAIN IF MISSED


        if(isBlueSide) {
            gyroTurnREV(1,get_where_angle(where_head)+10);
        }
        else {
            gyroTurnREV(1,get_where_angle(where_head)-10);

        }
        EncoderMoveDist(1, -25,false, false, 0);


        gyroTurnDirection(FldDirection.Face_Fld_Center_Foundation);
        EncoderMoveDist(1, -20,false, false, 0);

        gyroTurnDirection(FldDirection.Face_Fld_Foundation);
        EncoderMoveDist(0.8, 20,false, false, 0);
        foundation.setPosition(FOUNDATION_UP);

        //DSMove(0.75, 10, 26, useLeftSide, true, true);




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

    double averagerf = 0;
    double averagerb = 0;
    double averagelf = 0;
    double averagelb = 0;
    double averageff1 = 0;
    double averageffr = 0;
    double averagebbr = 0;
    int numberOfTimesRead = 1;
    //making the movingAverage of the last x values a hash define
    double getMovingAverage(Rev2mDistanceSensor ds){
        if(opModeIsActive() && !isStopRequested()) {
            if (ds==distanceSensor_rf){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);

                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averagerf == 0){
                    averagerf = dist;
                }
                averagerf = (((averagerf*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg RF " + averagerf);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averagerf;
            }
            else if (ds==distanceSensor_rb){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);
                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averagerb == 0){
                    averagerb = dist;
                }
                averagerb = (((averagerb*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg RB " + averagerb);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averagerb;
            }
            else if (ds==distanceSensor_lf){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);

                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averagelf == 0){
                    averagelf = dist;
                }
                averagelf = (((averagelf*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg LF " + averagelf);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averagelf;
            }
            else if (ds==distanceSensor_lb){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);

                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averagelb == 0){
                    averagelb = dist;
                }
                averagelb = (((averagelb*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg LB " + averagelb);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averagelb;
            }
            else if (ds==distanceSensor_ffl){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);

                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averageff1 == 0){
                    averageff1 = dist;
                }
                averageff1 = (((averageff1*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg FFL " + averageff1);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averageff1;
            }
            else if (ds==distanceSensor_ffr){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);

                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averageffr == 0){
                    averageffr = dist;
                }
                averageffr = (((averageffr*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg FFR " + averageffr);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averageffr;
            }
            else if (ds==distanceSensor_bbr){
                double dist = ds.getDistance(DistanceUnit.INCH);
                if (dist > 300 ) {
                    dist = ds.getDistance(DistanceUnit.INCH);

                    if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
                }
                if(averagebbr == 0){
                    averagebbr = dist;
                }
                averagebbr = (((averagebbr*(numberOfTimesRead-1)) + dist)/numberOfTimesRead);
                if(DEBUG) System.out.print("14564dbg BBR " + averagebbr);
                if(DEBUG) System.out.println(" DSRead " + dist);



                return averagebbr;
            }


        }
        return 0;
    }

    double DSReadAvg(Rev2mDistanceSensor ds) {
        //return getMovingAverage(ds);
        return DSRead(ds);

    }

    double DSRead(Rev2mDistanceSensor ds) {

        if(opModeIsActive() && !isStopRequested()) {
            double dist = ds.getDistance(DistanceUnit.INCH);
            if(DEBUG) System.out.println("14564dbg DSRead " + dist);
            if (dist > 300) {
                dist = ds.getDistance(DistanceUnit.INCH);
                if (DEBUG) System.out.println("14564dbg DSRead again " + dist);
            }
            return dist ;
        } else
            return 0 ;
    }

    public void makeParallelLeft(double distance_from_wall) {
        double sensor_gap = 11.3;
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
                EncoderMoveDist(0.8, (distance_from_wall - dist), true, false, 0);
            } else if (dist > distance_from_wall) {
                if ((dist - distance_from_wall) > 24) {
                    simpleStrafe(-0.8);
                    makeParallelLeft(distance_from_wall);
                } else
                    EncoderMoveDist(0.8, -1 * (dist - distance_from_wall), true, false, 0);
            }
        }
    }

    public void makeParallelRight(double distance_from_wall) {

        double sensor_gap = 11.3;
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
                EncoderMoveDist(0.8, -1 * (distance_from_wall - dist), true, false, 0);
            } else if (dist > distance_from_wall) {
                if ((dist - distance_from_wall) > 24) {
                    simpleStrafe(0.8);
                    makeParallelRight(distance_from_wall);
                } else
                    EncoderMoveDist(0.8, dist - distance_from_wall, true, false, 0);
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


        EncoderGoto(where_cam_x, 56, 0.8);
        closeGrabber();
        sleep(800);
        setLiftPosition(0.3);

        //step back
        EncoderGoto(where_cam_x, 38, 0.8);

        //turn
        gyroTurnDirection(FldDirection.Face_Fld_Foundation);

        if(doFoundation) {
            //go  to foundation
            EncoderGoto(122, where_cam_y, 1);



            setLiftPosition(5*REV_CORE_HEX_TICKS_PER_INCH);
            gyroTurnDirection(FldDirection.Face_Fld_Center);
            EncoderGoto(where_cam_x, 52, 0.8);

            //drop block
            grabCollection();

            sleep(200);
            EncoderGoto(where_cam_x, 46, 0.8);
            gyroTurnDirection(FldDirection.Face_Fld_Drivers);
            foundation.setPosition(0);
            EncoderGoto(where_cam_x, 54, 0.4);  //slow back into tray
            foundation.setPosition(0.7);
            sleep(200);
            EncoderGoto(where_cam_x, 20, 0.7);
            gyroTurnDirection(FldDirection.Face_Fld_Audience);
            EncoderGoto(where_cam_x+6, where_cam_y, 1);

            foundation.setPosition(0);

            setLiftPosition(0);

            if (isBlueSide) {
                makeParallelRight(30);
            } else {
                makeParallelLeft(30);

            }
        }
        else {
            EncoderGoto(96, where_cam_y, 1);
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
        EncoderMoveDist(0.6, -18,false, false, 0);

        //we always start from camera aligned with end of 5th block (i.e. 5*8 = 40 inch)
        //take to drop zone ( 2 tiles away towards the bridge )
        double dropDist = 0;
        if(isBlueSide){
            dropDist = -1*(where_cam_x) - 48;
        }
        else {
            dropDist = -1* (where_cam_x) + 48;
        }
        EncoderStrafe(dropDist);

        //drop the block
//        tray_left.setPosition(0.8);
    }

    public void runAutonomousTrayDS(Boolean isBlue) {

        isBlueSide = isBlue;

        powerReductionFactor = 1;

        initFn();

        waitForStart();


        makeParallelRight(21);


        if(isBlueSide) {
            where_head = FldDirection.Face_Fld_Audience;
        }
        else {
            where_head = FldDirection.Face_Fld_Foundation;
        }

        DS_MoveFoundation();

        //park

        if(isBlueSide) {
            makeParallelLeft(6);
        }
        else {
            makeParallelRight(6);
        }

        EncoderMoveDist(1, -55,false, true, 0);

        if(isBlueSide) {
            makeParallelLeft(1);
        }
        else {
            makeParallelRight(1);
        }


    }

        public void runAutonomousTray(Boolean isBlue) {

        initFn();

        waitForStart();

        isBlueSide = isBlue;

        //assume that robot is aligned such that it's
        //exactly two tiles away from the bridge towards the tray
        if(isBlueSide)
            EncoderStrafe(6);
        else
            EncoderStrafe(-6);

        foundation.setPosition(0);
        EncoderMoveDist(0.5, -40,false, false, 0);
        foundation.setPosition(0.7);
        sleep(500);
        gyroTurnREV(1, 180);
        EncoderMoveDist(0.5, -25,false, false, 0);
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
            where_cam_y = 18;
            where_cam_x = 36;
        }
        if(isBlueSide){
            where_cam_y = 18;
            where_cam_x = 44;
        }



        Boolean returnForSecond = true;
        Boolean doParking = true;
        Boolean testVuOnly = false;

        initFn();

        waitForStart();

        if(!testVuOnly) {
            new Thread(new extendThread()).start();
        }

        EncoderGoto(where_cam_x, 30, 1);


        Boolean blockSeen = vuFindBlock(isBlueSide);


        if (blockSeen) {

            double blockDist=0;


            blockDist = vu_x + CAM_SIDE_ARM_OFFSET;

            EncoderStrafe (blockDist);
        }

        double firstSkyStone_X = where_cam_x;

        if(!testVuOnly) {
            //grab a block (even if it's a random one)

            grabAndDropBlock_Arm(doFoundation);
        }




        if (!doFoundation && returnForSecond) {

            EncoderGoto(60, where_cam_y, 1);

            gyroTurnDirection(FldDirection.Face_Fld_Foundation);


//            if (isBlueSide) {
//                makeParallelLeft(20);
//            } else {
//                makeParallelRight(20);
//            }

            if(firstSkyStone_X < 32) {
                EncoderGoto(20, where_cam_y, 1);
                gyroTurnDirection(FldDirection.Face_Fld_Center);
            }
            else {
                EncoderGoto(firstSkyStone_X - 18, where_cam_y, 1);

                gyroTurnDirection(FldDirection.Face_Fld_Center);
                blockSeen = vuFindBlock(isBlueSide);

                if (blockSeen) {
                    double blockDist=0;
                    blockDist = vu_x + CAM_SIDE_ARM_OFFSET;
                    EncoderStrafe (blockDist);
                }
            }

            EncoderGoto(where_cam_x, where_cam_y + 14, 0.8);
            closeGrabber();
            sleep(800);
            setLiftPosition(0.3);
            EncoderGoto(where_cam_x, where_cam_y - 10, 0.8);

            gyroTurnDirection(FldDirection.Face_Fld_Foundation);

            EncoderGoto(96, where_cam_y, 1);
            //drop block
            grabCollection();
        }


        //park (1 tile back towards the bridge)
        if(doParking) {
            EncoderGoto(72, 40, 1);
        }
    }



    Boolean vuFindBlockSideCam(Boolean isBlueSide) {
        boolean blockSeen = false;

        int i = 0;
        int moveCount = 0;
        int retryCount = 0;
        targetsSkyStone.activate();

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

                double blockDist = vu_x + CAM_SIDE_ARM_OFFSET;

                if(DEBUG) System.out.println("14564dbg vu vx_x " + vu_x + " blockDist: " + blockDist);
                if(vu_x > 0){
                    if(isBlueSide)
                        where_cam_x = 36 - CAM_SIDE_ARM_OFFSET ;
                    else
                        where_cam_x = 44 + CAM_SIDE_ARM_OFFSET ;
                }
                else{
                    if(isBlueSide)
                        where_cam_x = 44 - CAM_SIDE_ARM_OFFSET;
                    else
                        where_cam_x = 36 + CAM_SIDE_ARM_OFFSET;
                }
//                where_cam_x += blockDist;
//                where_cam_y += vu_y-1;


//                EncoderMoveDist(1, blockDist, false, false, 0);
//                EncoderMoveDist(1, vu_y-1, true, false, 0);
//

//                if(isBlueSide) {
//                    DSMove(0.6, (where_cam_x-CAM_TO_FF) + blockDist, 32, false, false, true, 0);
//                }
//                else {
//                    DSMove(0.6, (where_cam_x-CAM_TO_BB) + blockDist, 32, false, true, true, 0);
//                }


                break;


            }
            else if (retryCount < 5) {
                retryCount++;
                if(DEBUG)  System.out.println("14564dbg vu retry count: " + retryCount);
                sleep(100);
                continue;
            }
            else {
                //assume third default block
                 where_cam_x = 28 - CAM_SIDE_ARM_OFFSET ;

                 break;
            }

        }

        targetsSkyStone.deactivate();

        return blockSeen;
    }

    public void dropOnFloorLeft(){
        sideArmSetStateLeft(SideArmState.GRAB);
        sleep(400);
        sideArmSetStateLeft(SideArmState.GRAB_HOLD_HIGH);
        sleep(400);

        double dist = 72 + CAM_TO_FF - where_cam_x + 2;
        EncoderStrafe(12);
        makeParallelRight(21);

        if(isBlueSide) {
            EncoderMoveDist(1, -dist ,false, true, 0);

        }
        else {
            EncoderMoveDist(1, dist,false, true, 0);

        }


        sideArmSetStateLeft(SideArmState.GRAB);
        sleep(200);
        sideArmSetStateLeft(SideArmState.PRE_GRAB);
        sleep(200);
        sideArmSetStateLeft(SideArmState.HOME);




        if(isBlueSide) {
            EncoderMoveDist(1, dist + 24 ,false, true, 0);

        }
        else {
            EncoderMoveDist(1, -dist - 8,false, true, 0);


        }


    }
    public void dropOnFloorRight(){
        sideArmSetStateRight(SideArmState.GRAB);
        sleep(400);
        sideArmSetStateRight(SideArmState.GRAB_HOLD_HIGH);
        sleep(400);

        double dist = 72 + CAM_TO_FF - where_cam_x + 2;
        EncoderStrafe(12);
        makeParallelLeft(21);

        if(isBlueSide) {
            EncoderMoveDist(1, -dist ,false, true, 0);

        }
        else {
            EncoderMoveDist(1, dist,false, true, 0);

        }


        sideArmSetStateRight(SideArmState.GRAB);
        sleep(200);
        sideArmSetStateRight(SideArmState.PRE_GRAB);
        sleep(200);
        sideArmSetStateRight(SideArmState.HOME);


        if(isBlueSide) {
            EncoderMoveDist(1, dist + 24 ,false, true, 0);

        }
        else {
            EncoderMoveDist(1, -dist - 8,false, true, 0);


        }


    }

    public void grab_SideArmLeft() {


       // makeParallelRight(32);


        sideArmSetStateLeft(SideArmState.GRAB);
        sleep(400);
        sideArmSetStateLeft(SideArmState.GRAB_HOLD_HIGH);
        sleep(400);

        EncoderStrafe(12);
        //makeParallelRight(21);

    }

    public void grab_SideArmRight() {


        // makeParallelRight(32);


        sideArmSetStateRight(SideArmState.GRAB);
        sleep(400);
        sideArmSetStateRight(SideArmState.GRAB_HOLD_HIGH);
        sleep(400);

        EncoderStrafe(-12);

    }

    public void drop_SideArm_FoundationLeft(){
        if(isBlueSide) {
            EncoderMoveDist(1, -84,false, true, 0);

            DSMove(0.5, 24-BB_DS_TO_SIDE_ARM, 32, false, true, true, 0, false);

        }
        else {
            EncoderMoveDist(1, 84,false, true, 0);
            DSMove(0.8, 24-FF_DS_TO_SIDE_ARM, 32, false, false, true, 0, false);

        }


        sideArmSetStateLeft(SideArmState.THROW);
        sleep(200);

        EncoderStrafe(8);
        sideArmSetStateLeft(SideArmState.HOME);
    }
    public void drop_SideArm_FoundationRight(){
        if(isBlueSide) {
            EncoderMoveDist(1, -84,false, true, 0);

            DSMove(0.5, 24-BB_DS_TO_SIDE_ARM, 32, false, true, true, 0, false);

        }
        else {
            EncoderMoveDist(1, 84,false, true, 0);
            DSMove(0.8, 24-FF_DS_TO_SIDE_ARM, 32, false, false, true, 0, false);

        }


        sideArmSetStateRight(SideArmState.THROW);
        sleep(200);

        EncoderStrafe(8);
        sideArmSetStateRight(SideArmState.HOME);
    }


    public void runAutonomousDS(Boolean isBlue, Boolean doFoundation, Boolean dropOnFloor) {


        USE_VUFORIA = true;

        isBlueSide = isBlue;

        powerReductionFactor = 1;

        where_head = FldDirection.Face_Fld_Audience;

        Boolean returnForSecond = true;
        if (doFoundation)
            returnForSecond = false;


        initFn();

        waitForStart();


        if (isBlueSide) {
            sideArmSetStateLeft(SideArmState.PRE_GRAB);
        }
        else {
            sideArmSetStateRight(SideArmState.PRE_GRAB);
        }

        if (isBlueSide) {
            where_cam_y = 15;
            where_cam_x = 40;
            DSMove(1, where_cam_x - CAM_TO_FF, where_cam_y, false, false, true, 0, false);
        } else {
            where_cam_y = 15;
            where_cam_x = 40;
            DSMove(1, where_cam_x - CAM_TO_BB, where_cam_y, true, true, true, 0, false);
        }

        //sleep(200);
        vuFindBlockSideCam(isBlueSide);
        //assume where_cam_x and where_cam_y are properly set by vuFindSizeCam
        where_cam_y = 30.5;
        double firstSkyStone_X = where_cam_x;

        if(DEBUG) System.out.println("FirstSS_X " + firstSkyStone_X);

        if(isBlueSide) {
            DSMove(0.6, where_cam_x - CAM_TO_FF, where_cam_y, false, false, true, 0, false);
        }
        else {
            DSMove(0.6, where_cam_x - CAM_TO_BB, where_cam_y, false, true, true, 0, false);

        }


        //TEST
        //sleep(30000);
       //TEST
        if(dropOnFloor){
            if(isBlue){
                grab_SideArmLeft();
                dropOnFloorLeft();
                EncoderStrafeGyro(6);
            }
            else{
                grab_SideArmRight();
                dropOnFloorRight();
                EncoderStrafeGyro(6);
            }
        }
        else {
            if (isBlue) {
                grab_SideArmLeft();
                drop_SideArm_FoundationLeft();
                EncoderStrafeGyro(6);
            }
            else {
                grab_SideArmRight();
                drop_SideArm_FoundationRight();
                EncoderStrafeGyro(6);
            }
        }




        if (doFoundation) {
            DS_MoveFoundation();

        } else {
            if (returnForSecond) {

                EncoderStraightGyro(96);

                if (isBlueSide) {

                    where_cam_y = 30.5;
                    where_cam_x = firstSkyStone_X - 24;

                    if(DEBUG) System.out.println("FirstSS_X on return" + firstSkyStone_X);
                    if(DEBUG) System.out.println("where_x on return" + where_cam_x);

                    sideArmSetStateLeft(SideArmState.PRE_GRAB);
                    DSMove(0.6, where_cam_x - CAM_TO_FF, where_cam_y, false, false, true, 0, false);

                }
                else {
                    where_cam_y = 30.5;
                    where_cam_x = firstSkyStone_X - 24;

                    sideArmSetStateRight(SideArmState.PRE_GRAB);

                    DSMove(0.6, where_cam_x - CAM_TO_BB, where_cam_y, true, false, true, 0, false);
                }
                if (DEBUG)
                    System.out.println("14564dbg second wherex " + where_cam_x + " where " + where_cam_y);

                if(isBlueSide && dropOnFloor) {
                    grab_SideArmLeft();
                    dropOnFloorLeft();
                }
                else{
                    grab_SideArmLeft();
                    drop_SideArm_FoundationLeft();
                }
                if (!isBlueSide && dropOnFloor) {
                    grab_SideArmRight();
                    dropOnFloorRight();
                }

                else{
                    grab_SideArmRight();
                    drop_SideArm_FoundationRight();
                }





            }


        }
        //park

        if (isBlueSide) {
            makeParallelLeft(23);
        } else {
            makeParallelRight(23);
        }


        EncoderMoveDist(1, -60, false, true, 0);
    }



}






