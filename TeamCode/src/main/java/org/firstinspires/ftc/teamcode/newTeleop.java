package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@TeleOp (name = "newTeleop")
public class newTeleop extends LinearOpMode {

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor lift;
    Servo grab;
    CRServo extend;

    BNO055IMU imu, imu1;
    Orientation angles, angles1;

    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone=false;
    double power_multiplier;

    double angleToTurn;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    public void initFn() {

        telemetry.addData("Init: start ","");

        lift = hardwareMap.dcMotor.get("lift");



        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
//        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD); //changed for 2020 config
//        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE); // changed for 2020 config
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE); //changed for 2020 config
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD); // changed for 2020 config
        //        extend.setDirection(CRServo.Direction.REVERSE);
        grab = hardwareMap.servo.get("grab");
        extend = hardwareMap.crservo.get("extend");

        strafing = false;

        motorRightFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);

        power_multiplier = 1;



        angleToTurn = 30;

        double driveSpeed = 0;
        motorLeftBack.setPower(driveSpeed);
        motorLeftFront.setPower(driveSpeed);
        motorRightBack.setPower(driveSpeed);
        motorRightFront.setPower(driveSpeed);
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

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    @Override
    public void runOpMode() {


        initFn();




        waitForStart();

        lift.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            float forward = -1*gamepad1.right_stick_y;
            float sideways = gamepad1.left_stick_x;


            if (Math.abs(sideways) > 0.1) {
                //strafe
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);  //changed for strafe
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD); //changed for strafe

                motorRightFront.setPower(-1*sideways*power_multiplier);
                motorRightBack.setPower(-1*sideways*power_multiplier);
                motorLeftFront.setPower(-1*sideways*power_multiplier);
                motorLeftBack.setPower(-1*sideways*power_multiplier);
            }
            else
            if (gamepad1.left_bumper && Math.abs(forward) > 0.1) {
                //right turn
                motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //changed
                motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);   //changed
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE); //default
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                motorRightFront.setPower(forward*power_multiplier);
                motorRightBack.setPower(forward*power_multiplier);
                motorLeftFront.setPower(forward*power_multiplier);
                motorLeftBack.setPower(forward*power_multiplier);
            }
            else
            if (gamepad1.right_bumper && Math.abs(forward) < 0.1) {
                //left turn
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD); //changed
                motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //changed
                motorRightFront.setPower(forward*power_multiplier);
                motorRightBack.setPower(forward*power_multiplier);
                motorLeftFront.setPower(forward*power_multiplier);
                motorLeftBack.setPower(forward*power_multiplier);
            }
            else
            if (Math.abs(forward) > 0.1) {
                //forward
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightFront.setPower(forward*power_multiplier);
                motorRightBack.setPower(forward*power_multiplier);
                motorLeftFront.setPower(forward*power_multiplier);
                motorLeftBack.setPower(forward*power_multiplier);
            }
            else
            {
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRightFront.setPower(0);
                motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRightBack.setPower(0);
                motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeftFront.setPower(0);
                motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeftBack.setPower(0);
            }

            if (gamepad1.b) {
                if (power_multiplier == 1)
                    power_multiplier = 0.5;
                else if (power_multiplier == 0.5)
                    power_multiplier = 1;
                sleep(200);
            }
            if(gamepad2.left_stick_y > 0.1) {
                lift.setPower(1);
            }
            else if(gamepad2.left_stick_y < -0.1){
                lift.setPower(-1);
            }
            else{
                lift.setPower(0);
            }

            if (gamepad2.right_trigger != 0){

                extend.setPower(1);
            }

            if(gamepad2.left_trigger != 0){

                extend.setPower(-1);
            }

            if (gamepad2.dpad_up){

                grab.setPosition(1);
            }

            if (gamepad2.dpad_down){

                grab.setPosition(0);
            }

            if (gamepad2.x) {

                lift.setMode(STOP_AND_RESET_ENCODER);
                lift.setMode(RUN_WITHOUT_ENCODER);
                lift.setDirection(DcMotorSimple.Direction.FORWARD);
                while (Math.abs(lift.getCurrentPosition()) < Math.abs(1*1120))
                {
                    lift.setPower(-1.0);
                }
                lift.setPower(0);

            }
            if (gamepad2.y) {

                lift.setMode(STOP_AND_RESET_ENCODER);
                lift.setMode(RUN_WITHOUT_ENCODER);
                lift.setDirection(DcMotorSimple.Direction.FORWARD);
                while (Math.abs(lift.getCurrentPosition()) < Math.abs(1*1120))
                {
                    lift.setPower(1.0);
                }
                lift.setPower(0);

            }

        }
//


//        if (gamepad2.y) {
//            telemetry.addData("grab position y", grabServo.getPosition() );
//            grabServo.setPosition(1);
//        }
//
//        if (gamepad2.x) {
//            telemetry.addData("grab position a", grabServo.getPosition() );
//            grabServo.setPosition(0);
//        }
//
//        if (gamepad2.y) {
//            extend.setPower(0.8);
//        }
//        grabServo.setPosition(0);
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        lift.setPower(0);


    }
}