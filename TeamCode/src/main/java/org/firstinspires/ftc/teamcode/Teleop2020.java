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

@TeleOp (name = "Teleop2020")
public class Teleop2020 extends LinearOpMode {

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor lift;
    DcMotor lift_assist;
    Servo grab_back;
    Servo grab_front;
    Servo extend;
    Servo turn;
    Servo foundation;




    BNO055IMU imu, imu1;
    Orientation angles, angles1;

    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone=false;
    double power_multiplier;
    double armPosition;
    double angleToTurn;
    double liftPosition;
    boolean extended = false;
    private static final double REV_CORE_HEX_TICKS_PER_INCH = 47.127;
    private static final double LIFT_NON_SLIP_POWER = 0.2;
    private static final double ARM_INCH_TO_TIME_MS = 200;


    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    public void teleopInitFn() {

        telemetry.addData("Init: start ","");

        lift = hardwareMap.dcMotor.get("lift");
        lift_assist = hardwareMap.dcMotor.get("lift_assist");
        lift.setMode(STOP_AND_RESET_ENCODER);





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
        grab_front = hardwareMap.servo.get("grab_front");
        grab_back = hardwareMap.servo.get("grab_back");
        turn = hardwareMap.servo.get("turn");
        foundation = hardwareMap.servo.get("foundation");
        armPosition = 0;

        //grab_front.setPosition(0.1);
        //grab_back.setPosition(0.1);

        extend = hardwareMap.servo.get("extend");

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

    public void liftInch(double inches) {

        lift.setMode(RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        double ticks = inches * REV_CORE_HEX_TICKS_PER_INCH;

        if ((Math.abs(lift.getCurrentPosition()) < Math.abs(ticks))) {
            while ((Math.abs(lift.getCurrentPosition()) < Math.abs(ticks))&& !isStopRequested()) {
                idle();
                lift.setPower(1.0);
                lift_assist.setPower(1.0);
            }
        } else {
            while ((Math.abs(lift.getCurrentPosition()) > Math.abs(ticks)) && !isStopRequested()) {
                idle();
                lift.setPower(-1.0);
                lift_assist.setPower(-1.0);
            }
        }

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(LIFT_NON_SLIP_POWER);
        lift_assist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_assist.setPower(LIFT_NON_SLIP_POWER);

    }

    public void setArmPosition_startPosition(){

    }
    public void armExtended(double inches){
        long currentTime = System.currentTimeMillis();
        double targetTime = Math.abs(inches) * ARM_INCH_TO_TIME_MS + currentTime;
        while((currentTime < targetTime ) && !isStopRequested()){
            idle();
            currentTime = System.currentTimeMillis();
            if(inches == Math.abs(inches)){
                extend.setPosition(0);

            }
            else{
                extend.setPosition(1);
            }

        }
        extend.setPosition(0.5);
        if(inches == Math.abs(inches)){
            armPosition+=inches;
        }
        else{
            armPosition-=inches;
        }
        extended = true;
    }
    public void grabCollection(){
        if(extended) {
            grab_front.setPosition(1);
            grab_back.setPosition(1);
        }
    }
    public void closeGrabber(){
        grab_front.setPosition(0.4);
        grab_back.setPosition(1);
    }

    @Override
    public void runOpMode() {


        teleopInitFn();

        waitForStart();

        lift.setPower(0);
        liftPosition = 2;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            idle();

            float forward = -1*gamepad1.right_stick_y;
            float sideways = gamepad1.left_stick_x;


            if (Math.abs(sideways) > 0.1) {
                //strafe
                motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //ƒchanged for strafe
                motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE); //changed for strafe

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
            if (gamepad1.right_bumper && Math.abs(forward) > 0.1) {
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
            if(gamepad1.right_trigger > 0 ){
                power_multiplier = 1 - (gamepad1.right_trigger*2/3);
                //Subtract from 1 to make the trigger give a reduction in power
                //Multiply by 2/3 to not completely reduce the power
            }
//            if (gamepad1.b) {
//                if (power_multiplier == 1)
//                    power_multiplier = 0.5;
//                else if (power_multiplier == 0.5)
//                    power_multiplier = 1;
//                sleep(200);
//            }
            if(lift.getCurrentPosition()>= 0 && gamepad2.left_stick_y > 0.1) {
                lift.setPower(-1);
                lift_assist.setPower(-1);

            }
            else if( gamepad2.left_stick_y < -0.1){
                lift.setPower(1);
                lift_assist.setPower(1);

            }
            else{
                lift.setPower(0);
                lift_assist.setPower(0);

            }

            if (gamepad2.right_trigger > 0.1){

                extend.setPosition(1);
            }
            else if (gamepad2.left_trigger > 0.1) {
                extend.setPosition(0);
            }
            else {
                extend.setPosition(0.5);
            }


//            if (gamepad1.y) {
//                 grab_front.setPosition(1);
//                 grab_back.setPosition(1);
//            }
            if (gamepad1. dpad_right){ //normal position
                turn.setPosition(1);
            }

            if (gamepad1. dpad_left){ //vertical position
                turn.setPosition(0.25);
            }

            if (gamepad2.dpad_up) {               //GRABBED POSITION
                grab_front.setPosition(0.4); //More than 90 degrees to add pressure
                grab_back.setPosition(1);
            }
            if (gamepad2.dpad_down) {               //OPEN FOR COLLECTION POSITION
                grab_front.setPosition(1);
                grab_back.setPosition(1);
            }
            if (gamepad2.dpad_left) {               //Dropping
                grab_front.setPosition(1);
                grab_back.setPosition(0.5);
            }

            if (gamepad2.dpad_right) {               //OPEN FOR COLLECTION POSITION
                grab_front.setPosition(1);           //AND LIFT TO NOT HIT BLOCK
                grab_back.setPosition(0.5);
                lift.setMode(STOP_AND_RESET_ENCODER);
                lift.setMode(RUN_WITHOUT_ENCODER);
                lift.setDirection(DcMotorSimple.Direction.FORWARD);
                while ((Math.abs(lift.getCurrentPosition()) < Math.abs((0.5*288))) && !isStopRequested())
                {
                    idle();
                    lift.setPower(1.0);
                }
                lift.setPower(0);

            }

            if (gamepad2.right_bumper) {

                grab_back.setPosition(0);
                grab_front.setPosition(0.5);

            }

            if (gamepad2.left_bumper) {

                grab_back.setPosition(0);
                grab_front.setPosition(0);

            }





//            if (gamepad2.dpad_up){
//
//                grab.setPosition(1);
//            }
//
//            if (gamepad2.dpad_down){
//
//                grab.setPosition(0);
//            }

            if (gamepad1.x){  //position up
                foundation.setPosition(0);

            }
            if (gamepad1.y) {   //position down
                foundation.setPosition(1);
            }


            if (gamepad2.x) {

                double target = lift.getCurrentPosition() + (2*REV_CORE_HEX_TICKS_PER_INCH);
                lift.setMode(RUN_WITHOUT_ENCODER);
                while(lift.getCurrentPosition()<target){
                    lift.setPower(1);
                    lift_assist.setPower(1);

                }

            }
            if (lift.getCurrentPosition()>= 2*REV_CORE_HEX_TICKS_PER_INCH && gamepad2.y) {

                double target = lift.getCurrentPosition() - (2*REV_CORE_HEX_TICKS_PER_INCH);
                lift.setMode(RUN_WITHOUT_ENCODER);
                while(lift.getCurrentPosition()>target){
                    lift.setPower(-0.5);
                    lift_assist.setPower(-0.5);

                }


            }


        }
//


//        if (gamepad2.y) {
//            telemetry.addData("grab position y", grabServo.getPosition() );
//            grabServo.setPosition(1);
//        }
//
//     z       if (gamepad2.x) {
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
        lift_assist.setPower(0);
        telemetry.addData("lift encoder value", lift.getCurrentPosition());


    }
}