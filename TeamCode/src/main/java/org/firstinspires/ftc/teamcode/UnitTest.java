package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@TeleOp (name = "UnitTest")
public class UnitTest extends Autonomous2020 {
final double TICKS_PER_INCH_STRAIGHT = 89.1;
final double TICKS_PER_INCH_STRAFE = 126.00;


    @Override
    public void runOpMode() {


        initFn();


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.x) {
                armExtended(10);
                grabCollection();
                straight_inch(1, 1, 10);
                closeGrabber();
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
            
            if(gamepad1.dpad_down){
                strafe_inch(1,1,12);
            }
            if(gamepad1.dpad_up){
                strafe_inch(1,1,24);
            }
            if(gamepad1.dpad_right){
                strafe_inch(1,-1,12);
            }
            if(gamepad1.dpad_left){
                strafe_inch(1,-1,24);
            }
        }

    }
}