package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.ObjectTracker;
import com.vuforia.Tracker;
import com.vuforia.TrackerManager;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Image;


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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;


import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.BlockingQueue;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name = "testblock")
public class testBlock extends LinearOpMode {

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armBottom;
    DcMotor armTop;
    DcMotor lift;

    Servo grabServo;
    Servo grabBase;

    double grabpos;

    private DistanceSensor sensorRange_rf;
    private DistanceSensor sensorRange_rb;
    private DistanceSensor sensorRange_lf;
    private DistanceSensor sensorRange_lb;

    double distanceToWall;

    Rev2mDistanceSensor distanceSensor_rf;
    Rev2mDistanceSensor distanceSensor_rb;
    Rev2mDistanceSensor distanceSensor_lf;
    Rev2mDistanceSensor distanceSensor_lb;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    final double TICKS_PER_INCH = 200;


    int pixyCounter;
    boolean isPixyObjectSeen;
    boolean pixyContinue = true;


    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone = false;
    boolean vuInitDone = false;


    I2cDeviceSynch pixy;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity, gravity1;
    BNO055IMU.Parameters parameters;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = "AYpOJ0H/////AAABGeEbm+5m+k5BrTnPlF3X9R177NGoUFUGl1kpgLa7MBwlsRdnD3IdxY7LmZ41NTQMASZ1MbCWaEpM4Sag7tDfQsJjqVvCwZr3qJm5y33J8rnMWz1ViOwwzZgnsSZqeGRY9+uPGa6cTMO/cxs+YF+4OqsD+iu4exeMCsxyAPYhXQrEIaW6h7zYVrdi9b5WsgNGUfP60Qz8U3szKTfVmaHmMFvc+iuJ1qmAM5AjlsBlc8MMHzLAL/3sf3UiCDe4tgo4mmYEsdl499QhqhhImEiKS8rTkap/53B8Hm89z3m5HuBoH4EKVUc65k2aCBg5c5jXVoZan8DkQFqSPnArwQnCHpaL/d1y79BRE44nJXj54E6V";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    private static final float stoneZ = 2.00f * mmPerInch;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private List<VuforiaTrackable> allTrackables;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia = null;
    VuforiaLocalizer.Parameters Vu_parameters;

    WebcamName webcamName = null;

    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    class InitThread_Depot implements Runnable {
        @Override
        public void run() {
            try {

                telemetry.addData("Init: Thread start ", "");

                parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = false;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                parameters.mode = BNO055IMU.SensorMode.IMU;

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
                // first.

                vuInitDone = true;
            } catch (Exception e) {
                e.printStackTrace();
            }

            initDone = true;
            pixy = hardwareMap.i2cDeviceSynch.get("pixy");


            //setting Pixy's I2C Address
            pixy.setI2cAddress(I2cAddr.create7bit(0x54));

            I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(1, 26,
                    I2cDeviceSynch.ReadMode.REPEAT);
            pixy.setReadWindow(readWindow);

            //required to "turn on" the device
            pixy.engage();


            //initDone = true;
            telemetry.addData("Init: Thread done ", "");

            while (pixyContinue && !isStopRequested()) {
                 /*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */

                //  int pixy_x;

                //  pixy_x = (int) pixy.read8(6);
                //  pixy_x = pixy_x << 8;
                //  pixy_x = (pixy_x & (0xff00) )| (int) pixy.read8(7);

                // int pixy_x = ((pixy.read8(4) & 0xff) << 8) | (pixy.read8(5) & 0xff);
                // telemetry.addData("Pixy_x", pixy_x);
                // https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:protocol_reference
                //getBlocks

//                pixy.write8(0,173);
//                pixy.write8(1,193);
//                pixy.write8(2,32);
//                pixy.write8(3,2);
//                pixy.write8(4,1);
//                pixy.write8(5,1);

                byte b0 = pixy.read8(0);
                telemetry.addData("Byte 0", b0);
                byte b1 = pixy.read8(1);
                telemetry.addData("Byte 1", b1);
                byte b2 = pixy.read8(2);
                telemetry.addData("Byte 2", b2);
                byte b3 = pixy.read8(3);
                telemetry.addData("Byte 3", b3);
                byte b4 = pixy.read8(4);
                telemetry.addData("Byte 4", b4);
                byte b5 = pixy.read8(5);
                telemetry.addData("Byte 5", b5);
                byte b6 = pixy.read8(6);
                telemetry.addData("Byte 6", b6);
                telemetry.update();
                if (b0 != 0) {
                    if (pixyCounter < 10)
                        pixyCounter += 4;

                } else {
                    if (pixyCounter > 1)
                        pixyCounter--;

                }

                if (pixyCounter > 1) {
                    isPixyObjectSeen = true;
                } else {
                    isPixyObjectSeen = false;
                }


                byte b = pixy.read8(7);
//                telemetry.addData("Byte 7", b);
                b = pixy.read8(8);
//                telemetry.addData("Byte 8", pixy.read8(8));
                b = pixy.read8(9);
//                telemetry.addData("Byte 9", pixy.read8(9));
                b = pixy.read8(10);
//                telemetry.addData("Byte 10", pixy.read8(10));
                b = pixy.read8(11);
//                telemetry.addData("Byte 11", pixy.read8(11));
                b = pixy.read8(12);
//                telemetry.addData("Byte 12", pixy.read8(12));
                b = pixy.read8(13);
//                telemetry.addData("Byte 13", pixy.read8(13));
                //  telemetry.addData("Byte 14", pixy.read8(14));
                //  telemetry.addData("Byte 15", pixy.read8(15));
                //  telemetry.addData("Byte 16", pixy.read8(16));
                //  telemetry.addData("Byte 17", pixy.read8(17));
                //telemetry.addData("Byte 18", pixy.read8(18));
                //telemetry.addData("Byte 19", pixy.read8(19));
                //telemetry.addData("Byte 20", pixy.read8(20));
                //telemetry.addData("Byte 21", pixy.read8(21));
                // telemetry.addData("pixyCounter", pixyCounter);
                // telemetry.update();
                sleep(20);


            }

        }

    }

    ;


    public void initFn() {

        telemetry.addData("Init: start ", "");

        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        strafing = false;

        motorRightFront.setMode(RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(RUN_WITHOUT_ENCODER);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(RUN_WITHOUT_ENCODER);


        armBottom = hardwareMap.dcMotor.get("armBottom");
        armTop = hardwareMap.dcMotor.get("armTop");
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setDirection(DcMotorSimple.Direction.REVERSE);
        armTop.setMode(RUN_WITHOUT_ENCODER);
        armBottom.setMode(RUN_WITHOUT_ENCODER);
        grabServo = hardwareMap.servo.get("grab_servo");
        grabBase = hardwareMap.servo.get("grab_base");


        grabpos = 0.5;


        double driveSpeed = 0;
        motorLeftBack.setPower(driveSpeed);
        motorLeftFront.setPower(driveSpeed);
        motorRightBack.setPower(driveSpeed);
        motorRightFront.setPower(driveSpeed);

        grabServo.setPosition(grabpos);

        sensorRange_rf = hardwareMap.get(DistanceSensor.class, "2m_rf");
        distanceSensor_rf = (Rev2mDistanceSensor) sensorRange_rf;
        sensorRange_rb = hardwareMap.get(DistanceSensor.class, "2m_rb");
        distanceSensor_rb = (Rev2mDistanceSensor) sensorRange_rb;
        sensorRange_lf = hardwareMap.get(DistanceSensor.class, "2m_lf");
        distanceSensor_lf = (Rev2mDistanceSensor) sensorRange_lf;
        sensorRange_lb = hardwareMap.get(DistanceSensor.class, "2m_lb");
        distanceSensor_lb = (Rev2mDistanceSensor) sensorRange_lb;

        new Thread(new InitThread_Depot()).start();
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

    double P_TURN_COEFF = -0.05;
    double TURN_THRESHOLD = 1;

    public void gyroTurnREV(double speed, double angle) {

        telemetry.addData("starting gyro turn", "-----");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !onTargetAngleREV(speed, angle, P_TURN_COEFF, 3)) {
            telemetry.update();
            idle();
            telemetry.addData("-->", "inside while loop :-(");
            telemetry.update();
        }
        //sleep(100);
        while (opModeIsActive() && !isStopRequested() && !onTargetAngleREV(speed, angle, P_TURN_COEFF / 3, 1)) {
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

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double weightConstant = 0.8;//this constant will depend on the robot. you need to test experimentally to see which is best

        while (Math.abs(weightConstant * leftSpeed) < 0.2)
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
        //telemetry.addData("Zvalue","%5.2f",gyro.getIntegratedZValue());
        //telemetry.update();

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
                //counter++;
                /*if(System.currentTimeMillis()-startTime > 29500 ){
                    break;
                }*/
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

    public void straight(double power, int direction, double distance) {

        distance /= 2.25;
        power /= 1;

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

        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Straight", "Still in straight()");
        telemetry.update();


        while (opModeIsActive() && !isStopRequested() && (Math.abs(motorLeftBack.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance))) {

            /*if(System.currentTimeMillis()-startTime > 29500 ){
                break;
            }*/
            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.update();
            if (Math.abs(motorLeftFront.getCurrentPosition()) < .1 * Math.abs(distance)) {

                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .2 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .7 * power);
                motorRightBack.setPower(direction * .7 * power);
                motorRightFront.setPower(direction * .7 * power);
                motorLeftBack.setPower(direction * .7 * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .7 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .8 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .7 * power);
                motorRightBack.setPower(direction * .7 * power);
                motorRightFront.setPower(direction * .7 * power);
                motorLeftBack.setPower(direction * .7 * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .9 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .5 * power);
                motorRightBack.setPower(direction * .5 * power);
                motorRightFront.setPower(direction * .5 * power);
                motorLeftBack.setPower(direction * .5 * power);
            } else {
                motorLeftFront.setPower(direction * .4 * power);
                motorRightBack.setPower(direction * .4 * power);
                motorRightFront.setPower(direction * .4 * power);
                motorLeftBack.setPower(direction * .4 * power);
            }
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

    public void strafe_inch(double power, int direction, double distance){
       double ticks = distance*TICKS_PER_INCH;
        strafe(power, direction, ticks);
    }

    /* direction : +1 is right , -1 is left
       distance: in ticks
     */
    public void strafe(double power, int direction, double distance) {

        distance /= 2.25;
        power /= 1.5;

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


        while (opModeIsActive() && !isStopRequested() && (Math.abs(motorLeftBack.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorLeftFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightFront.getCurrentPosition()) < Math.abs(distance)
                && Math.abs(motorRightBack.getCurrentPosition()) < Math.abs(distance))) {

            /*if(System.currentTimeMillis()-startTime > 29500 ){
                break;
            }*/
            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.update();
            if (Math.abs(motorLeftFront.getCurrentPosition()) < .1 * Math.abs(distance)) {

                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .8 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * power);
                motorRightBack.setPower(direction * power);
                motorRightFront.setPower(direction * power);
                motorLeftBack.setPower(direction * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .85 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .8 * power);
                motorRightBack.setPower(direction * .8 * power);
                motorRightFront.setPower(direction * .8 * power);
                motorLeftBack.setPower(direction * .8 * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .9 * Math.abs(distance)) {
                motorLeftFront.setPower(direction * .7 * power);
                motorRightBack.setPower(direction * .7 * power);
                motorRightFront.setPower(direction * .7 * power);
                motorLeftBack.setPower(direction * .7 * power);
            } else if (Math.abs(motorLeftFront.getCurrentPosition()) < .95 * Math.abs(distance)) {
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

    public void depositMarker() {
        try {
//            turnBottomArm(0.9, -1, 600);
//            turnTopArm(0.9, 1, 600);
//
//            grabBase.setPosition(0.2);
//            sleep(1000);
//            grabBase.setPosition(0.7);
//            turnTopArm(0.6, -1, 500);
//            turnBottomArm(0.6, 1, 500);

            grabServo.setPosition(1);
            sleep(1000);
            grabServo.setPosition(0.5);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void makeParallelLeft() {
        double sensor_gap = 24;
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

        double dis = distanceSensor_lb.getDistance(DistanceUnit.CM);
        if (dis < 9) {
            strafe(0.7, -1, 200);
        } else if (dis > 12) {
            strafe(0.7, 1, ((dis - 11) / 2.54) * 200);
        }
    }

    public void makeParallelRight() {

        double sensor_gap = 26;
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

        double dis = distanceSensor_rb.getDistance(DistanceUnit.CM);
        if (dis < 13) {
            strafe(0.7, 1, 200);
        } else if (dis > 16) {
            strafe(0.7, -1, ((dis - 15) / 2.54) * 200);
        }
    }





    @Override
    public void runOpMode() {


        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        //parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == FRONT) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();

        targetsSkyStone.activate();

        CVUtil cvUtil = new CVUtil();
        cvUtil.initCv(hardwareMap.appContext);

        VuforiaLocalizer.CloseableFrame frame;


        int count = 0;
        while (!isStopRequested()) {

            try {

                telemetry.addData("Trying to get OpenCV Frame", "none");
                telemetry.update();

                frame = vuforia.getFrameQueue().take();

                long numImages = frame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        Image rgb = frame.getImage(i);
                        Mat mat = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);

                        if (rgb != null) {
                            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(),
                                    Bitmap.Config.RGB_565);
                            bm.copyPixelsFromBuffer(rgb.getPixels());

                            cvUtil.detectColor(mat);
                            cvUtil.updateFrame(bm, frame);


                        }
                    }
                }



                telemetry.addData("Open CV Got Frame", count++);
                telemetry.update();


            } catch (InterruptedException e) {
                //Log.v(TAG, "Exception!!");
                System.out.println("get frame exception");
                telemetry.addData("Open CV Exception", "none");


                break;
            }


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

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();




        }
    }




        public void turnTopArm_E ( int degrees){

            int ticks = 1440 * degrees / (360 * 4);
            armTop.setMode(RUN_USING_ENCODER);
            armTop.setMode(STOP_AND_RESET_ENCODER);
            armTop.setMode(RUN_TO_POSITION);
            armTop.setTargetPosition(10);
            armTop.setPower(0.6);
            //while (armTop.getCurrentPosition()<ticks)
            sleep(1);
            armTop.setMode(RUN_WITHOUT_ENCODER);


        }

        public void turnBottomArm_E ( int degrees){

            int ticks = 3 * 1440 * degrees / (360 * 4);
            armBottom.setMode(RUN_USING_ENCODER);
            armBottom.setMode(STOP_AND_RESET_ENCODER);
            armBottom.setMode(RUN_TO_POSITION);
            armBottom.setTargetPosition(10);
            armBottom.setPower(0.6);
            //while(armBottom.getCurrentPosition()<ticks)
            sleep(1);
            armBottom.setMode(RUN_WITHOUT_ENCODER);
        }

        public void turnBottomArm ( double power, int direction, double distance){

            armBottom.setMode(STOP_AND_RESET_ENCODER);


            telemetry.addData("turnBottomArm", "In straight()");
            telemetry.update();


            armBottom.setMode(RUN_WITHOUT_ENCODER);
            armBottom.setDirection(DcMotorSimple.Direction.REVERSE);

            int count = 0;
            while (Math.abs(armBottom.getCurrentPosition()) < Math.abs(distance)
                    && count++ < 50 && opModeIsActive() && !isStopRequested()) {

                telemetry.addData("armBottom Position", armBottom.getCurrentPosition());
                telemetry.update();
                armBottom.setPower(direction * .4 * power);

                if (Math.abs(armBottom.getCurrentPosition()) < .05 * Math.abs(distance)) {
                    armBottom.setPower(direction * power);
                } else if (Math.abs(armBottom.getCurrentPosition()) < .5 * Math.abs(distance)) {
                    armBottom.setPower(direction * .8 * power);

                } else if (Math.abs(armBottom.getCurrentPosition()) < .6 * Math.abs(distance)) {
                    armBottom.setPower(direction * .6 * power);
                } else if (Math.abs(armBottom.getCurrentPosition()) < .9 * Math.abs(distance)) {
                    armBottom.setPower(direction * .5 * power);

                } else {
                    armBottom.setPower(direction * .2 * power);
                }
            }
            armBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armBottom.setPower(0);

            //stopRobot and change modes back to normal
            armBottom.setMode(STOP_AND_RESET_ENCODER);

            // sleep(100);


        }
        public void turnTopArm ( double power, int direction, double distance){

            armTop.setMode(STOP_AND_RESET_ENCODER);


            telemetry.addData("armTop", "In straight()");
            telemetry.update();


            armTop.setMode(RUN_WITHOUT_ENCODER);
            armTop.setDirection(DcMotorSimple.Direction.REVERSE);

            int count = 0;
            while (Math.abs(armTop.getCurrentPosition()) < Math.abs(distance)
                    && count++ < 30 && opModeIsActive() && !isStopRequested()) {

                telemetry.addData("armTop Position", armTop.getCurrentPosition());
                telemetry.update();
                armTop.setPower(direction * .4 * power);

                if (Math.abs(armTop.getCurrentPosition()) < .4 * Math.abs(distance)) {
                    armTop.setPower(direction * power);
                } else if (Math.abs(armTop.getCurrentPosition()) < .8 * Math.abs(distance)) {
                    armTop.setPower(direction * .8 * power);

                } else if (Math.abs(armTop.getCurrentPosition()) < .85 * Math.abs(distance)) {
                    armTop.setPower(direction * .7 * power);
                } else if (Math.abs(armTop.getCurrentPosition()) < .9 * Math.abs(distance)) {
                    armTop.setPower(direction * .5 * power);

                } else {
                    armTop.setPower(direction * .4 * power);
                }
            }
            armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armTop.setPower(0.2);

            //stopRobot and change modes back to normal
            armTop.setMode(STOP_AND_RESET_ENCODER);

            //    sleep(100);


        }


    }



