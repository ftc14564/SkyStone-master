package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@TeleOp (name = "Teleop2020")
public class Teleop2020 extends LinearOpMode {

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor lift;
    DcMotor lift_assist;
    DcMotor extend;
    DcMotor tape_extend;
    Servo capstone;
    Servo grab_left;
    Servo grab_right;
    Servo foundation;
    Servo sideArmWheelRight;
    Servo sideArmMainRight;
    Servo sideArmWheelLeft;
    Servo sideArmMainLeft;



    double basePower = 0.2;
    protected static final double DEFAULT_POWER_REDUCTION_FACTOR = 0.4;
    protected double powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR;
    double turnPowerFactor = 0.6;
    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone = false;
    double power_multiplier;

    double armPosition;
    double angleToTurn;
    double liftPosition;
    protected static final double REV_CORE_HEX_TICKS_PER_INCH = 47.127;
    protected static final double LIFT_JUMP_RESOLUTION_DOWN = 1;
    protected static final double LIFT_JUMP_RESOLUTION_UP = 3;
    protected static final double FOUNDATION_UP = 0.1;
    protected static final double FOUNDATION_DOWN = 0.67;
    protected static final double SIDE_ARM_WHEEL_OPEN_RIGHT = 0.6;
    protected static final double SIDE_ARM_WHEEL_OPEN_LEFT = 0.5;
    protected static final double SIDE_ARM_WHEEL_UP_RIGHT = 0.55;
    protected static final double SIDE_ARM_WHEEL_UP_LEFT = 0.53;

    protected static final double SIDE_ARM_MAIN_UP_RIGHT = 0.9;
    protected static final double SIDE_ARM_MAIN_UP_LEFT = 0.45;
    protected static final double SIDE_ARM_MAIN_PRE_LEFT = 0.75;


    protected static final double SIDE_ARM_LIFTED = 0.2;
    protected static final double SIDE_ARM_BASE_LIFTED = 0.7;
    protected static final double SIDE_ARM_DROP = 0.1;
    protected static final double SIDE_ARM_MAIN_DOWN_RIGHT = 0.55;
    protected static final double SIDE_ARM_MAIN_DOWN_LEFT = 0.85;
    protected static final double SIDE_ARM_MAIN_PRE_RIGHT = 0.65;


    protected static final double SIDE_ARM_WHEEL_GRAB_RIGHT = 0;
    protected static final double SIDE_ARM_WHEEL_GRAB_LEFT = 0;

    protected static final double SIDE_ARM_MAIN_HALF_UP_RIGHT = 0.9;
    protected static final double SIDE_ARM_MAIN_HALF_UP_LEFT = 0.6;


    enum SideArmState {
        HOME,
        PRE_GRAB,
        PRE_GRAB_LOW,
        GRAB,
        GRAB_HOLD_LOW,
        GRAB_HOLD_HIGH,
        THROW
    }









    protected static final double LIFT_MAX_INCH = 16.5;

    protected static final double CAM_SIDE_ARM_OFFSET_LEFT = -5.7;
    protected static final double CAM_SIDE_ARM_OFFSET_RIGHT = -6.5;
    protected  double CAM_SIDE_ARM_OFFSET = CAM_SIDE_ARM_OFFSET_LEFT;



    protected static final double CAM_TO_FF = 7.5;
    protected static final double CAM_TO_BB = 10.5;


    protected static final double FF_DS_TO_SIDE_ARM = 3.5;
    protected static final double BB_DS_TO_SIDE_ARM = 14.5;


    protected static boolean USE_VUFORIA = false;


    protected static final boolean DEBUG = true;


    protected static final double LIFT_NON_SLIP_POWER = 0.2;
//    private static final double ARM_INCH_PER_MS = 1471.724;
    protected static final double ARM_INCH_PER_MS = 225;

    protected  double liftStallPower = LIFT_NON_SLIP_POWER;
    private double liftPrevPosition = 0;

    private DistanceSensor sensorRange_rf;
    private DistanceSensor sensorRange_rb;
    private DistanceSensor sensorRange_lf;
    private DistanceSensor sensorRange_lb;
    private DistanceSensor sensorRange_ffl;
    private DistanceSensor sensorRange_ffr;
    private DistanceSensor sensorRange_bbr;



    Rev2mDistanceSensor distanceSensor_rf;
    Rev2mDistanceSensor distanceSensor_rb;
    Rev2mDistanceSensor distanceSensor_lf;
    Rev2mDistanceSensor distanceSensor_lb;
    Rev2mDistanceSensor distanceSensor_ffl;
    Rev2mDistanceSensor distanceSensor_ffr;
    Rev2mDistanceSensor distanceSensor_bbr;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    final double TICKS_PER_INCH_STRAFE = 140/3;
    final double TICKS_PER_INCH_STRAIGHT = 88/3;

    enum FldDirection{
        Face_Fld_Center,
        Face_Fld_Foundation,
        Face_Fld_Audience,
        Face_Fld_Drivers,
        Face_Fld_Center_Foundation
    }

    boolean vuInitDone = false;
    double vu_y = 0;
    double vu_x = 0;

    int ss_position=3;

    double where_cam_x = 0;
    double where_cam_y = 0;
    double ds_prev_read = 120;
    FldDirection where_head = FldDirection.Face_Fld_Center;
    boolean isBlueSide = false;

    protected double running_counter = 0;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity, gravity1;
    BNO055IMU.Parameters parameters;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = "AYpOJ0H/////AAABGeEbm+5m+k5BrTnPlF3X9R177NGoUFUGl1kpgLa7MBwlsRdnD3IdxY7LmZ41NTQMASZ1MbCWaEpM4Sag7tDfQsJjqVvCwZr3qJm5y33J8rnMWz1ViOwwzZgnsSZqeGRY9+uPGa6cTMO/cxs+YF+4OqsD+iu4exeMCsxyAPYhXQrEIaW6h7zYVrdi9b5WsgNGUfP60Qz8U3szKTfVmaHmMFvc+iuJ1qmAM5AjlsBlc8MMHzLAL/3sf3UiCDe4tgo4mmYEsdl499QhqhhImEiKS8rTkap/53B8Hm89z3m5HuBoH4EKVUc65k2aCBg5c5jXVoZan8DkQFqSPnArwQnCHpaL/d1y79BRE44nJXj54E6V";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    protected static final float mmPerInch = 25.4f;
    protected static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    protected static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    protected static final float bridgeZ = 6.42f * mmPerInch;
    protected static final float bridgeY = 23 * mmPerInch;
    protected static final float bridgeX = 5.18f * mmPerInch;
    protected static final float bridgeRotY = 59;                                 // Units are degrees
    protected static final float bridgeRotZ = 180;
    protected static final float stoneZ = 2.00f * mmPerInch;
    // Constants for perimeter targets
    protected static final float halfField = 72 * mmPerInch;
    protected static final float quadField = 36 * mmPerInch;
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;


    protected OpenGLMatrix lastLocation = null;
    protected  boolean targetVisible = false;
    protected List<VuforiaTrackable> allTrackables;

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
    boolean red = false;
    boolean blue = false;


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters vu_parameters;

    VuforiaTrackables targetsSkyStone ;
    VuforiaTrackable stoneTarget ;

    OpenGLMatrix robotFromCamera ;

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line




    public void teleopInitFn() {

        telemetry.addData("Init: start ", "");

        if(DEBUG) System.out.println(" 14564dbg : Init start ");


        sensorRange_rf = hardwareMap.get(DistanceSensor.class, "2m_rf");
        distanceSensor_rf = (Rev2mDistanceSensor)sensorRange_rf;
        sensorRange_rb = hardwareMap.get(DistanceSensor.class, "2m_rb");
        distanceSensor_rb = (Rev2mDistanceSensor)sensorRange_rb;
        sensorRange_lf = hardwareMap.get(DistanceSensor.class, "2m_lf");
        distanceSensor_lf = (Rev2mDistanceSensor)sensorRange_lf;
        sensorRange_lb = hardwareMap.get(DistanceSensor.class, "2m_lb");
        distanceSensor_lb = (Rev2mDistanceSensor)sensorRange_lb;

        sensorRange_ffl = hardwareMap.get(DistanceSensor.class, "ffLeft");
        distanceSensor_ffl = (Rev2mDistanceSensor)sensorRange_ffl;
        sensorRange_ffr = hardwareMap.get(DistanceSensor.class, "ffRight");
        distanceSensor_ffr = (Rev2mDistanceSensor)sensorRange_ffr;
        sensorRange_bbr = hardwareMap.get(DistanceSensor.class, "bbRight");
        distanceSensor_bbr = (Rev2mDistanceSensor)sensorRange_bbr;

        strafing = false;


        lift = hardwareMap.dcMotor.get("lift");
        lift_assist = hardwareMap.dcMotor.get("lift_assist");
        lift.setMode(STOP_AND_RESET_ENCODER);
        tape_extend= hardwareMap.dcMotor.get("tape_extend");
        tape_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        capstone = hardwareMap.servo.get("capstone");
        grab_right = hardwareMap.servo.get("grab_right");
        grab_left = hardwareMap.servo.get("grab_left");
        foundation = hardwareMap.servo.get("foundation");
        armPosition = 0;
        sideArmWheelRight = hardwareMap.servo.get("sideArmWheelRight");
        sideArmMainRight = hardwareMap.servo.get("sideArmMainRight");
        sideArmWheelLeft = hardwareMap.servo.get("sideArmWheelLeft");
        sideArmMainLeft = hardwareMap.servo.get("sideArmMainLeft");



        //grab_right.setPosition(0.1);
        //grab_left.setPosition(0.1);

        extend = hardwareMap.dcMotor.get("extend");

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


        if(USE_VUFORIA) {
            if(DEBUG) System.out.println(" 14564dbg : Init Vuforia");

            //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            vu_parameters.cameraDirection = FRONT;

            vu_parameters.vuforiaLicenseKey = VUFORIA_KEY;

            /**
             * We also indicate which camera on the RC we wish to use.
             */
            //vu_parameters.cameraName = webcamName;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);


            // Load the data sets for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
            stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsSkyStone);
            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            //Set the position of the bridge support targets with relation to origin (center of field)
//        blueFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
//
//        blueRearBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
//
//        redFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
//
//        redRearBridge.setLocation(OpenGLMatrix
//                .translation(bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
//
//        //Set the position of the perimeter targets with relation to origin (center of field)
//        red1.setLocation(OpenGLMatrix
//                .translation(quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        red2.setLocation(OpenGLMatrix
//                .translation(-quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        front1.setLocation(OpenGLMatrix
//                .translation(-halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        front2.setLocation(OpenGLMatrix
//                .translation(-halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blue1.setLocation(OpenGLMatrix
//                .translation(-quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        blue2.setLocation(OpenGLMatrix
//                .translation(quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        rear1.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        rear2.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            // Rotate the phone vertical about the X axis if it's in portrait mode
            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90;
            }


            robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vu_parameters.cameraDirection);
            }

            vuforia.setFrameQueueCapacity(1);

            targetsSkyStone.activate();
        }

        if(DEBUG) System.out.println(" 14564dbg : Init IMU");


        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // get a reference to the color sensor.
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        if(DEBUG) System.out.println(" 14564dbg : Init IMU 1");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        if(DEBUG) System.out.println(" 14564dbg : Init IMU 2");

        imu.initialize(parameters);
        if(DEBUG) System.out.println(" 14564dbg : Init IMU 3");

        telemetry.addData("Init: Thread Done ", "");
        telemetry.update();

        if(DEBUG) System.out.println(" 14564dbg : Init Done");

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


    public void vectorCombineSimple(double x, double y, double turn) {


        double a = powerReductionFactor*(x + y) + turn;
        double b = powerReductionFactor*(y - x) - turn;
        double c = -powerReductionFactor*(y - x) - turn;
        double d = -powerReductionFactor*(x + y) + turn;

        motorLeftFront.setPower(a);
        motorRightFront.setPower(b);
        motorLeftBack.setPower(c);
        motorRightBack.setPower(d);

        if(DEBUG) System.out.println("14564dbg vectorCombineSimple a " + a + " b " + b + " c " + c + " d " + d );

    }

    public void vectorCombine(double x, double y, double turn) {


        double a = powerReductionFactor*(x + y) + basePower * ((x + y) / Math.abs(x + y));
        double b = powerReductionFactor*(y - x) + basePower * ((y - x) / Math.abs(y - x));
        double c = -powerReductionFactor*(y - x) - basePower * ((y - x) / Math.abs(y - x));
        double d = -powerReductionFactor*(x + y) - basePower * ((x + y) / Math.abs(x + y));

        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setPower(a + turn);
        motorRightFront.setPower(b - turn);
        motorLeftBack.setPower(c - turn);
        motorRightBack.setPower(d + turn);

        if(DEBUG) System.out.println("14564dbg vectorCombine a " + a + " b " + b + " c " + c + " d " + d );

//        telemetry.addData("x:", x);
//        telemetry.addData("y:", y);
//
//        telemetry.addData("a:", a);
//        telemetry.addData("b:", b);
//        telemetry.addData("c:", c);
//        telemetry.addData("d:", d);
//
//        telemetry.update();
    }

    public void setLiftPosition(double position){
        lift.setMode(RUN_WITHOUT_ENCODER);
        double coarseMargin = 0.25* REV_CORE_HEX_TICKS_PER_INCH;
        double fineMargin = 0.1* REV_CORE_HEX_TICKS_PER_INCH;

        telemetry.addData("TargetLift Value", position);

        liftPrevPosition = lift.getCurrentPosition();

        if (Math.abs(position) > (LIFT_MAX_INCH*REV_CORE_HEX_TICKS_PER_INCH))
            position = LIFT_MAX_INCH*REV_CORE_HEX_TICKS_PER_INCH;

        double stall_counter = 0;
        while ((lift.getCurrentPosition() < (position - coarseMargin)) && !isStopRequested()){
            idle();
            stopWheels();
            if(DEBUG) System.out.println(" 14564dbg A: pos:"+ position + "curr:" + lift.getCurrentPosition());
            lift.setPower(1);
            lift_assist.setPower(1);

            if (liftPrevPosition == lift.getCurrentPosition()) {
                stall_counter++;
            } else {
                stall_counter = 0;
                liftPrevPosition = lift.getCurrentPosition();
            }
            if (stall_counter > 3) {
                position = liftPrevPosition;
                break;
            }
        }
        position+=1+fineMargin;

        while ((lift.getCurrentPosition() < (position - fineMargin)) && !isStopRequested()){
            while((lift.getCurrentPosition() != liftPrevPosition) && !isStopRequested()) {
                if(DEBUG) System.out.println("B: prev:"+ liftPrevPosition + "curr:" + lift.getCurrentPosition() + "pow:" + liftStallPower);
                stopWheels();
                lift.setPower(liftStallPower);
                lift_assist.setPower(liftStallPower);
                idle();
                liftPrevPosition = lift.getCurrentPosition();
                if(lift.getCurrentPosition() < position)
                    liftStallPower+=0.01;
                else
                    liftStallPower-=0.01;

            }

            idle();
            if(liftPrevPosition == lift.getCurrentPosition())
                break;
            else
                liftStallPower+=0.1;
        }

        while ((lift.getCurrentPosition() > (position + coarseMargin)) && !isStopRequested()) {
            idle();
            lift.setPower(-0.25);
            lift_assist.setPower(-0.25);


        }
        //double noSlipPower = LIFT_NON_SLIP_POWER + (position / (REV_CORE_HEX_TICKS_PER_INCH * 32));
        lift.setPower(liftStallPower);
     //   lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_assist.setPower(liftStallPower);
     //   lift_assist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void liftInch(double inches) {

        lift.setMode(RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        double ticks = inches * REV_CORE_HEX_TICKS_PER_INCH;

        if ((Math.abs(lift.getCurrentPosition()) < Math.abs(ticks))) {
            while ((Math.abs(lift.getCurrentPosition()) < Math.abs(ticks)) && !isStopRequested()) {
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



    public void setArmPosition_startPosition() {

    }

        public void armExtended(double inches){
        long currentTime = System.currentTimeMillis();
        double targetTime = Math.abs(inches) * ARM_INCH_PER_MS + currentTime;
        while((currentTime < targetTime ) && !isStopRequested()){
            idle();
            currentTime = System.currentTimeMillis();
            extend.setPower(1);
        }
        extend.setPower(0);
    }
    public void grabCollection() {
        grab_right.setPosition(0.8);
        grab_left.setPosition(0.5);
    }

    public void closeGrabber() {
        grab_right.setPosition(0.2);
        grab_left.setPosition(0.5);
    }

    public void captureFrame(CVUtil cvUtil) {
       int count = 0;

        try {

            telemetry.addData("Trying to get OpenCV Frame", "none");
            telemetry.update();

            Frame frame = vuforia.getFrameQueue().take();

            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    Mat mat = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);

                    if (rgb != null) {
                        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(),
                                Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(rgb.getPixels());

//                            cvUtil.detectColor(mat);
                        cvUtil.updateFrame(bm, frame);

                        telemetry.addData("Open CV Got Frame", count++);
                        telemetry.update();
                    }
                }
            }




        } catch (InterruptedException e) {
            //Log.v(TAG, "Exception!!");
            if(DEBUG) System.out.println("get frame exception");
            telemetry.addData("Open CV Exception", "none");
        }
    }
    public void sideArmSetStateLeft(SideArmState state)
    {


        if (state == SideArmState.HOME) { //GRABBER OPEN FOR COLLECTION
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_UP_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_UP_LEFT);
        }

        if (state == SideArmState.PRE_GRAB) { //GRABBER OPEN FOR COLLECTION
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_UP_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_PRE_LEFT);
        }

        if (state == SideArmState.PRE_GRAB_LOW) { //GRABBER OPEN FOR COLLECTION
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_UP_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_DOWN_LEFT);
        }

        if(state == SideArmState.GRAB){  //BLOCK GRABBED BUT ON GROUND
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_GRAB_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_DOWN_LEFT);
        }


        if(state == SideArmState.GRAB_HOLD_LOW){  //BLOCK LIFTED OFF GROUND
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_OPEN_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_UP_LEFT);
        }

        if(state == SideArmState.GRAB_HOLD_HIGH){  //BLOCK LIFTED OFF GROUND
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_GRAB_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_UP_LEFT);

        }

        if (state == SideArmState.THROW) { //DROP BLOCK
            sideArmWheelLeft.setPosition(SIDE_ARM_WHEEL_UP_LEFT);
            sideArmMainLeft.setPosition(SIDE_ARM_MAIN_HALF_UP_LEFT);
        }



    }

    public void sideArmSetStateRight(SideArmState state)
    {


        if (state == SideArmState.HOME) { //GRABBER OPEN FOR COLLECTION
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_UP_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_UP_RIGHT);
        }

        if (state == SideArmState.PRE_GRAB) { //GRABBER OPEN FOR COLLECTION
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_UP_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_PRE_RIGHT);
        }

        if (state == SideArmState.PRE_GRAB_LOW) { //GRABBER OPEN FOR COLLECTION
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_UP_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_DOWN_RIGHT);
        }

        if(state == SideArmState.GRAB){  //BLOCK GRABBED BUT ON GROUND
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_GRAB_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_DOWN_RIGHT);
        }


        if(state == SideArmState.GRAB_HOLD_LOW){  //BLOCK LIFTED OFF GROUND
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_OPEN_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_UP_RIGHT);
        }

        if(state == SideArmState.GRAB_HOLD_HIGH){  //BLOCK LIFTED OFF GROUND
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_GRAB_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_UP_RIGHT);

        }

        if (state == SideArmState.THROW) { //DROP BLOCK
            sideArmWheelRight.setPosition(SIDE_ARM_WHEEL_UP_RIGHT);
            sideArmMainRight.setPosition(SIDE_ARM_MAIN_HALF_UP_RIGHT);
        }



    }

    @Override
    public void runOpMode() {

        double liftTarget = 0;

        if(DEBUG) System.out.println(" 14564dbg : Opmode start ");

        USE_VUFORIA = false;

        teleopInitFn();
        if(DEBUG) System.out.println(" 14564dbg : Init done ");

        waitForStart();
        if(DEBUG) System.out.println(" 14564dbg : Starting  ...  ");


        lift.setPower(0);
        liftPosition = 2;

        if(USE_VUFORIA) {
            vuforia.setFrameQueueCapacity(1);
            vuforia.enableConvertFrameToBitmap();
            CVUtil cvUtil = new CVUtil();
            cvUtil.initCv(hardwareMap.appContext);
        }


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            idle();

            //captureFrame(cvUtil);

            float forward = -1 * gamepad1.right_stick_y;
            double turn_component = gamepad1.left_stick_x * turnPowerFactor;
            double x_component = gamepad1.right_stick_x;
            double y_component = -1 * gamepad1.right_stick_y;

            if (y_component == 0)
                y_component = 0.001;
            if (x_component == 0)
                x_component = 0.001;

            if (gamepad1.right_trigger > 0) {
                powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR + (gamepad1.right_trigger * 2 / 3);
                //Subtract from 1 to make the trigger give a reduction in power
                //Multiply by 2/3 to not completely reduce the power
            }


            if (gamepad1.left_bumper && Math.abs(forward) > 0.1) {
                //right turn
                motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);  //changed
                motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);   //changed
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE); //default
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default
                motorRightFront.setPower(forward * power_multiplier);
                motorRightBack.setPower(forward * power_multiplier);
                motorLeftFront.setPower(forward * power_multiplier);
                motorLeftBack.setPower(forward * power_multiplier);
            } else if (gamepad1.right_bumper && Math.abs(forward) > 0.1) {
                //left turn
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD); //changed
                motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);  //changed
                motorRightFront.setPower(forward * power_multiplier);
                motorRightBack.setPower(forward * power_multiplier);
                motorLeftFront.setPower(forward * power_multiplier);
                motorLeftBack.setPower(forward * power_multiplier);
            } else if (gamepad1.left_trigger > 0.1 && Math.abs(x_component) > 0.1) {
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE); //default
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default

                motorRightFront.setPower(-0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
                motorRightBack.setPower(0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
                motorLeftFront.setPower(0.5 * (x_component / Math.abs(x_component)) * power_multiplier);
                motorLeftBack.setPower(-0.5 * (x_component / Math.abs(x_component)) * power_multiplier);

            } else if (gamepad1.left_trigger > 0.1 && Math.abs(y_component) > 0.1) {
                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);  //default
                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE); //default
                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);  //default

                motorRightFront.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
                motorRightBack.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
                motorLeftFront.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);
                motorLeftBack.setPower(0.3 * (y_component / Math.abs(y_component)) * power_multiplier);

            }
//                else if (gamepad1.left_trigger > 0.1 && Math.abs(y_component) > 0.1){
//                    vectorCombineSimple(0, 0.5*(y_component/Math.abs(y_component)), 0);
//                }


            else if ((Math.abs(x_component) > 0.1) || (Math.abs(y_component) > 0.1)) {
                vectorCombine(x_component, y_component, turn_component * (y_component / Math.abs(y_component)));
            } else {
                motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRightFront.setPower(0);
                motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRightBack.setPower(0);
                motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeftFront.setPower(0);
                motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeftBack.setPower(0);
            }

//            else

//            }
//            else
//            {
//                motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
//                motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//                motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//                motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//                motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                motorRightFront.setPower(0);
//                motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                motorRightBack.setPower(0);
//                motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                motorLeftFront.setPower(0);
//                motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                motorLeftBack.setPower(0);
//            }


            if (gamepad1.b) {
                if (powerReductionFactor == 0.8)
                    powerReductionFactor = 0.4;
                else if (powerReductionFactor == 0.4)
                    powerReductionFactor = 0.8;
                sleep(200);
            }
//                if (lift.getCurrentPosition() >= 0 && gamepad2.left_stick_y > 0.1) {
//                    lift.setPower(-1);
//                    lift_assist.setPower(-1);
//
//                } else if (gamepad2.left_stick_y < -0.1) {
//                    lift.setPower(1);
//                    lift_assist.setPower(1);
//
//                } else {
//                    lift.setPower(0);
//                    lift_assist.setPower(0);
//
//                }

            if (gamepad2.a) {
                liftStallPower = LIFT_NON_SLIP_POWER;
                stopWheels();
                setLiftPosition(Math.abs(0));
                liftTarget = 0;
                powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR;

            }
            if (gamepad2.b) {

                liftTarget = 0.5 * REV_CORE_HEX_TICKS_PER_INCH;
            }

            if (gamepad2.right_trigger > 0.1) {

                extend.setPower(1);
            } else if (gamepad2.left_trigger > 0.1) {
                extend.setPower(-1);
            } else {
                extend.setPower(0);
            }


//            if (gamepad1.y) {
//                 grab_right.setPosition(1);
//                 grab_left.setPosition(1);
//            }

//                if (gamepad1.right_bumper) { //GRABBER OPEN FOR COLLECTION or Home with left trigger
//                        sideArmSetStateRight(SideArmState.HOME);
//                }
//                if (gamepad1.left_bumper) { //GRABBER OPEN FOR COLLECTION or Home with left trigger
//                    sideArmSetStateLeft(SideArmState.HOME);
//
//                }
            if (gamepad1.right_trigger > 0.5) {
                if (gamepad1.dpad_down) {  //BLOCK GRAB
                    sideArmSetStateRight(SideArmState.GRAB);
                }
                if (gamepad1.dpad_up) {  //BLOCK HOLD HIGH
                    sideArmSetStateRight(SideArmState.GRAB_HOLD_HIGH);
                }

                if (gamepad1.dpad_right) {  //BLOCK THROW
                    sideArmSetStateRight(SideArmState.THROW);
                }
                if (gamepad1.dpad_left) {  //BLOCK THROW
                    sideArmSetStateRight(SideArmState.PRE_GRAB);
                }
            }
            if (gamepad1.left_trigger > 0.5) {
                if (gamepad1.dpad_down) {  //BLOCK GRAB
                    sideArmSetStateLeft(SideArmState.GRAB);
                }
                if (gamepad1.dpad_up) {  //BLOCK HOLD HIGH
                    sideArmSetStateLeft(SideArmState.GRAB_HOLD_HIGH);
                }

                if (gamepad1.dpad_right) {  //BLOCK THROW
                    sideArmSetStateLeft(SideArmState.THROW);
                }
                if (gamepad1.dpad_left) {  //BLOCK THROW
                    sideArmSetStateLeft(SideArmState.PRE_GRAB);
                }
            }


            if (gamepad2.dpad_up) {               //GRABBED POSITION
                grab_right.setPosition(0.7); //More than 90 degrees to add pressure
                grab_left.setPosition(0.3);
            }
            if (gamepad2.dpad_down) {               //OPEN FOR COLLECTION POSITION
                grab_right.setPosition(0.9);
                grab_left.setPosition(0.15);
            }
            if (gamepad2.dpad_left){
                if (gamepad2.left_bumper) {
                    tape_extend.setPower(1);
//                } else if (gamepad2.right_bumper) {
//                    tape_extend.setPower(-1);
                } else {
                    tape_extend.setPower(0);
                }
            }
            if (gamepad2.dpad_right && gamepad2.left_bumper){
                    capstone.setPosition(0.0);
                }
            if (gamepad2.dpad_right && gamepad2.right_bumper){
                    capstone.setPosition(1);
                }

//               if (gamepad2.dpad_left) {               //Dropping
//                    grab_right.setPosition(1);
//                    grab_left.setPosition(0.52);
//                }
//
//                if (gamepad2.dpad_right) {               //OPEN FOR COLLECTION POSITION
//                    grab_right.setPosition(0.7);           //AND LIFT TO NOT HIT BLOCK
//                    grab_left.setPosition(0.52);

//                    liftTarget = liftTarget + (0.5 * REV_CORE_HEX_TICKS_PER_INCH);
//
//                }

            if (liftTarget > (0.5 * REV_CORE_HEX_TICKS_PER_INCH)) {
                powerReductionFactor = 0.3;
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

                if (gamepad1.x) {  //position up
                    foundation.setPosition(FOUNDATION_UP);

                }
                if (gamepad1.y) {   //position down
                    foundation.setPosition(FOUNDATION_DOWN);
                }


                if (lift.getCurrentPosition() < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH && gamepad2.y) {

                    liftTarget = liftTarget + (LIFT_JUMP_RESOLUTION_UP * REV_CORE_HEX_TICKS_PER_INCH);
                }

            if (((lift.getCurrentPosition() + REV_CORE_HEX_TICKS_PER_INCH*3) < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH) && (gamepad2.left_stick_y < - 0.2)) {

                liftTarget =  liftTarget +  REV_CORE_HEX_TICKS_PER_INCH*3;
            }
            if (((lift.getCurrentPosition() + REV_CORE_HEX_TICKS_PER_INCH*8) < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH)  && (gamepad2.left_stick_y > 0.2)) {

                liftTarget =  liftTarget +  REV_CORE_HEX_TICKS_PER_INCH*8;
            }
            if (((lift.getCurrentPosition() + REV_CORE_HEX_TICKS_PER_INCH*13) < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH)  && (gamepad2.right_stick_y < -0.2)) {

                liftTarget =  liftTarget +  REV_CORE_HEX_TICKS_PER_INCH*13;
            }
            if (((lift.getCurrentPosition() + REV_CORE_HEX_TICKS_PER_INCH*17) < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH)  && (gamepad2.right_stick_y > 0.2)) {

                liftTarget =  liftTarget +  REV_CORE_HEX_TICKS_PER_INCH*17;
            }

                if (gamepad2.x) {
                    liftTarget = liftTarget - (LIFT_JUMP_RESOLUTION_DOWN * REV_CORE_HEX_TICKS_PER_INCH);
                }

                if(Math.abs(liftTarget) > 5) {
                    setLiftPosition(Math.abs(liftTarget));
                }


            }
//


//        if (gamepad2.y) {
//            telemetry.addData("grab position y", grabServo.getPosition() );
//            grabServo.setPosition(1);
//        }
//
//            if (gamepad2.x) {
//            telemetry.addData("grab position a", grabServo.getPosition() );
//            grabServo.setPosition(0);
//        }
//
//        if (gamepad2.y) {
//            extend.setPower(0.8);
//        }
//        grabServo.setPosition(0);
            extend.setPower(0);
            motorLeftBack.setPower(0);
            motorLeftFront.setPower(0);
            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
            lift.setPower(0);
            lift_assist.setPower(0);
            telemetry.addData("lift encoder value", lift.getCurrentPosition());




        }
    }
