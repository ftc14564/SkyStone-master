package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
    Servo grab_back;
    Servo grab_front;
    DcMotor extend;
    Servo foundation;
    Servo sideArm;
    Servo sideArmBase;


    double basePower = 0.2;
    private static final double DEFAULT_POWER_REDUCTION_FACTOR = 0.8;
    double powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR;
    double turnPowerFactor = 0.6;
    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone = false;
    double power_multiplier;
    double armPosition;
    double angleToTurn;
    double liftPosition;
    private static final double REV_CORE_HEX_TICKS_PER_INCH = 47.127;
    private static final double LIFT_JUMP_RESOLUTION_DOWN = 1;
    private static final double LIFT_JUMP_RESOLUTION_UP = 3;

    private static final double LIFT_MAX_INCH = 16;

    protected static boolean USE_VUFORIA = false;


    private static final double LIFT_NON_SLIP_POWER = 0.2;
//    private static final double ARM_INCH_PER_MS = 1471.724;
    private static final double ARM_INCH_PER_MS = 735;

    private  double liftStallPower = LIFT_NON_SLIP_POWER;
    private double liftPrevPosition = 0;

    private DistanceSensor sensorRange_rf;
    private DistanceSensor sensorRange_rb;
    private DistanceSensor sensorRange_lf;
    private DistanceSensor sensorRange_lb;

    Rev2mDistanceSensor distanceSensor_rf;
    Rev2mDistanceSensor distanceSensor_rb;
    Rev2mDistanceSensor distanceSensor_lf;
    Rev2mDistanceSensor distanceSensor_lb;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    final double TICKS_PER_INCH_STRAFE = 174/3;
    final double TICKS_PER_INCH_STRAIGHT = 88/3;


    boolean vuInitDone = false;
    double y = 0;
    double x = 0;

    double where_x = 0;
    double where_y = 0;

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


        sensorRange_rf = hardwareMap.get(DistanceSensor.class, "2m_rf");
        distanceSensor_rf = (Rev2mDistanceSensor)sensorRange_rf;
        sensorRange_rb = hardwareMap.get(DistanceSensor.class, "2m_rb");
        distanceSensor_rb = (Rev2mDistanceSensor)sensorRange_rb;
        sensorRange_lf = hardwareMap.get(DistanceSensor.class, "2m_lf");
        distanceSensor_lf = (Rev2mDistanceSensor)sensorRange_lf;
        sensorRange_lb = hardwareMap.get(DistanceSensor.class, "2m_lb");
        distanceSensor_lb = (Rev2mDistanceSensor)sensorRange_lb;

        strafing = false;


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
        foundation = hardwareMap.servo.get("foundation");
        armPosition = 0;
        sideArm = hardwareMap.servo.get("sideArm");
        sideArmBase = hardwareMap.servo.get("sideArmBase");


        //grab_front.setPosition(0.1);
        //grab_back.setPosition(0.1);

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

        if(USE_VUFORIA)
            targetsSkyStone.activate();


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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Init: Thread Done ", "");
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

    public void vectorCombine(double x, double y, double turn) {
        telemetry.addData("x:", x);
        telemetry.addData("y:", y);
        telemetry.update();

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

    }

    public void setLiftPosition(double position){
        lift.setMode(RUN_WITHOUT_ENCODER);
        double coarseMargin = 0.25* REV_CORE_HEX_TICKS_PER_INCH;
        double fineMargin = 0.1* REV_CORE_HEX_TICKS_PER_INCH;

        telemetry.addData("TargetLift Value", position);

        while ((lift.getCurrentPosition() < (position - coarseMargin)) && !isStopRequested()){
            idle();
            System.out.println("A: pos:"+ position + "curr:" + lift.getCurrentPosition());
            lift.setPower(1);
            lift_assist.setPower(1);
            position-=1; //to avoid getting stuck at top position
        }

        while ((lift.getCurrentPosition() < (position - fineMargin)) && !isStopRequested()){
            while((lift.getCurrentPosition() != liftPrevPosition) && !isStopRequested()) {
                System.out.println("B: prev:"+ liftPrevPosition + "curr:" + lift.getCurrentPosition() + "pow:" + liftStallPower);

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
            grab_front.setPosition(1);
            grab_back.setPosition(0.5);
    }

    public void closeGrabber() {
        grab_front.setPosition(0.2);
        grab_back.setPosition(0.5);
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
            System.out.println("get frame exception");
            telemetry.addData("Open CV Exception", "none");
        }
    }


    @Override
    public void runOpMode() {

        double liftTarget = 0;
        teleopInitFn();

        waitForStart();

        lift.setPower(0);
        liftPosition = 2;

        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
        CVUtil cvUtil = new CVUtil();
        cvUtil.initCv(hardwareMap.appContext);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            idle();

            //captureFrame(cvUtil);

            float forward = -1 * gamepad1.right_stick_y;
            double turn_component = gamepad1.left_stick_x*turnPowerFactor;
            double x_component = gamepad1.right_stick_x;
            double y_component = -1 * gamepad1.right_stick_y;

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
                } else if (Math.abs(x_component) > 0.1 || (Math.abs(y_component)>0.1)) {
                    vectorCombine(x_component, y_component, turn_component);
                }
                else {
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


                if (gamepad1.right_trigger > 0) {
                    powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR - (gamepad1.right_trigger * 2 / 3);
                    //Subtract from 1 to make the trigger give a reduction in power
                    //Multiply by 2/3 to not completely reduce the power
                }

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

                if (gamepad2.a){
                    liftStallPower = LIFT_NON_SLIP_POWER;
                    setLiftPosition(Math.abs(0));
                    liftTarget = 0;
                    powerReductionFactor = DEFAULT_POWER_REDUCTION_FACTOR;

                }
                if (gamepad2.b){

                    liftTarget = 0.5*REV_CORE_HEX_TICKS_PER_INCH;
                }

                if (gamepad2.right_trigger > 0.1) {

                    extend.setPower(1);
                } else if (gamepad2.left_trigger > 0.1) {
                    extend.setPower(-1);
                } else {
                    extend.setPower(0);
                }


//            if (gamepad1.y) {
//                 grab_front.setPosition(1);
//                 grab_back.setPosition(1);
//            }

                if (gamepad1.dpad_left) { //GRABBER OPEN FOR COLLECTION
                    sideArm.setPosition(0.7);
                    sideArmBase.setPosition(0.9);
                }

                if(gamepad1.dpad_up){  //BLOCK LIFTED OFF GROUND
                    for (double i = 0.0 ; i<0.4;i+=0.05) {
                        sideArmBase.setPosition(0.7 - i);
                        sideArm.setPosition(0.2 + i/2);
                    }
                }
                if (gamepad1.dpad_right) { //GRABBER OPEN FOR COLLECTION
                    sideArm.setPosition(0.8);
                    sideArmBase.setPosition(0.7);
                }

                if(gamepad1.dpad_down){  //BLOCK GRABBED BUT ON GROUND
                    sideArmBase.setPosition(1);
                    sideArm.setPosition(0.2);

                }

                if (gamepad2.dpad_down) {               //GRABBED POSITION
                    grab_front.setPosition(0.2); //More than 90 degrees to add pressure
                    grab_back.setPosition(0.5);
                }
                if (gamepad2.dpad_up) {               //OPEN FOR COLLECTION POSITION
                    grab_front.setPosition(0.7);
                    grab_back.setPosition(0.5);
                }
                if (gamepad2.dpad_left) {               //Dropping
                    grab_front.setPosition(1);
                    grab_back.setPosition(0.4);
                }

                if (gamepad2.dpad_right) {               //OPEN FOR COLLECTION POSITION
                    grab_front.setPosition(0.7);           //AND LIFT TO NOT HIT BLOCK
                    grab_back.setPosition(0.4);

                    lift.setDirection(DcMotorSimple.Direction.FORWARD);
                    liftTarget = liftTarget + (0.5 * REV_CORE_HEX_TICKS_PER_INCH);

                }

                if (liftTarget > (0.5 * REV_CORE_HEX_TICKS_PER_INCH) ) {
                    powerReductionFactor = 0.3;
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

                if (gamepad1.x) {  //position up
                    foundation.setPosition(0);

                }
                if (gamepad1.y) {   //position down
                    foundation.setPosition(0.7);
                }


                if (lift.getCurrentPosition() < LIFT_MAX_INCH * REV_CORE_HEX_TICKS_PER_INCH && gamepad2.y) {

                    liftTarget = liftTarget + (LIFT_JUMP_RESOLUTION_UP * REV_CORE_HEX_TICKS_PER_INCH);
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
