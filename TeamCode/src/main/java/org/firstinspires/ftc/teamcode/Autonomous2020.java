package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;



@Autonomous(name = "Autonomous2020")
public class Autonomous2020 extends Teleop2020  {



    private Navigate navigator;
    private DistanceSensor sensorRange_rf;
    private DistanceSensor sensorRange_rb;
    private DistanceSensor sensorRange_lf;
    private DistanceSensor sensorRange_lb;

    double distanceToWall;

    Rev2mDistanceSensor distanceSensor_rf;
    Rev2mDistanceSensor distanceSensor_rb;
    Rev2mDistanceSensor distanceSensor_lf;
    Rev2mDistanceSensor distanceSensor_lb;
    Rev2mDistanceSensor distanceSensor_rf_Straight;
    Rev2mDistanceSensor distanceSensor_rb_Straight;
    Rev2mDistanceSensor distanceSensor_lf_Straight;
    Rev2mDistanceSensor distanceSensor_lb_Straight;
    boolean avoidObstacles;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    long timer = 0;
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    final double TICKS_PER_INCH_STRAFE = 126.00;
    final double TICKS_PER_INCH_STRAIGHT = 89.1;

    float power = 0;
    float track = 0;
    boolean strafing;
    boolean initDone = false;
    boolean vuInitDone = false;
    double y = 0;
    double x = 0;

    double where_x = 18;
    double where_y = 0;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity, gravity1;
    BNO055IMU.Parameters parameters;
    private static final boolean PHONE_IS_PORTRAIT = false;
    boolean strafedRightToAvoidObstacles;

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
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


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
            initDone = true;
            telemetry.addData("Init: Thread done ", "");

            while (!isStopRequested()) {

                sleep(20);

            }

        }

    }

    ;


    public void initFn() {
        teleopInitFn();

        navigator = new Navigate(this);
        telemetry.addData("Init: start ", "");


        strafing = false;

        double driveSpeed = 0;

        new Thread(new InitThread_Depot()).start();

        // webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        vu_parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vu_parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        // parameters.cameraName = webcamName;

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

        targetsSkyStone.activate();

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
    public void logging(boolean activate, String message){
        if(activate)
        System.out.println(message);
    }

    // test function for obstacle simulation
//    public void getTargetXAndY(){
//        Random random = new Random();
//        int targetX = random.nextInt(145);
//        int targetY = random.nextInt(145);
//        logging(true, "target x: " + targetX + "    target y: " + targetY);
//        //getting Target values
//
//        int obstacleTargetX = random.nextInt(145);
//        int obstacleTargetY = random.nextInt(145);
//        logging(true, "obstacleTarget x: " + obstacleTargetX  + "    obstacleTarget y: " + obstacleTargetY);
//        //getting obstacle Target values
//
//        if(targetY != obstacleTargetY && targetX != obstacleTargetX) {
//            EncoderMoveDist(1, targetY - where_y, false);
//            Rotate(1, 1, 90);
//            EncoderMoveDist(1, targetX - where_x, false);
//        }
//        else if (targetY == obstacleTargetY){
//            Rotate(1, 1, 90);
//            EncoderMoveDist(1, targetX - where_x, false);
//            Rotate(1,-1,90);
//            EncoderMoveDist(1, targetY - where_y, false);
//        }
//        else if(targetX == obstacleTargetX){
//            EncoderMoveDist(1, targetY - where_y, false);
//            Rotate(1, 1, 90);
//            EncoderMoveDist(1, targetX - where_x, false);
//        }
//    }

    public boolean goTo(double x, double y) {
       return navigator.goTo(x,y);
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
        EncoderMoveDist(1, dist,false, false);
    }

    public void EncoderStrafe(double dist){
        EncoderMoveDist(1, dist,true, false);
    }

    double P_FWD_COEFF = -0.005;
    double FWD_THRESHOLD = 1;

    public void EncoderMoveDist(double speed, double distance, Boolean strafe, Boolean checkForObstacles) {
        checkForObstacles = avoidObstacles;
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
            speed *= 0.7;
        }
        else {
            target_encoder = (distance * TICKS_PER_INCH_STRAFE);
        }

        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed, target_encoder, P_FWD_COEFF, TICKS_PER_INCH_STRAIGHT/5, strafe)) {
            idle();
        }
        //sleep(100);
        while (opModeIsActive() && !isStopRequested() && !onTargetDist(speed/3, target_encoder, P_FWD_COEFF / 3, TICKS_PER_INCH_STRAIGHT/10, strafe)) {
            idle();
        }

    }

    boolean onTargetDist(double speed, double distance, double PCoeff, double distThreshold, Boolean strafe) {
        double error;
        double steer;
        boolean onTarget = false;
        double power;


        //determine  power based on error
        error = getErrorDist(distance, strafe);

        if (Math.abs(error) <= distThreshold) {

            steer = 0.0;
            power = 0.0;
            onTarget = true;
            stopWheels();
            if(strafedRightToAvoidObstacles == true){
                long currentTime = 0;
                simpleStrafe(-1);
                while(currentTime <timer){
                    currentTime++;

                }
                stopWheels();
            }
            else{
                long currentTime = 0;
                simpleStrafe(1);
                while(currentTime <timer){
                    currentTime++;

                }
                stopWheels();
            }
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
            simpleStrafe(weightConstant*power);
        if(avoidObstacles && distance > 0 &&!strafe){
            //checking distance sensor to see if obstacle in front
            double rfStraight = distanceSensor_rf_Straight.getDistance(DistanceUnit.INCH);
            double lfStraight = distanceSensor_lf_Straight.getDistance(DistanceUnit.INCH);
            if(rfStraight<12){
                strafedRightToAvoidObstacles = false;
                simpleStrafe(-1);
                while(rfStraight <12){
                    timer ++;
                }
                stopWheels();
                //strafe left until no longer sees
            }
            else if (lfStraight <12){
                strafedRightToAvoidObstacles = true;
                simpleStrafe(1);{
                    while(lfStraight <12){
                        timer++;
                    }
                    stopWheels();
                }
            }

        }
        telemetry.addData("Target dist", "%5.2f", distance);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f", speed);

        return onTarget;
    }

    public double getErrorDist(double targetDist, Boolean strafe) {

        double robotError;

        double curr_encoder = 0;
        if(!strafe)
             curr_encoder = motorRightFront.getCurrentPosition();
        else
            curr_encoder = -1 * motorRightFront.getCurrentPosition();

        robotError = targetDist - curr_encoder;

        telemetry.addData("Robot Error", "%5.2f", robotError);
        telemetry.update();

        return robotError;

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
        //get distance sensor value from new lf and rf that are on the front of the robot instead of sides
        //if rf sees obstacle, strafe left until no longer see obstacle in front
        //if lf sees obstacle, strafe right until no longer see obstacle in front

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
    public void grabAndDropBlock_Arm(Boolean isBlueSide) {


        armExtended(10);
        grabCollection();
        EncoderStraight(18);
        closeGrabber();

        //step back
        EncoderStraight(-18);

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
        
        grabCollection();

    }
    public void grabAndDropBlock_Hook(Boolean isBlueSide) {

        EncoderStraight(18);
        sleep(700);
        tray_left.setPosition(0);
        sleep(600);

        //step back
        EncoderStraight(-18);

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
        tray_left.setPosition(0.8);
    }

    public void runAutonomousTray(Boolean isBlueSide) {

        initFn();

        waitForStart();

        //assume that robot is aligned such that it's
        //exactly two tiles away from the bridge towards the tray

        EncoderStraight(30);
        tray_left.setPosition(0);
        tray_right.setPosition(0);
        sleep(600);
        EncoderStraight(-30);

        //park after 20 seconds
        sleep(20000);


        double parkDist = 0;
        if(isBlueSide){
            parkDist = 55;  //48 + some extra to be over the line
        }
        else {
            parkDist = -55;
        }
        EncoderStrafe(parkDist);


    }

    public void runAutonomous(Boolean isBlueSide, Boolean useArm) {

        initFn();

        waitForStart();

        EncoderStraight(30);

        Boolean blockSeen = vuFindBlock(isBlueSide);

        if (blockSeen) {
            double blockDist=0;

            if(!useArm)
                blockDist = y + 11.5;
            else
                blockDist = y + 4.5;

            EncoderStrafe (blockDist);
        }

        //grab a block (even if it's a random one)
        if(!useArm)
            grabAndDropBlock_Hook(isBlueSide);
         else
            grabAndDropBlock_Arm(isBlueSide);


        //return for second
        double returnDist = 0;
        if(isBlueSide){
            returnDist = 24 + 48;
        }
        else {
            returnDist = -24 - 48;
        }
        EncoderStrafe(returnDist);

        //see second one ?
        blockSeen = vuFindBlock(isBlueSide);

        if (blockSeen) {
            double blockDist=0;

            if(!useArm)
                blockDist = y + 11.5;
            else
                blockDist = y + 4.5;

            EncoderStrafe (blockDist);
        }

        //grab a block (even if it's a random one)
        if(!useArm)
            grabAndDropBlock_Hook(isBlueSide);
        else
            grabAndDropBlock_Arm(isBlueSide);

        //park (1 tile back towards the bridge)
        double parkDist = 0;
        if(isBlueSide){
            parkDist = 24;
        }
        else {
            parkDist = -24;
        }
        EncoderStrafe(parkDist);

    }




}






