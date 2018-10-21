package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Util.TestRobotConstants;

public class HardwareTestRobot
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public  Servo servo = null;
    public DistanceSensor sensorRange;
    public boolean ishome = true;
    public double servoHomePosition = 0.5;
    BNO055IMU imu;
    Orientation angles;
    HardwareMap hwMap  = null;
    Telemetry telemetry;


    public GoldAlignDetector detector;
    int centerValue = 290;

    TestRobotConstants constants = new TestRobotConstants();



    public HardwareTestRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // assign devices to config
        leftDrive  = hwMap.get(DcMotor.class, "motor_1");
        rightDrive = hwMap.get(DcMotor.class, "motor_2");
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");

        servo = hwMap.get(Servo.class, "servo_1");
        imu = hwMap.get(BNO055IMU.class, "imu");
        //set motor direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set motor power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //set servo to initial position
        servo.setPosition(servoHomePosition);

        //Gyro


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        detector = new GoldAlignDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 75; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();






        //TODO
        /*
        Methods added to Hardware Map
        Raise and lower servo
         */


    }

    public void stopRobot()
    {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void move(double distance, double speed)
    {
        int newLeftDriveTarget;
        int newRightDriveTarget;

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //find how many encoder counts the motor is at, then add the distance to it
        newLeftDriveTarget = leftDrive.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_60());
        //Had "rightdrive.getTargetPosition()
        newRightDriveTarget = rightDrive.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_60());
        //set the target encoder count to the motors
        leftDrive.setTargetPosition(newLeftDriveTarget);
        rightDrive.setTargetPosition(newRightDriveTarget);
        //set mode to run to position
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set speed
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        //While loop is necessary!
        while (leftDrive.isBusy() || rightDrive.isBusy())
        {}

        stopRobot();
    }
    public void turn(double angle, double speed)
    {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initalAngle = angles.firstAngle;
        double motorPower;
        double minMotorPower = 0.15;
        double powerScaleFactor;
        double targetAngle;
        double currentAngle;
        double deltaAngle;
        double robotAngle = angles.firstAngle;
        double previousAngle = angles.firstAngle;

        targetAngle = initalAngle + angle;

        while (Math.abs(targetAngle - robotAngle)> .1)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = angles.firstAngle;

            //update speed dynamically to slow when approaching the target
            powerScaleFactor = Math.abs(targetAngle-robotAngle)/angle;
            motorPower = powerScaleFactor*speed;
            if (motorPower < minMotorPower)
            {
                motorPower = minMotorPower;
            }

            //determine which direction the robot should turn



            if ((targetAngle - robotAngle) > 0) {
                leftDrive.setPower(-motorPower);
                rightDrive.setPower(motorPower);
            } else {
                leftDrive.setPower(motorPower);
                rightDrive.setPower(-motorPower);
            }


            //define how the angle is changing and deal with the stupid 180 -> -180 thing
            deltaAngle = currentAngle - previousAngle;
            if (deltaAngle > 180)
            {
                deltaAngle -= 360;
            }
            else if(deltaAngle < -180)
            {
                deltaAngle += 360;
            }

            robotAngle += deltaAngle;
            previousAngle = currentAngle;

            telemetry.addData("robotangle", robotAngle);
            telemetry.addData("deltaAngle", deltaAngle);
            telemetry.addData("currentAngle", currentAngle);

            telemetry.update();

        }
        stopRobot();
    }
    public void alignRobot(double speed){
        while (!detector.getAligned()){
            if (detector.getXPosition() > centerValue){
                leftDrive.setPower(-speed);
                rightDrive.setPower(-speed);
            }else if (detector.getXPosition() < centerValue){
                leftDrive.setPower(speed);
                rightDrive.setPower(speed);
            }
        }
        if (detector.getAligned()){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }
    public void gyroMove(double distance, double speed)
    {
        double dYaw;
        double deltaSpeed = 0.05;// hardcoded
        double deltaA = 0;
        int newLeftDriveTarget;
        int newRightDriveTarget;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentangle = angles.firstAngle;



        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //find how many encoder counts the motor is at, then add the distance to it
        newLeftDriveTarget = leftDrive.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_60());
        newRightDriveTarget = rightDrive.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_60());

        //set the target encoder count to the motors
        leftDrive.setTargetPosition(newLeftDriveTarget);
        rightDrive.setTargetPosition(newRightDriveTarget);

        //set mode to run to position
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set speed
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        //While loop is necessary!
        while (leftDrive.isBusy() && rightDrive.isBusy())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            dYaw = currentangle - angles.firstAngle;
            if (dYaw > 300){
                dYaw -= 360;
            }else if (dYaw < -300){
                dYaw += 360;
            }
            if (dYaw > deltaA){
                telemetry.addData("left", 0);
               leftDrive.setPower(speed - deltaSpeed);
               rightDrive.setPower(speed + deltaSpeed);
           }else if (dYaw < deltaA){
                telemetry.addData("right", 0);
               leftDrive.setPower(speed + deltaSpeed);
               rightDrive.setPower(speed - deltaSpeed);
           }else {
                telemetry.addData("straight", 0);
               leftDrive.setPower(speed);
               rightDrive.setPower(speed);
           }
           telemetry.update();
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turn(currentangle - angles.firstAngle, .2);

        stopRobot();
    }

}
