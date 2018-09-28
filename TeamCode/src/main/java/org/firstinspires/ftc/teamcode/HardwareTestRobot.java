package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Util.TestRobotConstants;

public class HardwareTestRobot
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public  Servo servo = null;
    public boolean ishome = true;
    public double servoHomePosition = 0.5;
    BNO055IMU imu;
    Orientation angles;
    HardwareMap hwMap  = null;
    Telemetry telemetry;

    TestRobotConstants constants = new TestRobotConstants();


    public HardwareTestRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // assign devices to config
        leftDrive  = hwMap.get(DcMotor.class, "motor_1");
        rightDrive = hwMap.get(DcMotor.class, "motor_2");
        servo = hwMap.get(Servo.class, "servo_1");
        imu = hwMap.get(BNO055IMU.class, "imu");
        //set motor direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
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



        //TODO
        /*
        Methods added to Hardware Map
        Raise and lower servo
        Encoder Drive (DONE)
        Gyro turn
         */


    }

    public void move(double distance, double speed)
    {
        int newLeftDriveTarget;
        int newRightDriveTarget;

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //find how many encoder counts the motor is at, then add the distance to it
        newLeftDriveTarget = leftDrive.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_20());
        //Had "rightdrive.getTargetPosition()
        newRightDriveTarget = rightDrive.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_20());
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

        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }
    public void turn(double angle, double speed)
    {

        
        /*
        while (true)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("roll: ", angles.firstAngle);
            telemetry.update();
        }
        */



//        double targetAngle;
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        float initialAngle = angles.firstAngle;
//        targetAngle = initialAngle + angle;
//
//
//
//
//        while (Math.abs(targetAngle - angles.firstAngle)>0.05 )
//        {
//
//            if (angle > 0) {
//                leftDrive.setPower(-speed);
//                rightDrive.setPower(speed);
//            } else {
//                leftDrive.setPower(speed);
//                rightDrive.setPower(-speed);
//            }
//        }
//        leftDrive.setPower(0);
//        rightDrive.setPower(0);


        /*
        Vo Code
        if (angle > 0)
        {
            leftDrive.setPower(-speed);
            rightDrive.setPower(speed);
        }
        else{
            leftDrive.setPower(speed);
            rightDrive.setPower(-speed);
        }

        if (targetAngle == angles.firstAngle)
        {
            leftDrive.setPower(0);
            rightDrive.setPower(0);

        }
        while (leftDrive.isBusy() || rightDrive.isBusy())
        {

        }
        */


    }

}
