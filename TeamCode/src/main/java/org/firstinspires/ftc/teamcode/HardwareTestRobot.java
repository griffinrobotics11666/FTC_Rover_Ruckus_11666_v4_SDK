package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Util.TestRobotConstants;

public class HardwareTestRobot
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public  Servo servo = null;
    public boolean ishome = true;
    public double servoHomePosition = 0.5;
    HardwareMap hwMap  = null;

    TestRobotConstants constants = new TestRobotConstants();


    public HardwareTestRobot() {

    }


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // assign devices to config
        leftDrive  = hwMap.get(DcMotor.class, "motor_1");
        rightDrive = hwMap.get(DcMotor.class, "motor_2");
        servo = hwMap.get(Servo.class, "servo_1");
        //set motor direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //set motor power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //set servo to initial position
        servo.setPosition(servoHomePosition);


        //TODO
        /*
        Methods added to Hardware Map
        Raise and lower servo
        Encoder Drive
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

}
