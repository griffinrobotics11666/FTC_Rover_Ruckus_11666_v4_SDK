package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


public class HardwareTestRobot
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public  Servo servo = null;
    public boolean ishome = true;
    public double servoHomePosition = 0.5;
    HardwareMap hwMap  = null;

    public HardwareTestRobot() {
    }


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftDrive  = hwMap.get(DcMotor.class, "motor_1");
        rightDrive = hwMap.get(DcMotor.class, "motor_2");
        servo = hwMap.get(Servo.class, "servo_1");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        servo.setPosition(servoHomePosition);

        //TODO
        /*
        Methods added to Hardware Map
        Raise and lower servo
        Encoder Drive
        Gyro turn

         */


    }
}
