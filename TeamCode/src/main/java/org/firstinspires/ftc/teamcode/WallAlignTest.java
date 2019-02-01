package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareRobot;

import static java.lang.Thread.sleep;



@Autonomous (name="Autonomous Test", group="Autonomous")
public class WallAlignTest extends LinearOpMode{

    private ElapsedTime elapsedTime = new ElapsedTime();
    HardwareRobot robot = new HardwareRobot(telemetry);
    Orientation angles;

    public int sampleMove4(){
        int goldLocation;
        double centerSpaceAngle = 15;
        double strafeDistance = 14.5;
        double forwardMovement = 13;
        double movementSpeed = 1;
        double strafeSpeed = 1;
        double turnSpeed = .5;
        double turningSpeed = 1;
        robot.gyroMove(-14, 1);
        double startAngle = robot.getCurrentAngle();
        robot.turn(60,turningSpeed);


        //TODO if not see gold, then stop
        while(!robot.detector.getAligned()){
            if(robot.detector.getXPosition() < robot.centerValue) {
                robot.leftBack.setPower(turnSpeed);
                robot.leftFront.setPower(turnSpeed);
                robot.rightBack.setPower(-turnSpeed);
                robot.rightFront.setPower(-turnSpeed);
            }else{
                robot.leftBack.setPower(-turnSpeed/2);
                robot.leftFront.setPower(-turnSpeed/2);
                robot.rightBack.setPower(turnSpeed/2);
                robot.rightFront.setPower(turnSpeed/2);
            }
            telemetry.addData("Current angle", robot.getCurrentAngle());
            telemetry.update();
        }
        robot.stopRobot();


        double newAngle = robot.getNewAngle();
        double differenceAngle = newAngle - startAngle;


        if (differenceAngle > centerSpaceAngle){
            //then on left
            goldLocation = -1;
        }
        else if (differenceAngle < -centerSpaceAngle){
            // then on right
            goldLocation = 1;
        }else {
            //middle!
            goldLocation = 0;
        }

        telemetry.addData("newAngle",newAngle);
        telemetry.addData("Difference Angle", differenceAngle);
        telemetry.addData("gold location", goldLocation);
        telemetry.update();

        robot.turn(-differenceAngle,.5);  //robot is turned to straight

        if (goldLocation == 0){
            robot.gyroMove(-forwardMovement,movementSpeed);
            robot.gyroMove(forwardMovement - 2,movementSpeed);
        }else if (goldLocation == 1){
            robot.strafe(-strafeDistance,strafeSpeed);
            robot.gyroMove(-forwardMovement,movementSpeed);
            robot.gyroMove(forwardMovement - 4,movementSpeed);
        }else {
            robot.strafe(strafeDistance,strafeSpeed);
            robot.gyroMove(-forwardMovement,movementSpeed);
            robot.gyroMove(forwardMovement - 2,movementSpeed);
        }

        return goldLocation;
    }
    public int sampleMove5(){
        int goldLocation;
        double centerSpaceAngle = 15;
        double strafeDistance = 14.5;
        double forwardMovement = 13;
        double movementSpeed = 1;
        double strafeSpeed = 1;
        double turnSpeed = .5;
        double turningSpeed = 1;
        robot.gyroMove(-14, 1);
        double startAngle = robot.getCurrentAngle();

        robot.turn(60,turningSpeed);


        //TODO if not see gold, then stop
        while(!robot.detector.getAligned() || Math.abs((robot.centerValue) - robot.detector.getXPosition()) > 5){
            if((robot.centerValue) - robot.detector.getXPosition() > 5) {
                robot.leftBack.setPower(turnSpeed);
                robot.leftFront.setPower(turnSpeed);
                robot.rightBack.setPower(-turnSpeed);
                robot.rightFront.setPower(-turnSpeed);
            }else{
                robot.leftBack.setPower(-turnSpeed/2);
                robot.leftFront.setPower(-turnSpeed/2);
                robot.rightBack.setPower(turnSpeed/2);
                robot.rightFront.setPower(turnSpeed/2);
            }
            telemetry.addData("Current angle", robot.getCurrentAngle());
            telemetry.update();
        }
        robot.stopRobot();


        double newAngle = robot.getNewAngle();
        double differenceAngle = newAngle - startAngle;


        if (differenceAngle > centerSpaceAngle){
            //then on left
            goldLocation = -1;
        }
        else if (differenceAngle < -centerSpaceAngle){
            // then on right
            goldLocation = 1;
        }else {
            //middle!
            goldLocation = 0;
        }

        telemetry.addData("newAngle",newAngle);
        telemetry.addData("Difference Angle", differenceAngle);
        telemetry.addData("gold location", goldLocation);
        telemetry.update();


        if (goldLocation == 0){
            robot.gyroMove(-10,1);
            robot.gyroMove(10,1);
        }else {
            robot.gyroMove(-24,1);
            robot.gyroMove(24,1);
        }
        robot.turn(-differenceAngle,.5);  //robot is turned to straight

        return goldLocation;
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);

        waitForStart();
        robot.lift(1);
        robot.liftServoOpen();
        sleep(1000);


        int goldLocation = sampleMove5();
        robot.detector.disable();
        robot.turn(-90,1);
        robot.gyroMove(29 + 14.5,1);

        telemetry.addData("current angle", robot.getCurrentAngle());
        telemetry.update();

//        ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//        robot.gyroMove(-20 ,1);
//        if(goldLocation == -1) {
//            robot.gyroMove(29, 1);
//        }
//        if(goldLocation == 0){
//            robot.gyroMove(29 + 14.5,1);
//        }
//        if(goldLocation == 1){
//            robot.gyroMove(29 + 14.5 + 14.5,1);
//        }
//        |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

//        robot.turn(45,1);
//        robot.strafe(5,1);
//        robot.strafe(-2,1);
//        robot.gyroMove(44,1);
//        robot.turn(45,1);
//        robot.gyroMove(5,1);
//        robot.markerServoOpen();
//        sleep(1000);
//        robot.markerServoClose();
//        robot.gyroMove(-5,1);
//        robot.turn(-45,1);
//        robot.gyroMove(-45,1);//-70

    }


}

