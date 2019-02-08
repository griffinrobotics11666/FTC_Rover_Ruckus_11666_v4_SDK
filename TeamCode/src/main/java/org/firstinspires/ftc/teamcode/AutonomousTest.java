package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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
public class AutonomousTest extends LinearOpMode{

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
        double forwardMovement = 13;
        double movementSpeed = 1;
        double turnSpeed = .5;
        double turningSpeed = 1;
        double speed = 0;
        double goldXPosition = 0;
        robot.gyroMove(-8, 1);

        double startAngle = robot.getCurrentAngle();

        robot.turn(60,turningSpeed);

        while(Math.abs((robot.centerValue) - goldXPosition) > 5){
            if (!robot.detector.isFound()){
                goldXPosition = 0;
            }else {
                goldXPosition = robot.detector.getXPosition();
            }

            turnSpeed = Range.scale(Math.abs((robot.centerValue - goldXPosition)/robot.centerValue),0,1,.15,.5);
//            turnSpeed = Range.clip(Math.abs((robot.centerValue - robot.detector.getXPosition())/robot.centerValue),.2,.5);
            if((robot.centerValue - goldXPosition)/robot.centerValue < 0){
                turnSpeed = -turnSpeed;
            }



            robot.leftBack.setPower(turnSpeed);
            robot.leftFront.setPower(turnSpeed);
            robot.rightBack.setPower(-turnSpeed);
            robot.rightFront.setPower(-turnSpeed);

//            if((robot.centerValue) - robot.detector.getXPosition() > 0) {
//                robot.leftBack.setPower(turnSpeed);
//                robot.leftFront.setPower(turnSpeed);
//                robot.rightBack.setPower(-turnSpeed);
//                robot.rightFront.setPower(-turnSpeed);
//            }else{
//                robot.leftBack.setPower(-turnSpeed/1.5);
//                robot.leftFront.setPower(-turnSpeed/1.5);
//                robot.rightBack.setPower(turnSpeed/1.5);
//                robot.rightFront.setPower(turnSpeed/1.5);
//            }
            telemetry.addData("Current angle", robot.getCurrentAngle());
            telemetry.update();
        }
        robot.detector.disable();
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
        double finalAngle = robot.getCurrentAngle();


        if (goldLocation == 0){
            robot.gyroMove(-17,movementSpeed);
//            robot.turn(15,.4);
//            robot.gyroMove(-3,1);
            robot.gyroMove(10,movementSpeed);
            robot.turn(-differenceAngle + 90,.3);
            robot.gyroMove(47,1);
            robot.turn(-48,.5);
//            robot.turn(finalAngle - startAngle - 45,.5);
//            finalAngle = robot.getCurrentAngle();
//            robot.turn(finalAngle - startAngle - 45,.5);
//            robot.gyroMove(-25,1);
//            robot.strafe(-25,1);
//            robot.gyroMove(-5,1);
//            robot.strafe(-8,1);
            robot.gyroMove(-26,1);
        }else if (goldLocation == -1) {
            robot.turn(5,.3);
            robot.gyroMove(-29,movementSpeed);
//            robot.gyroMove(20,movementSpeed);
        }
        else{
            robot.turn(-5,.3);
            robot.gyroMove(-29,movementSpeed);
        }

//        robot.turn(-differenceAngle - 90,1);  //robot is turned to straight

        return goldLocation;
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);

        waitForStart();
        robot.lift(1);
        robot.liftServoOpen();
//        sleep(1000);


        int goldLocation = sampleMove5();
//        robot.isTop = true;
//        robot.detector.disable();
//        robot.liftServoClose();
//        robot.lift(1);
//        robot.gyroMove(29 + 14.5,1);

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

