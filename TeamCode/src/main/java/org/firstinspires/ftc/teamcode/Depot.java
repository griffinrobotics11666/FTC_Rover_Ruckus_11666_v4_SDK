package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareRobot;

import static java.lang.Thread.sleep;

@Autonomous (name="Depot ", group="Autonomous")
public class Depot extends LinearOpMode {


    HardwareRobot robot = new HardwareRobot(telemetry);
    Orientation angles;

    public int sampleMove2() {

        //drive forward to get away from
        double strafeSpeed = .5;
        double strafeDistance = 14.5;
        double movementSpeed = 1;
        robot.gyroMove(-16, 1);
        robot.markerServoClose();
        robot.strafe(-1.1 * strafeDistance, strafeSpeed); //strafe right
        int forwardMovement = 10;
        int goldLocation = 1;
        int scanNumber = 5;
        int scanDuration = 5;
        int timeOut = scanNumber * scanDuration;

        sleep(100);
        int sleepTimer = 0;
        while (!robot.detector.isFound() && (sleepTimer < timeOut + 20) && opModeIsActive()) {
            sleep(scanDuration);
            sleepTimer += scanDuration;
        }
        if (sleepTimer > timeOut) {
            goldLocation -= 1;
            robot.strafe(1.1 * strafeDistance, strafeSpeed);
            sleepTimer = 0;
            while (!robot.detector.isFound() && sleepTimer < timeOut + 20) {
                sleep(scanDuration);
                sleepTimer += scanDuration;
            }
            if (sleepTimer > timeOut) {
                goldLocation -= 1;
                robot.strafe(strafeDistance, strafeSpeed);
                sleepTimer = 0;
                /*while (!robot.detector.isFound() && sleepTimer < timeOut+20){
                    sleep(scanDuration);
                    sleepTimer += scanDuration;
                }
                if (sleepTimer > timeOut){
                    //stop();
                    //robot didn't find it
                }
                else
                {
                */
                //Align and hit block
                //robot.alignRobot(.5);
                robot.gyroMove(-forwardMovement, movementSpeed);
                robot.gyroMove(forwardMovement, movementSpeed);

            } else {
                //Align and hit block
                //robot.alignRobot(.5);
                robot.gyroMove(-forwardMovement, movementSpeed);
                robot.gyroMove(forwardMovement, movementSpeed);
            }

        } else {
            //robot.alignRobot(.5);
            robot.gyroMove(-1.5 * forwardMovement, movementSpeed);
            robot.gyroMove(forwardMovement, movementSpeed);
            //Align and hit block
        }
        return goldLocation;
    }
    public int sampleMove4(){
        int goldLocation;
        double centerSpaceAngle = 15;
        double strafeDistance = 14.5;
        double forwardMovement = 13;
        double movementSpeed = 1;
        double strafeSpeed = 1;
        double turnSpeed = .4;
        double turningSpeed = 1;
        robot.gyroMove(-14, 1);
        double startAngle = robot.getCurrentAngle();
        robot.turn(60,turningSpeed);
        double goldXPosition = 0;


        //TODO if not see gold, then stop
        while(Math.abs((robot.centerValue) - robot.detector.getXPosition()) > 5){

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
        }else if (goldLocation == 1){
            robot.strafe(-strafeDistance,strafeSpeed);
            robot.gyroMove(-forwardMovement,movementSpeed);
        }else {
            robot.strafe(strafeDistance,strafeSpeed);
            robot.gyroMove(-forwardMovement,movementSpeed);
        }

        return goldLocation;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);

        //TODO lower lift and move away servo

        waitForStart();

        robot.lift(1);
        robot.liftServoOpen();


        int goldPosition = sampleMove4();
        robot.gyroMove(-23,1);
        robot.turn(-90,1);

        if(goldPosition == -1){
            robot.markerServoOpen();
            sleep(1500);
            robot.markerServoClose();
            robot.turn(5,.5);
            robot.gyroMove(3,1);
        }
        if(goldPosition == 0){
            robot.markerServoOpen();
            sleep(1500);
            robot.markerServoClose();
            robot.gyroMove(14.5,1);
        }
        if(goldPosition == 1){
            robot.gyroMove(14.5,1);
            robot.markerServoOpen();
            sleep(1500);
            robot.markerServoClose();
            robot.gyroMove(14.5,1);
        }
        robot.turn(45,1);
        if (goldPosition == -1){
            robot.gyroMove(3,1);
        }
        robot.gyroMove(29,1);//58

        robot.isTop = true;
        robot.detector.disable();
        robot.liftServoClose();
        robot.lift(1);

        robot.detector.disable();

//        robot.detector.disable();
//        robot.gyroMove(-38,1);
//        robot.markerServoClose();
//        robot.gyroMove(20,1);
//        robot.turn(290,1);
    }
}
