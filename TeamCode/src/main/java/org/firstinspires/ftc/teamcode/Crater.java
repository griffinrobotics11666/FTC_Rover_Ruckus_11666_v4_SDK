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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareRobot;

import static java.lang.Thread.sleep;



@Autonomous (name="Crater ", group="Autonomous")
public class Crater extends LinearOpMode{



    private ElapsedTime elapsedTime = new ElapsedTime();
    HardwareRobot robot = new HardwareRobot(telemetry);
    Orientation angles;


    //TODO Can we speed this up?
    public int sampleMove2(){

        //drive forward to get away from
        double strafeSpeed = .5;
        double strafeDistance = 14.5;
        double movementSpeed = 1;
        robot.gyroMove(-16, 1);
        robot.markerServoClose();
        robot.strafe(-1.1*strafeDistance, strafeSpeed); //strafe right
        int forwardMovement = 10;
        int goldLocation = 1;
        int scanNumber = 5;
        int scanDuration = 5;
        int timeOut = scanNumber*scanDuration;

        sleep(100);
        int sleepTimer = 0;
        while (!robot.detector.isFound() && sleepTimer < timeOut+20){
            sleep(scanDuration);
            sleepTimer += scanDuration;
        }
        if (sleepTimer > timeOut)
        {
            goldLocation -= 1;
            robot.strafe(1.1*strafeDistance,strafeSpeed);
            sleepTimer = 0;
            while (!robot.detector.isFound() && sleepTimer < timeOut+20){
                sleep(scanDuration);
                sleepTimer += scanDuration;
            }
            if (sleepTimer > timeOut){
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
                    robot.gyroMove(-1.5*forwardMovement, movementSpeed);
//                    robot.gyroMove(forwardMovement - 2, movementSpeed);

            }
            else
            {
                //Align and hit block
                //robot.alignRobot(.5);
                robot.gyroMove(-1.5*forwardMovement, movementSpeed);
//                robot.gyroMove(forwardMovement, movementSpeed);
            }

        }
        else
        {
            //robot.alignRobot(.5);
            robot.gyroMove(-1.5*forwardMovement, movementSpeed);
//            robot.gyroMove(3*forwardMovement, movementSpeed);
            //Align and hit block
        }


        return goldLocation;
    }
    public int samplemove3(){
        double strafeSpeed = .5;
        double strafeDistance = 14.5;
        double movementSpeed = 1;
        robot.gyroMove(-16, 1);
        robot.markerServoClose();
        int forwardMovement = 10;
        int goldLocation = 1;
        int scanNumber = 5;
        int scanDuration = 5;
        double coolDownTime = 1000;
        elapsedTime.reset();
        double previousScanTime = elapsedTime.milliseconds();

        while(elapsedTime.milliseconds() - previousScanTime > coolDownTime){
            if (robot.detector.isFound()){

            }
        }
        //scans, if found will move forward
        if(robot.detector.isFound() && elapsedTime.milliseconds() - previousScanTime > coolDownTime){
            robot.gyroMove(forwardMovement,movementSpeed);
            robot.gyroMove(-forwardMovement,movementSpeed);
            //if not found, turn
        } else if (!robot.detector.isFound() && elapsedTime.milliseconds() - previousScanTime > coolDownTime) {
            robot.turn(51, 1);
            previousScanTime = elapsedTime.milliseconds();
            //if it is found , turn fully to 90 degree angle and then move to the gold piece
            if (robot.detector.isFound() && elapsedTime.milliseconds() - previousScanTime > coolDownTime) {
                robot.turn(39,1);
                robot.gyroMove(-14.5,1);
                robot.turn(-90,1);
                robot.gyroMove(forwardMovement,movementSpeed);
                robot.gyroMove(-forwardMovement,movementSpeed);
                goldLocation = -1;
                //if still not found, must be only one other option, so make it 90 degrees and move the opposite direction
            } else {
                robot.turn(39,1);
                robot.gyroMove(14.5,1);
                robot.turn(-90,1);
                robot.gyroMove(forwardMovement,movementSpeed);
                robot.gyroMove( -forwardMovement,movementSpeed);

            }

        }
        return goldLocation;
    }
    public int sampleMove4(){
        int goldLocation;
        double centerSpaceAngle = 10;
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
            robot.leftBack.setPower(turnSpeed);
            robot.leftFront.setPower(turnSpeed);
            robot.rightBack.setPower(-turnSpeed);
            robot.rightFront.setPower(-turnSpeed);
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

    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);

        //robot.imu.initialize(parameters);
        //Need to test to see if this fixes the running part.


        waitForStart();
//        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //TODO must lower the robot and take out the servo
        robot.lift(1);
        sleep(2000);
        robot.liftServoOpen();


        int goldLocation = sampleMove4();
        robot.detector.disable();
        robot.turn(-90,1);
        if(goldLocation == -1) {
            robot.gyroMove(29, 1);
        }
        if(goldLocation == 0){
            robot.gyroMove(29 + 14.5,1);
        }
        if(goldLocation == 1){
            robot.gyroMove(29 + 14.5 + 14.5,1);
        }
        robot.turn(45,1);
        robot.gyroMove(44,1);
        robot.turn(45,1);
        robot.gyroMove(5,1);
        robot.markerServoOpen();
        sleep(1500);
        robot.markerServoClose();
        robot.gyroMove(-5,1);
        robot.turn(-45,1);
        robot.gyroMove(-70,1);

//        if (goldLocation == 1){
//            robot.gyroMove(60,1);
//        }
//        if (goldLocation == 0){
//            robot.gyroMove(60 - 14.5, 1);
//        }
//        if (goldLocation == -1){
//            robot.gyroMove(60 - 14.5 - 14.5, 1);
//    }
//        robot.markerServoOpen ();
//        //TODO tweak the  code so it
//        robot.turn(42.5,1);
//        robot.gyroMove(46,1);
//        robot.markerServoCLose();
//        //new code 12/1
//        robot.gyroMove(-16 - 4.5*12, 1);


    }


}


