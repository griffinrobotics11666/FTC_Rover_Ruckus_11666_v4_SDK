package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


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
        robot.markerServoCLose();
        robot.strafe(-1.1 * strafeDistance, strafeSpeed); //strafe right
        int forwardMovement = 10;
        int goldLocation = 1;
        int scanNumber = 5;
        int scanDuration = 5;
        int timeOut = scanNumber * scanDuration;

        sleep(100);
        int sleepTimer = 0;
        while (!robot.detector.isFound() && sleepTimer < timeOut + 20) {
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

    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);

        //TODO lower lift and move away servo

        waitForStart();

        robot.lift(1);
        robot.liftServoOpen();
        sampleMove2();
        robot.detector.disable();
        robot.gyroMove(-38,1);
        robot.markerServoCLose();
        robot.gyroMove(20,1);
        robot.turn(290,1);

        //strafe?

        //TODO drop team marker
    }
}
