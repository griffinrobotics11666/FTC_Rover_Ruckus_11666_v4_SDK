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

@Autonomous (name="autonomous", group="Autonomous")

public class AutonomousMode extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot(telemetry);
    Orientation angles;



    @Override
    public void runOpMode() throws InterruptedException
    {
        //Init
        robot.init(hardwareMap);

        //robot.imu.initialize(parameters);
        //Need to test to see if this fixes the running part.

        //!!!!!DON'T USE 180 IN TURN WITH GYRO MOVEMENT AFTER!!!!!!!


        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Do stuff
        //TODO Rewrite SampleMove in Autonomous to use sleep or other functions

        robot.turn(-90,1);

        if (robot.direction == 1){
            robot.gyroMove(64,1);
        }
        if (robot.direction == 0){
            robot.gyroMove(64 - 14.5, 1);
        }
        if (robot.direction == -1){
            robot.gyroMove(64 - 14.5 - 14.5, 1);
        }
        robot.turn(45,1);
        robot.gyroMove(35.5,1);
//        while(opModeIsActive()){
//            telemetry.addData("distance", robot.getDistance());
//            telemetry.update();
//        }
//        robot.move(24,.4);
//        sleep(1000);
//        robot.turn(90,.7);
//        sleep(1000);
//        robot.turn(90,.7);
//        sleep(1000);
//        robot.turn(90,.7);
//        sleep(1000);
//        robot.turn(90, .7);
        robot.detector.disable();
 }


}
