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

@Autonomous (name="Red Crater", group="Autonomous")
public class redCrater extends LinearOpMode{




    HardwareRobot robot = new HardwareRobot(telemetry);
    Orientation angles;



    @Override
    public void runOpMode() throws InterruptedException
    {
        //Init
        robot.init(hardwareMap);

        //robot.imu.initialize(parameters);
        //Need to test to see if this fixes the running part.


        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //TODO must lower the robot and take out the servo

        robot.move(-20, .5);
        robot.turn(-90, 1);
        robot.move(49,.5);
        robot.turn(45, 1);
        robot.move(40, .5);

        //TODO Drop the Team Marker

    }


}

