package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareTestRobot;

@Autonomous(name="testautonomous", group="Autonomous")

public class TestRobotAutonomous extends LinearOpMode {

    //Declare Variables
    HardwareTestRobot robot = new HardwareTestRobot(telemetry);
    Orientation angles;




    @Override
    public void runOpMode() throws InterruptedException
    {
        //Init
        robot.init(hardwareMap);



        //robot.imu.initialize(parameters);
        //Need to test to see if this fixes the running part.


        waitForStart();
//        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while(opModeIsActive()) {
            telemetry.addData("IsAligned", robot.detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", robot.detector.getXPosition());

            robot.alignRobot(.2);




            telemetry.update();

        }
        robot.detector.disable();
        //Do stuff


    }


}
