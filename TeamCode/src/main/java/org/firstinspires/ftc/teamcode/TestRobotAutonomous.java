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

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        */

        //Init
        robot.init(hardwareMap);

        //robot.imu.initialize(parameters);
        //Need to test to see if this fixes the running part.


        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Do stuff
        robot.move(24,.4);
        sleep(1000);
        robot.turn(90,.7);
        sleep(1000);
        robot.turn(90,.7);
        sleep(1000);
        robot.turn(90,.7);
        sleep(1000);
        robot.turn(90, .7);



    }


}
