package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareTestRobot;

@Autonomous(name="testautonomous", group="Autonomous")

public class TestRobotAutonomous extends LinearOpMode {

    //Declare Variables
    HardwareTestRobot robot = new HardwareTestRobot();

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Init
        robot.init(hardwareMap);

        //Need to test to see if this fixes the running part.


        waitForStart();

        //Do stuff
        robot.move(24,.4);

    }
}
