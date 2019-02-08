package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous(name="Lower Lift", group="Autonomous")
public class LowerLift extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot(telemetry);



    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);
        robot.isTop = true;

        waitForStart();
        robot.lift(1);

        robot.detector.disable();
    }


}