package org.firstinspires.ftc.teamcode.SampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Blank OpMode", group="SAMPLE OPMODES")
//@Disabled
public class BlankOpMode extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    // Declare variables


    @Override
    public void runOpMode()
    {
        //map hardware variables to hardware map

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}