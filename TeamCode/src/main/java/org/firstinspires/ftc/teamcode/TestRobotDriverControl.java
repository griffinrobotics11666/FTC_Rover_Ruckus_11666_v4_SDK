package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver Control", group="Competition")
//@Disabled
public class TestRobotDriverControl extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    // Declare variables
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private double speedRight;
    private double speedLeft;
    private double drive;
    private double turn;
    private double maxSpeed;
    private double topSpeed = .5;
    private Servo servo = null;
    private double servoHomePosition = 0.5;
    private double servoDropPosition = 0.0;
    private boolean ishome = true;

    @Override
    public void runOpMode()
    {
        //map hardware variables to hardware map
        leftDrive  = hardwareMap.get(DcMotor.class, "motor_1");
        rightDrive = hardwareMap.get(DcMotor.class, "motor_2");
        servo  = hardwareMap.get(Servo.class, "servo_1");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        servo.setPosition(servoHomePosition);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            if (gamepad1.a && ishome)
            {
                servo.setPosition(servoDropPosition);
                ishome = false;
                sleep(300); //button delay time
            }
            if (gamepad1.a && !ishome)
            {
                servo.setPosition(servoHomePosition);
                ishome = true;
                sleep(300);
            }

            //gamepad values
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;

            //toggle speed for fine adjustment
            if (gamepad1.right_bumper)
            {
                maxSpeed = topSpeed /2;
            }
            else
            {
                maxSpeed = topSpeed;
            }

            //RANGE CLIP
            speedLeft = Range.clip(drive + turn, -maxSpeed, maxSpeed);
            speedRight = Range.clip(drive - turn, -maxSpeed, maxSpeed);

            //set motor speed
            leftDrive.setPower(speedLeft);
            rightDrive.setPower(speedRight);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}