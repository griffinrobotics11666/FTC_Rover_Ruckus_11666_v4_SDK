package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver Control", group="Competition")
//@Disabled
public class DriverControl extends LinearOpMode
{
    HardwareRobot robot = new HardwareRobot(telemetry);
    private ElapsedTime runtime = new ElapsedTime();
    // Declare variables
    private double speedRightFront;
    private double speedLeftFront;
    private double speedRightBack;
    private double speedLeftBack;
    private double drive;
    private double turn;
    private double maxSpeed;
    private double topSpeed = 1;
    //private Servo servo = null;
    private double servoHomePosition = 0.5;
    private double servoDropPosition = 0.0;
    private boolean ishome = true;
    private double strafe;

    @Override
    public void runOpMode()
    {
        //map hardware variables to hardware map
        robot.init(hardwareMap);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            /*
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
            */

            //gamepad values
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;
            strafe = gamepad1.right_stick_x;

            if (gamepad2.a)
            {
                robot.lift(1);
                sleep(1000);
            }

            if (gamepad2.x) {robot.leftServoOpen();     }
            if (gamepad2.y) {robot.leftServoClose();    }
            if (gamepad2.a) {robot.rightServoOpen();    }
            if (gamepad2.b) {robot.rightServoClose();   }
            if (gamepad2.right_bumper){robot.liftServoClose();  }
            if (gamepad2.left_bumper) {robot.liftServoOpen();   }

            //toggle speed for fine adjustment
            if (gamepad1.right_bumper){maxSpeed = topSpeed /3;}
            else {maxSpeed = topSpeed; }

            //RANGE CLIP
            speedLeftFront = Range.clip(drive + turn + strafe, -maxSpeed, maxSpeed);
            speedRightFront = Range.clip(drive - turn + strafe, -maxSpeed, maxSpeed);
            speedLeftBack = Range.clip(drive + turn - strafe, -maxSpeed, maxSpeed);
            speedRightBack = Range.clip(drive - turn - strafe, -maxSpeed, maxSpeed);

            //set motor speed
            robot.leftFront.setPower(speedLeftFront);
            robot.rightFront.setPower(speedRightFront);
            robot.rightBack.setPower(speedRightBack);
            robot.leftBack.setPower(speedLeftBack);


            //Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("left servo position:", robot.leftServo.getPosition());
            telemetry.addData("right servo position:", robot.rightServo.getPosition());
            telemetry.addData("Lift Servo Position:", robot.liftServo.getPosition());
            telemetry.update();
        }
        robot.detector.disable();
    }
}