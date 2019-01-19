package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
DRIVER MAPPING

Gamepad1
x: 180 turn
y:
b:
a:
Dpad Up: Move lift up/down
Dpad Down:
Dpad left:
Dpad right:
left joystick: Drive
right joystick:
left joystick press:
right joystick press:
right trigger:
right bumper: Slow robot
left trigger:
left bumper:

Gamepad2
x:
y:
b: Right Servo Open/Close
a: Right Servo Open/Close
Dpad_Up:
Dpad_Down:
Dpad_left: Stop lift (emergencies only!)
Dpad_right:
left_joystick: Upper Arm?
right_joystick: Lower Arm?
left_joystick press:
right_joystick press:
right_trigger: Lift up  (emergencies only!)
right_bumper:
left_trigger:  Lift down (emergencies only!)
left_bumper:

*/

@TeleOp(name="Driver Control", group="Competition")
//@Disabled
public class DriverControl extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot(telemetry);
    private ElapsedTime elapsedTime = new ElapsedTime();
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

    public double cooldownTime = 250;

    @Override
    public void runOpMode() {
        //map hardware variables to hardware map
        robot.init(hardwareMap);
        robot.detector.disable();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        elapsedTime.reset();
        double previousLiftTime = elapsedTime.milliseconds();
        double previousRightTime = elapsedTime.milliseconds();
        double previousLeftTime = elapsedTime.milliseconds();
        double previousLockTime = elapsedTime.milliseconds();

        int armLock = 0;
        boolean isLiftOpen = false;
        boolean isRightOpen = false;
        boolean isLeftOpen = false;// changed this position.
        boolean isLocked = false;
        boolean lastLock = isLocked;

        double firstArmPower;
        //true if not pressed
        boolean isPressed = !robot.button.getState();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            isPressed = !robot.button.getState();

            if(gamepad2.dpad_down && (elapsedTime.milliseconds() - previousLockTime > cooldownTime)){
                if(!isLocked){
                    armLock = robot.middleArm.getCurrentPosition();
                }
                isLocked = !isLocked;
            }


            //gamepad values
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;
            //strafe = gamepad1.right_stick_x;
            //changed strafe to be on triggers for Gamepad1
            strafe = gamepad1.right_trigger - gamepad1.left_trigger;



            firstArmPower = gamepad2.left_stick_y;


            //TODO Vo needs to change this so it doesn't just skip a lot.  Positive rotates arm up!
            if(isPressed){
                firstArmPower = Math.abs(firstArmPower);
            }

            robot.firstArm.setPower(firstArmPower);



            if(!isLocked) {
                robot.middleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.middleArm.setPower(gamepad2.right_stick_y);
            }
            /*
            if(gamepad2.right_stick_y < 0){
                robot.middleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.middleArm.setPower(gamepad2.right_stick_y / 2);
            }
            */
            if(isLocked && gamepad2.right_stick_y == 0){
                robot.middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.middleArm.setTargetPosition(armLock);
                robot.middleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.middleArm.setPower(.15);
            }

/*
            if(isLocked && !lastLock)
            {
                robot.middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.middleArm.setTargetPosition(armLock);
                robot.middleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.middleArm.setPower(.15);
            }
            lastLock = isLocked;
*/
            if (gamepad1.x) {
                robot.turn(180, 1);
            }

//            if (gamepad2.a){
//                robot.rightServoOpen();
//            }
//
//            if(gamepad2.b){
//                robot.rightServoClose();
//            }
//            if(gamepad2.x){
//                robot.leftServoOpen();
//            }
//            if(gamepad2.y){
//                robot.leftServoClose();
//            }


            if (gamepad2.right_bumper && (elapsedTime.milliseconds() - previousLeftTime > cooldownTime)) {
                if (isLeftOpen) {
                    robot.leftServoClose();
                }
                if (!isLeftOpen) {
                    robot.leftServoOpen();
                }
                isLeftOpen = !isLeftOpen;
                previousLeftTime = elapsedTime.milliseconds();
            }
            if (gamepad2.left_bumper && (elapsedTime.milliseconds() - previousRightTime > cooldownTime)) {
                if (isRightOpen) {
                    robot.rightServoClose();
                }
                if (!isRightOpen) {
                    robot.rightServoOpen();
                }
                isRightOpen = !isRightOpen;
                previousRightTime = elapsedTime.milliseconds();
            }

            if (gamepad1.left_bumper && (elapsedTime.milliseconds() - previousLiftTime > cooldownTime)) {
                if (isLiftOpen) {
                    robot.liftServoClose();
                }
                if (!isLiftOpen) {
                    robot.liftServoOpen();
                }
                isLiftOpen = !isLiftOpen;
                previousLiftTime = elapsedTime.milliseconds();
                }




//            if (gamepad1.dpad_up || gamepad2.dpad_up) {
//                robot.lift(1);
//            }

            if (gamepad2.right_trigger > 0) {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger > 0) {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(-gamepad2.left_trigger);
            }
            else
            {
                robot.lift.setPower(0);
            }

            if (gamepad2.dpad_left) {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(0);
            }

            //toggle speed for fine adjustment
            if (gamepad1.right_bumper) {
                maxSpeed = topSpeed / 2.5; //was 3 on 1/5/19
            } else {
                maxSpeed = topSpeed;
            }

            if (gamepad2.a)
            {
                robot.markerServoOpen();
            }
            if (gamepad2.b)
            {
                robot.markerServoClose();
            }

            //arm control

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
            telemetry.addData("Is the arm locked?", isLocked);
            telemetry.addData("Arm Power:", gamepad2.right_stick_y / 2);
            telemetry.addData("Status", "Run Time: " + elapsedTime.toString());
            telemetry.addData("RT ms: ", elapsedTime.milliseconds());
            telemetry.addData("left servo position:", robot.leftServo.getPosition());
            telemetry.addData("right servo position:", robot.rightServo.getPosition());
            telemetry.addData("Lift Servo Position:", robot.liftServo.getPosition());
            telemetry.addData("Lift motor Encoder Value:", robot.lift.getCurrentPosition());
            telemetry.addData("First arm motor", robot.firstArm.getCurrentPosition());
            telemetry.addData("Arm joint motor", robot.middleArm.getCurrentPosition());
            telemetry.addData("right Open?", isRightOpen);
            telemetry.addData("left Open?", isLeftOpen);
            telemetry.addData("Is Top?", robot.isTop);
            telemetry.addData("Left Previous Time", previousLeftTime);
            telemetry.addData("Right Previous Time", previousRightTime);
            telemetry.addData("Lift Previous Time", previousLiftTime);
            telemetry.update();
            }
            robot.detector.disable();
        }
    }