/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Sample Op SERVO", group="SAMPLE OPMODES")
@Disabled
public class SampleOpMode_SERVO extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private Servo servo = null;
    public double servo1Speed = 0.01;
    public final static double servoInitPosition = 0.2;
    public double servoMinPosition = 0;
    public double servoMaxPosition = 1;
    public double servoPosition;
    public double servo1DisplaySpeed;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        servo  = hardwareMap.get(Servo.class, "servo_1");
        servo.setPosition(servoInitPosition);
        servoPosition = servoInitPosition;

        telemetry.addData("Servo Position", "%.2f", servo.getPosition()); // Display current position of servo
        telemetry.addData("Servo Min Position", "%.2f", servoMinPosition);
        telemetry.addData("Servo Max Position", "%.2f", servoMaxPosition);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            if (gamepad1.dpad_up && servoMaxPosition < 1)
            {
                servoMaxPosition += 0.05;
                sleep(200);
            }
            if (gamepad1.dpad_down && servoMaxPosition > servoMinPosition+0.1)
            {
                servoMaxPosition -= 0.05;
                sleep(200);
            }
            if (gamepad1.dpad_right && servoMinPosition < servoMaxPosition - 0.1)
            {
                servoMinPosition += 0.05;
                sleep(200);
            }
            if (gamepad1.dpad_left && servoMinPosition > 0.1)
            {
                servoMinPosition -= 0.05;
                sleep(200);
            }
            if (gamepad1.right_bumper && servo1Speed < 0.02)
            {
                sleep(200);
                servo1Speed += 0.001;
            }
            if (gamepad1.left_bumper && servo1Speed > .001)
            {
                sleep(200);
                servo1Speed -= 0.001;
            }
            if (gamepad1.a && servoPosition < servoMaxPosition)
            {
                servoPosition += servo1Speed;
            }
            if (gamepad1.b && servoPosition > servoMinPosition)
            {
                servoPosition -= servo1Speed;
            }
            servo.setPosition(servoPosition);
            servo1DisplaySpeed = servo1Speed*100;  //for display purposes.  Unnecessary

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Position", "%.2f", servo.getPosition()); // Display current position of servo
            telemetry.addData("Servo Speed", "%.2f", servo1DisplaySpeed);
            telemetry.addData("Servo Min Position", "%.2f", servoMinPosition);
            telemetry.addData("Servo Max Position", "%.2f", servoMaxPosition);
            telemetry.update();
            sleep(25);
        }
    }
}
