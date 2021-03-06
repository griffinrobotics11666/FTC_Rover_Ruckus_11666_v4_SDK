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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Sample Op ROB DR USING HWMAP", group="SAMPLE OPMODES")
@Disabled
public class SampleOpMode_ROBOTDRIVE_USING_HWMAP extends LinearOpMode {

    // Declare OpMode members.
    HardwareSampleBot robot = new HardwareSampleBot();


    public int driveMode = 0;
    public String driveModeDisplay = "Arcade Mode";
    public double maxSpeed = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();
        robot.period.reset(); //resets the amount of runtime (called from the hardware map)
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or Arcade Mode
            // Comment out the method that's not used.  The default below is Arcade.

            // Arcade Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;

            if (driveMode == 0)
            {
                leftPower = Range.clip(drive + turn, -1*maxSpeed, 1*maxSpeed);
                rightPower = Range.clip(drive - turn, -1*maxSpeed, 1*maxSpeed);
            }
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            else
            {
                leftPower = Range.clip(-gamepad1.left_stick_y, -1*maxSpeed, 1*maxSpeed);
                rightPower = Range.clip(-gamepad1.right_stick_y, -1*maxSpeed, 1*maxSpeed);
            }
            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            if (gamepad1.b && driveMode == 0)
            {
                driveMode = 1;
                driveModeDisplay = "Tank Drive";
                sleep(200);
            }
            if (gamepad1.b && driveMode == 1)
            {
                driveMode = 0;
                driveModeDisplay = "Arcade Drive";
                sleep(200);
            }
            if (gamepad1.right_bumper && maxSpeed < 1)
            {
                sleep(200);
                maxSpeed += 0.1;
            }
            if (gamepad1.left_bumper && maxSpeed > .2)
            {
                sleep(200);
                maxSpeed -= 0.1;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + robot.period.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Max Speed ", "%.2f", maxSpeed);
            telemetry.addData("Robot is currently in ", driveModeDisplay);
            telemetry.update();

        }
    }
}
