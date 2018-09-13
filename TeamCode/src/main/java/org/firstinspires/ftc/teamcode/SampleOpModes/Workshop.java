package org.firstinspires.ftc.teamcode.SampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Workshop extends OpMode {

    DcMotor left;
    DcMotor right;

    @Override
    public void init() {

       left = hardwareMap.dcMotor.get("right");
       right= hardwareMap.dcMotor.get("left");

    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            right.setPower(1);
        }

        if (gamepad1.b) {

            left.setPower(1);
        }

        if (gamepad1.x){
            right.setPower(0);


        }

    }
}
