package org.firstinspires.ftc.teamcode.Util;

public class TestRobotConstants {
    private final double WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;


    //TODO: Test for values
    private final int ENCODER_TICKS_PER_REVOLUTION_20 = 140*4; // NeverRest 20

    //Methods
    public int getTICKS_PER_INCH_20()
    {
        return (int)(ENCODER_TICKS_PER_REVOLUTION_20 / WHEEL_CIRCUMFERENCE);
    }
}


