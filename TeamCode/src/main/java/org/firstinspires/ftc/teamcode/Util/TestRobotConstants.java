package org.firstinspires.ftc.teamcode.Util;

public class TestRobotConstants {


    public TestRobotConstants(){

    }

    private final double WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double GEAR_RATIO = 1;
    private final double FUDGE_FACTOR = 1;



    //TODO: Test for values
    private final int ENCODER_TICKS_PER_REVOLUTION_20 = 140*4; // NeverRest 20
    private final int ENCODER_TICKS_PER_REVOLUTION_60 = 420*4; // NeverRest 60

    //Methods
    public double getTICKS_PER_INCH_60()
    {
        return (double)(GEAR_RATIO * ENCODER_TICKS_PER_REVOLUTION_60 / WHEEL_CIRCUMFERENCE);
    }
    public double getWHEEL_DIAMETER()
    {
        return WHEEL_DIAMETER;
    }


}


