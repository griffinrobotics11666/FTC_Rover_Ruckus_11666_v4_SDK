package org.firstinspires.ftc.teamcode.Util;

public class RobotConstants {
    private final double WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;



    //TODO: Test for values
    private final int ENCODER_TICKS_PER_REVOLUTION_20 = 140*4; // NeverRest 20
    private final int ENCODER_TICKS_PER_REVOLUTION_40 = 280*4; // NeverRest 40
    private final int ENCODER_TICKS_PER_REVOLUTION_60 = 420*4; // NeverRest 60

    //Methods
    public int getTICKS_PER_INCH_40()
    {
        return (int)(ENCODER_TICKS_PER_REVOLUTION_40 / WHEEL_CIRCUMFERENCE);
    }
    public double getStrafe()
    {
        return (10000/98)*(10/9.25);
    }
    public double getLift(){
        return 10950;
    }
    public double getLiftServoOpen(){
        return .4677;
    }
    public double getLiftServoClose(){
        return .403;
        //.32
    }
    public double getLeftServoOpen(){
        return .55;
    }
    public double getLeftServoClose(){
        return .647;
    } //.613
    public double getRightServoOpen(){
        return .25;
    }
    public double getRightServoClose(){
        return .13;
    }
    public double getArm(){
        return 100;
    }
    public double getElbow(){
        return 100;
    }
    public double getElbowMax(){
        return 100;
    }
    public double getElbowMin(){
        return 100;
    }
}


