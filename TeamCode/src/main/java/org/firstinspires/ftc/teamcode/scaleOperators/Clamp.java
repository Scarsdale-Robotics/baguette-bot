package org.firstinspires.ftc.teamcode.scaleOperators;

public class Clamp extends ScaleOperator {

    private final double LOWER, UPPER;

    public Clamp(double lower, double upper) {
        LOWER = lower;
        UPPER = upper;
    }

    @Override
    public double apply(double x) {
        return Math.min(Math.max(x, LOWER), UPPER);
    }

}
