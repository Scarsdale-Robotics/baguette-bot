package org.firstinspires.ftc.teamcode.scaleOperators;

public class Rescale extends ScaleOperator {

    private double CENTER_OFFSET, RANGE_RATIO;

    public Rescale(double rangeRatio) {
        init(0, rangeRatio);
    }

    public Rescale(double centerOffset, double rangeRatio) {
        init(centerOffset, rangeRatio);
    }

    public Rescale(double lower1, double upper1, double lower2, double upper2) {
        double center1 = (upper1 + lower1) / 2.0;
        double center2 = (upper2 + lower2) / 2.0;
        double centerOffset = center2 - center1;

        double range1 = upper1 - lower1;
        double range2 = upper2 - lower2;
        double rangeRatio = range2 / range1;

        init(centerOffset, rangeRatio);
    }

    private void init(double centerOffset, double rangeRatio) {
        CENTER_OFFSET = centerOffset;
        RANGE_RATIO = rangeRatio;
    }

    @Override
    public double apply(double x) {
        return (x + CENTER_OFFSET) * RANGE_RATIO;
    }

}
