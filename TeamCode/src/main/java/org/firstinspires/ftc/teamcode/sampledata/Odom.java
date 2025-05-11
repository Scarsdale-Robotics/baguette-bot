package org.firstinspires.ftc.teamcode.sampledata;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odom {
    private final DcMotor leftEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor centerEncoder;
    // last pos
    private int lastLeft = 0;
    private int lastRight = 0;
    private int lastCenter = 0;

    // curent pos
    public double x = 0.0;       // inch
    public double y = 0.0;       // inch
    public double heading = 0.0; // radians (IDK HOW TO IMPLMENT THIS)


    private static final double ticks = 8192.0; // i think so idk fosho
    private static final double wheel_di = 2.0;   // inch

    private static final double ticks_per_inch  = (ticks) / (Math.PI * wheel_di);

    public Odom(HardwareMap hw, double x, double y, double heading) {
        leftEncoder = hw.get(DcMotor.class, "leftEncoder");
        rightEncoder = hw.get(DcMotor.class, "rightEncoder");
        centerEncoder = hw.get(DcMotor.class, "centerEncoder");

        leftEncoder.setDirection(DcMotor.Direction.FORWARD);
        rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        centerEncoder.setDirection(DcMotor.Direction.FORWARD);

        this.x = x;
        this.y = y;
        this.heading = heading; //HELP

        lastLeft = leftEncoder.getCurrentPosition();
        lastRight = rightEncoder.getCurrentPosition();
        lastCenter = centerEncoder.getCurrentPosition();
    }

    public void update(){
        int curLeft = leftEncoder.getCurrentPosition();
        int curRight = rightEncoder.getCurrentPosition();
        int curCenter = centerEncoder.getCurrentPosition();

        int deltaLeftT = curLeft - lastLeft;
        int deltaRightT = curRight - lastRight;
        int deltaCenterT = curCenter - lastCenter;

        lastLeft = curLeft;
        lastRight = curRight;
        lastCenter = curCenter;

        double deltaLeft = deltaLeftT / ticks_per_inch;
        double deltaRight = deltaRightT / ticks_per_inch;
        double deltaCenter = deltaCenterT / ticks_per_inch;

        double deltaHeading = deltaRight - deltaLeft;

        double deltaY = (deltaLeft + deltaRight) / 2.0;
        double deltaX = deltaCenter;

        heading += deltaHeading;

        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        double fieldX = deltaX * cos - deltaY * sin;
        double fieldY = deltaX * sin + deltaY * cos;

        x += fieldX;
        y += fieldY;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading(){
        return heading;
    }
}
