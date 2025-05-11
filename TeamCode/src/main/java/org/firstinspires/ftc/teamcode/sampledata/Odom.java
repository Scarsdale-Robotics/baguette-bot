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

        int deltaLeftTicks = curLeft - lastLeft;
        int deltaRightTicks = curRight - lastRight;
        int deltaCenterTicks = curCenter - lastCenter;

        lastLeft = curLeft;
        lastRight = curRight;
        lastCenter = curCenter;

        double deltaLeft = deltaLeftTicks / ticks_per_inch;
        double deltaRight = deltaRightTicks / ticks_per_inch;
        double deltaCenter = deltaCenterTicks / ticks_per_inch;


    }
}
