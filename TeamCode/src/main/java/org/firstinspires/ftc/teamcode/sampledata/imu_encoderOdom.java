package org.firstinspires.ftc.teamcode.sampledata;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class imu_encoderOdom {
    private final DcMotor centerEncoder;
    private final IMU imu;

    private double x;
    private double y;
    private double heading;

    private double d;

    private double deltaHeading;
    private double lastheading;
    private double lastd;

    public imu_encoderOdom(HardwareMap hm, double x1, double y1, double heading1){
        centerEncoder = hm.get(DcMotor.class, "forwardOdom");
        imu = hm.get(IMU.class, "imu");
        x = x1;
        y= y1;
        heading = heading1;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void update(){
        d = centerEncoder.getCurrentPosition();
        double deltaD = (d - lastd);
        lastd = d;

        double headingRad = getCurHeading();
        //im not rlly sure if i need deltaHeading cus i dont use it
        deltaHeading = headingRad - lastheading;
        lastheading = headingRad;

        double TICKS_PER_INCH = 1000; // update i forgor
        double deltaDistanceInInches = deltaD / TICKS_PER_INCH;

        x += deltaDistanceInInches * Math.cos(headingRad);
        y += deltaDistanceInInches * Math.sin(headingRad);
    }

    public double getCurHeading(){
        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        heading = rawHeading + Math.PI/2;
        //heading i think should only be between -pi and pi
        heading = Math.atan2(Math.sin(heading), Math.cos(heading));
        return heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDeltaHeading(){return deltaHeading;}

    public double getEncoderPosition() {return centerEncoder.getCurrentPosition();}

}