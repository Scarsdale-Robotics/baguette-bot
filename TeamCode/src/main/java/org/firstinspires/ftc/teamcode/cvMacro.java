package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class cvMacro extends LinearOpMode {
    public Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

    private double PEX = 0;
    private double PEY = 0;
    private double PEBEARING = 0;


    private DcMotor left;
    private DcMotor right;

    private double x = 0.0; // change based on predicted pos from odon
    private double u = 0.0; //
    private double p = 0.0;
    private double q = 0.0;





    @Override
    public void runOpMode() throws InterruptedException {
        filter();
    }

    public void filter(){
        while(opModeIsActive()){
            predict();
            update();
        }
    }

    public void predict(){
        x = x + u;
        p = p + q;
    }

    public void update(){

    }

}
