package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MoveOdom {
    private  Odom Odom;
    private Limelight3A limelight;


    public MoveOdom() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(120);
        limelight.start();
        Odom = new Odom(hardwareMap, 0, 0,0);
        limelight.pipelineSwitch(0);
    }

    public LLResult getAprilTag(){
        return limelight.getLatestResult();
    }

    public double getTagY(){
        return getAprilTag().getTy();
    }

    //so now we have the distance to tag based off of april tag and we have the current pos based on odom
    //we can also get current bot pos based on lime light

}
