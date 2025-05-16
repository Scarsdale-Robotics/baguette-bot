package org.firstinspires.ftc.teamcode.sampledata;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "AprilTag Distance with Limelight", group = "Vision")
public class AprilTagDistanceOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");


        waitForStart();

        while (opModeIsActive()) {
            // Get the latest botpose from the Limelight
            LLResult tag = limelight.getLatestResult();


        }
    }
}
