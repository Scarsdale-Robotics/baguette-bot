package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.teamcode.MainTeleop.LEFT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.MainTeleop.RIGHT_POWER_FACTOR;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Motor;
import org.firstinspires.ftc.teamcode.scaleOperators.Clamp;
import org.firstinspires.ftc.teamcode.scaleOperators.Rescale;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
@TeleOp(name = "xxyz")
public class MoveOdom extends LinearOpMode {

    imu_encoderOdom odom;
    DriveSubsystem drive;
    Limelight3A limelight;

    double[] apriltagvals;
    // 0 = x // 1 = y // 2  = heading


    public void runOpMode() throws InterruptedException {
        odom =  new imu_encoderOdom(hardwareMap, 0,0,0);
        Motor left = new Motor.Builder(hardwareMap, "left")
                .setOperators(
                        new Rescale(LEFT_POWER_FACTOR),
                        new Clamp(-LEFT_POWER_FACTOR, LEFT_POWER_FACTOR)
                )
                .build();
        Motor right = new Motor.Builder(hardwareMap, "right")
                .setOperators(
                        new Rescale(RIGHT_POWER_FACTOR),
                        new Clamp(-RIGHT_POWER_FACTOR, RIGHT_POWER_FACTOR)
                )
                .invert()
                .build();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        drive = new DriveSubsystem(left, right);
        apriltagvals = new double[3];

        waitForStart();

        while (opModeIsActive()){
            odom.update();
            apriltagUpdate();
            double leftPower = gamepad1.left_stick_y * LEFT_POWER_FACTOR;
            double rightPower = gamepad1.right_stick_y * RIGHT_POWER_FACTOR;
            drive.motorsByPowers(leftPower, rightPower);
            telemetry.addData("Odom", 0);
            telemetry.addData("x", odom.getX());
            telemetry.addData("y", odom.getY());
            telemetry.addData("Heading", odom.getCurHeading());
            telemetry.addData("AprilTag", 0);
            telemetry.addData("x", apriltagvals[0]);
            telemetry.addData("y", apriltagvals[1]);
            telemetry.update();
        }


    }
    public void apriltagUpdate(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> tags = result.getFiducialResults();
            if (!tags.isEmpty()) {
                apriltagvals[0] = tags.get(0).getTargetPoseRobotSpace().getPosition().x;
                apriltagvals[1] = tags.get(0).getTargetPoseRobotSpace().getPosition().y;
            }
        }
    }
}
