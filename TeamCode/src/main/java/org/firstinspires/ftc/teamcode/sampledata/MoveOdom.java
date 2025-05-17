package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.teamcode.MainTeleop.LEFT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.MainTeleop.RIGHT_POWER_FACTOR;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Motor;

import java.util.List;

@TeleOp(name = "xxyz")
public class MoveOdom extends LinearOpMode {

    imu_encoderOdom odom;
    DriveSubsystem drive;
    Limelight3A limelight;

    double[] apriltagvals;
    private int detectionCount = 0;
    private boolean triangle;
    private DcMotor left = hardwareMap.get(DcMotor.class, "left");
    private DcMotor right = hardwareMap.get(DcMotor.class, "right");

    public void runOpMode() throws InterruptedException {
        odom = new imu_encoderOdom(hardwareMap, 0, 0, 0);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        drive = new DriveSubsystem(left, right);
        apriltagvals = new double[3]; // [x, y, heading]


        waitForStart();

        while (opModeIsActive()) {
            detectionCount = 0;
            odom.update();
            apriltagUpdate();

            double leftPower = gamepad1.left_stick_y * LEFT_POWER_FACTOR;
            double rightPower = gamepad1.right_stick_y * RIGHT_POWER_FACTOR;
            drive.motorsByPowers(leftPower, rightPower);

            triangle = gamepad1.triangle;
            if(triangle && gamepad1.triangle ){
                //
            }

            // Odom Telemetry
            telemetry.addLine("--- Odom Telemetry ---");
            telemetry.addData("X", odom.getX());
            telemetry.addData("Y", odom.getY());
            telemetry.addData("Heading", odom.getCurHeading());

            // at Telemetry
            telemetry.addLine("--- AprilTag Telemetry ---");
            telemetry.addData("# Tags", detectionCount);
            telemetry.addData("X", apriltagvals[0]);
            telemetry.addData("Y", apriltagvals[1]);


            telemetry.update();
        }
    }

    public void apriltagUpdate() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("no:(");
            return;
        }

        if (result.isValid()) {
            List<FiducialResult> tags = result.getFiducialResults();

            if (!tags.isEmpty()) {
                detectionCount++;
                Pose3D pose = tags.get(0).getTargetPoseRobotSpace();

                apriltagvals[0] = pose.getPosition().x;
                apriltagvals[1] = pose.getPosition().y;
                apriltagvals[2] = tags.get(0).getTargetXDegrees();
            }

        }
    }


    //pre-req value has to be in between 0-pi rad

    public void turn(double radians, boolean direction) {
        double startHeading = odom.getCurHeading();
        double targetHeading = direction ? startHeading + radians : startHeading - radians;

        // Is this the right math ????
        if (targetHeading > Math.PI) targetHeading -= Math.PI;
        if (targetHeading < 0) targetHeading += Math.PI;

        // TUNE (i think)
        double kP = 2.0;
        double kD = 0.5;

        double prevError = 0;
        long prevTime = System.currentTimeMillis();

        while (true) {
            double currentHeading = odom.getCurHeading();

            double error = targetHeading - currentHeading;
            if (error > Math.PI) error -= Math.PI;
            if (error < -Math.PI) error += Math.PI;

            if (Math.abs(error) < 0.02) break;

            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - prevTime) / 1000.0;
            double derivative = (error - prevError) / (deltaTime != 0 ? deltaTime : 0.001);


            double output = kP * error + kD * derivative;

            // set motor power
            if (output > 0.1) {
                left.setPower(1);
                right.setPower(-1);
            } else if (output < -0.1) {
                left.setPower(-1);
                right.setPower(1);
            } else {
                left.setPower(0);
                right.setPower(0);
            }

            prevError = error;
            prevTime = currentTime;

            telemetry.addData("Current", currentHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();

        }


    }






}
