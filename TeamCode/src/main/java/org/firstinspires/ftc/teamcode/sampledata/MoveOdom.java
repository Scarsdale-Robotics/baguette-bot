package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.teamcode.MainTeleop.LEFT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.MainTeleop.RIGHT_POWER_FACTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.DriveSubsystem;

import java.util.List;

@TeleOp(name = "xxyz")
public class MoveOdom extends LinearOpMode {

    imu_encoderOdom odom;
    DriveSubsystem drive;
    Limelight3A limelight;


    double[] apriltagvals;
    private int detectionCount = 0;
    private boolean triangle;
    private Kolman kolman;

    private DcMotor left;
    private DcMotor right;


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odom = new imu_encoderOdom(hardwareMap, 0, 0, 0);


        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        drive = new DriveSubsystem(left, right);
        apriltagvals = new double[3]; // [x, y, heading]
        kolman = new Kolman(0, odom, limelight);



        waitForStart();

        while (opModeIsActive()) {
            detectionCount = 0;
            odom.update();
            apriltagUpdate();

            double leftPower = gamepad1.left_stick_y * LEFT_POWER_FACTOR;
            double rightPower = -gamepad1.right_stick_y * RIGHT_POWER_FACTOR;
            drive.motorsByPowers(leftPower, rightPower);

            triangle = gamepad1.triangle;
            if(triangle && gamepad1.triangle ){
                turn((Math.PI/2), true);
            }

            // Odom Telemetry
            telemetry.addLine("--- Odom Telemetry ---");
            telemetry.addData("X", odom.getX());
            telemetry.addData("Y", odom.getY());
            telemetry.addData("Heading", odom.getCurHeading());

            // AprilTag Telemetry
            telemetry.addLine("--- AprilTag Telemetry ---");
            telemetry.addData("# Tags", detectionCount);
            telemetry.addData("X", apriltagvals[0]);
            telemetry.addData("Y", apriltagvals[1]);

            // Kolman
            telemetry.addLine("--- Kolman Telemetry ---");
            telemetry.addData("Distance", kolman.getPosition());
            kolman.updateKF();


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

        //targetHeading = Math.atan2(Math.sin(targetHeading), Math.cos(targetHeading));
       // if (targetHeading > Math.PI) targetHeading -= 2*Math.PI;
        //if (targetHeading < 0) targetHeading += 2*Math.PI;

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

            if(direction){
                left.setPower(-output);
                right.setPower(output);
            }else{
                left.setPower(output);
                right.setPower(-output);
            }

            // set motor power
            //if (output > 0.1) {
            //left.setPower(1);
            //right.setPower(-1);
            //} else if (output < -0.1) {
            //left.setPower(-1);
            //right.setPower(1);
            //} else {
            //left.setPower(0);
            //right.setPower(0);
            //}

            prevError = error;
            prevTime = currentTime;

        }


    }






}
