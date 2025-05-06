package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.scaleOperators.Clamp;
import org.firstinspires.ftc.teamcode.scaleOperators.Rescale;

import java.util.List;

@Config
@TeleOp(name = "Baguette Teleop")
public class MainTeleop extends LinearOpMode {

    public static double LEFT_POWER_FACTOR = 1;
    public static double RIGHT_POWER_FACTOR = 1;

    public static double MACRO_RADS_TOLERANCE = 0.17;  // \pm 0.17 tolerance
    public static double MACRO_DISP_TOLERANCE = 2;  // \pm 2 inches (displacement)

    public static double Ksqu_ROT = 0.5;
    public static double Ksqu_TRANS = 0.5;

    private static double ENCODER_CPR = 4096; // Optii v1
    private static double ODOM_DIAMETER = 1.37795276; // inches
    private static double ODOM_TICKS_PER_INCH = ENCODER_CPR / (Math.PI * ODOM_DIAMETER) * 0.864583;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor left = new Motor.Builder(hardwareMap, "left")
                .setOperators(
                        new Rescale(LEFT_POWER_FACTOR),
                        new Clamp(-LEFT_POWER_FACTOR, LEFT_POWER_FACTOR)
                )
                .build();
        Motor right = new Motor.Builder(hardwareMap, "left")
                .setOperators(
                        new Rescale(RIGHT_POWER_FACTOR),
                        new Clamp(-RIGHT_POWER_FACTOR, RIGHT_POWER_FACTOR)
                )
                .invert()
                .build();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;  // TODO: UPDATE
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        DcMotor forwardEncoder = hardwareMap.get(DcMotor.class, "forwardOdom");

        int lastForwardEnc = forwardEncoder.getCurrentPosition();
//        double forwardEncoderOffset = 0, strafeEncoderOffset = 0;

        DriveSubsystem drive = new DriveSubsystem(left, right);

        //////////////////////
        // GAMEPAD CONTROLS //
        //////////////////////
        Button square1Button = new Button();
        Button triangle1Button = new Button();

        waitForStart();

        boolean macroActive = false;
        double macroTargetRads = 0, macroTargetX = 0, macroTargetY = 0;

        double currentX = 0, currentY = 0, currentRads;

        while (opModeIsActive()) {
            ////////////////////
            // UPDATE BUTTONS //
            ////////////////////

            Button.Data square1 = square1Button.update(gamepad1.square);
            Button.Data triangle1 = triangle1Button.update(gamepad1.triangle);

            telemetry.addData("Square 1", square1);
            telemetry.addData("Triangle 1", triangle1);

            /////////////////////////
            // UPDATE LOCALIZATION //
            /////////////////////////

            currentRads = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            int dForwardEnc = forwardEncoder.getCurrentPosition() - lastForwardEnc;

            double dForwardInches = dForwardEnc / ODOM_TICKS_PER_INCH;

            double dForwardInchesX = dForwardInches * Math.sin(currentRads);
            double dForwardInchesY = dForwardInches * -Math.cos(currentRads);

            currentX += dForwardInchesX;
            currentY += dForwardInchesY;

            lastForwardEnc = forwardEncoder.getCurrentPosition();

            telemetry.addData("Theta (rad)", currentRads);
            telemetry.addData("X", currentX);
            telemetry.addData("Y", currentY);

            /////////////////////
            // SET DRIVE MACRO //
            /////////////////////

            // Simple forward left motion macro
            if (square1.onPress) {
                macroTargetRads = Math.PI / 4;
                macroTargetX = currentX - 10;
                macroTargetY = currentY + 10;
                macroActive = true;
            }

            // Align to apriltag macro
            if (triangle1.onPress) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (!tags.isEmpty()) {
                        LLResultTypes.FiducialResult tag = tags.get(0);
                        Pose3D pose = tag.getCameraPoseTargetSpace();

                        double target_rads = currentRads + pose
                                .getOrientation().getYaw(AngleUnit.RADIANS);
                        double target_ds = Math.hypot(pose.getPosition().x, pose.getPosition().y);

                        double target_x = currentX + target_ds * Math.sin(target_rads);
                        double target_y = currentY + target_ds * Math.cos(target_rads);

                        macroTargetRads = target_rads;
                        macroTargetX = target_x;
                        macroTargetY = target_y;
                        macroActive = true;
                    }
                }
            }

            ///////////
            // DRIVE //
            ///////////

            double leftPower = gamepad1.left_stick_y * LEFT_POWER_FACTOR;
            double rightPower = gamepad1.right_stick_y * RIGHT_POWER_FACTOR;

            if (Math.abs(leftPower) + Math.abs(rightPower) > 0)
                macroActive = false;

            drive.motorsByPowers(leftPower, rightPower);

            //////////////////////////
            // EXECUTE DRIVE MACROS //
            //////////////////////////

            if (macroActive) {
                double e_theta = macroTargetRads - currentRads;
                double e_dx = macroTargetX - currentX;
                double e_dy = macroTargetY - currentY;
                double e_ds = Math.hypot(e_dx, e_dy);

                telemetry.addData("e_theta", e_theta);
                telemetry.addData("e_ds", e_ds);

                telemetry.addData("macroTargetX", macroTargetX);
                telemetry.addData("macroTargetY", macroTargetY);
                telemetry.addData("macroTargetRads", macroTargetRads);

                if (Math.abs(e_ds) > MACRO_DISP_TOLERANCE) {
                    // translation
                    double approachRads = Math.atan2(e_dx, e_dy);

                    e_theta = approachRads - currentRads;
                    if (Math.abs(e_theta) > MACRO_RADS_TOLERANCE / 2.0) {  // div. 2 for extra precision
                        // angle align

                        drive.rotateByPowers(Ksqu_ROT * Math.signum(e_theta) * Math.sqrt(Math.abs(e_theta)));
                    } else {
                        // angle aligned
                        drive.forwardByPowers(Ksqu_TRANS * Math.sqrt(Math.abs(e_ds)));
                    }

                } else if (Math.abs(e_theta) > MACRO_RADS_TOLERANCE) {
                    // rotation
                    drive.rotateByPowers(Ksqu_ROT * Math.signum(e_theta) * Math.sqrt(Math.abs(e_theta)));
                } else {
                    macroActive = false;
                }
            }

            telemetry.addData("leftEnc", left.getCurrentPosition());
            telemetry.addData("rightEnc", right.getCurrentPosition());
            telemetry.update();
        }
    }

}
