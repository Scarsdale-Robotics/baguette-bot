package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.teamcode.MainTeleop.LEFT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.MainTeleop.RIGHT_POWER_FACTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Motor;
import org.firstinspires.ftc.teamcode.scaleOperators.Clamp;
import org.firstinspires.ftc.teamcode.scaleOperators.Rescale;

public class MoveOdom extends LinearOpMode {

    imu_encoderOdom odom;
    DriveSubsystem drive;

    @Override
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




        drive = new DriveSubsystem(left, right);
        while (opModeIsActive()){
            odom.update();
            double leftPower = gamepad1.left_stick_y * LEFT_POWER_FACTOR;
            double rightPower = gamepad1.right_stick_y * RIGHT_POWER_FACTOR;
            drive.motorsByPowers(leftPower, rightPower);
            telemetry.addLine("x: " + odom.getX());
            telemetry.addLine("y: " + odom.getY());
            telemetry.addLine("Heading " + odom.getCurHeading());
        }
    }
}
