package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Config
public class DriveSubsystem {

    Motor left, right;

    /**
     * Initializes a DriveSubsystem
     * @param left A left-side mounted DcMotor with positive power assumed as "forward"
     * @param right A right-side mounted DcMotor with positive power assumed as "forward"
     */
    public DriveSubsystem(Motor left, Motor right) {
        this.left = left;
        this.right = right;
    }

    /**
     * Rotates the robot using powers.
     * Do NOT call when forwardByPowers is also called.
     * @param power Positive is counterclockwise, negative is clockwise, in [-1, 1]
     */
    public void rotateByPowers(double power) {
        left.setPower(-power);
        right.setPower(power);
    }

    /**
     * Moves the robot forward/backward using powers.
     * Do NOT call when rotateByPowers is also called.
     * @param power Positive is forward, negative is backward, in [-1, 1]
     */
    public void forwardByPowers(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    /**
     * Powers drive motors individually.
     * @param powerLeft Positive is forward, negative is backward, in [-1, 1], for left motor
     * @param powerRight Positive is forward, negative is backward, in [-1, 1], for right motor
     */
    public void motorsByPowers(double powerLeft, double powerRight) {
        left.setPower(powerLeft);
        right.setPower(powerRight);
    }

}
