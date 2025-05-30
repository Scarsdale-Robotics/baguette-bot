package org.firstinspires.ftc.teamcode.sampledata;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@Config
public class Kolman {
    private double x = 0; // your initial state
    public static double Q = 0.1; // your model covariance
    public static double R = 0.4; // your sensor covariance
    public static double p = 1; // your initial covariance guess
    public static double K = 1;// your initial Kalman gain guess

    private double o_previous;
    private imu_encoderOdom odom;
    private Limelight3A light;

    public Kolman(double x1, imu_encoderOdom odom1, Limelight3A light1){
        x = x1;
        odom = odom1;
        o_previous = odom.getEncoderPosition();
        light = light1;
    }

    public double getPosition() { return x; }

    public void updateKF(){
        predict();
        update();
    }

    private void predict() {
        x += odom.getEncoderPosition() - o_previous;
        p += Q;
    }

    private void update() {
        K = p/(p+R);
        LLResult result = light.getLatestResult();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        Position pose = tags.get(0).getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);

        double x = pose.x;
        double y = pose.y;
        double z = pose.z;
        double distance = Math.hypot(x, Math.hypot(y, z));
        x += K*(distance-x);
        p = ( 1-K)*p;
    }

}
