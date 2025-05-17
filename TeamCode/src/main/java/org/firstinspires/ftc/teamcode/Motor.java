package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.scaleOperators.Clamp;
import org.firstinspires.ftc.teamcode.scaleOperators.ScaleOperator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class Motor {

    private DcMotor dcMotor;
    private ArrayList<ScaleOperator> operators = new ArrayList<>(Collections.singletonList(new Clamp(-1, 1)));
    private double TICKS_PER_INCH = 0;

    public static class Builder {
        private DcMotor dcMotor;
        private Motor motor;

        public Builder(HardwareMap hardwareMap, String id) {
            dcMotor = hardwareMap.get(DcMotor.class, id);
            motor.dcMotor = dcMotor;
        }

        public Builder invert() {
            dcMotor.setDirection(dcMotor.getDirection().inverted());
            return this;
        }

        public Builder setTicksPerInch(double value) {
            motor.TICKS_PER_INCH = value;
            return this;
        }

        public Builder setOperators(ScaleOperator... operators) {
            motor.operators = new ArrayList<>(Arrays.asList(operators));
            return this;
        }

        public Motor build() {
            return motor;
        }

    }

    enum PositionUnit {
        TICKS,
        INCHES
    }

    public double getCurrentPosition() {
        return getCurrentPosition(PositionUnit.TICKS);
    }

    public double getCurrentPosition(PositionUnit unit) {
        return dcMotor.getCurrentPosition() * (unit==PositionUnit.INCHES ? TICKS_PER_INCH : 1);
    }

    public void setPower(double power) {
        for (ScaleOperator operator : operators) {
            power = operator.apply(power);
        }

        dcMotor.setPower(power);
    }



}
