package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorEncoder {

    private DcMotorEx motor;
    private int currentPosition = 0;
    private int previousPosition = 0;
    private double currentVelocity = 0.0;

    public MotorEncoder(DcMotorEx motor) {
        this.motor = motor;
    }

    public void update(double dt) {
        currentPosition = motor.getCurrentPosition();
        currentVelocity = Derivatives.getDerivative(currentPosition, previousPosition, dt);
    }

    public void updateLast() {
        previousPosition = currentPosition;
    }

    public int getPosition() {
        return this.currentPosition;
    }

    public double getVelocity() {
        return this.currentVelocity;
    }
}
