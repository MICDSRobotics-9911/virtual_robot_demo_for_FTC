package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double Kp;
    double Ki;
    double Kd;
    double lastError = 0;
    double integralSum = 0;
    boolean angleWrap = false;

    long lastLoopTime = System.nanoTime();
    int counter = 0;
    double loopTime;

    /**
     * Set PID gains
     * @param Kp proportional gain
     * @param Ki integral gain
     * @param Kd derivative gain
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    /**
     * calculate PID output given the reference and the current system state
     * @param error where we would like our system to be
     * @return the signal to send to our motor or other actuator
     */
    public double update(double error) {
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }
        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime) / 1000000000.0;
        lastLoopTime = currentTime;
        double derivative;
        // check if we need to unwrap angle
        // forward euler integration
        integralSum += error * loopTime;
        derivative = (error - lastError) / loopTime;

        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);

        lastError = error;
        counter++;

        return output;
    }


    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }


}
