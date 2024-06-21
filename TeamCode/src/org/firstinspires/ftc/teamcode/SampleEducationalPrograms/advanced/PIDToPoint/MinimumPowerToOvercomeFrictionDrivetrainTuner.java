package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.PIDToPoint;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.DriveEncoderLocalizer;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Pose2d;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;

import java.util.ArrayList;

@Autonomous
public class MinimumPowerToOvercomeFrictionDrivetrainTuner extends LinearOpMode {

    double[] sums = new double[4];
    int iterations = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain robot = new Drivetrain(hardwareMap, telemetry);
        DriveEncoderLocalizer localizer = new DriveEncoderLocalizer(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight, robot.imu);

        double[] minPowersToOvercomeFriction = new double[4];

        Pose2d robotPose;

        ArrayList<DcMotorEx> motors = new ArrayList<>();
        motors.add(robot.backRight);
        motors.add(robot.frontRight);
        motors.add(robot.backLeft);
        motors.add(robot.frontLeft);

        double[] targetPowers = new double[4];

        waitForStart();

        for (int i = 0; i < 4; i++) {

            for (int a = 0; a < iterations; a++) {
                localizer.setStartPose(new Pose2d(0, 0, 0));
                long start = System.currentTimeMillis();
                for (double j = 0; j < 1; j = (double) (System.currentTimeMillis() - start) / (15000.0)) {
                    motors.get(0).setPower(targetPowers[0]);
                    motors.get(1).setPower(targetPowers[1]);
                    motors.get(2).setPower(targetPowers[2]);
                    motors.get(3).setPower(targetPowers[3]);
                    localizer.update();

                    targetPowers[i] = j;

                    robotPose = localizer.getPoseEstimate();
                    if (Math.abs(robotPose.getX()) > 0.1 || Math.abs(robotPose.getY()) > 0.1 || Math.abs(robotPose.getHeading()) > Math.toRadians(1)) {
                        minPowersToOvercomeFriction[i] = j;
                        break;
                    }
                    telemetry.addData(i + " current power: ", j);
                    telemetry.update();
                }

                targetPowers[i] = 0.0;

                sums[i] += minPowersToOvercomeFriction[i] * (12 / hardwareMap.voltageSensor.iterator().next().getVoltage());

                long waitStart = System.currentTimeMillis();
                while (System.currentTimeMillis() - waitStart < 1000) {
                    motors.get(0).setPower(targetPowers[0]);
                    motors.get(1).setPower(targetPowers[1]);
                    motors.get(2).setPower(targetPowers[2]);
                    motors.get(3).setPower(targetPowers[3]);
                }
            }

            System.out.println(i + " AVERAGE min power with voltage correction: " + sums[i]/iterations + "");
        }
        // Front Right: (0.02074285714285714 + 0.02065142857142857) / 2
        // Back Left: 0.020857142857142855
        // Front Left: 0.020822857142857143
        // Back Right: 0.020925714285714284
    }
}