package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.PIDController;

@TeleOp(name="Sample_Gyro_Control", group="advanced")
public class Sample_Gyro_Control extends OpMode {
    private double integralSum = 0;
    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 2.5;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private Drivetrain drive;
    private IMU imu;
    private double referenceAngle;
    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap, telemetry);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    }

    @Override
    public void init_loop() {
        imu.resetYaw();
        telemetry.addData("Robot Heading: ", drive.getHeading());
    }

    public void start() {
        telemetry.addData("Target IMU Angle: ", 180);
        telemetry.addData("Current IMU Angle", drive.getHeading());
        drive.setTargetPosition(2000);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setPower(0.5);
        while (drive.isBusy()) {
            telemetry.update();
        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        referenceAngle = Math.toRadians(90);
    }

    @Override
    public void loop() {
        double power = PIDControl(referenceAngle, Math.toRadians(drive.getHeading()));
        drive.frontLeft.setPower(-power);
        drive.backLeft.setPower(-power);
        drive.frontRight.setPower(power);
        drive.backRight.setPower(power);
        telemetry.addData("Current Angle: ", drive.getHeading());
        telemetry.addData("Error(in degrees): ", Math.toDegrees(referenceAngle) - drive.getHeading());
        telemetry.update();
    }

    public double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
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
