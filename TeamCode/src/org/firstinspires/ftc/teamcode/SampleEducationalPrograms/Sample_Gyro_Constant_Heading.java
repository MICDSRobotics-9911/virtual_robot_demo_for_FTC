package org.firstinspires.ftc.teamcode.SampleEducationalPrograms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.PIDController;

@TeleOp(name="Sample_Gyro_Constant_Heading", group="advanced")
public class Sample_Gyro_Constant_Heading extends OpMode {
    public static double kP = 3;
    public static double kI = 0;
    public static double kD = 0;
    private Drivetrain drive;
    private PIDController gyroController;
    private IMU imu;
    private double referenceAngle;
    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap, telemetry);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyroController = new PIDController(kP, kI, kD, true);
        referenceAngle = Math.toRadians(180);
    }

    @Override
    public void init_loop() {
        imu.resetYaw();
        telemetry.addData("Robot Heading: ", drive.getHeading());
    }

    public void start() {
        telemetry.addData("Target IMU Angle: ", 180);
        telemetry.addData("Current IMU Angle", drive.getHeading());
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setTargetPosition(2000);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setPower(0.5);
        while (drive.isBusy()) {
            telemetry.update();
        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double referenceAngle = 180;
        drive.turnTo(180);
        drive.holdHeading();
        telemetry.update();
    }

}
