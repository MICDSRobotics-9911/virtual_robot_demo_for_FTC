package org.firstinspires.ftc.teamcode.SampleEducationalPrograms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Sample_Time_Based_Auto")
public class Sample_Time_Based_Auto extends LinearOpMode {
    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;

    public static int turnMilliseconds = 1700;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        imu = hardwareMap.get(IMU.class, "imu");
        // Logo Up, USB Forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Current Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            for (int i = 0; i < 4; i++) {
                backLeft.setPower(0.5);
                frontLeft.setPower(0.5);
                frontRight.setPower(0.5);
                backRight.setPower(0.5);

                sleep(2000);

                backLeft.setPower(-0.5);
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backRight.setPower(0.5);

                sleep(turnMilliseconds);
                telemetry.update();

            }
            backLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }
}
