package org.firstinspires.ftc.teamcode.SampleEducationalPrograms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Sample_Encoder_Based_Auto")
public class Sample_Encoder_Based_Auto extends OpMode {
    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    IMU imu;

    @Override
    public void init() {
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
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void start() {
        for (int i = 0; i < 4; i++) {
            setTargetPosition(2000);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            setPower(0.5);
            while (isBusy()) {
            }

            setRightTarget(2100);
            setLeftTarget(-2100);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setRightPower(0.5);
            setLeftPower(-0.5);
            while (isBusy()) {
            }
        }
    }
    @Override
    public void loop() {
        telemetry.addData("backLeftPos: ", backLeft.getCurrentPosition());
        telemetry.addData("frontLeftPos: ", frontLeft.getCurrentPosition());
        telemetry.addData("frontRightPos: ", frontRight.getCurrentPosition());
        telemetry.addData("backRightPos: ", backRight.getCurrentPosition());
        telemetry.update();
    }

    public void setMode(DcMotor.RunMode mode) {
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void setTargetPosition(int target) {
        setRightTarget(target);
        setLeftTarget(target);
    }

    public void setRightTarget(int target) {
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + target);
        backRight.setTargetPosition(backRight.getCurrentPosition() + target);
    }

    public void setLeftTarget(int target) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + target);
    }

    public void setPower(double power) {
        setRightPower(power);
        setLeftPower(power);
    }

    public void setRightPower(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void setLeftPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public boolean isBusy() {
        return frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

}
