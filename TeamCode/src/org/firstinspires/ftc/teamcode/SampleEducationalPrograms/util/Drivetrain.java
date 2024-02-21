package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util;

import static org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.RobotConstants.COUNTS_PER_INCH;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Drivetrain {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    private PIDController gyroController;
    private PIDController driveController;
    private IMU imu;
    private Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/

        // Control HUb
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
         * **/
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**
         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         * **/

        //THIS IS THE CORRECT ORIENTATION
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        /**
         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
         * **/

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.resetYaw();
        gyroController = new PIDController(3, 0, 0, true);
    }

    private void turnAdjust(double output){
        frontLeft.setPower(-output);
        backLeft.setPower(-output);
        frontRight.setPower(output);
        backRight.setPower(output);
    }



    public void encoderDrive(double speed, double leftFrontInches,
                             double rightFrontInches, double leftBackInches, double rightBackInches) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        newLFTarget = frontLeft.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
        newRFTarget = frontRight.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
        newLBTarget = backLeft.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
        newRBTarget = backRight.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(Math.abs(speed));

        while (isBusy()) {
            holdHeading();
            telemetry.addData("frontLeftPos: ", frontLeft.getCurrentPosition());
            telemetry.addData("frontRightPos: ", frontRight.getCurrentPosition());
            telemetry.addData("backLeftPos: ", backLeft.getCurrentPosition());
            telemetry.addData("backRightPos: ", backRight.getCurrentPosition());
            telemetry.update();
        }
        setPower(0);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void moveForward(double speed, double inches) {
        encoderDrive(speed, inches, inches, inches, inches);
    }

    public void moveBackward(double speed, double inches) {
        moveForward(speed, -inches);
    }

    public void strafeLeft(double speed, double inches) {
        encoderDrive(speed, -inches, inches, inches, -inches);
    }

    public void strafeRight(double speed, double inches) {
        encoderDrive(speed, inches, -inches, -inches, inches);
    }

    public void turnTo(double referenceAngle) {
        double power = gyroController.update(Math.toRadians(referenceAngle),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        turnAdjust(power);
        telemetry.addData("Target Angle:  ", referenceAngle);
        telemetry.addData("Current IMU Angle", getHeading());
        telemetry.addData("Error: ", referenceAngle - getHeading());
    }

    public void holdHeading() {
        turnTo(getHeading());
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

    public  double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}