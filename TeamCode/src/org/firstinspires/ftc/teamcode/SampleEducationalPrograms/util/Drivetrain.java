package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivetrain {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    public Drivetrain(HardwareMap hardwareMap) {

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

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void turnAdjust(double output){
        frontLeft.setPower(-output);
        backLeft.setPower(-output);
        frontRight.setPower(output);
        backRight.setPower(output);
    }
    public void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        /**
         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
         * **/
        double topLeftPower = leftStickY + leftStickX + rightStickX;
        double bottomLeftPower = leftStickY - leftStickX + rightStickX;
        double topRightPower = leftStickY - leftStickX - rightStickX;
        double bottomRightPower = leftStickY + leftStickX - rightStickX;

        /**
         * Sets the wheel's power
         * **/
        frontLeft.setPower(topLeftPower);
        frontRight.setPower(topRightPower);
        backLeft.setPower(bottomLeftPower);
        backRight.setPower(bottomRightPower);
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