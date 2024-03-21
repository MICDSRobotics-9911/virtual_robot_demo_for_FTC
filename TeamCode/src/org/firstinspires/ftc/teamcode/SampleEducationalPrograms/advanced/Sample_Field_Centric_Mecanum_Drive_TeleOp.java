package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Sample_Field_Centric_Mecanum_Drive_TeleOp", group="advanced")
public class Sample_Field_Centric_Mecanum_Drive_TeleOp extends OpMode {
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

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // Logo Up, USB Forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            imu.resetYaw();
        }
        double backLeftPower, frontLeftPower, frontRightPower, backRightPower;
        double x, y, turn, power, theta, botHeading;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        x = gamepad1.left_stick_x;
        // This is negative because the natural gamepad values for y axis on stick is flipped(down is 1 and up is -1)
        y = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        /*(power = Math.hypot(rotX, rotY);
        theta = Math.atan2(rotY, rotX);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));*/
        frontLeftPower = (rotY + rotX + turn) / denominator;
        backLeftPower = (rotY - rotX + turn) / denominator;
        frontRightPower = (rotY - rotX - turn) / denominator;
        backRightPower = (rotY + rotX - turn) / denominator;

        /*if ((power + Math.abs(turn)) > 1) {
            frontLeftPower /= power + Math.abs(turn);
            frontRightPower /= power + Math.abs(turn);
            backLeftPower /= power + Math.abs(turn);
            backRightPower /= power + Math.abs(turn);
        }*/

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }
}
