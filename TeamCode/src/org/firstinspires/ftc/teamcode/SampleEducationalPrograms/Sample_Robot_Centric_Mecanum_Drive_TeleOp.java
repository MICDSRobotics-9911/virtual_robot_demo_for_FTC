package org.firstinspires.ftc.teamcode.SampleEducationalPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="Sample_Robot_Centric_Mecanum_Drive_TeleOp")
public class Sample_Robot_Centric_Mecanum_Drive_TeleOp extends OpMode {

    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;

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
    }

    @Override
    public void loop() {
        double backLeftPower, frontLeftPower, frontRightPower, backRightPower;
        double x, y, turn, power, theta;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        power = Math.hypot(x, y);
        theta = Math.atan2(y, x);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        backLeftPower = power * sin / max + turn;
        backRightPower = power * cos / max - turn;
        frontLeftPower = power * cos / max + turn;
        frontRightPower = power * sin / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            frontLeftPower /= power + Math.abs(turn);
            frontRightPower /= power + Math.abs(turn);
            backLeftPower /= power + Math.abs(turn);
            backRightPower /= power + Math.abs(turn);
        }

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }
}
