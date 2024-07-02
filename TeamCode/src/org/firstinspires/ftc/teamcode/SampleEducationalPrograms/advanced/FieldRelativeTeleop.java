package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Relative")
public class FieldRelativeTeleop extends LinearOpMode {




    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private IMU imu;

    double speedModifier = 1; //@TODO If your drivers complain that the robot is too fast fix this :)
    double robotAngle = 0; //For Field Relative
    double angleZeroValue = 0;  //gets value from auton. if auton fails for some reason the
    //default angle is 0 degrees. Remember that your drivers should recalibrate if this happens by facing
    //the front of the robot away from them (i.e. the front of the robot is facing your opponents) and then press x.
    @Override
    public void runOpMode(){
        //@TODO Check hardware mappings
        leftFront = hardwareMap.get(DcMotor.class,"front_left_motor");
        leftBack = hardwareMap.get(DcMotor.class,"back_left_motor");
        rightFront = hardwareMap.get(DcMotor.class,"front_right_motor");
        rightBack = hardwareMap.get(DcMotor.class,"back_right_motor");
        imu = hardwareMap.get(IMU.class, "imu");
        initIMU();
        //since this is mecanum
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            drivetrain();
        }


    }

    public void initIMU() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.resetYaw();

    }
    public void drivetrain() {
        //x will calibrate field relative
        if (gamepad1.x) {
            //the calibration angle
            imu.resetYaw();

        }
        //getting the current angle of the robot and subtracting the calibration angle lets us know delta_theta
        //or the robot's angle relative to its calibration angle
        robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //angle of robot

        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
        double LeftStickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //get angle
        double rightX = -gamepad1.right_stick_x; //rotation
        rightX *= 0.8; //optionally reduce rotation value for better turning
        //linear the angle by the angle of the robot to make it field relative
        double leftFrontPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX;
        double rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX;
        double leftBackPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX;
        double rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX;


        speedModifier = .8 + (.8 * gamepad1.right_trigger) - (.4 * gamepad1.left_trigger);
        //Our drivers are video game players so this is why we added this ^
        speedModifier = 1;

        //setting powers correctly
        leftFront.setPower(leftFrontPower * speedModifier);
        rightFront.setPower(rightFrontPower * speedModifier);
        leftBack.setPower(leftBackPower * speedModifier);
        rightBack.setPower(rightBackPower * speedModifier);

    }



}
