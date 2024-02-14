package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            drive.localizer.update();

            org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Pose2d otherPoseEstimate = drive.localizer.getPoseEstimate();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("otherX: ", otherPoseEstimate.getX());
            telemetry.addData("otherY: ", otherPoseEstimate.getY());
            telemetry.addData("otherHeading: ", otherPoseEstimate.getHeading());

            telemetry.addData("xError: ", poseEstimate.getX() - otherPoseEstimate.getX());
            telemetry.addData("yError: ", poseEstimate.getY() - otherPoseEstimate.getY());
            telemetry.addData("headingError: ", poseEstimate.getHeading() - otherPoseEstimate.getHeading());
            telemetry.update();
        }
    }
}
