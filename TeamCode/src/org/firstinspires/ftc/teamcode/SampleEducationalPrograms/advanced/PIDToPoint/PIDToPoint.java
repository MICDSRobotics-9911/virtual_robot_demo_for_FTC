package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.PIDToPoint;

import static org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.AngleWrap.angleWrap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.DriveEncoderLocalizer;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Pose2d;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.PIDController;

@Autonomous(name="PIDToPoint", group="advanced")
public class PIDToPoint extends LinearOpMode {
    public enum State {
        GO_TO_POINT,
        DRIVE,
        FINAL_ADJUSTMENT,
        BRAKE,
        WAIT_AT_POINT,
        IDLE
    }

    public State state = State.IDLE;
    private DriveEncoderLocalizer localizer;
    private Drivetrain drive;
    private double xError;
    private double yError;
    private double turnError;

    /**
     * OpModes must override the abstract runOpMode() method.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d currentPose = localizer.getPoseEstimate();
            telemetry.addLine(currentPose.toString());
            Pose2d target = new Pose2d(30, 10, Math.toRadians(270));
            switch (state) {
                case GO_TO_POINT:
                    calculateErrors(target);
                    goToPoint();
                    if (atPoint()) {
                        state = State.FINAL_ADJUSTMENT;
                        System.out.println(currentPose);
                        System.out.println("TARGET");
                        System.out.println(target);
                    }
                    break;
                case FINAL_ADJUSTMENT:
                    calculateErrors(target);
                    finalAdjustment();
                    if (atPoint()) {
                        state = State.BRAKE;
                    }
                    break;
                case BRAKE:
                    drive.setPower(0);
                    state = State.WAIT_AT_POINT;
                    break;
                case WAIT_AT_POINT:
                    break;
                case DRIVE:
                    break;
                case IDLE:
                    break;
            }
            telemetry.addData("xError: ", xError);
            telemetry.addData("yError: ", yError);
            telemetry.addData("headingError: ", turnError);
            telemetry.addData("Current State: ", state);
            telemetry.update();
            localizer.update();
        }
    }

    public void initialize() throws InterruptedException {
        drive = new Drivetrain(hardwareMap, telemetry);
        localizer = new DriveEncoderLocalizer(drive.frontLeft, drive.frontRight, drive.backLeft, drive.backRight, drive.imu);
        state = State.GO_TO_POINT;
    }


    // Non PID implementation of a movement to a point
    public void goToPosition(double targetX, double targetY, double targetHeading, double movementSpeed) {
        Pose2d currentPose = localizer.getPoseEstimate();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double heading = currentPose.getHeading();

        double distance = Math.sqrt(Math.pow(targetX-currentX, 2) + Math.pow(targetY-currentY, 2));

        double absoluteAngleToTarget = Math.atan2(targetY-currentY, targetX-currentX);
        double relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (heading - Math.toRadians(90)));

        double relativeX = Math.cos(relativeAngleToPoint) * distance;
        double relativeY = Math.sin(relativeAngleToPoint) * distance;

        double movementXPower = relativeX / (Math.abs(relativeX) + Math.abs(relativeY));
        double movementYPower = relativeY / (Math.abs(relativeX) + Math.abs(relativeY));

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + targetHeading;

        double movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1);
        if (distance < 10) {
            movementTurn = 0;
        }
        applyKinematics(movementXPower * movementSpeed, movementYPower * movementSpeed, movementTurn * movementSpeed);
    }

    public static PIDController xPID = new PIDController(0.04,0.0,0.003);
    public static PIDController yPID = new PIDController(0.125,0.0,0.175);
    public static PIDController turnPID = new PIDController(0.25,0.0,0.01);

    public static double xThreshold = 0.75;
    public static double yThreshold = 0.75;
    public static double turnThreshold = 3.0;

    // PID implementation of it
    public void goToPoint() {
        Pose2d currentPose = localizer.getPoseEstimate();
        turnError = angleWrap(turnError);

        double forward = Math.abs(xError) > xThreshold / 2 ? xPID.update(xError) : 0;
        double strafe = Math.abs(yError) > yThreshold / 2 ? -yPID.update(yError) : 0;

        // Rotate the vectors in a 2d space around the heading
        Vector2D rotated = new Vector2D(forward, strafe);
        rotated = Vector2D.rotate(rotated, -currentPose.getHeading());
        double x_rotated = forward * Math.cos(-currentPose.getHeading()) - strafe * Math.sin(-currentPose.getHeading());
        double y_rotated = forward * Math.sin(-currentPose.getHeading()) + strafe * Math.cos(-currentPose.getHeading());

        double turn = Math.abs(turnError) > Math.toRadians(turnThreshold) / 2 ? -turnPID.update(turnError) : 0;


        applyKinematics(forward, strafe, turn);
    }

    public void applyKinematics(double x, double y, double turn) {
        double theta = Math.atan2(x, y);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = power * cos / max + turn;
        double rightFront = power * sin / max - turn;
        double leftBack = power * sin / max + turn;
        double rightBack = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftBack /= power + Math.abs(turn);
            rightBack /= power + Math.abs(turn);
        }
        telemetry.addData("FL: ", leftFront);
        telemetry.addData("FR: ", rightFront);
        telemetry.addData("BL: ", leftBack);
        telemetry.addData("BR: ", rightBack);
        drive.frontLeft.setPower(leftFront);
        drive.frontRight.setPower(rightFront);
        drive.backLeft.setPower(leftBack);
        drive.backRight.setPower(rightBack);
    }

    public static PIDController finalXPID = new PIDController(0.035, 0.0,0.0);
    public static PIDController finalYPID = new PIDController(0.1, 0.0,0.0);
    public static PIDController finalTurnPID = new PIDController(0.01, 0.0,0.0);

    public static double finalXThreshold = 0.5;
    public static double finalYThreshold = 0.5;
    public static double finalTurnThreshold = 2.5;

    public void finalAdjustment() {
        double forward = Math.abs(xError) > finalXThreshold / 2 ? finalXPID.update(xError) : 0;
        double strafe = Math.abs(yError) > finalYThreshold / 2 ? -finalYPID.update(yError) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(finalTurnThreshold) / 2 ? -finalTurnPID.update(turnError) : 0;

        applyKinematics(forward, strafe, turn);
    }

    public void calculateErrors(Pose2d target) {
        Pose2d currentPose = localizer.getPoseEstimate();
        xError = target.getX() - currentPose.getX();
        yError = target.getY() - currentPose.getY();
        turnError = target.getHeading() - currentPose.getHeading();
    }

    public boolean atPoint() {
        if (state == State.FINAL_ADJUSTMENT) {
            return Math.abs(xError) < finalXThreshold && Math.abs(yError) < finalYThreshold && Math.abs(turnError) < Math.toRadians(finalTurnThreshold);
        }
        return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < Math.toRadians(turnThreshold);
    }

}
