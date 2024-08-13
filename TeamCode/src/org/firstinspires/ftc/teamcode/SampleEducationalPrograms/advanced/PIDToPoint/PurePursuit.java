package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.PIDToPoint;

import static org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.AngleWrap.angleWrap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.StaticField;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.DriveEncoderLocalizer;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Pose2d;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Vector2D;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.PIDController;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous(name="PurePursuit", group="advanced")
public class PurePursuit extends LinearOpMode {
    private enum State {
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
    private ArrayList<Pose2d> path;
    private int currentPoint;
    private State lastState;
    private PurePursuitPath purePursuitPath;
    private double angleZeroValue;

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
            switch (state) {
                case GO_TO_POINT:
                    if (!purePursuitPath.isFinished()) {
                        goToPoint(purePursuitPath.update(currentPose));
                    } else {
                        currentPoint = path.size() - 1;
                        state = State.FINAL_ADJUSTMENT;
                        System.out.println("xError: " + xError);
                        System.out.println("yError: " + yError);
                        System.out.println("Heading Error: " + turnError);
                    }
                    break;
                case FINAL_ADJUSTMENT:
                    goToPoint(path.get(currentPoint));
                    if (atPoint()) {
                        state = State.BRAKE;
                        System.out.println("xError: " + xError);
                        System.out.println("yError: " + yError);
                        System.out.println("Heading Error: " + turnError);
                    }
                    break;
                case BRAKE:
                    drive.setPower(0);
                    state = State.WAIT_AT_POINT;
                    break;
                case WAIT_AT_POINT:
                    if (!atPointThresholds(1.5, 1.5, 5)) {
                        resetIntegrals();
                        state = State.GO_TO_POINT;
                    }
                    StaticField.autonHeading = currentPose.getHeading();
                    break;
                case DRIVE:
                    angleZeroValue = currentPose.getHeading();
                    applyKinematics(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                    break;
                case IDLE:
                    break;
            }

            if (gamepad1.a && state != State.DRIVE) {
                lastState = state;
                state = State.DRIVE;
            } else if (gamepad1.b) {
                state = lastState;
            }

            telemetry.addData("xError: ", xError);
            telemetry.addData("yError: ", yError);
            telemetry.addData("headingError: ", turnError);
            telemetry.addData("Current State: ", state);
            telemetry.addData("Current Point: ", (currentPoint < path.size()) ? "\n" + path.get(currentPoint) : "No more points");
            telemetry.update();
            localizer.update();
        }
    }

    public void initialize() throws InterruptedException {
        drive = new Drivetrain(hardwareMap, telemetry);
        localizer = new DriveEncoderLocalizer(drive.frontLeft, drive.frontRight, drive.backLeft, drive.backRight, drive.imu);
        state = State.GO_TO_POINT;
        path = new ArrayList<>();
        currentPoint = 0;
        angleZeroValue = 0;
        Pose2d turn = new Pose2d(0, 0, Math.toRadians(20));
        Pose2d point1 = new Pose2d(30, 10, Math.toRadians(-45));
        Pose2d point2 = new Pose2d(20, -35, Math.toRadians(-180));
        Pose2d point3 = new Pose2d(40, 45, Math.toRadians(130));
        path.add(turn);
        path.add(point1);
        path.add(point2);
        path.add(point3);
        purePursuitPath = new PurePursuitPath(path);
    }


    // These PIDS should be more aggressive than the final pids

    /*public static PIDController xPID = new PIDController(0.04,0.0,0.003);
    public static PIDController yPID = new PIDController(0.125,0.0,0.175);
    public static PIDController turnPID = new PIDController(0.25,0.0,0.01);*/
    public static PIDController xPID = new PIDController(1, 0, 0.003);
    public static PIDController yPID = new PIDController(1, 0, 0.175);
    public static PIDController turnPID = new PIDController(1, 0, 0.01);





    public static double xThreshold = 0.75;
    public static double yThreshold = 0.75;
    public static double turnThreshold = 3.0;

    // PID implementation of it
    public void goToPoint(Pose2d target) {
        calculateErrors(target);
        turnError = angleWrap(turnError);

        double forward = Math.abs(xError) > xThreshold / 2 ? xPID.update(xError, -maxPower, maxPower) : 0;
        double strafe = Math.abs(yError) > yThreshold / 2 ? yPID.update(yError, -maxPower, maxPower) : 0;

        double turn = Math.abs(turnError) > Math.toRadians(turnThreshold) / 2 ? -turnPID.update(turnError, -maxPower, maxPower) : 0;
        applyKinematics(forward, strafe, turn);
    }

    public void applyKinematics(double x, double y, double turn) {
        y *= -1;
        Pose2d currentPose = localizer.getPoseEstimate();
        double botHeading = currentPose.getHeading();
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        if (state == State.DRIVE) {
            rotX = -x;
            rotY = -y;
        }


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotX + rotY + turn) / denominator;
        double backLeftPower = (rotX - rotY + turn) / denominator;
        double frontRightPower = (rotX - rotY - turn) / denominator;
        double backRightPower = (rotX + rotY - turn) / denominator;
        drive.frontLeft.setPower(frontLeftPower);
        drive.frontRight.setPower(frontRightPower);
        drive.backLeft.setPower(backLeftPower);
        drive.backRight.setPower(backRightPower);
    }



    // These PIDS need to be less aggreessive but these constants are not aggressive enough(it moves too slowly)
    public static PIDController finalXPID = new PIDController(0.035, 0.0,0.0);
    public static PIDController finalYPID = new PIDController(0.1, 0.0,0.0);
    public static PIDController finalTurnPID = new PIDController(0.01, 0.0,0.0);

    public static double finalXThreshold = 0.4;
    public static double finalYThreshold = 0.4;
    public static double finalTurnThreshold = 2.0;

    double maxPower = 1;

    public void finalAdjustment() {

        turnError = angleWrap(turnError);
        double forward = Math.abs(xError) > finalXThreshold / 2 ? finalXPID.update(xError, -maxPower, maxPower) : 0;
        double strafe = Math.abs(yError) > finalYThreshold / 2 ? -finalYPID.update(yError, -maxPower, maxPower) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(finalTurnThreshold) / 2 ? -finalTurnPID.update(turnError, -maxPower, maxPower) : 0;

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

    public boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(turnError) < Math.toRadians(headingThresh);
    }

    public void resetIntegrals() {
        xPID.resetIntegral();
        yPID.resetIntegral();
        turnPID.resetIntegral();
        finalXPID.resetIntegral();
        finalYPID.resetIntegral();
        finalTurnPID.resetIntegral();
    }

}
