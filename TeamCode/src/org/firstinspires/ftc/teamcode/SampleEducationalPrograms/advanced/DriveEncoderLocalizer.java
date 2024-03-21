package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.MotorEncoder;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Pose2d;
import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Derivatives;



import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class DriveEncoderLocalizer {

    private final double LONGITUDINAL_MULTIPLIER = 1.0;
    private final double LATERAL_MULTIPLIER = 1.0;
    private final double WHEEL_RADIUS = 1.88976; // inches
    private final double GEARBOX_RATIO = 1;
    private final int TICKS_PER_REV = 1120; // from motor spec sheet
    private final IMU imu;
    private final ElapsedTime time;

    private final List<MotorEncoder> motorEncoders;
    private List<Double> angularVelocity = new ArrayList<>();

    private double lateralVelocity = 0;
    private double lastLateralVelocity = 0;
    private double lateralAcceleration = 0;

    private double longitudinalVelocity = 0;
    private double lastLongitudinalVelocity = 0;
    private double longitudinalAcceleration = 0;

    private double headingOffset = 0;
    private Pose2d currentPoseEstimate = new Pose2d(0, 0, 0);

    public DriveEncoderLocalizer(DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, IMU imu) {
        this.imu = imu;
        this.time = new ElapsedTime();

        List<DcMotorEx> motors = Arrays.asList(FL, FR, BL, BR);
        motorEncoders = motors.stream().map(MotorEncoder::new).collect(Collectors.toList());
    }

    public void setStartPose(double x, double y, double headingDegs) {
        this.headingOffset = Math.toRadians(headingDegs);
        currentPoseEstimate = new Pose2d(x, y, Math.toRadians(headingDegs));
    }

    public void setStartPose(Pose2d pose) {
        currentPoseEstimate = pose;
    }

    public Pose2d getPoseEstimate() {
        return currentPoseEstimate;
    }

    public void update() {
        double seconds = time.seconds();
        for (MotorEncoder encoder : motorEncoders) {
            encoder.update(seconds);
        }
        convertTicksToRads();

        getRobotCentricVelocity(seconds);
        updatePoseEstimate(seconds);

        updateLast();
    }

    private double ticksToRads(double ticks) {
        return (2 * Math.PI * ticks) / (TICKS_PER_REV * GEARBOX_RATIO);
    }

    private void convertTicksToRads() {
        angularVelocity = motorEncoders.stream().map(encoder -> ticksToRads(encoder.getVelocity())).collect(Collectors.toList());
    }

    private void getRobotCentricVelocity(double seconds) {
        longitudinalVelocity = angularVelocity.stream().mapToDouble(a -> a).sum() * WHEEL_RADIUS / 4.0;
        lateralVelocity = (angularVelocity.get(0) - angularVelocity.get(1) - angularVelocity.get(2) + angularVelocity.get(3)) * WHEEL_RADIUS / 4.0;
        longitudinalAcceleration = Derivatives.getDerivative(longitudinalVelocity, lastLongitudinalVelocity, seconds);
        lateralAcceleration = Derivatives.getDerivative(lateralVelocity, lastLateralVelocity, seconds);
    }

    private void updateLast() {
        for (MotorEncoder encoder : motorEncoders) {
            encoder.updateLast();
        }
        lastLateralVelocity = lateralVelocity;
        lastLongitudinalVelocity = longitudinalVelocity;
        time.reset();
    }

    private void updatePoseEstimate(double seconds) {
        double heading = angleWrapper(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset);

        double longitudinalDisplacement = longitudinalVelocity * seconds + 0.5 * longitudinalAcceleration * seconds * seconds;
        double lateralDisplacement = lateralVelocity * seconds + 0.5 * lateralAcceleration * seconds * seconds;

        longitudinalDisplacement *= LONGITUDINAL_MULTIPLIER;
        lateralDisplacement *= LATERAL_MULTIPLIER;

        double xDisplacement = (longitudinalDisplacement * Math.cos(heading)) + (lateralDisplacement * Math.sin(heading));
        double yDisplacement = (-lateralDisplacement * Math.cos(heading)) + (longitudinalDisplacement * Math.sin(heading));

        currentPoseEstimate.plus(new Pose2d(xDisplacement, yDisplacement, 0));
        currentPoseEstimate.setHeading(heading);
    }

    private double angleWrapper(double radians) {

        double normalizedAngle = radians % (2 * Math.PI);

        if (normalizedAngle > Math.PI) {
            normalizedAngle -= 2 * Math.PI;
        } else if (normalizedAngle <= -Math.PI){
            normalizedAngle += 2 * Math.PI;
        }
        return normalizedAngle;
    }

    public void reset() {
        currentPoseEstimate = new Pose2d(0, 0, 0);
    }
}
