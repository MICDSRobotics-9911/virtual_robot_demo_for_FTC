package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotConstants {
    public static final int TICKS_PER_REV = 1120;
    public static final double MAX_RPM = 160;
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static final double COUNTS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);
    public static double TRACK_WIDTH = 17.91; // in

    // TODO: Tune these constants
    public static double MAX_LINEAR_SPEED = 79.2;
    public static double MAX_LINEAR_ACCELERATION = 20;
    public static double MAX_ROTATIONAL_SPEED = Math.PI / 3;

    public static double STRAFE_GAIN = 8;
    public static double FORWARD_GAIN = 6;
    public static double ROTATIONAL_GAIN = 0.65;
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static double inchesToEncoderTicks(double inches) {
        return (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);
    }

}
