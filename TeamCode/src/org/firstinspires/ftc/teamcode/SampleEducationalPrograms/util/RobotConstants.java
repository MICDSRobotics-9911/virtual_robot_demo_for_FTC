package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotConstants {
    public static final double TICKS_PER_REV = 1120;
    public static final double MAX_RPM = 160;
    public static final double COUNTS_PER_INCH = inchesToEncoderTicks(1);
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 17.91; // in
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static double inchesToEncoderTicks(double inches) {
        return (inches * TICKS_PER_REV) / (2 * WHEEL_RADIUS * Math.PI * GEAR_RATIO);
    }
}
