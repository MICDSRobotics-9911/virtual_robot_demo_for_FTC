package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotConstants {
    public static final double TICKS_PER_REV = 1120;
    public static final double MAX_RPM = 160;
    public static double WHEEL_RADIUS = 1.88976; // in
    public static final double COUNTS_PER_INCH = (WHEEL_RADIUS * 2) * Math.PI / TICKS_PER_REV;
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 17.91; // in
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static double inchesToEncoderTicks(double inches) {
        return (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);
    }
}
