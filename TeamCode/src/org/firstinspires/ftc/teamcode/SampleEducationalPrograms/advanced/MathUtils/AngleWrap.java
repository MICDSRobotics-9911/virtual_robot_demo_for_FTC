package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils;

public class AngleWrap {
    /**
     * Makes sure an angle is within the range -180 to 180 degrees
     * @param radians
     * @return
     */
    public static double angleWrap(double radians) {
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        return radians;
    }
}
