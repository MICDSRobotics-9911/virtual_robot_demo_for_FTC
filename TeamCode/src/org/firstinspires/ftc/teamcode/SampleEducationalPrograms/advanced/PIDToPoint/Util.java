package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.PIDToPoint;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Util {
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + Math.pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 + Math.pow(m1, 2) + x1);

        double quadraticC = (Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            // put back the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint1.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint1.x ? linePoint1.x : linePoint2.x;

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            //double xRoot2 = (-quadraticB)
        } finally {

        }
        return allPoints;
    }
}
