package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils;

public class Pose2d {

    private double x, y, heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getHeading() {
        return this.heading;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void plus(Pose2d pose2d) {
        this.x += pose2d.getX();
        this.y += pose2d.getY();
        this.heading = angleWrapper(this.heading + pose2d.getHeading());
    }

    public void minus(Pose2d pose2d) {
        this.x -= pose2d.getX();
        this.y -= pose2d.getY();
        this.heading = angleWrapper(this.heading - pose2d.getHeading());
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
}
