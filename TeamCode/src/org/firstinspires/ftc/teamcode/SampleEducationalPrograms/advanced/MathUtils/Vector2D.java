package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils;

public class Vector2D {

    private double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void plus(Vector2D vector2D) {
        this.x += vector2D.getX();
        this.y += vector2D.getY();
    }

    public void minus(Vector2D vector2D) {
        this.x -= vector2D.getX();
        this.y -= vector2D.getY();
    }

    public static Vector2D rotate(Vector2D vector2D, double angle) {
        return new Vector2D(
                vector2D.getX() * Math.cos(angle) - vector2D.getY() * Math.sin(angle),
                vector2D.getY() * Math.cos(angle) + vector2D.getX() * Math.sin(angle)
        );
    }
}
