package org.firstinspires.ftc.teamcode.Utils.Poses;

public class Vector2d {
    private double x = 0;
    private double y = 0;

    public Vector2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector2d(Vector2d vector2D) {
        this.x = vector2D.getX();
        this.y = vector2D.getY();
    }

    public Vector2d rotateBy(double angle) {
        double cos = Math.cos(-angle);
        double sin = Math.sin(-angle);
        double x = this.x * cos - this.y * sin;
        double y = this.x * sin + this.y * cos;
        return new Vector2d(x,y);
    }

    public Vector2d rotateByDegrees(double degrees) {
        return rotateBy(Math.toRadians(degrees));
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
    public double angle() {
        return Math.atan2(y, x);
    }
}
