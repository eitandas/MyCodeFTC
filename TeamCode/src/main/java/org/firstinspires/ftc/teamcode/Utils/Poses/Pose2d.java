package org.firstinspires.ftc.teamcode.Utils.Poses;

public class Pose2d {
    private double x = 0;
    private double y = 0;
    private double angle = 0;

    public Pose2d(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
    public Pose2d(Vector2d vec, double angle){
        new Pose2d(vec.getX(), vec.getY(), angle);
    }

    public Pose2d(Pose2d pose){
        new Pose2d(pose.getX(), pose.getY(), pose.getAngle());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

}
