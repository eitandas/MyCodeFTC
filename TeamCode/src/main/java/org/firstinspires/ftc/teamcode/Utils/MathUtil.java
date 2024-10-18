package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Utils.Poses.Vector2d;

public class MathUtil {
    public static Vector2d rotateVec(Vector2d vec, double angle) {
        double cos = Math.cos(-angle);
        double sin = Math.sin(-angle);

        double x = vec.getX() * cos - vec.getY() * sin;
        double y = vec.getX() * sin + vec.getY() * cos;
        return new Vector2d(x,y);
    }

    public static double aplayDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return input;
    }
    public static double convertTicksToDistance(double CPR,double diameter,double ticks) {
        double circumference = Math.PI * diameter;
        double revolutions = ticks/CPR;
        double distance = circumference * revolutions;
        return distance;
    }
    public static double convertDistanceToTicks(double CPR,double diameter,double distance) {
        double circumference = Math.PI * diameter;
        double revolutions = distance/circumference;
        double ticks = revolutions*CPR;
        return ticks;
    }
    public static double convertTicksToDegries(double CPR,double ticks){
        double revolutions = ticks/CPR;
        double angle = revolutions * 360;
        double angleNormalized = angle % 360;
        return angleNormalized;
    }
    public static double convertDegriesToTicks(double CPR, double angle){
        double revolutions = angle/360;
        double ticks = revolutions *CPR;
        return ticks;

    }
}

