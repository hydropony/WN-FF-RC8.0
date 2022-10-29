package org.firstinspires.ftc.teamcode.math;

public class Pose2d {
    private double x, y, heading;
    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {return x;}

    public double getY() {return y;}

    public double getHeading() {return heading;}

    public Vector2d toVector() {return new Vector2d(x, y);}
}
