package org.firstinspires.ftc.teamcode.math;

public class Operations {
    public static Vector2d minus(Vector2d vec1, Vector2d vec2) {
        return new Vector2d(vec1.getX() - vec2.getX(), vec1.getY() - vec2.getY());
    }

    public static Vector2d poseToVector(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    /**
     * Makes sure angle is within the range of -PI to PI radians
     * @param angle
     * @return
     */
    public static double angleWrap(double angle) {
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        return angle;
    }
}
