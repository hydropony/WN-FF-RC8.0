package org.firstinspires.ftc.teamcode.math;

public class Vector2d {
    private double x, y;
    private double velocity;
    private double curvature;
    private double distance;

    public double getLengthSquared() {
        return x * x + y * y;
    }

    public double getX() {return x;}

    public double getY() {return y;}

    public void setX(double x) {this.x = x;}

    public void setY(double y) {this.y = y;}

    public Vector2d() {

    }

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    public Vector2d(Vector2d v) {
        this.x = v.x;
        this.y = v.y;
        this.velocity = v.velocity;
        this.curvature = v.curvature;
        this.distance = v.distance;
    }

    public static Vector2d add(Vector2d a, Vector2d b, Vector2d target) {
        if (target == null) {
            target = new Vector2d(a.x + b.x, a.y + b.y);
        } else {
            target.set(a.x + b.x, a.y + b.y);
        }
        return target;
    }

    public static Vector2d add(Vector2d a, Vector2d b) {
        return add(a, b, null);
    }

    public static Vector2d sub(Vector2d a, Vector2d b, Vector2d target) {
        if (target == null) {
            target = new Vector2d(a.x - b.x, a.y - b.y);
        } else {
            target.set(a.x - b.x, a.y - b.y);
        }
        return target;
    }

    public static Vector2d sub(Vector2d a, Vector2d b) {
        return sub(a, b, null);
    }

    public static Vector2d mult(Vector2d a, double n, Vector2d target) {
        if (target == null) {
            target = new Vector2d(a.x * n, a.y * n);
        } else {
            target.set(a.x * n, a.y * n);
        }
        return target;
    }

    public static Vector2d mult(Vector2d a, double n) {
        return mult(a, n, null);
    }

    public static Vector2d mult(Vector2d a, Vector2d b, Vector2d target) {
        if (target == null) {
            target = new Vector2d(a.x * b.x, a.y * b.y);
        } else {
            target.set(a.x * b.x, a.y * b.y);
        }
        return target;
    }

    public static Vector2d mult(Vector2d a, Vector2d b) {
        return mult(a, b, null);
    }

    public static Vector2d div(Vector2d a, float n, Vector2d target) {
        if (target == null) {
            target = new Vector2d(a.x / n, a.y / n);
        } else {
            target.set(a.x / n, a.y / n);
        }
        return target;
    }

    public static Vector2d div(Vector2d a, float n) {
        return div(a, n, null);
    }

    public static Vector2d div(Vector2d a, Vector2d b, Vector2d target) {
        if (target == null) {
            target = new Vector2d(a.x / b.x, a.y / b.y);
        } else {
            target.set(a.x / b.x, a.y / b.y);
        }
        return target;
    }

    static public double dist(Vector2d a, Vector2d b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    static public double dot(Vector2d a, Vector2d b) {
        return a.x * b.x + a.y * b.y;
    }

    static public double angleBetween(Vector2d a, Vector2d b) {
        double dot = dot(a, b);
        double amag = Math.sqrt(Math.pow(a.x, 2) + Math.pow(a.y, 2));
        double bmag = Math.sqrt(Math.pow(b.x, 2) + Math.pow(b.y, 2));
        return Math.acos(dot / (amag * bmag));
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public Vector2d duplicate() {
        return new Vector2d(x, y);
    }

    public double norm() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public Vector2d add(Vector2d v) {
        x += v.x;
        y += v.y;
        return this;
    }

    public void add(double x, double y, double z) {
        this.x += x;
        this.y += y;
    }

    public Vector2d sub(Vector2d a) {
        x -= a.x;
        y -= a.y;
        return this;
    }

    public void sub(double x, double y, double z) {
        this.x -= x;
        this.y -= y;
    }

    public void mult(double n) {
        x *= n;
        y *= n;
    }

    public Vector2d mult(Vector2d a) {
        x *= a.x;
        y *= a.y;
        return this;
    }

    public void div(double n) {
        x /= n;
        y /= n;
    }

    public Vector2d div(Vector2d a) {
        x /= a.x;
        y /= a.y;
        return this;
    }

    public Vector2d div(Vector2d a, Vector2d b) {
        return div(a, b, null);
    }

    public double dist(Vector2d v) {
        double dx = x - v.x;
        double dy = y - v.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    public double dot(Vector2d v) {
        return x * v.x + y * v.y;
    }

    public double dot(double x, double y, double z) {
        return this.x * x + this.y * y;
    }

    public void normalize() {
        double m = norm();
        if (m != 0 && m != 1) {
            div(m);
        }
    }

    public Vector2d normalize(Vector2d target) {
        if (target == null) {
            target = new Vector2d();
        }
        double m = norm();
        if (m > 0) {
            target.set(x / m, y / m);
        } else {
            target.set(x, y);
        }
        return target;
    }

    public void limit(float max) {
        if (norm() > max) {
            normalize();
            mult(max);
        }
    }

    @Override
    public String toString() {
        return "x: " + x + ", y: " + y;
    }

    @Override
    public boolean equals(Object object) {
        if (!(object instanceof Vector2d))
            return false;
        Vector2d vector = (Vector2d) object;
        return vector.x == x && vector.y == y && vector.curvature == curvature && vector.velocity == velocity && vector.distance == distance;
    }
}
