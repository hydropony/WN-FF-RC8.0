package org.firstinspires.ftc.teamcode.misc;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class PIDController {
    public double kP;
    public double kI;
    public double kD;
    public double maxI;
    public int m;
    private ArrayList<Double> func = new ArrayList<>();
    private List<Double> timeList;
    private int n = -1;
    private double averageTimeDelta = 0;
    private double previousTime = 0;
    private double p = 0, i = 0, d = 0;
    private Telemetry telemetry;

    public PIDController(double kP, double kI, double kD, double maxI, int m) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
        this.m = m;
    }

    public PIDController(double kP, double kI, double kD, double maxI, int m, Telemetry tele) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
        this.m = m;
        telemetry = tele;
        reset();
    }

    public double update(double err, double currentTime) {
        n++;
        func.add(n, err);

        if (n == 0) {
            averageTimeDelta = currentTime;
        } else {
            averageTimeDelta = (averageTimeDelta + currentTime - previousTime) / 2;
        }

        p = 0;
        d = 0;

        p = err * kP;

        if (kI != 0) {
            if (n > 0) {
                i += ((func.get(n) + func.get(n - 1)) / 2) * (currentTime - previousTime);
            }
            if (i > maxI) {
                i = maxI;
            }
        }
        i *= kI;

        if (kD != 0) {
            for (int k = 0; k <= m; k++) {
                double ai = 0;
                for (int l = 1; l <= m; l++) {
                    if (l - k >= 0) {
                        ai += 1.0 / (getFactorial(k) * getFactorial(l - k));
                    }
                }
                ai *= Math.pow(-1, k);
                if (n - k >= 0) {
                    d += ai * func.get(n - k) / averageTimeDelta;
                }
            }
                if (n > 0)
                    d = (func.get(n) - func.get(n - 1)) / (currentTime - previousTime);
        }
        if (Double.isNaN(d) || Double.isInfinite(d)) {
            d = 0;
        } else {
            d *= kD;
        }
        if (telemetry != null) {
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("delta", averageTimeDelta);
        }

        previousTime = currentTime;
        return p + i + d;
    }

    public void reset() {
        n = -1;
        averageTimeDelta = 0;
        previousTime = 0;
        func.clear();
    }

    private int getFactorial(int f) {
        if (f <= 1) {
            return 1;
        }
        else {
            return f * getFactorial(f - 1);
        }
    }
}
