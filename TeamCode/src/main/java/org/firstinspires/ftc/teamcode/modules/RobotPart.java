package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RobotPart {
    LinearOpMode opMode;
    public abstract void init(LinearOpMode opMode);

    public abstract void control(Gamepad gamepad);

    private double to_square(double n) {
        return (n * n) * (n / Math.abs(n));
    }

    public final void delay(long milliseconds) {
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 < milliseconds){}
    }
}
