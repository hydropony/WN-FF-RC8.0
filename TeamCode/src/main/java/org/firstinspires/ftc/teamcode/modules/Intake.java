package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.INTAKE_SENSOR;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.RobotMotor;

public class Intake extends RobotPart{
    public RobotMotor motor;
    private RevColorSensorV3 distanceSensor;
    public boolean isFull = true;
    public static double intakeMaxTime = 4000;
    public static double intakePwr = 0.9;
    public static double offset = 60;
    private View relativeLayout;
    private float gain = 2;
    final float[] hsvValues = new float[3];
    private double red, green, blue;
    public static double redLowShelf = 100;
    public static double greenLowShelf = 100;
    public static double blueLowShelf = 70;
    public static double redHighShelf = 500;
    public static double greenHighShelf = 800;
    public static double blueHighShelf = 300;
    @Override
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        int relativeLayoutId = opMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", opMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) opMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        motor = new RobotMotor(opMode.hardwareMap.get(DcMotorEx.class, HardwareConfig.INTAKE_MOTOR));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distanceSensor = opMode.hardwareMap.get(RevColorSensorV3.class, INTAKE_SENSOR);
        if (distanceSensor instanceof SwitchableLight) {
            ((SwitchableLight)distanceSensor).enableLight(true);
        }
        distanceSensor.setGain(gain);
        opMode.telemetry.addData("Intake initialized!", null);
        opMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    @Override
    public void control(Gamepad gamepad) {
        update();
        if (Math.abs(gamepad.left_stick_y) >= 0.1) {
            if (gamepad.left_bumper) {
                motor.setPower(-gamepad.left_stick_y * 0.3);
            }
            else {
                if (gamepad.left_stick_y < 0)
                    motor.setPower(-gamepad.left_stick_y * 0.5);
                else
                    motor.setPower(-gamepad.left_stick_y);
            }
        } else {
            motor.setPower(0);
        }

//        opMode.telemetry.addData("inp voltage", averageVoltage);
//        opMode.telemetry.addData("isFull", isFull);
    }

    public void autocontrol(double pwr, double maxTime) {
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < maxTime && !opMode.isStopRequested()) {
            motor.setPower(pwr);
        }
        motor.setPower(0);
    }

    public void autoIn() {
        update();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < intakeMaxTime && !opMode.isStopRequested() && !isFull) {
            motor.setPower(-intakePwr);
            update();
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200)
            motor.setPowerClassic(-intakePwr);
        motor.setPower(0);
    }

    public void autoIn(double power) {
        update();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < intakeMaxTime && !opMode.isStopRequested() && !isFull) {
            motor.setPower(-power);
            update();
        }
//        t = new ElapsedTime();
//        while (t.milliseconds() < 200)
//            motor.setPowerClassic(intakePwr);
        motor.setPower(0);
    }

    public void autoInAsync() {
        update();
        if (!isFull)
            motor.setPowerClassic(-intakePwr);
        else
            motor.setPowerClassic(0);
    }

    public void autoInAsync(double power) {
        update();
        if (!isFull)
            motor.setPowerClassic(-power);
        else
            motor.setPowerClassic(0);
    }

    public void autoOutAsync() {
        update();
        motor.setPowerClassic(intakePwr);
    }

    public void autoOutAsync(double pwr) {
        update();
        motor.setPowerClassic(pwr);
    }

    public void autoOut() {
        update();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < intakeMaxTime && !opMode.isStopRequested() && isFull) {
            motor.setPower(intakePwr);
            update();
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200)
            motor.setPowerClassic(intakePwr);
        motor.setPower(0);
    }

    public void update() {
        isFull = distanceSensor.getDistance(DistanceUnit.MM) < offset;
        red = distanceSensor.red();
        green = distanceSensor.green();
        blue = distanceSensor.blue();
        opMode.telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.MM));
        if (isFull) {
            opMode.telemetry.addData("<font color='green'>IS FULL</font>", null);
        }
        else {
            opMode.telemetry.addData("<font color='red'>IS FULL</font>", null);
        }
//        opMode.telemetry.update();
    }

    public boolean isFull() {
        update();
        return isFull;
    }

    public boolean isYellow() {
        update();
        return (red > redLowShelf && red < redHighShelf) && (green > greenLowShelf && green < greenHighShelf) && (blue > blueLowShelf && blue < blueHighShelf);
    }

    public boolean isFullAndYellow() {
        update();
        return ((red > redLowShelf && red < redHighShelf) && (green > greenLowShelf && green < greenHighShelf) && (blue > blueLowShelf && blue < blueHighShelf)) && isFull;
    }

    public void setPower(double power) {
        motor.setPowerClassic(power);
    }
}
