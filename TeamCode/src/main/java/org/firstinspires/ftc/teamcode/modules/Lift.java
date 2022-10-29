package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.misc.PIDController;
import org.firstinspires.ftc.teamcode.misc.RobotMotor;

import java.util.ArrayList;
import java.util.List;

@Config
public class Lift extends RobotPart {
    public static double initialkF = 0.29;
    public static double kSlow = 0.4;
    public static double kSlowDown = 0.4;
    public RobotMotor motor;
    public double currentPos = 0;
    public static double offset = 20;
    public static double high_pos = 1900;
    public static double mid_pos = 1400;
    public static double low_pos = 850
            ;
    public double error;
    private double averageCycletime;
    private List<Double> timeList = new ArrayList<>();
    private double prevtime = 0;
    private double output = 0, prevoutput = 0;
    public static double maxAccel = 10;
    public static double highLimit = 1900;
    public static double lowLimit = 0;
    public enum LIFT_STATE {
        DOWN,
        UP
    }
    public LIFT_STATE currentState = LIFT_STATE.DOWN;
    public static PIDController pidController = new PIDController(0.001, 0, 0, 0, 0);
    private double power = 0;
    private VoltageSensor batteryVoltageSensor;
    @Override
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = new RobotMotor(opMode.hardwareMap.get(DcMotorEx.class, HardwareConfig.LIFT_MOTOR));
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
    }

    private double rateLimiter(double input, double maxRate) {
        double maxChange = averageCycletime * maxRate;
        output += Range.clip(input - prevoutput, -maxChange, maxChange);
        prevoutput = output;
        return output;
    }

    @Override //deprecated
    public void control(Gamepad gamepad) {
        double kF = initialkF * 12 / batteryVoltageSensor.getVoltage();
        if (Math.abs(gamepad.right_stick_y) >= 0.1) {
            if (!gamepad.right_bumper) {
                if (motor.getCurrentPosition() < 150) {
                    if (gamepad.right_stick_y < 0) {
                        power = Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 50);
                        motor.setPowerClassic(power + kF);
                    } else {
                        motor.setPowerClassic(kF);
                    }
                } else {
                    power = Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 50);
                    if (motor.getCurrentPosition() < mid_pos && gamepad.right_stick_y > 0)
                        power *= kSlow;
                    motor.setPowerClassic(power + kF);
                }
            }
            else {
                power = Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 50);
                motor.setPowerClassic(power + kF);
            }
            currentPos = motor.getCurrentPosition();

        } else {
//            if (gamepad.left_bumper)
//                motor.setPowerClassic(kF);
//            else
            update(currentPos);
        }

//        opMode.telemetry.addData("LIFT POSITION ", motor.getCurrentPosition());
    }

    public void control2(Gamepad gamepad, double currtime) {
        if (gamepad.dpad_up) {
            motor.setPowerClassic(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentPos = 0;
        }
//        double kF = initialkF * 12 / batteryVoltageSensor.getVoltage();
        double kF = initialkF;
        timeList.add((currtime - prevtime));
        prevtime = currtime;
        double sum = 0;
        for (int i = 0; i < timeList.size(); i++) {
            sum += timeList.get(i);
        }
        double k = 1;
        averageCycletime = sum / timeList.size();
        if (gamepad.left_bumper)
            k = kSlow;
        else
            k = 1;
        if (Math.abs(gamepad.right_stick_y) >= 0.1) {
            if (!gamepad.right_bumper) {
                if (motor.getCurrentPosition() < lowLimit) {
                    if (gamepad.right_stick_y < 0) {
                        power = k * Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 2) + kF;
                    } else {
                        power = kF;
                    }
//                    opMode.telemetry.addData("over low", null);
                }
                if (motor.getCurrentPosition() > highLimit) {
                    if (gamepad.right_stick_y > 0)
                        power = k * Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 2) + kF;
                    else
                        power = kF;
//                    opMode.telemetry.addData("over high", null);
                }
                if (motor.getCurrentPosition() <= highLimit && motor.getCurrentPosition() >= lowLimit) {
                    power = k * Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 2);
                    if (motor.getCurrentPosition() < mid_pos && gamepad.right_stick_y > 0)
                        power *= kSlowDown;
                    power += kF;
//                    opMode.telemetry.addData("ok", null);
                }
            }
            else {
                power = k * Math.signum(-gamepad.right_stick_y) * Math.pow(gamepad.right_stick_y, 2) + kF;
//                opMode.telemetry.addData("bypass", null);
            }
            currentPos = motor.getCurrentPosition();
            motor.setPowerClassic(rateLimiter(power, maxAccel));
        } else {
            update(currentPos);
//            opMode.telemetry.addData("stay", null);
        }
//        opMode.telemetry.addData("currentpos", currentPos);
//        opMode.telemetry.update();
    }

    public void setToHigh() {
        motor.setPower(1);
        delay(500);
        while (motor.getVelocity() > 40) {
            motor.setPower(1);
        }
        motor.setPower(0);
    }

    public void setToHighAsync() { // need to setpower and delay first!
        if (motor.getVelocity() > 40) {
            motor.setPower(1);
        }
        else
            motor.setPower(0);
    }

    public void setToDown() {
        motor.setPower(-1);
        delay(800);
        motor.setPower(0);
    }



    public void setToDownFull() {
        while (Math.abs(motor.getCurrentPosition()) > 20) {
            motor.setPower(-1);
        }
        motor.setPower(0);
    }

    public void setToDownFullAsync() {
        if (Math.abs(motor.getCurrentPosition()) > 20) {
            motor.setPower(-1);
        }
        else
            motor.setPower(0);
    }

    public void update(double target) {
        double kF = initialkF * 12 / batteryVoltageSensor.getVoltage();
        error = target - motor.getCurrentPosition();
        if (Math.abs(error) > offset)
            motor.setPowerClassic(pidController.update(target - motor.getCurrentPosition() + kF, 1));
        else
            motor.setPowerClassic(kF);
    }

    public void setPower(double power) {
        motor.setPowerClassic(power);
    }

//    public Thread thread = new Thread() {
//        @Override
//        public void run(){
//            while (opMode.opModeIsActive()) {
//                switch (currentState) {
//                    case UP:
//                        error = high_pos - motor.getCurrentPosition();
//                        motor.setPower(error);
//                        break;
//                    case DOWN:
//                        error = 0 - motor.getCurrentPosition();
//                        motor.setPower(error);
//                        break;
//                }
//            }
//        }
//    };
}
