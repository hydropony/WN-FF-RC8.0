package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.PIDController;
import org.firstinspires.ftc.teamcode.misc.RobotMotor;

//@Config
public class Carousel extends RobotPart{
    private RobotMotor motor_left, motor_right;

    public static PIDController PID = new PIDController(1, 0, 0, 0, 3);
    public static double Power = -0.4; //-0.65
    public static double PowerDelta = 0.05;
    private double power = Power;
    public static double baseVoltage = 14;
    private VoltageSensor voltageSensor;
    public static double reverseTime = 80;

    @Override
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        motor_left = new RobotMotor(opMode.hardwareMap.get(DcMotorEx.class, HardwareConfig.CAROUSEL_MOTOR_LEFT));
        motor_right = new RobotMotor(opMode.hardwareMap.get(DcMotorEx.class, HardwareConfig.CAROUSEL_MOTOR_RIGHT));
        voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_left.setAccelerationLimiter(true, PowerDelta);
        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_right.setAccelerationLimiter(true, PowerDelta);
        opMode.telemetry.addData("Carousel initialized!", null);
    }

    @Override
    public void control(Gamepad gamepad) {

    }

    private boolean wasTurning = false;
    private double delta = 0.025;
    private ElapsedTime t = new ElapsedTime();
    public void telecontrol(Gamepad gamepad, boolean isRed) {
        if (isRed) {
            if (gamepad.a) {
                if (gamepad.right_trigger > 0.1)
                {
                    motor_left.setPower(Power - gamepad.right_trigger / 2, voltageSensor.getVoltage(), baseVoltage);
                    motor_right.setPower(Power - gamepad.right_trigger / 2, voltageSensor.getVoltage(), baseVoltage);
                }
                else {
                    motor_left.setPower(Power, voltageSensor.getVoltage(), baseVoltage);
                    motor_right.setPower(Power, voltageSensor.getVoltage(), baseVoltage);
                }
                wasTurning = true;
            } else {
                if (wasTurning) {
                    t = new ElapsedTime();
                    wasTurning = false;
                }
                if (t.milliseconds() < reverseTime) {
                    motor_left.setPowerClassic(0.3);
                    motor_right.setPowerClassic(0.3);
                }
                else {
                    motor_left.setPower(0);
                    motor_right.setPower(0);
                }
            }
        }
        else {
            if (gamepad.a) {
                if (gamepad.right_trigger > 0.1)
                {
                    motor_left.setPower(-(Power - gamepad.right_trigger / 2), voltageSensor.getVoltage(), baseVoltage);
                    motor_right.setPower(-(Power - gamepad.right_trigger / 2), voltageSensor.getVoltage(), baseVoltage);
                }
                else {
                    motor_left.setPower(-Power, voltageSensor.getVoltage(), baseVoltage);
                    motor_right.setPower(-Power, voltageSensor.getVoltage(), baseVoltage);
                }
                wasTurning = true;
            } else {
                if (wasTurning) {
                    t = new ElapsedTime();
                    wasTurning = false;
                }
                if (t.milliseconds() < reverseTime) {
                    motor_left.setPowerClassic(-0.3);
                    motor_right.setPowerClassic(-0.3);
                }
                else {
                    motor_left.setPower(0);
                    motor_right.setPower(0);
                }
            }
        }
    }

    public void autocontrol(double time, boolean isRed) {
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < time && opMode.opModeIsActive()) {
            if (isRed) {
                motor_left.setPower(-0.3, voltageSensor.getVoltage(), baseVoltage);
                motor_right.setPower(-0.3, voltageSensor.getVoltage(), baseVoltage);
            }
            else {
                motor_left.setPower(0.3, voltageSensor.getVoltage(), baseVoltage);
                motor_right.setPower(0.3, voltageSensor.getVoltage(), baseVoltage);
            }
        }
        motor_left.setPowerClassic(0);
        motor_right.setPowerClassic(0);
    }
}
