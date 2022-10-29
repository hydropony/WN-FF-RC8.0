package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.math.Operations.angleWrap;
import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.RIGHT_FRONT_MOTOR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Operations;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.misc.DrivePower;
import org.firstinspires.ftc.teamcode.misc.Encoder;
import org.firstinspires.ftc.teamcode.misc.PIDController;
import org.firstinspires.ftc.teamcode.misc.RobotMotor;
import org.opencv.core.Mat;

//@Config
public class DrivetrainTank extends RobotPart {
    double OFFSET_ANGLE = 34;
    double BASE_VOLTAGE = 12;

    public RobotMotor lf, rf;

    public Vector2d prevTarget = new Vector2d(0,0);

    private LinearOpMode opMode;

    public enum Mode {
        DRIVER_CONTROLLED,
        STOP,
        FOLLOW_VECTOR
    }
    public Mode mode;

    private Telemetry telemetry;

    private HardwareMap hardwareMap;

    public boolean isCurved = false;

    public boolean isAccelerationLimiterOn = false;

    private Encoder leftEncoder, rightEncoder;

    @Override
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        lf = new RobotMotor(hardwareMap.get(DcMotorEx.class, HardwareConfig.LEFT_FRONT_MOTOR));
        rf = new RobotMotor(hardwareMap.get(DcMotorEx.class, HardwareConfig.RIGHT_FRONT_MOTOR));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//        setAccelerationLimiter(true);

        opMode.telemetry.addData("DrivetrainTank initialized!", null);
    }

    public void setAccelerationLimiter(boolean isOn) {
        lf.setAccelerationLimiter(isOn, MaxPowerDelta);
        rf.setAccelerationLimiter(isOn, MaxPowerDelta);
        isAccelerationLimiterOn = isOn;
    }

    public void set_Power(double pwr_lf, double pwr_rf) {
        lf.setPower(pwr_lf);
        rf.setPower(pwr_rf);
    }

    public void set_Power(double pwr_lf, double pwr_rf, double k) {
        lf.setPower(pwr_lf * k);
        rf.setPower(pwr_rf * k);
    }

    public void set_Power(double pwr_lf, double pwr_rf, double base_voltage, double voltage) {
        lf.setPower(pwr_lf * base_voltage / voltage);
        rf.setPower(pwr_rf * base_voltage / voltage);
    }

    public void set_Power(double pwr_lf, double pwr_rf, double k, double base_voltage, double voltage) {
        lf.setPower(pwr_lf * k * base_voltage / voltage);
        rf.setPower(pwr_rf * k * base_voltage / voltage);
    }

    public void setPowerSimple(double leftPower, double rightPower) {
        lf.setPowerClassic(leftPower);
        rf.setPowerClassic(rightPower);
    }

    public void setPower(DrivePower drivePower) {
        setPowerSimple(drivePower.getLeftPower(), drivePower.getRightPower());
    }

    //-----------------------------------Thread----------------------------------------------------

    public Thread thread = new Thread() {
        @Override
        public void run(){
            while (opMode.opModeIsActive()) {
                switch (mode) {
                    case DRIVER_CONTROLLED:
                        if (!isAccelerationLimiterOn) {
                            setAccelerationLimiter(false);
                        }
                            control(opMode.gamepad1);
                        break;
                    case FOLLOW_VECTOR:
                        setAccelerationLimiter(false);
//                        if (isTargetUpdated) {
//                            update();
//                        }
                        break;
                    case STOP:
                        if (isTargetUpdated) {
                            update();
                        }
//                        reset();
//                        set_Power(0, 0, 0, 0);
                }
//                telemetry.update();
            }
            set_Power(0, 0, 0, 0);
        }
    };

    //----------------------------------TeleOp-----------------------------------------------------

    @Override
    public void control(Gamepad gamepad1) {
        double y = -Math.pow(gamepad1.left_stick_y, 5); //-gamepad1.left_stick_y
        double x = gamepad1.right_trigger - gamepad1.left_trigger;
        x = Math.signum(x) * Math.pow(x, 2);
        if (gamepad1.right_bumper)
            x += 0.4;
        if (gamepad1.left_bumper)
            x -= 0.4;

        if (gamepad1.dpad_up)
            y = 0.4;
        if (gamepad1.dpad_down)
            y = -0.4;

        lf.setPowerClassic(y - x);
        rf.setPowerClassic(y + x);
    }

    //-----------------------------------For_Autonomous--------------------------------------------

    private double leftTargetSpeed = 0;
    private double rightTargetSpeed = 0;
    private double currentLeftSpeed = 0;
    private double currentRightSpeed = 0;
    private double previousLeftSpeed = 0;
    private double previousRightSpeed = 0;
    private double pwrL = 0;
    private double pwrR = 0;
    private boolean isTargetUpdated = false;
    private double currentTime = 0;
    private double previousTime = 0;


    //TODO: Настроить ПИД
    public static PIDController MOTOR_PID_LEFT = new PIDController(0.00003, 0, 0, 0, 3);
    public static PIDController MOTOR_PID_RIGHT = new PIDController(0.00003, 0, 0, 0, 3);
    public static PIDController HEADING_PID = new PIDController(2, 1, 0, 1, 3);
    double kP = 4;
    public void goToHeading(double targetHeading, double currentHeading, double currentTime) {
        this.currentTime = currentTime;
        double dif = targetHeading - currentHeading;
        double angle = 0;
        if (Math.abs(dif - 2 * Math.PI * Math.signum(dif)) < Math.abs(dif))
            angle = dif - 2 * Math.PI * Math.signum(dif);
        else
            angle = dif;
        double speed = kP * angle;
//        setTargetSpeed(speed, -speed, currentTime);
//        set_Power(-speed, speed);
        setPowerSimple(-speed, speed);
    }

    public static double MaxPowerDelta = 0.022;

    public void setTargetSpeed(double leftSpeed, double rightSpeed, double currentTime) {
        this.currentTime = currentTime;
        leftTargetSpeed = leftSpeed;
        rightTargetSpeed = rightSpeed;
        isTargetUpdated = true;
    }

    double maxPwr;
    double pwrRout = 0;
    double pwrLout = 0;

    public void update() {
        currentLeftSpeed = leftEncoder.getRawVelocity();//(lf.getCurrentPosition() - previousLeftSpeed) / (currentTime - previousTime);
        currentRightSpeed = rightEncoder.getRawVelocity();//(rf.getCurrentPosition() - previousRightSpeed) / (currentTime - previousTime);

        pwrL += MOTOR_PID_LEFT.update(leftTargetSpeed - currentLeftSpeed, currentTime);
        pwrR += MOTOR_PID_RIGHT.update(rightTargetSpeed - currentRightSpeed, currentTime);

        maxPwr = Math.max(Math.abs(pwrL), Math.abs(pwrR));

        if (maxPwr > 1) {
            pwrLout = pwrL / maxPwr;
            pwrRout = pwrR / maxPwr;
        }
        else {
            pwrLout = pwrL;
            pwrRout = pwrR;
        }

        set_Power(pwrLout, pwrRout);

        previousLeftSpeed = currentLeftSpeed;
        previousRightSpeed = currentRightSpeed;
        previousTime = currentTime;

        isTargetUpdated = false;

        telemetry.addData("pwrL", pwrL);
        telemetry.addData("pwrR", pwrR);
        telemetry.addData("pwrLout", pwrLout);
        telemetry.addData("pwrRout", pwrRout);
        telemetry.addData("Lspeed", currentLeftSpeed);
        telemetry.addData("Rspeed", currentRightSpeed);
    }

    public void test() {
        lf.setPower(1);
        delay(1000);
        lf.setPower(0);
        rf.setPower(1);
        delay(1000);
        rf.setPower(0);
    }

    public void reset() {
        MOTOR_PID_LEFT.reset();
        MOTOR_PID_RIGHT.reset();
        leftTargetSpeed = 0;
        rightTargetSpeed = 0;
        currentLeftSpeed = 0;
        currentRightSpeed = 0;
        currentTime = 0;
        previousTime = 0;
        pwrL = 0;
        pwrR = 0;
    }
}
