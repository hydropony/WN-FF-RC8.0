package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.LINE_SENSOR_BACK;
//import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.LINE_SENSOR_MID;
import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.RIGHT_FRONT_MOTOR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.Encoder;
import org.firstinspires.ftc.teamcode.pathFollowing.TankLocalizer;

import java.util.Arrays;
import java.util.List;

@Config
public class Localizer extends TankLocalizer {
    public static double TICKS_PER_REV = 560; //751.8;
    private static double WHEEL_RADIUS = 1.65; // in
    private static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed


    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder leftEncoder, rightEncoder;
    private RobotIMU imu = new RobotIMU();

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
//    private AnalogInput lineSensorBack, lineSensorMid;

    public static double backvalue = 0.7;
    public static double midvalue = 0.35;

    public Localizer(LinearOpMode opMode) {
        super(opMode);
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        init();
    }

    public void init() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR));

        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);

//        lineSensorMid = hardwareMap.get(AnalogInput.class, LINE_SENSOR_MID);
//        lineSensorBack = hardwareMap.get(AnalogInput.class, LINE_SENSOR_BACK);

//        lineSensor.setMode(DigitalChannel.Mode.INPUT);

        imu.init(opMode);

        telemetry.addData("Odometry is Ready!", null);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double getHeadingInternal() {
        return imu.getImu1Heading();
    }

    private double getHeadingVelocity() {
        return imu.getVelocity();
    }

    @Override
    public List<Double> getOdoPositions() {
        return Arrays.asList(encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                getHeadingInternal());
    }

    @Override
    public List<Double> getOdoVelocities() {
        return Arrays.asList(encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                getHeadingVelocity());
    }

    public static double inchesToTicks(double inches) {
        return inches / WHEEL_RADIUS / 2 / Math.PI / GEAR_RATIO * TICKS_PER_REV;
    }

//    @Override
//    public boolean isLineBack() {
//        return lineSensorBack.getVoltage() < backvalue;
//    }
//
//    @Override
//    public boolean isLineMiddle() {
//        return lineSensorMid.getVoltage() < midvalue;
//    }
}
