package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class RobotIMU extends RobotPart {
    BNO055IMU imu1, imu2;
    private boolean wasTilted = false;
    public int tiltNum = 0;
    public static double tiltVal = 0.5;
    public static double zeroTilt = 1.52;

    @Override
    public void init(LinearOpMode opMode) {
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json";
        imu1 = opMode.hardwareMap.get(BNO055IMU.class, HardwareConfig.IMU_1);
        imu1.initialize(parameters1);
        while (!imu1.isGyroCalibrated() && !opMode.isStopRequested()) {
        }

//        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
//        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
//        imu2 = opMode.hardwareMap.get(BNO055IMU.class, HardwareConfig.IMU_2);
//        imu2.initialize(parameters2);
//        while (!imu2.isGyroCalibrated()){}
        opMode.telemetry.addData("IMU is calibrated", null);
    }

    @Override
    public void control(Gamepad gamepad) {

    }

    private double getAverageHeading() {
        float angle1 = imu1.getAngularOrientation().firstAngle;
        float angle2 = imu2.getAngularOrientation().firstAngle;

        if (angle1 <= -90 && angle1 >= -180 && angle2 >= 90 && angle2 <= 180) {
            angle1 += 360;
        }
        if (angle2 <= -90 && angle2 >= -180 && angle1 >= 90 && angle1 <= 180) {
            angle2 += 360;
        }
        double average = (angle1 + angle2) / 2;
        if (average > 180) {
            average -= 360;
        }
        if (average < -180) {
            average += 360;
        }
        return average;
    }

    public double getImu1Heading() {
        return imu1.getAngularOrientation().firstAngle;
    }

    public double getTiltHeading() {
        return Math.toDegrees(Math.abs(imu1.getAngularOrientation().secondAngle) - zeroTilt);
    }

    public double tiltVelocity() {
        return imu1.getAngularVelocity().toAngleUnit(AngleUnit.RADIANS).yRotationRate;
    }

    public double getVelocity() {
        return imu1.getAngularVelocity().toAngleUnit(AngleUnit.RADIANS).zRotationRate;
    }

    public double tiltCheck() {
        if (tiltVelocity() > tiltVal && !wasTilted) {
            wasTilted = true;
            tiltNum++;
        }
        if (tiltVelocity() < tiltVal)
            wasTilted = false;
        return tiltNum;
    }
}
