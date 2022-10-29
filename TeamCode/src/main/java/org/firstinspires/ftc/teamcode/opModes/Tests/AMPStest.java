package org.firstinspires.ftc.teamcode.opModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.RobotMotor;

@TeleOp
@Disabled
public class AMPStest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotMotor motor = new RobotMotor(hardwareMap.get(DcMotorEx.class, "motor"));

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        ElapsedTime t = new ElapsedTime();
        double t1 = 0;

        while (opModeIsActive()) {
            double t0 = t.milliseconds();
            double pwr = gamepad1.left_stick_y;
            motor.setPower(pwr);
            telemetry.addData("pwr", pwr);
            telemetry.addData("delta", t0 - t1);
            telemetry.addData("Amps:", motor.getAmps());
            telemetry.update();
            t1 = t0;
        }
    }
}
