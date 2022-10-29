package org.firstinspires.ftc.teamcode.opModes.Teleop;

import android.net.wifi.WifiEnterpriseConfig;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.modules.DrivetrainTank;

//@Config
@TeleOp
public class TeleBlue extends LinearOpMode {
    Robot21 R = new Robot21();
    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this);
        R.attachGamepads(gamepad1, gamepad2);
        R.drivetrainTank.mode = DrivetrainTank.Mode.DRIVER_CONTROLLED;


        waitForStart();

        R.drivetrainTank.thread.start();
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            R.control(false, t.milliseconds());
            R.localizer.updateEstimated(t.milliseconds() / 1000.0);
            telemetry.update();
        }

        R.drivetrainTank.thread.interrupt();
    }
}
