package org.firstinspires.ftc.teamcode.opModes.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.modules.DrivetrainTank;
import org.firstinspires.ftc.teamcode.modules.Localizer;

@TeleOp
public class WheelRadiusTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot21 R = new Robot21();
        R.init(this);
        telemetry.addData("Drive 48in(2 squares) forward", null);
        telemetry.update();

        double effectiveRadiusLeft = 0;
        double effectiveRadiusRight = 0;

        R.drivetrainTank.mode = DrivetrainTank.Mode.DRIVER_CONTROLLED;

        waitForStart();
        R.drivetrainTank.thread.start();

        while (!isStopRequested()) {
            effectiveRadiusLeft = 48.0 / 2 / Math.PI / R.drivetrainTank.lf.getCurrentPosition() * Localizer.TICKS_PER_REV;
            effectiveRadiusRight = 48.0 / 2 / Math.PI / R.drivetrainTank.rf.getCurrentPosition() * Localizer.TICKS_PER_REV;
            telemetry.addData("Effective wheel radius left: ", effectiveRadiusLeft);
            telemetry.addData("Effective wheel radius right: ", effectiveRadiusRight);
            telemetry.update();
        }
    }
}
