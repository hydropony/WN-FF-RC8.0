package org.firstinspires.ftc.teamcode.opModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.modules.Lift;

//@Config
@Autonomous
public class LiftTest extends LinearOpMode {
    public static double target = 2000;
    Robot21 R = new Robot21();
    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this);

        waitForStart();

//        R.lift.update(Lift.high_pos);
        while (!isStopRequested())
            R.lift.update(target);
        R.lift.setPower(0);
    }
}
