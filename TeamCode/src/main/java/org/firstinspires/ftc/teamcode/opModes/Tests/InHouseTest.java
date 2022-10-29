package org.firstinspires.ftc.teamcode.opModes.Tests;

import static org.firstinspires.ftc.teamcode.modules.Intake.intakePwr;
import static org.firstinspires.ftc.teamcode.modules.Lift.high_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.low_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.mid_pos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.common.reflection.qual.GetConstructor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.misc.DrivePower;
import org.firstinspires.ftc.teamcode.modules.Localizer;
import org.firstinspires.ftc.teamcode.modules.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.pathFollowing.Path;
import org.firstinspires.ftc.teamcode.pathFollowing.PurePursuitTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous
public class InHouseTest extends LinearOpMode {
    public static double power = 0.8;
    public static double maxtilt  = 0.5;
    public static double t1 = 1000;
    private double dist, begleft = 0, begright = 0, deltaleft, deltaright;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime t;
        Robot21 R = new Robot21();

        R.init(this);

        PurePursuitTracker tracker = new PurePursuitTracker(R, this);

        waitForStart();

        tracker.forwardlift(36, 0.8);
        tracker.rotateTo(-45);
        begleft = R.drivetrainTank.lf.getCurrentPosition();
        begright = R.drivetrainTank.rf.getCurrentPosition();
        while (!isStopRequested() && !R.intake.isFull()) {
            R.drivetrainTank.setPowerSimple(0.6, 0.6);
//            while (!isStopRequested() && Math.abs(R.imu.getTiltHeading()) > 3) {
//                R.drivetrainTank.setPowerSimple(-1, -1);
//            }
            R.intake.autoInAsync(0.8);
            R.lift.update(0);
            deltaleft = Localizer.encoderTicksToInches(Math.abs(R.drivetrainTank.lf.getCurrentPosition() - begleft));
            deltaright = Localizer.encoderTicksToInches(Math.abs(R.drivetrainTank.rf.getCurrentPosition() - begright));
            dist = (deltaleft + deltaright) / 2;
            telemetry.addData("dist", dist);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.intake.setPower(0);
        tracker.forwardlift(dist, -0.8);
        tracker.rotateTo(0);
        tracker.forwardlift(36, -0.8);
//        while (!isStopRequested() && R.imu.tiltCheck() <= 2) {
//            tracker.runInWarehouseAsync(power);
//            R.lift.update(low_pos);
//        }
//        t = new ElapsedTime();
//        while ((!isStopRequested() && R.imu.getTiltHeading() > 2) || t.milliseconds() < t1) {
//            tracker.runInWarehouseAsync(power);
//            R.lift.update(low_pos);
//        }
//        R.drivetrainTank.setPowerSimple(0,0);
    }
}
