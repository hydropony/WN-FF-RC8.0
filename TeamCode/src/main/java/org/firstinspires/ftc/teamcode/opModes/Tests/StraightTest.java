package org.firstinspires.ftc.teamcode.opModes.Tests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.misc.DrivePower;
import org.firstinspires.ftc.teamcode.pathFollowing.Path;
import org.firstinspires.ftc.teamcode.pathFollowing.PurePursuitTracker;

import java.util.ArrayList;
import java.util.List;

//@Config
@Autonomous
public class StraightTest extends LinearOpMode {
    Path path = new Path(true);
    public static double k = 5;
    public static double lookaheadDistance = 5;
    public static double maxVel = 40;
    public static double maxAccel = 40;
    public static double startHeading = 0;
    ElapsedTime t;
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        Robot21 R = new Robot21();
        R.init(this);
        R.localizer.setStartPose(new Pose2d(0, 0, Math.toRadians(startHeading)));
        PurePursuitTracker tracker = new PurePursuitTracker(R, this);
        path.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.straighttest);
        path.initializePath(maxVel, maxAccel, k, 5);

        List<Path> paths = new ArrayList<>();
        paths.add(path);

        tracker.setRobotTrack(17);
        tracker.setPaths(paths, lookaheadDistance);
        tracker.setPath(0);

        waitForStart();

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower power = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPowerSimple(power.getLeftPower(), power.getRightPower());
        }
//        R.delay(1500);
//        tracker.forward(24, -1);
//        tracker.rotateTo(-160);
    }
}
