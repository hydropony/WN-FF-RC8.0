package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import static org.firstinspires.ftc.teamcode.modules.Intake.intakePwr;
import static org.firstinspires.ftc.teamcode.modules.Lift.high_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.initialkF;
import static org.firstinspires.ftc.teamcode.modules.Lift.low_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.mid_pos;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.misc.DrivePower;
import org.firstinspires.ftc.teamcode.modules.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.pathFollowing.Path;
import org.firstinspires.ftc.teamcode.pathFollowing.PurePursuitTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;

//@Config
@Autonomous
public class BlueCarousel2 extends LinearOpMode { //mirrored
    public static double lookaheadDistance = 4;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private TeamElementDetectionPipeline pipeline;
    private OpenCvCamera camera;
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime t;
        Robot21 R = new Robot21();

        Path toCarousel = new Path(true);
        Path toHub1 = new Path(false);
        Path toWarehouse = new Path(true);
        Path intake = new Path(true);
        Path back = new Path(false);
        Path toHub = new Path(false);
        Path toParking = new Path(true);
        toCarousel.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar0);
        toHub1.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar1);
        toWarehouse.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar2);
        intake.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar3);
        back.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar4);
        toHub.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar5);
        toParking.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.bluecar6);

        toCarousel.initializePath(40, 40, 100, 4);
        toHub1.initializePath(40, 40, 100, 8);
        toWarehouse.initializePath(40, 40, 100, 8);
        intake.initializePath(30, 30, 20, 8);
        back.initializePath(40, 40, 20, 8);
        toHub.initializePath(40, 40, 5, 8);
        toParking.initializePath(40, 40, 100, 8);
        List<Path> paths = new ArrayList<>();
        paths.add(toCarousel);
        paths.add(toHub1);
        paths.add(toWarehouse);
        paths.add(intake);
        paths.add(back);
        paths.add(toHub);
        paths.add(toParking);

        R.init(this);

        PurePursuitTracker tracker = new PurePursuitTracker(R, this);
        tracker.setRobotTrack(19);
        tracker.setPaths(paths, lookaheadDistance);
        tracker.setPath(0);

        R.localizer.setStartPose(new Pose2d(-66, -36, Math.toRadians(-90)));

        TeamElementDetectionPipeline.Position position = TeamElementDetectionPipeline.Position.RIGHT;

        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new TeamElementDetectionPipeline(telemetry, DEBUG, false, this));

        TeamElementDetectionPipeline.CAMERA_WIDTH = CAMERA_WIDTH;

        TeamElementDetectionPipeline.HORIZON = HORIZON;

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 30);
        dashboard.setTelemetryTransmissionInterval(500);

        while (!isStarted()) {
            position = pipeline.position;
//            telemetry.addData("rings", "" + height);
            telemetry.update();
        }

        waitForStart();

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone() && t.milliseconds() < 3000) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(low_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.carousel.autocontrol(5000, false);

        tracker.setPath(1);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            if (position == TeamElementDetectionPipeline.Position.RIGHT || position == TeamElementDetectionPipeline.Position.NONE)
                R.lift.update(high_pos);
            if (position == TeamElementDetectionPipeline.Position.MIDDLE)
                R.lift.update(mid_pos);
            if (position == TeamElementDetectionPipeline.Position.LEFT)
                R.lift.update(low_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);
        if (position == TeamElementDetectionPipeline.Position.RIGHT || position == TeamElementDetectionPipeline.Position.NONE)
            tracker.rotateTo(new Vector2d(-24, -12), high_pos);
        if (position == TeamElementDetectionPipeline.Position.MIDDLE)
            tracker.rotateTo(new Vector2d(-24, -12), mid_pos);
        if (position == TeamElementDetectionPipeline.Position.LEFT)
            tracker.rotateTo(new Vector2d(-24, -12), low_pos);
        tracker.forward(1, 0.8);
        t = new ElapsedTime();
        R.intake.update();
        while (R.intake.isFull() && t.milliseconds() < 2000 && !isStopRequested()) {
            if (position == TeamElementDetectionPipeline.Position.RIGHT || position == TeamElementDetectionPipeline.Position.NONE)
                R.lift.update(high_pos);
            if (position == TeamElementDetectionPipeline.Position.MIDDLE)
                R.lift.update(mid_pos);
            if (position == TeamElementDetectionPipeline.Position.LEFT)
                R.lift.update(low_pos);
            if (position == TeamElementDetectionPipeline.Position.RIGHT || position == TeamElementDetectionPipeline.Position.NONE)
                R.intake.autoOutAsync(0.7);
            else
                R.intake.autoOutAsync();
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 500)
            R.intake.motor.setPowerClassic(intakePwr);
        while (R.intake.isFull())
            R.intake.motor.setPowerClassic(1);
        R.intake.motor.setPowerClassic(0);

        tracker.forward(1, -0.8);

        tracker.rotateTo(5, low_pos - 200);

        tracker.setPath(2);
        t = new ElapsedTime();
        while (!tracker.isDone() && !isStopRequested()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(low_pos - 200);
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);

        tracker.rotateTo(30);

        tracker.setPath(3);
        t = new ElapsedTime();
        while (!tracker.isDone() && !R.intake.isFull() && !isStopRequested()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0, true);
            if (Math.abs(R.imu.getTiltHeading()) > 3)
                tracker.forward(5, -0.7, true);
            else
                R.drivetrainTank.setPower(drivePower);
            R.lift.update(0);
            R.intake.autoInAsync(0.8);
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);
        R.intake.setPower(0);

        tracker.setPath(4);
        t = new ElapsedTime();
        while (!tracker.isDone() && !isStopRequested()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(low_pos);
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);

        tracker.rotateTo(0);

        tracker.setPath(5);
        t = new ElapsedTime();
        while (!tracker.isDone() && !isStopRequested()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(low_pos - 200);
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);

//        t = new ElapsedTime();
//        while (t.milliseconds() < 500)
//            R.lift.update(high_pos);
//        R.lift.setPower(initialkF);
        tracker.rotateTo(new Vector2d(-24, -11), high_pos);
        R.lift.setPower(initialkF);

        tracker.forward(1, 0.8);
        while (R.intake.isFull() && t.milliseconds() < 3000 && !isStopRequested()) {
            R.lift.update(high_pos);
            R.intake.autoOutAsync(0.5);
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 800)
            R.intake.motor.setPowerClassic(1);
        while (R.intake.isFull())
            R.intake.motor.setPowerClassic(1);
        R.intake.motor.setPowerClassic(0);
        tracker.forward(1, -0.8);
        R.delay(300);

        tracker.rotateTo(5, low_pos - 200);

        tracker.setPath(6);
        t = new ElapsedTime();
        while (!tracker.isDone() && !R.intake.isFull() && !isStopRequested()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(low_pos - 200);
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);

        t = new ElapsedTime();
        while (!isStopRequested() && t.milliseconds() < 3000)
            R.lift.update(100);
        R.lift.setPower(0);
    }
}
