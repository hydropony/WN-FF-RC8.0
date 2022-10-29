package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import static org.firstinspires.ftc.teamcode.modules.Intake.intakePwr;
import static org.firstinspires.ftc.teamcode.modules.Lift.high_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.low_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.mid_pos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.dataflow.qual.Pure;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot;
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
public class BlueAutoprikol extends LinearOpMode { //mirrored
    public static double lookaheadDistance = 4;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private TeamElementDetectionPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime t;
        Robot21 R = new Robot21();

        Path ctohub = new Path(true);
        Path back = new Path(false);
        Path ctoparking = new Path(true);

        ctohub.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.blueautoprikol0);
//        back.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.blueautoprikol1);
        ctoparking.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.blueautoprikol2);

        ctohub.initializePath(20, 20, 10, 4);
//        back.initializePath(20, 20, 10, 4);
        ctoparking.initializePath(30, 30, 10, 10);
        List<Path> paths = new ArrayList<>();
        paths.add(ctohub);
//        paths.add(ctoparking);
        paths.add(ctoparking);

        R.init(this);

        PurePursuitTracker tracker = new PurePursuitTracker(R.drivetrainTank, R.localizer, this);
        tracker.setRobotTrack(17);
        tracker.setPaths(paths, lookaheadDistance);
        tracker.setPath(0);

        R.localizer.setStartPose(new Pose2d(-66, 12, Math.toRadians(-90)));

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

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
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

//        tracker.setLift(R.lift);

        waitForStart();

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
        R.drivetrainTank.setPowerSimple(0, 0);
//        R.lift.setPower(0);
        tracker.rotateTo(new Vector2d(-24, -12));
        tracker.forward(4, 0.6);
//        R.delay(500);
//        R.intake.autocontrol(0.8, 2000);
        t = new ElapsedTime();
        while (R.intake.isFull() && t.milliseconds() < 4000 && !isStopRequested()) {
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
        while (t.milliseconds() < 500 && !isStopRequested())
            R.intake.motor.setPowerClassic(intakePwr);
        R.intake.motor.setPowerClassic(0);

        tracker.forward(12, -1); //5

        t = new ElapsedTime();
        while (t.milliseconds() < 500)
            R.lift.update(low_pos);
        R.lift.setPower(0);


        tracker.rotateTo(0);

        tracker.forward(16, 1);

        tracker.rotateTo(-90);

        R.intake.motor.setPowerClassic(-1);

//        tracker.forward(90, 1);

//        tracker.setPath(1);
//
//        t = new ElapsedTime();
//        while (!isStopRequested() && !tracker.isDone()) {
//            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
//            R.drivetrainTank.setPower(drivePower);
//            R.lift.update(low_pos);
//            telemetry.update();
//        }
//        R.drivetrainTank.setPowerSimple(0,0);
//        t = new ElapsedTime();
//        while (t.milliseconds() < 1000 && !isStopRequested())
//            R.lift.update(0);
//        R.lift.setPower(0);
////        tracker.rotateTo(90);
//
//
//        R.intake.motor.setPowerClassic(-1);
        tracker.setPath(1);

        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(0);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0, 0);
    }
}