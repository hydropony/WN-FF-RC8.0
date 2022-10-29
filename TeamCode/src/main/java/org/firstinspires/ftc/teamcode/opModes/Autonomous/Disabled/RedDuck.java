package org.firstinspires.ftc.teamcode.opModes.Autonomous.Disabled;

import static org.firstinspires.ftc.teamcode.modules.Intake.intakePwr;
import static org.firstinspires.ftc.teamcode.modules.Lift.high_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.low_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.mid_pos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
//@Config
@Autonomous
public class RedDuck extends LinearOpMode {
    public static double lookaheadDistance = 4;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static double intakepwr = 0.3;

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

        Path tohub1 = new Path(true);
        Path back1 = new Path(false);
        Path intake1 = new Path(true);
        Path tohub2 = new Path(true);
        Path back2 = new Path(false);
        Path intake2 = new Path(true);
        Path back3 = new Path(false);
        Path tohub3 = new Path(true);
        Path parking = new Path(false);

//        tohub1.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck0);
//        back1.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck1);
//        intake1.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck2);
//        tohub2.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck3);
//        back2.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck4);
//        intake2.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck5);
//        back3.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck6);
//        tohub3.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck7);
//        parking.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redduck8);

        tohub1.initializePath(25, 25, 10, 4);
        back1.initializePath(25, 25, 10, 4);
        intake1.initializePath(20, 25, 3, 4);
        tohub2.initializePath(25, 25, 10, 4);
        back2.initializePath(25, 25, 10, 4);
        intake2.initializePath(20, 25, 3, 4);
        back3.initializePath(25, 25, 10, 4);
        tohub3.initializePath(25, 25, 10, 4);
        parking.initializePath(40, 40, 100, 4);

        List<Path> paths = new ArrayList<>();
        paths.add(tohub1);
        paths.add(back1);
        paths.add(intake1);
        paths.add(tohub2);
        paths.add(back2);
        paths.add(intake2);
        paths.add(back3);
        paths.add(tohub3);
        paths.add(parking);

        R.init(this);

        PurePursuitTracker tracker = new PurePursuitTracker(R.drivetrainTank, R.localizer, this);
        tracker.setRobotTrack(17);
        tracker.setPaths(paths, lookaheadDistance);
        tracker.setPath(0);

        R.localizer.setStartPose(new Pose2d(66, -36, Math.toRadians(90)));

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

        camera.setPipeline(pipeline = new TeamElementDetectionPipeline(telemetry, DEBUG, true, this));

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

        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("URA");
        dashboard.sendTelemetryPacket(packet);

        while (!isStarted()) {
            position = pipeline.position;
            telemetry.update();
        }
        camera.closeCameraDevice();

        waitForStart();

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            if (position == TeamElementDetectionPipeline.Position.RIGHT)
                R.lift.update(high_pos);
            if (position == TeamElementDetectionPipeline.Position.MIDDLE)
                R.lift.update(mid_pos);
            if (position == TeamElementDetectionPipeline.Position.LEFT)
                R.lift.update(low_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.lift.setPower(0);
        tracker.rotateTo(new Vector2d(24, -12));
        R.delay(200);
        t = new ElapsedTime();
        while (R.intake.isFull() && t.milliseconds() < 2000 && !isStopRequested()) {
            if (position == TeamElementDetectionPipeline.Position.RIGHT)
                R.lift.update(high_pos);
            if (position == TeamElementDetectionPipeline.Position.MIDDLE)
                R.lift.update(mid_pos);
            if (position == TeamElementDetectionPipeline.Position.LEFT)
                R.lift.update(low_pos);
            R.intake.autoOutAsync();
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200 && !isStopRequested())
            R.intake.motor.setPowerClassic(intakePwr);
        R.intake.motor.setPower(0);

        tracker.setPath(1);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone() && t.milliseconds() < 8000) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            if (position == TeamElementDetectionPipeline.Position.RIGHT)
                R.lift.update(high_pos);
            if (position == TeamElementDetectionPipeline.Position.MIDDLE)
                R.lift.update(mid_pos);
            if (position == TeamElementDetectionPipeline.Position.LEFT)
                R.lift.update(low_pos);
            telemetry.addData("2nd path executing...", null);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);

        tracker.setPath(2);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone() && !R.intake.isFull()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(0);
            R.intake.autoInAsync(intakepwr);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        t = new ElapsedTime();
        while (t.milliseconds() < 500 && !isStopRequested())
            R.intake.autoInAsync(intakepwr);
        R.intake.motor.setPower(0);
        R.intake.motor.setPowerClassic(0);

        tracker.setPath(3);

//        tracker.rotateTo(new Vector2d(21, -12));
        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(high_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        tracker.rotateTo(new Vector2d(21, -12));
        R.delay(200);
        t = new ElapsedTime();
        while (t.milliseconds() < 1000 && !isStopRequested()) {
            R.lift.update(high_pos);
            R.intake.autoOutAsync(intakepwr + 0.1);
        }
        R.intake.motor.setPowerClassic(0);

        tracker.setPath(4);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(high_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);

        tracker.rotateTo(new Vector2d(31, 16));

        tracker.setPath(5);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone() && !R.intake.isFull()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(0);
            R.intake.autoInAsync(intakepwr);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.intake.autoIn(intakepwr);

        tracker.setPath(6);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone() && !R.intake.isFull()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(0);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        tracker.rotateTo(new Vector2d(19, -12));

        tracker.setPath(7);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone() && t.milliseconds() < 2000) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(high_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        t = new ElapsedTime();
        while (t.milliseconds() < 1000 && !isStopRequested()) {
            R.lift.update(high_pos);
            R.intake.motor.setPowerClassic(0.5);
        }
        R.intake.motor.setPowerClassic(0);

        tracker.setPath(8);

        t = new ElapsedTime();
        while (!isStopRequested() && !tracker.isDone()) {
            DrivePower drivePower = tracker.update(t.milliseconds() / 1000.0);
            R.drivetrainTank.setPower(drivePower);
            R.lift.update(low_pos);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(-1,-1);
        R.lift.update(0);
    }
}