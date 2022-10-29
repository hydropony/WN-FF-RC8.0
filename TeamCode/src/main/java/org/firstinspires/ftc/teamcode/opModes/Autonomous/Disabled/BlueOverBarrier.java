package org.firstinspires.ftc.teamcode.opModes.Autonomous.Disabled;

import static org.firstinspires.ftc.teamcode.modules.Intake.intakePwr;
import static org.firstinspires.ftc.teamcode.modules.Lift.high_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.low_pos;
import static org.firstinspires.ftc.teamcode.modules.Lift.mid_pos;

import android.os.Build;

import androidx.annotation.RequiresApi;

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

@Disabled
//@Config
@Autonomous
public class BlueOverBarrier extends LinearOpMode { //mirrored
    public static double lookaheadDistance = 4;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private TeamElementDetectionPipeline pipeline;
    private OpenCvCamera camera;

    private double dist, begleft = 0, begright = 0, deltaleft, deltaright;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime t;
        Robot21 R = new Robot21();

        Path tohub1 = new Path(true);
//        Path back1 = new Path(false);

        tohub1.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.blueob);
//        back1.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.redob1);

        tohub1.initializePath(25, 25, 10, 4);
//        back1.initializePath(25, 25, 10, 4);

        List<Path> paths = new ArrayList<>();
        paths.add(tohub1);
//        paths.add(back1);

        R.init(this);

        PurePursuitTracker tracker = new PurePursuitTracker(R, this);
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
        while (!isStopRequested() && !tracker.isDone()) { //tohub1
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
        tracker.rotateTo(new Vector2d(-24, -14));
        tracker.forward(1, 0.7);
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
        while (t.milliseconds() < 200)
            R.intake.motor.setPowerClassic(intakePwr);
        R.intake.motor.setPowerClassic(0);

        tracker.forward(6, -0.8);
        t = new ElapsedTime();
        while (t.milliseconds() < 500)
            R.lift.update(low_pos);
        tracker.rotateTo(0);
        tracker.forwardlift(36, 0.8);
        tracker.rotateTo(30);
        begleft = R.drivetrainTank.lf.getCurrentPosition();
        begright = R.drivetrainTank.rf.getCurrentPosition();
        while (t.milliseconds() < 1000) //not working need elapsedtime init
            R.lift.update(0);
        R.intake.update();
        while (!isStopRequested() && !R.intake.isFull) {
            double velocity = Localizer.encoderTicksToInches((R.drivetrainTank.lf.getVelocity() + R.drivetrainTank.rf.getVelocity() / 2));
            R.drivetrainTank.setPowerSimple(0.4, 0.4);
            while (!isStopRequested() && Math.abs(R.imu.getTiltHeading()) > 3 && !R.intake.isFull()) {
                R.drivetrainTank.setPowerSimple(-0.8, -0.8);
                R.intake.update();
            }
//            if (Math.abs(velocity) < 5) {
//                R.drivetrainTank.setPowerSimple(-1, -1);
//                R.delay(500);
//                R.drivetrainTank.setPowerSimple(0,0);
//            }
            R.intake.autoInAsync(0.4);
            R.lift.update(0);
            R.localizer.update();
            deltaleft = Localizer.encoderTicksToInches(Math.abs(R.drivetrainTank.lf.getCurrentPosition() - begleft));
            deltaright = Localizer.encoderTicksToInches(Math.abs(R.drivetrainTank.rf.getCurrentPosition() - begright));
            dist = (deltaleft + deltaright) / 2;
            telemetry.addData("dist", dist);
//            telemetry.addData("isFull", R.intake.isFull());
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.intake.setPower(0);
        tracker.forwardlift(dist, -0.6);
        tracker.rotateTo(0);
        tracker.forwardlift(36, -0.8);
        tracker.rotateTo(new Vector2d(-24, -12));
        t = new ElapsedTime();
        while (t.milliseconds() < 500)
            R.lift.update(high_pos);
        tracker.forward(5, 0.5);
        R.intake.update();
        while (R.intake.isFull() && t.milliseconds() < 2000 && !isStopRequested()) {
            R.lift.update(high_pos);
            R.intake.autoOutAsync(0.7);
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200)
            R.intake.motor.setPowerClassic(intakePwr);
        R.intake.motor.setPowerClassic(0);
        tracker.forward(5, -0.8);

        t = new ElapsedTime();
        while (t.milliseconds() < 500)
            R.lift.update(low_pos);
        tracker.rotateTo(0);

        tracker.forwardlift(36, 0.8);
        tracker.rotateTo(30);
        begleft = R.drivetrainTank.lf.getCurrentPosition();
        begright = R.drivetrainTank.rf.getCurrentPosition();
        R.intake.setPower(-0.8);
        while (t.milliseconds() < 1000)
            R.lift.update(0);
        R.intake.update();
        while (!isStopRequested() && !R.intake.isFull) {
            double velocity = Localizer.encoderTicksToInches((R.drivetrainTank.lf.getVelocity() + R.drivetrainTank.lf.getVelocity()) / 2);
            R.drivetrainTank.setPowerSimple(0.4, 0.4);
            while (!isStopRequested() && Math.abs(R.imu.getTiltHeading()) > 3 && !R.intake.isFull()) {
                R.drivetrainTank.setPowerSimple(-0.8, -0.8);
                R.intake.update();
            }
//            if (Math.abs(velocity) < 1) {
//                R.drivetrainTank.setPowerSimple(-1, -1);
//                R.delay(500);
//                R.drivetrainTank.setPowerSimple(0,0);
//            }
            R.intake.autoInAsync(0.4);
            R.lift.update(0);
            R.localizer.update();
            deltaleft = Localizer.encoderTicksToInches(Math.abs(R.drivetrainTank.lf.getCurrentPosition() - begleft));
            deltaright = Localizer.encoderTicksToInches(Math.abs(R.drivetrainTank.rf.getCurrentPosition() - begright));
            dist = (deltaleft + deltaright) / 2;
            telemetry.addData("dist", dist);
//            telemetry.addData("isFull", R.intake.isFull());
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        R.intake.setPower(0);
        tracker.forwardlift(dist, -0.6);
        tracker.rotateTo(0);
        tracker.forwardlift(36, -0.8);
        tracker.rotateTo(new Vector2d(-24, -12));
        t = new ElapsedTime();
        while (t.milliseconds() < 500)
            R.lift.update(high_pos);
        tracker.forward(5, 0.5);
        R.intake.update();
        while (R.intake.isFull() && t.milliseconds() < 2000 && !isStopRequested()) {
            R.lift.update(high_pos);
            R.intake.autoOutAsync(0.7);
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200)
            R.intake.motor.setPowerClassic(intakePwr);
        R.intake.motor.setPowerClassic(0);
        tracker.forward(5, -0.8);

        tracker.rotateTo(0);
        tracker.forwardlift(36, 1);
        t = new ElapsedTime();
        while (!isStopRequested() && t.milliseconds() < 3000)
            R.lift.update(100);
        R.lift.setPower(0);
    }
}
