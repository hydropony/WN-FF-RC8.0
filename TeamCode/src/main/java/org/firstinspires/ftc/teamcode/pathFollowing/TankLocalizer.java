package org.firstinspires.ftc.teamcode.pathFollowing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Operations;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.opencv.core.Mat;

import java.util.List;

public abstract class TankLocalizer {
    private Pose2d currentPose;
    private LinearOpMode opMode;
    public double startheading = 0;
    private double heading = 0;
    private double headingVelocity = 0;
    public double delta_left_encoder_pos, left_encoder_pos, prev_left_encoder_pos;
    public double delta_right_encoder_pos, right_encoder_pos, prev_right_encoder_pos;
    private double delta_middle_pos, delta_x, delta_y, delta_heading, prevheading;
    public double x_pos;
    public double y_pos;
    private List<Double> encoderPos, encoderVelocities;
    private boolean debug = false;
    public double trajstart_left_encoder_pos, trajstart_right_encoder_pos;
    public Pose2d velocity;
    private double leftVelocity, rightVelocity;
    private double prevtime = 0, deltatime = 0;

    public TankLocalizer(LinearOpMode opMode, boolean debug) {
        this.opMode = opMode;
        this.debug = true;
    }

    public TankLocalizer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void setStartPose(Pose2d startPose) {
        x_pos = startPose.getX();
        y_pos = startPose.getY();
        startheading = startPose.getHeading();
        prevheading = startheading;
    }

    public Pose2d update() {
        encoderPos = getOdoPositions();
        left_encoder_pos = encoderPos.get(0);
        right_encoder_pos = encoderPos.get(1);
        heading = startheading + encoderPos.get(2);

        delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;

        delta_y = delta_middle_pos * Math.cos(heading);
        delta_x = -delta_middle_pos * Math.sin(heading);

//        if (isLine()) {
//            x_pos = 68;
//            y_pos = 26;
//        }

        x_pos += delta_x;
        y_pos += delta_y;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;

        currentPose = new Pose2d(x_pos, y_pos, heading);
        if (debug) {
            opMode.telemetry.addData("Left encoder:", left_encoder_pos);
            opMode.telemetry.addData("Right encoder:", right_encoder_pos);
            opMode.telemetry.addData("x: ", currentPose.getX());
            opMode.telemetry.addData("y: ", currentPose.getY());
            opMode.telemetry.addData("heading: ", currentPose.getHeading());
//            opMode.telemetry.update();
        }
        return currentPose;
    }

    public Pose2d updateEstimated(double currtime) {
        encoderPos = getOdoPositions();
//        left_encoder_pos = encoderPos.get(0);
//        right_encoder_pos = encoderPos.get(1);
        heading = startheading + encoderPos.get(2);
        encoderVelocities = getOdoVelocities();
        headingVelocity = encoderVelocities.get(2);
        leftVelocity = encoderVelocities.get(0);
        rightVelocity = encoderVelocities.get(1);

//        delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
//        delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        delta_middle_pos = (leftVelocity + rightVelocity) / 2;

        deltatime = currtime - prevtime;

//        delta_x = (-Math.cos(heading) * Math.pow(deltatime, 2) * headingVelocity / 2 - Math.sin(heading) *
//                (deltatime - Math.pow(deltatime, 3) * Math.pow(headingVelocity, 2) / 6)) * delta_middle_pos;
//        delta_y = (-Math.sin(heading) * Math.pow(deltatime, 2) * headingVelocity / 2 + Math.cos(heading) *
//                (deltatime - Math.pow(deltatime, 3) * Math.pow(headingVelocity, 2) / 6)) * delta_middle_pos;

//        delta_y = (Math.cos(heading) * (deltatime - Math.pow(deltatime, 3) * Math.pow(headingVelocity, 2) / 6) - Math.sin(heading) * Math.pow(deltatime, 2) * headingVelocity / 2) * delta_middle_pos;
//        delta_x = (Math.sin(heading) * (deltatime - Math.pow(deltatime, 3) * Math.pow(headingVelocity, 2) / 6) + Math.cos(heading) * Math.pow(deltatime, 2) * headingVelocity / 2) * delta_middle_pos;

        delta_y = -(Math.cos(heading) * (deltatime - Math.pow(deltatime, 3) * Math.pow(headingVelocity, 2) / 6) - Math.sin(heading) * Math.pow(deltatime, 2) * headingVelocity / 2) * delta_middle_pos;
        delta_x = (Math.sin(heading) * (deltatime - Math.pow(deltatime, 3) * Math.pow(headingVelocity, 2) / 6) + Math.cos(heading) * Math.pow(deltatime, 2) * headingVelocity / 2) * delta_middle_pos;

        x_pos += delta_x;
        y_pos += delta_y;

//        prev_left_encoder_pos = left_encoder_pos;
//        prev_right_encoder_pos = right_encoder_pos;
        prevtime = currtime;

        currentPose = new Pose2d(x_pos, y_pos, heading);
        if (debug) {
//            opMode.telemetry.addData("Left encoder:", left_encoder_pos);
//            opMode.telemetry.addData("Right encoder:", right_encoder_pos);
            opMode.telemetry.addData("x: ", currentPose.getX());
            opMode.telemetry.addData("y: ", currentPose.getY());
            opMode.telemetry.addData("heading: ", currentPose.getHeading());
            opMode.telemetry.addData("deltatime: ", deltatime);
            opMode.telemetry.addData("currtime: ", currtime);
            opMode.telemetry.addData("headingvel: ", headingVelocity);
//            opMode.telemetry.update();
        }
        return currentPose;
    }

    public Pose2d update2() {
        delta_x = 0;
        delta_y = 0;
        delta_heading = 0;
        encoderPos = getOdoPositions();
        left_encoder_pos = encoderPos.get(0);
        right_encoder_pos = encoderPos.get(1);
        heading = startheading + encoderPos.get(2);

        delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        delta_heading = Operations.angleWrap(heading - prevheading);

        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;

        double radius = (delta_heading == 0) ? 0 : delta_middle_pos / delta_heading;
        delta_x = (delta_heading == 0) ? 0 : (radius - radius * Math.cos(delta_heading));
        delta_y = (delta_heading == 0) ? delta_middle_pos : radius * Math.sin(delta_heading);

        x_pos += delta_x * Math.cos(heading) + delta_y * Math.sin(heading);
        y_pos -= delta_y * Math.cos(heading) - delta_x * Math.sin(heading);

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prevheading = heading;

        currentPose = new Pose2d(x_pos, y_pos, heading);
        if (debug) {
            opMode.telemetry.addData("Left encoder:", left_encoder_pos);
            opMode.telemetry.addData("Right encoder:", right_encoder_pos);
            opMode.telemetry.addData("x: ", currentPose.getX());
            opMode.telemetry.addData("y: ", currentPose.getY());
            opMode.telemetry.addData("heading: ", currentPose.getHeading());
//            opMode.telemetry.update();
        }
        return currentPose;
    }

    public double getHeading() {
        update();
        return heading;
    }

    public double getX() {
        update();
        return x_pos;
    }

    public double getY() {
        update();
        return y_pos;
    }

    public void trajstart() {
        update();
        trajstart_left_encoder_pos = left_encoder_pos;
        trajstart_right_encoder_pos = right_encoder_pos;
    }

    public double getLeftRawPosition() {
        update();
        return left_encoder_pos - trajstart_left_encoder_pos;
    }

    public double getRightRawPosition() {
        update();
        return right_encoder_pos - trajstart_right_encoder_pos;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public abstract List<Double> getOdoPositions();

    public abstract List<Double> getOdoVelocities();

//    public abstract boolean isLineBack();
//
//    public abstract boolean isLineMiddle();
}
