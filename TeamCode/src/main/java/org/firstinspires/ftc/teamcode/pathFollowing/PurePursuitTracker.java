package org.firstinspires.ftc.teamcode.pathFollowing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.math.Operations;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.misc.DrivePower;
import org.firstinspires.ftc.teamcode.modules.DrivetrainTank;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Localizer;
import org.firstinspires.ftc.teamcode.modules.RobotIMU;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.math.Operations.angleWrap;

import android.os.Build;

import androidx.annotation.RequiresApi;

/**
 * Given a path and a couple parameters, this class will handle much of the processing of the Pure Pursuit algorithm
 */
@Config
public class PurePursuitTracker {
	private double forwardOffset = 0.5;
	private DrivetrainTank drivetrainTank;
	private Localizer localizer;
	private LinearOpMode opMode;
	private Lift lift;
	private Intake intake;
	private int lastClosestPoint;
	private List<Path> paths = new ArrayList<>();
	private Path path;
	public double lookaheadDistance;
	private double robotTrack = 18;
	public static double feedbackMultiplier = 0.005;
	private int closestPointLimit = 0;
	public FtcDashboard dashboard;
	private double averageCycletime;
	private double prevtime = 0;
	private List<Double> timeList = new ArrayList<>();
	private boolean isLift = false;
	private RobotIMU imu;
	private double errorSumLeft = 0;
	private double errorSumRight = 0;

	public static double kV = 0.0265;
	public static double kA = 0.0008;
	public static double kI = 0;

	public static double ANGLE_OFFSET = 5; //IN DEGREES

	private DrivePower targetVels = new DrivePower(0, 0);

	private PurePursuitTracker() {
		reset();
	}

	private double rightOutput = 0;
	private double prevRightOutput = 0;
	private double leftOutput = 0;
	private double prevLeftOutput = 0;
	public double currleft = 0, curright = 0;
//	public static PurePursuitTracker getInstance() {
//		return instance == null ? instance = new PurePursuitTracker() : instance;
//	}

	public PurePursuitTracker(Robot21 R, LinearOpMode opMode) {
		this.drivetrainTank = R.drivetrainTank;
		this.localizer = R.localizer;
		this.opMode = opMode;
		this.lift = R.lift;
		this.imu = R.imu;
		this.intake = R.intake;
		dashboard = FtcDashboard.getInstance();
		dashboard.setTelemetryTransmissionInterval(25);
		reset();
	}

	public PurePursuitTracker(DrivetrainTank drivetrainTank, Localizer localizer, LinearOpMode opMode) {
		this.drivetrainTank = drivetrainTank;
		this.localizer = localizer;
		this.opMode = opMode;
		dashboard = FtcDashboard.getInstance();
		dashboard.setTelemetryTransmissionInterval(25);
		reset();
	}

	public void setLift(Lift lift) {
		this.lift = lift;
		isLift = true;
	}

	/**
	 * Sets limit for calculation of the closest point on the path to the robot
	 * @param closestPointLimit: maximum number of points in front of previous closest point searched to be current closest point,
	 *                         nonpositive values means the whole path is searched
	 */
	public void setClosestPointLimit(int closestPointLimit){
		this.closestPointLimit = closestPointLimit;
	}

	/**
	 * Sets the path to be tracked
	 *
	 * @param lookaheadDistance lookahead distance (ideally between 15-24 inches)
	 */
	public void setPaths(List<Path> paths, double lookaheadDistance) {
		reset();
		this.paths = paths;
		this.lookaheadDistance = lookaheadDistance;
	}

	/**
	 * Sets robot track
	 * @param robotTrack width of robot measured from center of each side of drivetrain
	 */
	public void setRobotTrack(double robotTrack) {
		this.robotTrack = robotTrack;
	}

	/**
	 * Sets the feedback multiplier (proportional feedback constant) for velocities
	 *
	 * @param feedbackMultiplier feedback multiplier
	 */
	public void setFeedbackMultiplier(double feedbackMultiplier) {
		this.feedbackMultiplier = feedbackMultiplier;
	}

	public void reset() {
		this.lastClosestPoint = 0;
		rightOutput = 0;
		prevRightOutput = 0;
		leftOutput = 0;
		prevLeftOutput = 0;
		targetVels = new DrivePower(0, 0);
		prevtime = 0;
		errorSumLeft = 0;
		errorSumRight = 0;
		timeList = new ArrayList<>();
	}

	/**
	 * Updates the tracker and returns left and right velocities
	 *
//	 * @param currPose current position of robot
//	 * @param currLeftVel current left velocity of robot
//	 * @param currRightVel current right velocity of robot
//	 * @param heading current angle of robot
	 * @return left and right velocities to be sent to drivetrain
	 */
	@RequiresApi(api = Build.VERSION_CODES.N)
	public DrivePower update(double currtime) {
		timeList.add((currtime - prevtime));
		prevtime = currtime;
		double sum = 0;
		for (int i = 0; i < timeList.size(); i++) {
			sum += timeList.get(i);
		}
		averageCycletime = sum / timeList.size();
		TelemetryPacket packet = new TelemetryPacket();
		Pose2d currPose2d = localizer.update();
		Vector2d currPose = currPose2d.toVector();
		double heading = currPose2d.getHeading();
		boolean onLastSegment = false;
		int closestPointIndex = getClosestPointIndex(currPose);
		Vector2d lookaheadPoint = new Vector2d(0, 0);
		ArrayList<Vector2d> robotPath = path.getRobotPath();
		Optional<Vector2d> lookaheadPtOptional = null;
		int starti = closestPointIndex + 1;
		for (int i = closestPointIndex + 1; i < robotPath.size(); i++) {
			Vector2d startPoint = robotPath.get(i - 1);
			Vector2d endPoint = robotPath.get(i);
			if (i == robotPath.size() - 1) {
//				if (path.isForward()) {
//					rotateTo(endPoint);
//					forward(currPose.sub(endPoint).norm(), 0.7);
//				}
//				else {
//					rotateTo(endPoint, false);
//					forward(currPose.sub(endPoint).norm(), -0.7);
//				}
				onLastSegment = true;
			}
			lookaheadPtOptional = calculateLookAheadPoint(startPoint, endPoint, currPose, lookaheadDistance, onLastSegment);
			if (lookaheadPtOptional.isPresent()) {
				lookaheadPoint = lookaheadPtOptional.get();
				opMode.telemetry.addData("FOUND", null);
				break;
			}
		}

		double velocity = robotPath.get(getClosestPointIndex(currPose)).getVelocity();
		double curvature = path.calculateCurvatureLookAheadArc(currPose, heading, lookaheadPoint, lookaheadDistance);
		double leftTargetVel = calculateLeftTargetVelocity(velocity, curvature);
		double rightTargetVel = calculateRightTargetVelocity(velocity, curvature);

		if (!path.isForward()) {
//			double lt = leftTargetVel;
//			leftTargetVel = -rightTargetVel;
//			rightTargetVel = -lt;
			leftTargetVel = -leftTargetVel;
			rightTargetVel = -rightTargetVel;
		}

		double currLeftVel = Localizer.encoderTicksToInches(drivetrainTank.lf.getVelocity());
		double currRightVel = Localizer.encoderTicksToInches(drivetrainTank.rf.getVelocity());

		if (errorSumLeft * kI > 0.5 && Math.signum(leftTargetVel - currLeftVel) > 0) {}
		else if (errorSumLeft * kI < -0.5 && Math.signum(leftTargetVel - currLeftVel) < 0) {}
		else
			errorSumLeft += (leftTargetVel - currLeftVel);
		if (errorSumRight * kI > 0.5 && Math.signum(rightTargetVel - currRightVel) > 0) {}
		else if (errorSumRight * kI < -0.5 && Math.signum(rightTargetVel - currRightVel) < 0) {}
		else
			errorSumRight += (rightTargetVel - currRightVel);

		double leftFeedback = feedbackMultiplier * (leftTargetVel - currLeftVel) + kI * errorSumLeft;
		double rightFeedback = feedbackMultiplier * (rightTargetVel - currRightVel) + kI * errorSumRight;

        double rightFF = calculateFeedForward(rightTargetVel, currRightVel, true);
        double leftFF = calculateFeedForward(leftTargetVel, currLeftVel, false);
//        double rightFB = calculateFeedback(rightTargetVel, currVel);
//        double leftFB = calculateFeedback(leftTargetVel, currVel);

		double leftPower = leftFeedback + leftFF; //+lefttargetvel
		double rightPower = rightFeedback + rightFF; //+righttargetvel

		targetVels = new DrivePower(leftPower, rightPower);
//		opMode.telemetry.addData("curvature", curvature);
//		opMode.telemetry.addData("leftMotorCounts", drivetrainTank.lf.getCurrentPosition());
//		opMode.telemetry.addData("rightMotorCounts", drivetrainTank.rf.getCurrentPosition());
//		opMode.telemetry.addData("currLeftVel", currLeftVel);
//		opMode.telemetry.addData("currRightVel", currRightVel);
//		opMode.telemetry.addData("target velocity", velocity);
//		opMode.telemetry.addData("current velocity", (currLeftVel + currRightVel) / 2);
//		opMode.telemetry.addData("leftTargetVel", leftTargetVel);
//		opMode.telemetry.addData("rightTargetVel", rightTargetVel);
//		opMode.telemetry.addData("leftPower", leftPower);
//		opMode.telemetry.addData("rightPower", rightPower);
//		opMode.telemetry.addData("lookaheadX", lookaheadPoint.getX());
//		opMode.telemetry.addData("lookaheadY", lookaheadPoint.getY());
//		opMode.telemetry.addData("curvature", curvature);
//		opMode.telemetry.addData("currentX", currPose.getX());
//		opMode.telemetry.addData("currentY", currPose.getY());
//		opMode.telemetry.addData("isLast", onLastSegment);
//		opMode.telemetry.addData("closestPointX", robotPath.get(getClosestPointIndex(currPose)).getX());
//		opMode.telemetry.addData("closestPointY", robotPath.get(getClosestPointIndex(currPose)).getY());
//		opMode.telemetry.addData("closestPointIndex", getClosestPointIndex(currPose));
		packet.put("LeftTarget", leftTargetVel);
		packet.put("LeftCurrent", currLeftVel);
		packet.put("RightTarget", rightTargetVel);
		packet.put("RightCurrent", currRightVel);
		dashboard.sendTelemetryPacket(packet);
		opMode.telemetry.update();
		return targetVels;
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	public DrivePower update(double currtime, boolean isIntaking) {
		timeList.add((currtime - prevtime));
		prevtime = currtime;
		double sum = 0;
		for (int i = 0; i < timeList.size(); i++) {
			sum += timeList.get(i);
		}
		averageCycletime = sum / timeList.size();
		TelemetryPacket packet = new TelemetryPacket();
		Pose2d currPose2d = localizer.update();
		Vector2d currPose = currPose2d.toVector();
		double heading = currPose2d.getHeading();
		boolean onLastSegment = false;
		int closestPointIndex = getClosestPointIndex(currPose);
		Vector2d lookaheadPoint = new Vector2d(0, 0);
		ArrayList<Vector2d> robotPath = path.getRobotPath();
		Optional<Vector2d> lookaheadPtOptional = null;
		int starti = closestPointIndex + 1;
		for (int i = closestPointIndex + 1; i < robotPath.size(); i++) {
			Vector2d startPoint = robotPath.get(i - 1);
			Vector2d endPoint = robotPath.get(i);
			if (i == robotPath.size() - 1) {
//				if (path.isForward()) {
//					rotateTo(endPoint);
//					forward(currPose.sub(endPoint).norm(), 0.7, isIntaking);
//				}
//				else {
//					rotateTo(endPoint, false);
//					forward(currPose.sub(endPoint).norm(), -0.7, isIntaking);
//				}
				onLastSegment = true;
			}
			lookaheadPtOptional = calculateLookAheadPoint(startPoint, endPoint, currPose, lookaheadDistance, onLastSegment);
			if (lookaheadPtOptional.isPresent()) {
				lookaheadPoint = lookaheadPtOptional.get();
//				opMode.telemetry.addData("FOUND", null);
				break;
			}
		}

		double velocity = robotPath.get(getClosestPointIndex(currPose)).getVelocity();
		double curvature = path.calculateCurvatureLookAheadArc(currPose, heading, lookaheadPoint, lookaheadDistance);
		double leftTargetVel = calculateLeftTargetVelocity(velocity, curvature);
		double rightTargetVel = calculateRightTargetVelocity(velocity, curvature);

		if (!path.isForward()) {
//			double lt = leftTargetVel;
//			leftTargetVel = -rightTargetVel;
//			rightTargetVel = -lt;
			leftTargetVel = -leftTargetVel;
			rightTargetVel = -rightTargetVel;
		}

		double currLeftVel = Localizer.encoderTicksToInches(drivetrainTank.lf.getVelocity());
		double currRightVel = Localizer.encoderTicksToInches(drivetrainTank.rf.getVelocity());

		if (errorSumLeft * kI > 0.5 && Math.signum(leftTargetVel - currLeftVel) > 0) {}
		else if (errorSumLeft * kI < -0.5 && Math.signum(leftTargetVel - currLeftVel) < 0) {}
		else
			errorSumLeft += (leftTargetVel - currLeftVel);
		if (errorSumRight * kI > 0.5 && Math.signum(rightTargetVel - currRightVel) > 0) {}
		else if (errorSumRight * kI < -0.5 && Math.signum(rightTargetVel - currRightVel) < 0) {}
		else
			errorSumRight += (rightTargetVel - currRightVel);

		double leftFeedback = feedbackMultiplier * (leftTargetVel - currLeftVel) + kI * errorSumLeft;
		double rightFeedback = feedbackMultiplier * (rightTargetVel - currRightVel) + kI * errorSumRight;

		double rightFF = calculateFeedForward(rightTargetVel, currRightVel, true);
		double leftFF = calculateFeedForward(leftTargetVel, currLeftVel, false);
//        double rightFB = calculateFeedback(rightTargetVel, currVel);
//        double leftFB = calculateFeedback(leftTargetVel, currVel);

		double leftPower = leftFeedback + leftFF; //+lefttargetvel
		double rightPower = rightFeedback + rightFF; //+righttargetvel

		targetVels = new DrivePower(leftPower, rightPower);
//		opMode.telemetry.addData("curvature", curvature);
//		opMode.telemetry.addData("leftMotorCounts", drivetrainTank.lf.getCurrentPosition());
//		opMode.telemetry.addData("rightMotorCounts", drivetrainTank.rf.getCurrentPosition());
//		opMode.telemetry.addData("currLeftVel", currLeftVel);
//		opMode.telemetry.addData("currRightVel", currRightVel);
//		opMode.telemetry.addData("velocity", velocity);
		opMode.telemetry.addData("leftTargetVel", leftTargetVel);
		opMode.telemetry.addData("rightTargetVel", rightTargetVel);
		opMode.telemetry.addData("leftPower", leftPower);
		opMode.telemetry.addData("rightPower", rightPower);
//		opMode.telemetry.addData("lookaheadX", lookaheadPoint.getX());
//		opMode.telemetry.addData("lookaheadY", lookaheadPoint.getY());
//		opMode.telemetry.addData("curvature", curvature);
//		opMode.telemetry.addData("currentX", currPose.getX());
//		opMode.telemetry.addData("currentY", currPose.getY());
//		opMode.telemetry.addData("isLast", onLastSegment);
//		opMode.telemetry.addData("closestPointX", robotPath.get(getClosestPointIndex(currPose)).getX());
//		opMode.telemetry.addData("closestPointY", robotPath.get(getClosestPointIndex(currPose)).getY());
//		opMode.telemetry.addData("closestPointIndex", getClosestPointIndex(currPose));
		packet.put("LeftTarget", leftTargetVel);
		packet.put("LeftCurrent", currLeftVel);
		packet.put("RightTarget", rightTargetVel);
		packet.put("RightCurrent", currRightVel);
		dashboard.sendTelemetryPacket(packet);
		opMode.telemetry.update();
		return targetVels;
	}
    //calculates the feedForward and the feedBack that will get passed through to the motors

    private double calculateFeedForward(double targetVel, double currVel, boolean right) {
        double targetAcc = (targetVel - currVel)/(averageCycletime);
		double maxAccel = path.getMaxAcceleration();
		targetAcc = Range.clip(targetAcc, -maxAccel, maxAccel);
        double rateLimitedVel = rateLimiter(targetVel, maxAccel, right);
        return (kV * rateLimitedVel) + (kA * targetAcc);
    }

//    private double calculateFeedback(double targetVel, double currVel) {
//        return Constants.kP * (targetVel - currVel);
//    }


	//calculates the left and right target velocities given the targetRobotVelocity

	/**
	 * Calculates the left target velocity given target overall velocity
	 *
	 * @param targetRobotVelocity target overall robot velocity
	 * @param curvature           curvature of path at current point
	 * @return left target velocity
	 */
	private double calculateLeftTargetVelocity(double targetRobotVelocity, double curvature) {
		return targetRobotVelocity * ((2 + (robotTrack * curvature))) / 2;
	}

	/**
	 * Calculates the right target velocity given target overall velocity
	 * @param targetRobotVelocity target overall robot velocity
	 * @param curvature curvature of path at current point
	 * @return right target velocity
	 */
	private double calculateRightTargetVelocity(double targetRobotVelocity, double curvature) {
		return targetRobotVelocity * ((2 - (robotTrack * curvature))) / 2;
	}


    //limits the rate of change of a value given a maxRate parameter

    private double rateLimiter(double input, double maxRate, boolean right) {
        double maxChange = averageCycletime * maxRate;
        if (right) {
            rightOutput += Range.clip(input - prevRightOutput, -maxChange, maxChange);
            prevRightOutput = rightOutput;
            return rightOutput;
        }
        else {
            leftOutput += Range.clip(input - prevLeftOutput, -maxChange, maxChange);
            prevLeftOutput = leftOutput;
            return leftOutput;
        }
    }


	//calculates the intersection point between a point and a circle

	/**
	 * Calculates the intersection t-value between a line and a circle, using quadratic formula
	 *
	 * @param startPoint        start of line
	 * @param endPoint          end of line
	 * @param currPos           current robot position (or center of the circle)
	 * @param lookaheadDistance lookahead distance along the path (or radius of the circle)
	 * @return intersection t-value (scaled from 0-1, representing proportionally how far along the segment the intersection is)
	 */
	@RequiresApi(api = Build.VERSION_CODES.N)
	private Optional<Double> calcIntersectionTVal(Vector2d startPoint, Vector2d endPoint, Vector2d currPos, double lookaheadDistance) {

		Vector2d d = Vector2d.sub(endPoint, startPoint);
		Vector2d f = Vector2d.sub(startPoint, currPos);

		double a = d.dot(d);
		double b = 2 * f.dot(d);
		double c = f.dot(f) - Math.pow(lookaheadDistance, 2);
		double discriminant = Math.pow(b, 2) - (4 * a * c);

		if (discriminant < 0) {
			return Optional.empty();
		} else {
			discriminant = Math.sqrt(discriminant);
			double t1 = (-b - discriminant) / (2 * a);
			double t2 = (-b + discriminant) / (2 * a);

			if (t1 >= 0 && t1 <= 1) {
				return Optional.of(t1);
			}
			if (t2 >= 0 && t2 <= 1) {
				return Optional.of(t2);
			}

		}

		return Optional.empty();
	}

	//uses the calculated intersection point to get a Vector value on the path that is the lookahead point

	/**
	 * Uses the calculated intersection t-value to get a point on the path of where to look ahead
	 *
	 * @param startPoint        starting point
	 * @param endPoint          ending point
	 * @param currPos           current robot position
	 * @param lookaheadDistance lookahead distance along the path
	 * @param onLastSegment     whether or not we are on the last path segment
	 * @return lookahead point
	 */
	@RequiresApi(api = Build.VERSION_CODES.N)
	private Optional<Vector2d> calculateLookAheadPoint(Vector2d startPoint, Vector2d endPoint, Vector2d currPos, double lookaheadDistance, boolean onLastSegment) {
		Optional<Double> tIntersect = calcIntersectionTVal(startPoint, endPoint, currPos, lookaheadDistance);
		if (!tIntersect.isPresent() && onLastSegment) {
			return Optional.of(path.getRobotPath().get(path.getRobotPath().size() - 1));
		} else if (!tIntersect.isPresent()) {
			return Optional.empty();
		} else {
			Vector2d intersectVector = Vector2d.sub(endPoint, startPoint, null);
			Vector2d vectorSegment = Vector2d.mult(intersectVector, tIntersect.get());
			Vector2d point = Vector2d.add(startPoint, vectorSegment);
			return Optional.of(point);
		}
	}

	/**
	 * Calculates the index of the point on the path that is closest to the robot. Also avoids going backwards
	 * @param currPos current robot position
	 * @return index of closest point on path
	 */
	private int getClosestPointIndex(Vector2d currPos) {
		double shortestDistance = Double.MAX_VALUE;
		ArrayList<Vector2d> robotPath = path.getRobotPath();
		int closestPoint = robotPath.size()-1;
		int lastPointToSearch = closestPointLimit > 0 ? lastClosestPoint + closestPointLimit : robotPath.size()-1;
		lastPointToSearch = Math.min(lastPointToSearch, robotPath.size()-1);	// make sure the searched points do not exceed robotPath
		for (int i = lastClosestPoint; i <= lastPointToSearch; i++) {
			if (Vector2d.dist(robotPath.get(i), currPos) < shortestDistance) {
				closestPoint = i;
				shortestDistance = Vector2d.dist(robotPath.get(i), currPos);
			}
		}
		lastClosestPoint = closestPoint;
		return closestPoint;
	}

	/**
	 * Tells PurePursuitAction when we are done, indicated by the closest point being the last point of the path
	 * @return whether or not we should finish
	 */
	public boolean isDone() {
		return getClosestPointIndex(localizer.update().toVector()) == path.getRobotPath().size() - 1;
	}

	public void setPath(Integer pathIndex) {
		this.reset();
		this.path = paths.get(pathIndex);
		this.lookaheadDistance = path.getLookaheadDistance();
	}

	public Path getPath() {
		return path;
	}

	public DrivePower getTargetVels() {
		return targetVels;
	}

	public void rotateTo(double targetHeading) {
		ElapsedTime t = new ElapsedTime();
		double currentheading = localizer.getHeading();
		double dif = Math.toRadians(targetHeading) - currentheading;
//		double dif = Math.atan2(Math.cos(Math.toRadians(targetHeading) - currentheading), Math.sin(Math.toRadians(targetHeading) - currentheading));
		double angle;
		if (Math.abs(dif) > Math.abs(dif - 2 * Math.PI * Math.signum(dif)))
			angle = dif - 2 * Math.PI * Math.signum(dif);
		else
			angle = dif;
		while (Math.abs(angle) > Math.toRadians(ANGLE_OFFSET) && !opMode.isStopRequested()) {
			currentheading = localizer.getHeading();
			dif = Math.toRadians(targetHeading) - currentheading;
			if (Math.abs(dif) > Math.abs(dif - 2 * Math.PI * Math.signum(dif)))
				angle = dif - 2 * Math.PI * Math.signum(dif);
			else
				angle = dif;
			if (isLift)
				lift.update(Lift.low_pos);
			drivetrainTank.goToHeading(Math.toRadians(targetHeading), currentheading, t.milliseconds());
			localizer.update();
			opMode.telemetry.addData("dif", Math.toDegrees(dif));
			opMode.telemetry.addData("dif2",Math.toDegrees(dif - 2 * Math.PI * Math.signum(dif)));
			opMode.telemetry.addData("angle", Math.toDegrees(angle));
			opMode.telemetry.update();
		}
		drivetrainTank.setPowerSimple(0, 0);
		drivetrainTank.HEADING_PID.reset();
	}

	public void rotateTo(double targetHeading, double liftTarget) {
		ElapsedTime t = new ElapsedTime();
		double currentheading = localizer.getHeading();
		double dif = Math.toRadians(targetHeading) - currentheading;
//		double dif = Math.atan2(Math.cos(Math.toRadians(targetHeading) - currentheading), Math.sin(Math.toRadians(targetHeading) - currentheading));
		double angle;
		if (Math.abs(dif) > Math.abs(dif - 2 * Math.PI * Math.signum(dif)))
			angle = dif - 2 * Math.PI * Math.signum(dif);
		else
			angle = dif;
		while (Math.abs(angle) > Math.toRadians(ANGLE_OFFSET) && !opMode.isStopRequested()) {
			lift.update(liftTarget);
			currentheading = localizer.getHeading();
			dif = Math.toRadians(targetHeading) - currentheading;
			if (Math.abs(dif) > Math.abs(dif - 2 * Math.PI * Math.signum(dif)))
				angle = dif - 2 * Math.PI * Math.signum(dif);
			else
				angle = dif;
			if (isLift)
				lift.update(Lift.low_pos);
			drivetrainTank.goToHeading(Math.toRadians(targetHeading), currentheading, t.milliseconds());
			localizer.update();
			opMode.telemetry.addData("dif", Math.toDegrees(dif));
			opMode.telemetry.addData("dif2",Math.toDegrees(dif - 2 * Math.PI * Math.signum(dif)));
			opMode.telemetry.addData("angle", Math.toDegrees(angle));
			opMode.telemetry.update();
		}
		drivetrainTank.setPowerSimple(0, 0);
		drivetrainTank.HEADING_PID.reset();
	}

	public void rotateTo(Vector2d targetPoint) {
		double targetHeading = Math.toDegrees(Operations.angleWrap(Math.atan2(targetPoint.getY() - localizer.getY(), targetPoint.getX() - localizer.getX()) + Math.toRadians(-90)));
//		if (targetHeading == 0)
//			targetHeading = 160;
		rotateTo(targetHeading);
	}

	public void rotateTo(Vector2d targetPoint, double liftTarget) {
		double targetHeading = Math.toDegrees(Operations.angleWrap(Math.atan2(targetPoint.getY() - localizer.getY(), targetPoint.getX() - localizer.getX()) + Math.toRadians(-90)));
//		if (targetHeading == 0)
//			targetHeading = 160;
		rotateTo(targetHeading, liftTarget);
	}

	public void rotateTo(Vector2d targetPoint, boolean isForward) {
		if (isForward) {
			double targetHeading = Math.toDegrees(Operations.angleWrap(Math.atan2(targetPoint.getY() - localizer.getY(), targetPoint.getX() - localizer.getX()) + Math.toRadians(-90)));
			rotateTo(targetHeading);
		}
		else {
			double targetHeading = Math.toDegrees(Operations.angleWrap(Math.atan2(targetPoint.getY() - localizer.getY(), targetPoint.getX() - localizer.getX()) + Math.toRadians(90)));
			rotateTo(targetHeading);
		}
	}

	public static double kP = 0.1;
	public static double kIforward = 0.001;
	public void forward(double dist, double power) {
		double begleft = Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition());
		double begright = Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition());
		double currleft = 0;
		double curright = 0;
		double avg = 0, error = 0;
		double sum = 0;
		while ((currleft + curright) / 2 < Math.abs(dist) && !opMode.isStopRequested()) {
			drivetrainTank.setPowerSimple(power, power);
			currleft = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition()) - begleft);
			curright = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition()) - begright);
			avg = (currleft + curright) / 2;
			error = Math.abs(dist) - avg;
			if (sum > 0.8 && Math.signum(error) > 0) {}
			else if (sum < -0.8 && Math.signum(error) < 0) {}
			else
				sum += error;
			localizer.update();
			drivetrainTank.setPowerSimple(Math.signum(power) * 0.2 + power * error * kP + sum * kI, Math.signum(power) * 0.2 + power * error * kP + sum * kI);
		}
		drivetrainTank.setPowerSimple(0, 0);
	}

	public void forward(double dist, double power, boolean isIntaking) {
		double begleft = Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition());
		double begright = Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition());
		double currleft = 0;
		double curright = 0;
		double avg = 0, error = 0;
		double sum = 0;
		if (isIntaking) {
			while ((currleft + curright) / 2 < Math.abs(dist) && !opMode.isStopRequested() && !intake.isFull()) {
				drivetrainTank.setPowerSimple(power, power);
				currleft = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition()) - begleft);
				curright = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition()) - begright);
				avg = (currleft + curright) / 2;
				error = Math.abs(dist) - avg;
				if (sum > 0.8 && Math.signum(error) > 0) {
				} else if (sum < -0.8 && Math.signum(error) < 0) {
				} else
					sum += error;
				localizer.update();
				drivetrainTank.setPowerSimple(Math.signum(power) * 0.2 + power * error * kP + sum * kI, Math.signum(power) * 0.2 + power * error * kP + sum * kI);
			}
		}
		else {
			while ((currleft + curright) / 2 < Math.abs(dist) && !opMode.isStopRequested()) {
				drivetrainTank.setPowerSimple(power, power);
				currleft = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition()) - begleft);
				curright = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition()) - begright);
				avg = (currleft + curright) / 2;
				error = Math.abs(dist) - avg;
				if (sum > 0.8 && Math.signum(error) > 0) {
				} else if (sum < -0.8 && Math.signum(error) < 0) {
				} else
					sum += error;
				localizer.update();
				drivetrainTank.setPowerSimple(Math.signum(power) * 0.2 + power * error * kP + sum * kI, Math.signum(power) * 0.2 + power * error * kP + sum * kI);
			}
		}
		drivetrainTank.setPowerSimple(0, 0);
	}

	public void forwardlift(double dist, double power) {
		double begleft = Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition());
		double begright = Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition());
		double currleft = 0;
		double curright = 0;
		double avg = 0, error = 0, sum = 0;
		while ((currleft + curright) / 2 < Math.abs(dist) && !opMode.isStopRequested()) {
			currleft = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.lf.getCurrentPosition()) - begleft);
			curright = Math.abs(Localizer.encoderTicksToInches(drivetrainTank.rf.getCurrentPosition()) - begright);
			avg = (currleft + curright) / 2;
			error = Math.abs(dist) - avg;
			if (sum > 0.8 && Math.signum(error) > 0) {}
			else if (sum < -0.8 && Math.signum(error) < 0) {}
			else
				sum += error;
			localizer.update();
			lift.update(Lift.low_pos);
			opMode.telemetry.addData("current", (currleft + curright) / 2);
			opMode.telemetry.update();
			drivetrainTank.setPowerSimple(Math.signum(power) * 0.3 + power * error * kP + sum * kI,  Math.signum(power) * 0.3 + power * error * kP + sum * kI);
		}
		drivetrainTank.setPowerSimple(0, 0);
	}

	boolean isMid = false;
//	public void runInWarehouseAsync(double power) {
//		if (localizer.isLineMiddle())
//			isMid = true;
//		if (!localizer.isLineBack()) {
//			if (isMid)
//				drivetrainTank.setPowerSimple(0.7, 0.7);
//			else
//				drivetrainTank.setPowerSimple(power, power);
//		}
//		else
//			drivetrainTank.setPowerSimple(0, 0);
//		drivetrainTank.setPowerSimple(power, power);
//		localizer.update();
//	}
}