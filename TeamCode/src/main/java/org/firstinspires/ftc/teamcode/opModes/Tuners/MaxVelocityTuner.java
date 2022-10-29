package org.firstinspires.ftc.teamcode.opModes.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.modules.Localizer;

@Autonomous
public class MaxVelocityTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot21 R = new Robot21();
        R.init(this);
        double maxLinearSpeed = 0;
        double maxLinearAcceleration = 0;
        double currentSpeed = 0;
        double prevSpeed = 0;
        double currentAcceleration = 0;
        double currentPosition = 0;
        double currenttime = 0;
        double prevtime = 0;
        double deltatime = 0;

        waitForStart();

        ElapsedTime t = new ElapsedTime();
        R.drivetrainTank.setPowerSimple(1,1);
        while (currentPosition < 72) {
            currentPosition = Localizer.encoderTicksToInches((R.drivetrainTank.lf.getCurrentPosition() + R.drivetrainTank.rf.getCurrentPosition())) / 2;

            currentSpeed = Math.abs(Localizer.encoderTicksToInches(R.drivetrainTank.lf.getVelocity() + R.drivetrainTank.rf.getVelocity()) / 2);
            currenttime = t.milliseconds() / 1000.0;
            deltatime = currenttime - prevtime;
            prevtime = currenttime;
            if (currentSpeed > maxLinearSpeed)
                maxLinearSpeed = currentSpeed;
            currentAcceleration = Math.abs(currentSpeed - prevSpeed) / deltatime;
//            currentAcceleration = Math.abs(currentSpeed - prevSpeed);
            if (currentAcceleration > maxLinearAcceleration)
                maxLinearAcceleration = currentAcceleration;
            prevSpeed = currentSpeed;
            telemetry.addData("Current speed: ", currentSpeed);
            telemetry.addData("Max linear speed: ", maxLinearSpeed);
            telemetry.addData("Effective max linear speed: ", maxLinearSpeed * 0.9);
            telemetry.addData("Current acceleration: ", currentAcceleration);
            telemetry.addData("Max linear acceleration: ", maxLinearAcceleration);
            telemetry.addData("Effective max linear acceleration: ", maxLinearAcceleration * 0.9);
            telemetry.addData("Current position: ", currentPosition);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0,0);
        while (!isStopRequested()) {}
    }
}
