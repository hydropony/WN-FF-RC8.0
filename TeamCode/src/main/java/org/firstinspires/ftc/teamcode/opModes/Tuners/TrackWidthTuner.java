package org.firstinspires.ftc.teamcode.opModes.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.modules.Localizer;

@Autonomous
public class TrackWidthTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot21 R = new Robot21();
        R.init(this);

        waitForStart();

        double effectiveTrackWidth = 0;
        double leftPos, rightPos;
        while (Math.abs(R.localizer.getHeading()) < Math.toRadians(170) && !isStopRequested()) {
            R.drivetrainTank.setPowerSimple(-0.8, 0.8);
            leftPos = Math.abs(Localizer.encoderTicksToInches(R.drivetrainTank.lf.getCurrentPosition()));
            rightPos = Math.abs(Localizer.encoderTicksToInches(R.drivetrainTank.rf.getCurrentPosition()));
            if (R.localizer.getHeading() != 0)
                effectiveTrackWidth = (leftPos + rightPos) / Math.abs(R.localizer.getHeading());
            telemetry.addData("leftPos", leftPos);
            telemetry.addData("rightPos", rightPos);
            telemetry.addData("current Heading: ", R.localizer.getHeading());
            telemetry.addData("effectiveTrackWidth: ", effectiveTrackWidth);
            telemetry.update();
        }
        R.drivetrainTank.setPowerSimple(0, 0);


        while (!isStopRequested()) {}
    }
}
