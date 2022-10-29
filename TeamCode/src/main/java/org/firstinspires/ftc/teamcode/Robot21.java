package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.modules.Carousel;
import org.firstinspires.ftc.teamcode.modules.DrivetrainTank;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Localizer;
import org.firstinspires.ftc.teamcode.modules.RobotIMU;

import java.util.List;
//import org.firstinspires.ftc.teamcode.modules.WheelBase;

public class Robot21 extends Robot {
    public DrivetrainTank drivetrainTank;
    public Intake intake;
    public Carousel carousel;
    public RobotIMU imu;
    public Localizer localizer;
    public Lift lift;

    @Override
    public void initHardware(LinearOpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drivetrainTank = new DrivetrainTank();
        carousel = new Carousel();
        intake = new Intake();
        imu = new RobotIMU();
        localizer = new Localizer(opMode);
        lift = new Lift();

        intake.init(opMode);
        drivetrainTank.init(opMode);
        carousel.init(opMode);
        imu.init(opMode);
        localizer.init();
        lift.init(opMode);

//        delay(500);
        telemetry.update();
        telemetry.addData("Robot is ready! ", ")");
        telemetry.update();
    }

    @Override
    public void init(LinearOpMode opMode) {
        initHardware(opMode);
        telemetry.update();
    }

    public void control(boolean isRed, double currtime) {
        intake.control(opGamepad2);
        carousel.telecontrol(opGamepad2, isRed);
//        lift.control(opGamepad2);
        lift.control2(opGamepad2, currtime / 1000.0);
    }
}
