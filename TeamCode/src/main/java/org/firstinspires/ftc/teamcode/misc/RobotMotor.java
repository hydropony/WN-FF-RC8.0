 package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.Localizer;

 public class RobotMotor {
     private final double POWER_OFFSET = 0.0;

     private double last_power = 0;
     private DcMotorEx motor;

     public boolean isAccelerationLimiterOn = false;
     private double maxPowerDelta = 0;

     public RobotMotor(DcMotorEx motor) {
         this.motor = motor;
     }

     public void setDirection(DcMotorEx.Direction direction){
         motor.setDirection(direction);
     }

     public void setMode(DcMotorEx.RunMode mode){
         motor.setMode(mode);
     }

     public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
         motor.setZeroPowerBehavior(zeroPowerBehavior);
     }

     public void setAccelerationLimiter(boolean isOn, double maxPowerDelta) {
          isAccelerationLimiterOn = isOn;
          this.maxPowerDelta = maxPowerDelta;
     }

     public void setPower(double power) {
         if (isAccelerationLimiterOn && Math.abs(power - last_power) >= maxPowerDelta) {
             power = last_power + maxPowerDelta * sign(power - last_power);
         }
         if (power < last_power - POWER_OFFSET || power > last_power + POWER_OFFSET) {
             motor.setPower(power);
         }
         last_power = power;
     }

     public void setPower(double power, double voltage, double base_voltage) {
         if (isAccelerationLimiterOn && Math.abs(power - last_power) >= maxPowerDelta) {
             power = last_power + maxPowerDelta * sign(power - last_power);
         }
         if (power < last_power - POWER_OFFSET || power > last_power + POWER_OFFSET) {
             motor.setPower(power * (base_voltage / voltage));
         }
         last_power = power;
     }

     public void setPowerClassic(double power){
         motor.setPower(power);
     }

     public void setPowerClassic(double power, double voltage, double base_voltage) {
         motor.setPower(power * (base_voltage / voltage));
     }

     public double getCurrentPosition() {
         return motor.getCurrentPosition();
     }

     public double getVelocity() {
         return motor.getVelocity();
     }

     public void setVelocity(double velocityips) {motor.setVelocity(Localizer.inchesToTicks(velocityips));}

     public double getMilliAmps(){
         return motor.getCurrent(CurrentUnit.MILLIAMPS);
     }

     public double getAmps(){
         return motor.getCurrent(CurrentUnit.AMPS);
     }

     private int sign(double a){
         if (a > 0)
             return 1;
         else if (a == 0)
             return 0;
         else
             return -1;
     }

     public DcMotor.RunMode getMode() {
         return motor.getMode();
     }
 }
