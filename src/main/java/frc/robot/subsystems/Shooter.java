/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotMap;
import frc.robot.util.BooleanAverager;
import frc.robot.util.RobotUtils;

public class Shooter extends SubsystemBase {
  TalonSRX leftShooterMotor, rightShooterMotor;
  boolean isSpeedPursuit;
  double speedSetpoint;
  BooleanAverager speedReadyAverager;
  double errorSum;
  boolean isTelemetry;

  public Shooter(boolean isTelemetry){
    leftShooterMotor = new TalonSRX(RobotMap.LEFT_SHOOTER_MOTOR_PORT);
    rightShooterMotor = new TalonSRX(RobotMap.RIGHT_SHOOTER_MOTOR_PORT);
    rightShooterMotor.setSensorPhase(true);
    
    isSpeedPursuit = false;
    speedSetpoint = 0;
    
    speedReadyAverager = new BooleanAverager(ShooterConstants.SPEED_BUCKET_SIZE);
    
    this.isTelemetry = isTelemetry;
  }

  public Shooter() {
    this(false);
  }

   public void setSpeedSetpoint(double setpoint){
    this.speedSetpoint = setpoint;
  }

  public void setIsSpeedPursuit(boolean isSpeedPersuit){
    this.isSpeedPursuit = isSpeedPersuit;
  }

  public void setShooterMotorPower(double p){
    leftShooterMotor.set(ControlMode.PercentOutput, RobotUtils.clip(p,1));
    rightShooterMotor.set(ControlMode.PercentOutput, RobotUtils.clip(-p,1));
  }

   public double getLeftMotorSpeed(){
    return leftShooterMotor.getSelectedSensorVelocity();
  }

  public double getRightMotorSpeed(){
    return rightShooterMotor.getSelectedSensorVelocity();
  }

  public boolean getRawIsOnSpeed(){
    boolean motor1 = Math.abs(speedSetpoint - getLeftMotorSpeed()) < ShooterConstants.SPEED_TOLERANCE;
    boolean motor2 = Math.abs(speedSetpoint - getRightMotorSpeed()) < ShooterConstants.SPEED_TOLERANCE;
    return motor1 && motor2;
  }

  public boolean isOnSpeed(){
    return speedReadyAverager.getAverage();
  }

  public boolean isOnSpeed(double speedTolerance){
    boolean motor1 = Math.abs(speedSetpoint - getLeftMotorSpeed()) < speedTolerance;
    boolean motor2 = Math.abs(speedSetpoint - getRightMotorSpeed()) < speedTolerance;
    return motor1 && motor2;
  }

   @Override
  public void periodic() {
    if (isSpeedPursuit){
      var AFF = compensateVoltage(ShooterConstants.MOTOR_KV * speedSetpoint)/12.0;

      var error = speedSetpoint - leftShooterMotor.getSelectedSensorVelocity();

      var P = RobotUtils.clip(ShooterConstants.SPEED_KP * error, ShooterConstants.kPEffectiveness);

      var I = RobotUtils.clip(ShooterConstants.SPEED_KI * errorSum, 0.12);

      var power = AFF + P + I;

      setShooterMotorPower(power);

      errorSum += RobotUtils.clip(error,2000);

    } else {
      leftShooterMotor.set(ControlMode.PercentOutput, 0);
      rightShooterMotor.set(ControlMode.PercentOutput, 0);

      errorSum = 0;
    }
    
    speedReadyAverager.update(getRawIsOnSpeed());

    if (isTelemetry){
      SmartDashboard.putNumber("DEBUG_LEFTSPEED", getLeftMotorSpeed());
      SmartDashboard.putNumber("DEBUG_RIGHTSPEED", getRightMotorSpeed());
           // SmartDashboard.putNumber("DEBUG_ANGLE_POWER", Robot.m_shooter.getAngleMotorPower());
      SmartDashboard.putBoolean("DEBUG_isOnSpeed", getRawIsOnSpeed());
           // SmartDashboard.putNumber("DEBUG_ROBOT_VOLTAGE", RobotController.getBatteryVoltage());
    }
  }
 
  public void setLeftMotorSpeed(double s){
    leftShooterMotor.set(ControlMode.PercentOutput, s);
  }

  public void setRightMotorSpeed(double s){
    rightShooterMotor.set(ControlMode.PercentOutput, s);
  }

  private double compensateVoltage(double originalVoltage){
    return originalVoltage * (ShooterConstants.VOLTAGE_AT_TOP_SPEED/RobotController.getBatteryVoltage());
  }

  public boolean isSpeedPursuit(){
    return isSpeedPursuit;
  }
}
