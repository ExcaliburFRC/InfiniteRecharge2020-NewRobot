/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Set;
import java.util.stream.Stream;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotMap;
import frc.robot.util.BooleanAverager;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterMotor;
  boolean isSpeedPursuit;
  double speedSetpoint;
  BooleanAverager speedReadyAverager;
  double errorSum;
  boolean isTelemetry;

  public Shooter(boolean isTelemetry){
    shooterMotor = new CANSparkMax(RobotMap.LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    initPID(shooterMotor.getPIDController());
    isSpeedPursuit = false;
    speedSetpoint = 0;
    
    speedReadyAverager = new BooleanAverager(ShooterConstants.SPEED_BUCKET_SIZE);
    
    this.isTelemetry = isTelemetry;
  }

  public Shooter() {
    this(false);
  }
  @Deprecated
  public void setSpeedSetpoint(double setpoint){
    this.speedSetpoint = setpoint;
    setVelocityPID(setpoint);
  }
  
  private void setIsSpeedPursuit(boolean isSpeedPersuit){
    this.isSpeedPursuit = isSpeedPersuit;
  }
  /**
   * @deprecated : set a wanted velocity via {@link #setVelocityPID(double)}.
   * This function sets an absolute speed to the motor, and will probably interfere with the Velocity PID.
   */
  @Deprecated
  public void setShooterMotorPower(double p){
    shooterMotor.set(p);
  }
  
  public double getShooterMotorVelocity(){
    return shooterMotor.getEncoder().getVelocity();
  }

  public boolean getRawIsOnSpeed(){
    return isOnSpeed(ShooterConstants.SPEED_TOLERANCE);
  }

  public boolean isOnSpeed(){
    return speedReadyAverager.getAverage();
  }

  public boolean isOnSpeed(double speedTolerance){
    return Math.abs(speedSetpoint - getShooterMotorVelocity()) < speedTolerance;
  }

  @Override
  public void periodic() {
    
    speedReadyAverager.update(getRawIsOnSpeed());

    if (isTelemetry){
      SmartDashboard.putNumber("DEBUG_SPEED", getShooterMotorVelocity());
      SmartDashboard.putBoolean("DEBUG_isOnSpeed", getRawIsOnSpeed());
    }
  }

  private double compensateVoltage(double originalVoltage){
    return originalVoltage * (ShooterConstants.VOLTAGE_AT_TOP_SPEED/RobotController.getBatteryVoltage());
  }
  
  public boolean isSpeedPursuit(){
    return isSpeedPursuit;
  }

  public void setVelocityPID(double target){
    shooterMotor.getPIDController().setReference(target, ControlType.kVelocity);
  }

  public void releaseVelocityPID(){
    setIsSpeedPursuit(false);
    shooterMotor.getPIDController().setReference(0, ControlType.kDutyCycle);
  }

  
  private void initPID(CANPIDController controller){
    Stream<CANError> errorStream = Set.of( controller.setP(ShooterConstants.SPEED_KP),
      controller.setI(ShooterConstants.SPEED_KI),
      controller.setD(0),
      controller.setOutputRange(-1,1))
      .stream()
      .filter(err -> err != CANError.kOk);
    long errorCount = errorStream.count();
    if(errorCount > 0){
      System.err.printf("SparkMax PID not initialized correctly, %d errors:\n", errorCount);
      errorStream.map(CANError::toString).forEach(System.err::println);
    }
  }
}
