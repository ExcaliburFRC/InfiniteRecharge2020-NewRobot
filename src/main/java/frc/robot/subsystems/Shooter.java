/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotMap;
import frc.robot.util.BooleanAverager;
import frc.robot.util.RobotUtils;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterMotor;
  boolean isSpeedPursuit;
  double speedSetpoint;
  BooleanAverager speedReadyAverager;
  double errorSum;
  boolean isTelemetry, isSparkMaxControl;

  public Shooter(boolean isTelemetry, boolean isSparkMaxControl){
    shooterMotor = new CANSparkMax(RobotMap.SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setInverted(true);

    if (isSparkMaxControl){
      initPID(shooterMotor.getPIDController());
    }

    isSpeedPursuit = false;
    speedSetpoint = 0;
    
    speedReadyAverager = new BooleanAverager(ShooterConstants.SPEED_BUCKET_SIZE);
    
    this.isTelemetry = isTelemetry;
    this.isSparkMaxControl = isSparkMaxControl;
  }
  
  public Shooter() {
    this(false, false);
  }
    
  private void setIsSpeedPursuit(boolean isSpeedPersuit){
    this.isSpeedPursuit = isSpeedPersuit;
  }

  public void setAbsoluteShooterMotorPower(double p){
    if (this.isSparkMaxControl){
      shooterMotor.getPIDController().setReference(p, ControlType.kDutyCycle);  
    } else {
      shooterMotor.set(p);
    }
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
    var vel_delta = this.speedSetpoint - getShooterMotorVelocity();
    // SmartDashboard.putNumber("vel_delta", vel_delta);
    return Math.abs(vel_delta) < speedTolerance;
  }

  @Override
  public void periodic() {
    speedReadyAverager.update(getRawIsOnSpeed());

    if (!this.isSparkMaxControl){
      externalPID();
    }

    if (isTelemetry){
      SmartDashboard.putNumber("DEBUG_speedSpetpoint", this.speedSetpoint);
      SmartDashboard.putNumber("DEBUG_SPEED", getShooterMotorVelocity());
      SmartDashboard.putBoolean("DEBUG_isOnSpeed", getRawIsOnSpeed());
    }
  }

  private void externalPID(){
    if (isSpeedPursuit){
      var F = (ShooterConstants.MOTOR_KV * speedSetpoint)/12.0;
      var error = speedSetpoint - getShooterMotorVelocity();
      var P = RobotUtils.clip(ShooterConstants.NOSP_SPEED_KP * error, ShooterConstants.kPEffectiveness);
      var I = RobotUtils.clip(ShooterConstants.NOSP_SPEED_KI * errorSum, ShooterConstants.kIEffectiveness);
      var power = F + P + I;
  
      setAbsoluteShooterMotorPower(power);
        
      if (Math.abs(error) < ShooterConstants.I_RANGE){
        errorSum += error;
      }
    } else {
      setAbsoluteShooterMotorPower(0);
      errorSum = 0;
    }
  }

  // private double compensateVoltage(double originalVoltage){
  //   return originalVoltage * (ShooterConstants.VOLTAGE_AT_TOP_SPEED/RobotController.getBatteryVoltage());
  // }
  
  public boolean isSpeedPursuit(){
    return isSpeedPursuit;
  }

  public void setVelocityPID(double target){
    if (isSparkMaxControl){
      shooterMotor.getPIDController().setReference(target / ShooterConstants.SPEED_TO_RPM_CONVERSION, ControlType.kVelocity);
      this.speedSetpoint = target;
    } else {
      setIsSpeedPursuit(true);
      speedSetpoint = target;
    }
  }

  public void releaseVelocityPID(){
    setIsSpeedPursuit(false);
    if (isSparkMaxControl){
      setAbsoluteShooterMotorPower(0);
    }
  }

  
  private void initPID(CANPIDController controller){
    Set<String> errorSet = Stream.of(
      controller.setFeedbackDevice(shooterMotor.getEncoder()),
      controller.setP(ShooterConstants.SPEED_KP),
      controller.setI(ShooterConstants.SPEED_KI),
      controller.setD(ShooterConstants.SPEED_KD),
      controller.setOutputRange(0,1),
      controller.setFF(ShooterConstants.SPEED_KFF)
      )
    .filter(err -> err != CANError.kOk).map(CANError::toString).collect(Collectors.toSet());
    
    if(!errorSet.isEmpty()){
      StringBuilder sb = new StringBuilder();
      sb.append("SparkMax PID not initialized correctly, errors:\n");
      errorSet.forEach(s -> sb.append(s).append("\n"));
      DriverStation.reportError(sb.toString(), false);
    }else DriverStation.reportWarning("SparkMax PID initialized correctly!", false);

  }
}
