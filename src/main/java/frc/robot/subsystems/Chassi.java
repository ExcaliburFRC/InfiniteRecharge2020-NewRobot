/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;


public class Chassi extends SubsystemBase {
  //4 spark max + neo
  private CANSparkMax LBM,LFM,RBM,RFM;
  private SpeedControllerGroup leftMotor, rightMotor;
  private DifferentialDrive differentialDrive;
  //2 encoders (PWM)
  private Encoder rightEncoder, leftEncoder;
  //AHRS gyro
  private AHRS gyro;
  //Compressor
  private Compressor compressor;
  private boolean compressorMode;

  DifferentialDriveOdometry driveOdometry;

  public Chassi() {
    gyro = new AHRS(SPI.Port.kMXP);

    LBM = new CANSparkMax(RobotMap.LEFT_BACK_MOTOR_PORT, MotorType.kBrushless);
    LFM = new CANSparkMax(RobotMap.LEFT_FRONT_MOTOR_PORT, MotorType.kBrushless);
    RBM = new CANSparkMax(RobotMap.RIGHT_BACK_MOTOR_PORT, MotorType.kBrushless);
    RFM = new CANSparkMax(RobotMap.RIGHT_FRONT_MOTOR_PORT, MotorType.kBrushless);

    leftMotor = new SpeedControllerGroup(LBM, LFM);
    rightMotor = new SpeedControllerGroup(RBM, RFM);

    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

    leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_P[0], RobotMap.LEFT_ENCODER_P[1], RobotConstants.DriveConstants.isLeftEncoderReversed);
    leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);

    rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_P[0], RobotMap.RIGHT_ENCODER_P[1], RobotConstants.DriveConstants.isRightEncoderReversed);
    rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    
    resetEncoders();

    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()));

    setIdleMode(IdleMode.kBrake);

    compressor = new Compressor();
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean quickTurn){
    this.differentialDrive.curvatureDrive(xSpeed, zRotation * DriveConstants.MANUAL_TURN_MAX, quickTurn);
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    this.differentialDrive.arcadeDrive(xSpeed, zRotation * DriveConstants.MANUAL_TURN_MAX);
  }

  public void tankDrive(double lSpeed, double rSpeed){
    this.differentialDrive.tankDrive(lSpeed, rSpeed);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }
  
  public void resetGyro(){
    gyro.reset();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(-rightVolts);
  }

  public void setIdleMode(IdleMode mode){
    LBM.setIdleMode(mode);
    LFM.setIdleMode(mode);
    RBM.setIdleMode(mode);
    RFM.setIdleMode(mode);
  }

  public IdleMode getIdleMode(){
    return LBM.getIdleMode();
  }

  @Override
  public void periodic() {
    driveOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), leftEncoder.getDistance(),
                      rightEncoder.getDistance());
  }

  public void setCompressorMode(boolean mode){
    compressor.setClosedLoopControl(mode);
    compressorMode = mode;
  }

  public boolean getCompressorMode(){
    return compressorMode;
  }
}
