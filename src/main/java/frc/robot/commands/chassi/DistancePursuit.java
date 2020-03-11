package frc.robot.commands.chassi;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.util.BooleanAverager;
import frc.robot.util.CalculateVisionValues;
import frc.robot.util.RobotUtils;

public class DistancePursuit extends CommandBase {
  private double txError, distError, turnPower, forwardPower; //angleChange is the change in angle needed for every run
  private BooleanAverager txErrorAverager, distErrorAverager;
  private boolean isEnd;
  private double distSetpoint, currentDistance;

  public DistancePursuit(double distance, boolean isEnd) {
    addRequirements(Robot.m_chassi);
    txErrorAverager = new BooleanAverager(30);
    distErrorAverager = new BooleanAverager(30);
    this.isEnd = isEnd;
    distSetpoint = distance;
  }

  public DistancePursuit(double distance) {
    this(distance, true);
  }

  @Override
  public void initialize() {
    txErrorAverager.reset();
    distErrorAverager.reset();
  }

  @Override
  public void execute() {
    txError = Robot.m_limelight.getTx();
    turnPower = txError * DriveConstants.TURN_KP;
    turnPower += txError > 0 ? DriveConstants.TURN_AFF : -DriveConstants.TURN_AFF; 
    turnPower = RobotUtils.clip(turnPower, DriveConstants.MAX_TURN);

    currentDistance = CalculateVisionValues.calculateDistanceShooter(Robot.m_limelight.getTy());
    distError = distSetpoint - currentDistance;
    forwardPower = distError * DriveConstants.DISTANCE_KP;
    forwardPower = RobotUtils.clip(forwardPower, 0.5);

    Robot.m_chassi.arcadeDrive(forwardPower, turnPower);

    txErrorAverager.update(Math.abs(txError) < DriveConstants.ANGLE_TOLERACE);
    distErrorAverager.update(Math.abs(distError) < 0.2);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.m_chassi.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return isEnd ? isReady() : false;
  }

  public boolean isReady(){
    return (txErrorAverager.getAverage() && distErrorAverager.getAverage());
  }
}
