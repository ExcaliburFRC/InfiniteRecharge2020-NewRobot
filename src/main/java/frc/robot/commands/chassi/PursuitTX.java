package frc.robot.commands.chassi;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.util.BooleanAverager;
import frc.robot.util.RobotUtils;

public class PursuitTX extends CommandBase {
  private double error, turnPower; //angleChange is the change in angle needed for every run
  private BooleanAverager errorAverager;
  private boolean isEnd;

  public PursuitTX(boolean isEnd) {
    addRequirements(Robot.m_chassi);
    errorAverager = new BooleanAverager(30);
    this.isEnd = isEnd;
  }

  public PursuitTX() {
    this(true);
  }

  @Override
  public void initialize() {
    errorAverager.reset();
  }

  @Override
  public void execute() {
    error = Robot.m_limelight.getTx();
    turnPower = error * DriveConstants.TURN_KP;
    turnPower += error > 0 ? DriveConstants.TURN_AFF : -DriveConstants.TURN_AFF; 
    turnPower = RobotUtils.clip(turnPower, DriveConstants.MAX_TURN);

    Robot.m_chassi.arcadeDrive(OI.driverJoystick.getRawAxis(1)/2.0, turnPower);

    errorAverager.update(Math.abs(error) < DriveConstants.ANGLE_TOLERACE);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.m_chassi.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return isEnd ? errorAverager.getAverage() : false;
  }

  public boolean isReady(){
    return errorAverager.getAverage();
  }
}
