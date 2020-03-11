/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.transporter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants.TransporterConstants;

public class TransporterDrive extends CommandBase {
  /**
   * Creates a new DebugTransport.
   */
  double timeSinceTop;
  boolean wasInTop;

  public TransporterDrive() {
    addRequirements(Robot.m_transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceTop = 0;
    wasInTop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var isOkToShoot = (Robot.m_transporter.getIsAutoShoot() || OI.armJoystick.getRawButton(1)) && Robot.m_shooter.isOnSpeed();
    // var isInNoReturnMode = (System.currentTimeMillis() - timeSinceTop) < TransporterConstants.NO_RETURN_TIME;
    SmartDashboard.putBoolean("transport_switch", Robot.m_transporter.getRawShooterSensor());
    
    // If you really want the transporter to work, regardless of the shooter speed readiness state
    // just uncomment this line:
    
    // isOkToShoot = false;

    if (OI.armJoystick.getRawButton(OI.collectorTakeInBallButton) && !Robot.m_transporter.getRawShooterSensor()){ //get the raw shooter sensor to get quicker feedback
      Robot.m_transporter.setFlickerMotorSpeed(TransporterConstants.MANUAL_SHOOT_FLICKER_SPEED);
      Robot.m_transporter.setLoadingMotorSpeed(TransporterConstants.MANUAL_SHOOT_LOAD_SPEED);
    } else if (isOkToShoot){
      Robot.m_transporter.setFlickerMotorSpeed(TransporterConstants.AUTO_SHOOT_FLICKER_SPEED);
      Robot.m_transporter.setLoadingMotorSpeed(TransporterConstants.AUTO_SHOOT_LOAD_SPEED);
    } else if  (OI.armJoystick.getRawButton(8)) {
      Robot.m_transporter.setFlickerMotorSpeed(-TransporterConstants.MANUAL_SHOOT_FLICKER_SPEED);
      Robot.m_transporter.setLoadingMotorSpeed(-TransporterConstants.MANUAL_SHOOT_LOAD_SPEED);
    } else {
      Robot.m_transporter.setFlickerMotorSpeed(0);
      Robot.m_transporter.setLoadingMotorSpeed(0);
    }

    // if (!wasInTop && Robot.m_transporter.isBallInShooter()){
    //   timeSinceTop = System.currentTimeMillis();
    // }

    wasInTop = Robot.m_transporter.isBallInShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
