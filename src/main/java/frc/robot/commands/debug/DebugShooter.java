/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.debug;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class DebugShooter extends CommandBase {
  /**
   * Creates a new ShooterDrive.
   */
  public DebugShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shootSpeed = OI.driverJoystick.getRawAxis(3);
    Robot.m_shooter.setShooterMotorPower(shootSpeed);
    SmartDashboard.putNumber("shoot_speed_debug", shootSpeed);
    
    SmartDashboard.putNumber("left_shooter_speed_debug", Robot.m_shooter.getLeftMotorSpeed());
    SmartDashboard.putNumber("right_shooter_speed_debug", Robot.m_shooter.getRightMotorSpeed());
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
