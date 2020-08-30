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
    double shootSpeed = OI.armJoystick.getRawAxis(1);
    // double shootSpeed = SmartDashboard.getNumber("target", 0);

    
    if (OI.armJoystick.getRawAxis(3) > 0){ //this is always false - for debugging purposes
      Robot.m_shooter.setVelocityPID(0/*-Math.abs(SmartDashboard.getNumber("target", 0))*/);
    } else {
      Robot.m_shooter.releaseVelocityPID();
      Robot.m_shooter.setAbsoluteShooterMotorPower(shootSpeed);
    }

    SmartDashboard.putNumber("SHOOT_SPEED", Robot.m_shooter.getShooterMotorVelocity());
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
