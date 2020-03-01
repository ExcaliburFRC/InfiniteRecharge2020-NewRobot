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

public class DebugTransporter extends CommandBase {
  /**
   * Creates a new DebugTransporter.
   */
  public DebugTransporter() {
    addRequirements(Robot.m_transporter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var flickerSpeed = OI.armJoystick.getRawAxis(1);
    Robot.m_transporter.setFlickerMotorSpeed(flickerSpeed);
    SmartDashboard.putNumber("flickerSpeed_DEBUG", flickerSpeed);

    var loadSpeed = OI.driverJoystick.getRawAxis(1);
    Robot.m_transporter.setLoadingMotorSpeed(loadSpeed);
    SmartDashboard.putNumber("loadSpeed_DEBUG", loadSpeed);
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
