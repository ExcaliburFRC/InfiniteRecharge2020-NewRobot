/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterSetup extends CommandBase {
  private DoubleSupplier speedTarget;

  public ShooterSetup(DoubleSupplier speedTarget) {
    addRequirements(Robot.m_shooter);
    this.speedTarget = speedTarget;
  }

  @Override
  public void initialize() {
    // Robot.m_shooter.setVelocityPID(speedTarget.getAsDouble());
  }

  @Override
  public void execute() {
    Robot.m_shooter.setVelocityPID(speedTarget.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.m_shooter.releaseVelocityPID();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isReady(){
    return Robot.m_shooter.isOnSpeed();
  }
}
