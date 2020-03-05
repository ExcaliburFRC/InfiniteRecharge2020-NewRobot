/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class ClimberDrive extends CommandBase {

  public ClimberDrive() {
    addRequirements(Robot.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_climber.setIsOn(true);
    // Robot.m_leds.setMode(LEDMode.YELLOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OI.driverJoystick.getRawButton(3)){
      Robot.m_climber.setHangerState(true);
    }else if(OI.driverJoystick.getRawButton(2)){
      Robot.m_climber.setHangerState(false);
    }

    if (OI.driverJoystick.getRawButton(6)){ //forward
      Robot.m_climber.setRobotClimbersPower(0.9);
    } else if (OI.driverJoystick.getRawButton(5)){ //backwards
      Robot.m_climber.setRobotClimbersPower(-0.9);
    } else {
      Robot.m_climber.setRobotClimbersPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_climber.setIsOn(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
