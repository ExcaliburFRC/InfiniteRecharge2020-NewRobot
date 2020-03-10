/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.collector.CollectorDrive;
import frc.robot.commands.leds.DefaultLED;
import frc.robot.commands.transporter.TransporterDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDMode;

public class Robot extends TimedRobot {

  public static Chassi m_chassi;
  public static Climber m_climber;
  public static Collector m_collector;
  public static Limelight m_limelight;
  public static Shooter m_shooter;
  public static Transporter m_transporter;
  public static LEDs m_leds;
  private static boolean isEnabled;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chassi = new Chassi();
    m_climber = new Climber();
    m_collector = new Collector();
    // m_limelight = new Limelight();
    m_shooter = new Shooter(true, false);
    m_transporter = new Transporter();
    m_leds = new LEDs();
    
    OI.init();
    initDefaultCommands();
    initSystemsStates();
  }
  public static boolean enabled() {
    return isEnabled;
  }

  @Override
  public void robotPeriodic() {
    isEnabled = isEnabled();
  }

  @Override
  public void disabledInit() {
    m_shooter.releaseVelocityPID();
  }

  @Override
  public void autonomousInit() {
    initSystemsStates();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    initSystemsStates();
    m_chassi.tankDrive(0, 0);
  }

  @Override
  public void teleopPeriodic(){
    CommandScheduler.getInstance().run();
  }
  
  private void initDefaultCommands(){
    m_chassi.setDefaultCommand(new RunCommand(()->{
     m_chassi.arcadeDrive(-1*OI.driverJoystick.getRawAxis(1), OI.driverJoystick.getRawAxis(2));
    }, m_chassi));

    m_leds.setDefaultCommand(new DefaultLED());

    m_collector.setDefaultCommand(new CollectorDrive());

    m_transporter.setDefaultCommand(new TransporterDrive());

    // m_shooter.setDefaultCommand(new ShooterSetup(()->4500));

    // m_climber.setDefaultCommand(new ClimberDrive());
  }

  private void initSystemsStates(){
    CommandScheduler.getInstance().cancelAll();
    m_chassi.setCompressorMode(false);

    m_shooter.releaseVelocityPID();

    m_collector.setLifterPistonPosition(false);
    
    m_climber.setHangerState(false);

    m_leds.setMode(LEDMode.RAINBOW);
  }
  
  @Override
  public void testPeriodic() {
    

    testChassi();
  }
  void testChassi(){
    if(OI.armJoystick.getRawButton(3)){
      Robot.m_chassi.test(1);
    }else if(OI.armJoystick.getRawButton(4)){
      Robot.m_chassi.test(2);
    }else if(OI.armJoystick.getRawButton(5)){
      Robot.m_chassi.test(3);
    }else if(OI.armJoystick.getRawButton(6)){
      Robot.m_chassi.test(4);
      }else  Robot.m_chassi.test(0);
  }

  void testClimb(){
    if(OI.armJoystick.getRawButton(10)){
      Robot.m_climber.setHangerState(false);
    }else if(OI.armJoystick.getRawButton(9)){
      Robot.m_climber.setHangerState(true);
    }

    if(OI.armJoystick.getRawButton(11)){
      Robot.m_climber.test(1);
    }else if(OI.armJoystick.getRawButton(12)){
      Robot.m_climber.test(2);
    }else Robot.m_climber.test(0);
  }
}