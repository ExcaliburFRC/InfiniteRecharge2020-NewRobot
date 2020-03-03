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
import frc.robot.commands.debug.DebugShooter;
import frc.robot.commands.leds.DefaultLED;
import frc.robot.commands.transporter.TransporterDrive;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  public static Chassi m_chassi;
  public static Climber m_climber;
  public static Collector m_collector;
  public static Limelight m_limelight;
  public static Shooter m_shooter;
  public static Transporter m_transporter;
  public static LEDs m_leds;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chassi = new Chassi();
    m_climber = new Climber();
    m_collector = new Collector();
    m_limelight = new Limelight();
    m_shooter = new Shooter(true);
    m_transporter = new Transporter();
    m_leds = new LEDs();

    OI.init();
    initDefaultCommands();
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    m_chassi.tankDrive(0, 0);
  }

  @Override
  public void teleopPeriodic(){
    CommandScheduler.getInstance().run();
  }
  
  private void initDefaultCommands(){
    m_chassi.setDefaultCommand(new RunCommand(()->{
     m_chassi.arcadeDrive(-OI.driverJoystick.getRawAxis(OI.xSpeedAxis), OI.driverJoystick.getRawAxis(OI.zRotationAxis));
    }, m_chassi));

    m_leds.setDefaultCommand(new DefaultLED());

    m_collector.setDefaultCommand(new CollectorDrive());

    m_transporter.setDefaultCommand(new TransporterDrive());

    m_shooter.setDefaultCommand(new DebugShooter());
  }
}