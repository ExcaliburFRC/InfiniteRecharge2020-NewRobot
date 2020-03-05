/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber extends SubsystemBase {
  private DoubleSolenoid hanger;
  private Spark robotLifterMotor1, robotLifterMotor2;
  private boolean isOn;

  public Climber() {
    isOn = false;
    hanger = new DoubleSolenoid(RobotMap.HANGER_PORT1, RobotMap.HANGER_PORT2);
    robotLifterMotor1 = new Spark(RobotMap.ROBOT_LIFTER_MOTOR_PORT1);
    robotLifterMotor2 = new Spark(RobotMap.ROBOT_LIFTER_MOTOR_PORT2);
  }

  public void setHangerState(boolean open){
    hanger.set(open? kForward : kReverse);
  }

  public void setRobotClimbersPower(double power){
    robotLifterMotor1.set(power);
    robotLifterMotor2.set(power);
  }
  public void setIsOn(boolean isOn){
    this.isOn = isOn;
  }
  public boolean getIsOn(){
    return isOn;
  }
}