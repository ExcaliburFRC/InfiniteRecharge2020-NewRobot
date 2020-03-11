/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private DoubleSolenoid hanger;
  private TalonSRX robotLifterMotor1, robotLifterMotor2;
  private boolean isOn;

  public Climber() {
    isOn = false;
    hanger = new DoubleSolenoid(RobotMap.HANGER_PORT1, RobotMap.HANGER_PORT2);
    robotLifterMotor1 = new TalonSRX(RobotMap.ROBOT_LIFTER_MOTOR1_PORT);
    robotLifterMotor1.setInverted(true);
    robotLifterMotor2 = new TalonSRX(RobotMap.ROBOT_LIFTER_MOTOR2_PORT);
  }

  public void setHangerState(boolean open){
    hanger.set(open? kForward : kReverse);
  }

  public void setRobotClimbersPower(double power){
    robotLifterMotor1.set(ControlMode.PercentOutput,power);
    robotLifterMotor2.set(ControlMode.PercentOutput,power);
  }
  public void setIsOn(boolean isOn){
    this.isOn = isOn;
  }
  public boolean getIsOn(){
    return isOn;
  }

  public void test(int motor){
    if(motor == 1){
      robotLifterMotor1.set(ControlMode.PercentOutput, 0.4);
    }else if(motor == 2){
      robotLifterMotor2.set(ControlMode.PercentOutput, 0.4);
    }else {
      robotLifterMotor1.set(ControlMode.PercentOutput, 0);
      robotLifterMotor2.set(ControlMode.PercentOutput, 0);
    }
  }
}