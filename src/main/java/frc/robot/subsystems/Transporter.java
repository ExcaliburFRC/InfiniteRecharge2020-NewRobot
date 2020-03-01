package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Transporter extends SubsystemBase {
    VictorSPX flicker, loading;
    public Transporter(){
        loading = new VictorSPX(RobotMap.LOADING_MOTOR_PORT);
        flicker = new VictorSPX(RobotMap.FLICKER_MOTOR_PORT);
    }
    public void setLoadingMotorSpeed(double speed){
        loading.set(ControlMode.PercentOutput, speed);
    }
    public void setFlickerMotorSpeed(double speed){
        flicker.set(ControlMode.PercentOutput, speed);
    }
}