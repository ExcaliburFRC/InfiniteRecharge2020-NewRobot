package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.BooleanAverager;

public class Transporter extends SubsystemBase {
    VictorSPX flicker, loading;
    boolean isAutoShoot;
    DigitalInput shooterSwitch;
    BooleanAverager shooterSensorAverager;

    public Transporter(){
        loading = new VictorSPX(RobotMap.LOADING_MOTOR_PORT);
        flicker = new VictorSPX(RobotMap.FLICKER_MOTOR_PORT);
        shooterSwitch = new DigitalInput(RobotMap.SHOOTER_MICRO_SWITCH);
        isAutoShoot = false;
        shooterSensorAverager = new BooleanAverager(10);
    }

    public void setLoadingMotorSpeed(double speed){
        loading.set(ControlMode.PercentOutput, speed);
    }

    public void setFlickerMotorSpeed(double speed){
        flicker.set(ControlMode.PercentOutput, speed);
    }

    public boolean getIsAutoShoot(){
        return isAutoShoot;
    }

    public void setAutoShoot(boolean isAuto){
        isAutoShoot = isAuto;
    }

    public boolean getRawShooterSensor(){
        return shooterSwitch.get();
    }

    public boolean isBallInShooter(){
        return shooterSensorAverager.getAverage();
    }

    @Override
    public void periodic() {
        shooterSensorAverager.update(getRawShooterSensor());
    }
}