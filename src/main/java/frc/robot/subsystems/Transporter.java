package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.BallDetector;
import frc.robot.util.BooleanAverager;
import frc.robot.util.ColorSensorBallDetector;

public class Transporter extends SubsystemBase {
    VictorSPX flicker, loading;
    boolean isAutoShoot;
    ColorSensorBallDetector shooterSwitch;
    BooleanAverager shooterSensorAverager;

    public Transporter(){
        loading = new VictorSPX(RobotMap.LOADING_MOTOR_PORT);
        loading.setInverted(true);
        flicker = new VictorSPX(RobotMap.FLICKER_MOTOR_PORT);
        flicker.setInverted(false);
        shooterSwitch = new ColorSensorBallDetector(Port.kOnboard, 1000);
        isAutoShoot = false;
        shooterSensorAverager = new BooleanAverager(10);
    }

    public void setLoadingMotorSpeed(double speed){
        loading.set(ControlMode.PercentOutput, speed);
    }

    public void setFlickerMotorSpeed(double speed){
        flicker.set(ControlMode.PercentOutput, speed);
    }

    public double getColorSensorDist(){
        return shooterSwitch.getDistance();
    }

    public boolean getIsAutoShoot(){
        return isAutoShoot;
    }

    public void setAutoShoot(boolean isAuto){
        isAutoShoot = isAuto;
    }


    public boolean getRawShooterSensor(){
        return shooterSwitch.isBallDetected();
    }

    public boolean isBallInShooter(){
        return shooterSensorAverager.getAverage();
    }

    @Override
    public void periodic() {
        shooterSensorAverager.update(getRawShooterSensor());

        SmartDashboard.putBoolean("isInShooter", isBallInShooter());
    }
}