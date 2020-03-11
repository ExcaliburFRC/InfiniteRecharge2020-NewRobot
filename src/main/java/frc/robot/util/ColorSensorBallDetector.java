/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Add your docs here.
 */
public class ColorSensorBallDetector implements BallDetector{
    ColorSensorV3 sensor;
    double tolerance;

    public ColorSensorBallDetector(Port port, double tolerance){
        sensor = new ColorSensorV3(port);
        this.tolerance = tolerance;
    }

    @Override
    public boolean isBallDetected() {
        return getDistance() < tolerance;
    }
    
    public double getDistance(){
        return sensor.getProximity();
    }
}
