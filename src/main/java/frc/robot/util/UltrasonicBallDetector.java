package frc.robot.util;

import edu.wpi.first.wpilibj.Ultrasonic;

public class UltrasonicBallDetector implements BallDetector{
    private Ultrasonic distanceSensor;
    private double MINIMUM_DISTANCE;

    public UltrasonicBallDetector(int ping, int echo){
        distanceSensor = new Ultrasonic(ping, echo);
        // MINIMUM_DISTANCE =  TransporterConstants.BALL_DETECTION_TOLERANCE;
    }

    public UltrasonicBallDetector(int ping, int echo, double minimum){
        distanceSensor = new Ultrasonic(ping, echo);
        MINIMUM_DISTANCE =  minimum;
    }

    public double getMeasuredDistance() {
        return distanceSensor.getRangeMM() / 10;
    }

    public void setAuto(boolean state){
        distanceSensor.setAutomaticMode(state);
    }

    @Override
    public boolean isBallDetected() {
        return getMeasuredDistance() < MINIMUM_DISTANCE;
    }
}