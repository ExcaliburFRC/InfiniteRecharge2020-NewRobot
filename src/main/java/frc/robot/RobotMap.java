package frc.robot;

public class RobotMap{
    //Chassi
    public static final int LEFT_BACK_MOTOR_PORT = 14;
    public static final int LEFT_FRONT_MOTOR_PORT = 13;
    public static final int RIGHT_BACK_MOTOR_PORT = 11;
    public static final int RIGHT_FRONT_MOTOR_PORT = 12;
    public static final int[] RIGHT_ENCODER_P = {12,13};
    public static final int[] LEFT_ENCODER_P = {19,20};

    //LED
    public static final int LED_PWM_PORT = 4;

    //Collector
    public static final int ROLLER_MOTOR_PORT = 3;
    public static final int[] LIFTER_PORTS = {2, 3};

    //Shooter
    public static final int LEFT_SHOOTER_MOTOR_PORT = 41;
    public static final int RIGHT_SHOOTER_MOTOR_PORT = 42;

    //Transporter
    public static final int FLICKER_MOTOR_PORT = 33;
    public static final int LOADING_MOTOR_PORT = 34;
    
    // Climber ports
    public static final int CLIMBER_LIFTER_MOTOR_PORT = 2;
    public static final int ROBOT_LIFTER_MOTOR_PORT1 = 1;
    public static final int HEIGHT_ENCODER_PORT1 = 21;
    public static final int HEIGHT_ENCODER_PORT2 = 22;   

}