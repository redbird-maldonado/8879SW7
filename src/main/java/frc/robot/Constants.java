package frc.robot;

public class Constants {
    public static final double tau = 2.0 * Math.PI;

    public static final double kElevatorGearRatio = 0.05; // 20:1 gear ratio on the two elevator motors
    public static final double kElevatorPositionFactor = kElevatorGearRatio * tau;
    public static final double kElevatorVelocityFactor = kElevatorPositionFactor / 60.0;

    public static final double kWristGearRatio = 0.0277; // 20:1 gear ratio on the wrist drive now its 36 to 1
    public static final double kWristPositionFactor = kWristGearRatio * tau;
    public static final double kWristVelocityFactor = kWristPositionFactor / 60.0;

    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -.5;
    public static final double CLIMBER_SPEED_UP = .5;

    public static final double PROCESSOR_HEIGHT = 0;
    public static final double SOURCE_HEIGHT = 4.5;
    public static final double L0_HEIGHT = -0.5;
    public static final double L1_HEIGHT = 4.05; // 5.25
    public static final double L2_HEIGHT = 12.35; // 14.5
    public static final double L3_HEIGHT = 25.1; //18
    
    public static final double L1_HEIGHT_ALGAE = 10.15; // 9.25
    public static final double L2_HEIGHT_ALGAE = 19; // 17
    
    public static final double ZERO_ANGLE = 0;

    public static final double ARM_HEIGHT = 0.6; // NEED TO SET
    public static final double CLIMB_HEIGHT = 10; // NEED TO SET

    public static final double PROCESSOR_ANGLE = 0.5; // TEST VALUE FOR THE WRIST (1 was about 50°)///////////////////
    public static final double SOURCE_ANGLE = 1.2;
    public static final double L0_ANGLE = 2.65;    // 1.56
    public static final double L1_ANGLE = 2.55;    // 1.85
    public static final double L2_ANGLE = 2.55; //1.85
    public static final double L3_ANGLE = 2.45; //.5
    public static final double TOP_ALGAE_ANGLE = 0;
}
