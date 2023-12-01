package frc.robot;

public class Constants {
    public static final double XBOXDEADBAND = .1;
    
    // Index and Intake IDs
    public static final int IndexBottomMotorID = 6;
    public static final int IndexTopMotorID = 9;
    public static final int IntakeMotorID = 7;
    public static final int IntakeSolenoidForwardID = 4;
    public static final int IntakeSolenoidReverseID = 5;
    public static final int BottomIntakeSwitchID = 0;
    public static final int TopIntakeSwitchID = 1;

    // Shooter IDs
    public static final int ShooterMotorID = 10;
    public static final int HoodMotorID = 49; // <--- Evan's fault for this weird number
    public static final int LeftHoodSwitchID = 4;
    public static final int RightHoodSwitchID = 5;

    // Climber IDs
    public static final int ClimberMotorId = 8;
    public static final int ClimberSolenoidForwardID = 2;
    public static final int ClimberSolenoidReverseID = 3;
    public static final int LeftClimberSwitchID = 2;
    public static final int RightClimberSwitchID = 3;
}
