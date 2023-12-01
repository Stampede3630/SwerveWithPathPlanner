package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            1,
                                            1,
                                            new ReplanningConfig(),
                                            0.004),
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command applyNeutralModeBrake(){
        return runOnce(() -> {             
                SwerveModule[] modulesToApply = this.Modules;
                for (int i = 0; i < this.ModuleCount; ++i) {
                    modulesToApply[i].getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
                    modulesToApply[i].getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
                }
            }).ignoringDisable(true);
    }

    public Command waitThenApplyNeutralModeCoast(){
        return Commands.sequence(
            Commands.parallel(new WaitUntilCommand(5),
            new WaitUntilCommand(
                this::isChassisMoving
                )),
            this.runOnce(
                () -> {             
                    SwerveModule[] modulesToApply = this.Modules;
                    for (int i = 0; i < this.ModuleCount; ++i) {
                        modulesToApply[i].getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
                        modulesToApply[i].getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
                    }
                })).ignoringDisable(true).until(DriverStation::isEnabled).unless(()->getCurrentRobotChassisSpeeds().equals(null));
    }    

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public boolean isChassisMoving(){
        return Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < .25 &&
                      Math.abs(getCurrentRobotChassisSpeeds().vyMetersPerSecond) <.25;

    }
}
