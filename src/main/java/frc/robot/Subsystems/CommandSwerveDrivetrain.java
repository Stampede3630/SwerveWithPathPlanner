package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.ParentDevice;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.networktables.TopicInfo;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final Translation2d[] swerveModuleLocationArray = this.m_moduleLocations;
    private final DoubleArrayPublisher swerveModuleLocationArrayPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("/SwerveDrive Base/wheelLocations").publish(PubSubOption.periodic(.1));
    private final DoubleArrayPublisher swerveModuleStateArrayPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("/SwerveDrive Base/measuredStates").publish(PubSubOption.periodic(.1));
    private final DoublePublisher  moduleCountPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive Base/moduleCount").publish(PubSubOption.periodic(.1));
    private final DoublePublisher  robotRotationPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive Base/robotRotation").publish(PubSubOption.periodic(.1));
    private final DoublePublisher  sizeLeftRightPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive Base/sizeLeftRight").publish(PubSubOption.periodic(.1));
    private final DoublePublisher  sizeFrontBackPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive Base/sizeFrontBack").publish(PubSubOption.periodic(.1));
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        sendToElasticDashboard();
        SmartDashboard.putData(rezeroGyro());
        initFWCSwerveDrive();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        sendToElasticDashboard();
        SmartDashboard.putData(rezeroGyro());
        initFWCSwerveDrive();
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
        return run(() -> {this.setControl(requestSupplier.get());
        updateFWCSwerveDrive();});
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

    public Command rezeroGyro(){
        return runOnce(()->this.m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, new Pose2d()))
            .ignoringDisable(true)
            .withName("reZero Gyro");
    }

    public void initFWCSwerveDrive(){
        moduleCountPublisher.set(this.ModuleCount);
        sizeFrontBackPublisher.set(Math.abs(this.m_moduleLocations[0].getX()*2));
        sizeLeftRightPublisher.set(Math.abs(this.m_moduleLocations[0].getY()*2));
        double[] myDoubleArray = new double[this.ModuleCount*2];
        for (int i=0; i<this.ModuleCount; i++) {
            myDoubleArray[2*i] = this.m_moduleLocations[i].getX();
            myDoubleArray[2*i+1] = this.m_moduleLocations[i].getY();
        }
        swerveModuleLocationArrayPublisher.set(myDoubleArray);
    }

    public void updateFWCSwerveDrive(){
        robotRotationPublisher.set(this.m_odometry.getEstimatedPosition().getRotation().getDegrees());

        double[] myMeasuredStatesArray = new double[this.ModuleCount*2];
        double[] myDesiredStatesArray = new double[this.ModuleCount*2];
        ;
        SwerveDriveState SDstate = this.getState();
        if (SDstate.ModuleStates != null) {
            
        
        for (int i=0; i<this.ModuleCount; i++) {
            myMeasuredStatesArray[2*i] = SDstate.ModuleStates[i].angle.getCos()*SDstate.ModuleStates[i].speedMetersPerSecond;
            myMeasuredStatesArray[2*i+1] = SDstate.ModuleStates[i].angle.getSin()*SDstate.ModuleStates[i].speedMetersPerSecond;
        }
        swerveModuleStateArrayPublisher.set(myMeasuredStatesArray);
        }
    }

    

    /**
     * @param module 0-i
     * @param type 0 is drive, n is steer
     * @return ParentDevice
     */
    public ParentDevice getSwerveParent(int module, int type){
        SwerveModule[] modulesToApply = this.Modules;
        if (type==0) {
            return modulesToApply[module].getDriveMotor();
        } else {
            return modulesToApply[module].getSteerMotor();
        }
    }

    public void sendToElasticDashboard(){
        SwerveModule[] module = this.Modules;

        SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> module[0].getCurrentState().angle.getDegrees(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> module[0].getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Front Right Angle", () -> module[1].getCurrentState().angle.getDegrees(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> module[1].getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Left Angle", () -> module[2].getCurrentState().angle.getDegrees(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> module[2].getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Right Angle", () -> module[3].getCurrentState().angle.getDegrees(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> module[3].getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle",()-> m_odometry.getEstimatedPosition().getRotation().getDegrees(), null);
        }
        });
    }
}
