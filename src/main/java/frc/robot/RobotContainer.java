// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = 2 * Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric requestFODrive = new SwerveRequest
                                          .FieldCentric()
                                          .withIsOpenLoop(true)
                                          .withDeadband(MaxSpeed * 0.1)
                                          .withRotationalDeadband(MaxAngularRate * 0.1); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake requestSetBrake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric requestRoboCentric = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
  SwerveRequest.PointWheelsAt requestPointWheelsAt = new SwerveRequest.PointWheelsAt();

  Intake intake = new Intake();
  Indexer indexer = new Indexer();
  
  /* Path follower */
  // Command runAuto = drivetrain.getAutoPath("Tests");

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
          () -> requestFODrive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed)             // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)             // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));                   // Calculates requests during disabled
    intake.setDefaultCommand(intake.intakeDefaultOff());
    indexer.setDefaultCommand(indexer.indexersDefaultOff());

    //REVERSEINTAKE
    joystick.b().debounce(.2, DebounceType.kFalling)
      .whileTrue(
      Commands.parallel(
          intake.setIntakeSpeed(-.8).andThen(intake.extendIntake()),
        indexer.setTopBottomIndexer(-.8, -.7))
        );

    //INTAKE
    joystick.rightTrigger(.5).debounce(.2,DebounceType.kFalling)
    .onTrue(
      Commands.parallel(
        intake.setIntakeSpeed(.8).andThen(intake.extendIntake()),
        indexer.setTopBottomIndexer(-.35, -.35).until(indexer::getTopLimitSwitch)
        )
    )
    .onFalse(
        intake.intakeDefaultOff()
        .andThen(Commands.waitSeconds(.2))
        .andThen(indexer.indexersDefaultOff()));


    
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> requestPointWheelsAt.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);
 }

  public RobotContainer() {
    configureBindings();
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return Commands.runOnce(() -> {});
  }

  public Command getDisabledExitSwerveInitCommand() {
    return Commands.sequence(
      drivetrain.applyRequest(() -> requestSetBrake));
  }




}
