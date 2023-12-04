// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.TunerConstants;
import monologue.Logged;

public class RobotContainer implements Logged {
  final DigitalInput topLimitSwitch = new DigitalInput(Constants.TopIntakeSwitchID);
  final Trigger topLimitSwitchTrigger = new Trigger(()->!topLimitSwitch.get());
  final Intake intake = new Intake();
  final Indexer indexer = new Indexer(topLimitSwitchTrigger);
  final Shooter shooter = new Shooter();

  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = 2 * Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  final SwerveRequest.FieldCentric requestFODrive = new SwerveRequest
                                          .FieldCentric()
                                          .withIsOpenLoop(true)
                                          .withDeadband(MaxSpeed * 0.1)
                                          .withRotationalDeadband(MaxAngularRate * 0.1); // I want field-centric
                                                                                            // driving in open loop
  final SwerveRequest.SwerveDriveBrake requestSetBrake = new SwerveRequest.SwerveDriveBrake();
  final SwerveRequest.RobotCentric requestRoboCentric = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
  final SwerveRequest.PointWheelsAt requestPointWheelsAt = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  // Command runAuto = drivetrain.getAutoPath("Tests");

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  
  public RobotContainer() {
    configureBindings();
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
          () -> requestFODrive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed)             // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)             // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));                   // Calculates requests during disabled
    intake.setDefaultCommand(intake.cDefault());
    indexer.setDefaultCommand(indexer.cDefault());
    
    shooter.setDefaultCommand(Commands.either(
      shooter.setShooterRPS(()->40),
      shooter.cCoastShooter(),
      topLimitSwitchTrigger).repeatedly());

    //REVERSEINTAKE
    joystick.b().debounce(.2)
      .whileTrue(
      Commands.parallel(
        intake.cExtendAndSetSpeed(-.8),
        indexer.setTopBottomIndexer(.8, .7))
        );

    //INTAKE
    joystick.rightTrigger(.5).debounce(.2)
      .whileTrue(
        Commands.parallel(
          intake.cExtendAndSetSpeed(.8),
          indexer.setTopBottomIndexer(-.35, -.35).until(topLimitSwitchTrigger)
          )
      );

    //SHOOT
    joystick.leftTrigger(.5).debounce(.2)
      .whileTrue(
        Commands.sequence(
          Commands.print("I'm shooting!"),
          shooter.setShooterRPS(()->{return 60;}),

          Commands.either(
            indexer.setTopBottomIndexer(-.7, -.8),
            indexer.setTopBottomIndexer(0, 0),
            shooter::getShooterAtSpeed)).repeatedly()
          )
      .whileFalse(shooter.cCoastShooter());
   
    joystick.x().debounce(.2, DebounceType.kFalling).whileTrue(drivetrain
        .applyRequest(() -> requestPointWheelsAt.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);
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
