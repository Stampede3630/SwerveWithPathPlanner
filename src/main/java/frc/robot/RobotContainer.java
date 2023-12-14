// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  final TalonFX disabledClimber = new TalonFX(Constants.ClimberMotorId, "rio");
  final DigitalInput topLimitSwitch = new DigitalInput(Constants.TopIntakeSwitchID);
  final Trigger topLimitSwitchTrigger = new Trigger(()->!topLimitSwitch.get());
  final Intake intake = new Intake();
  final Indexer indexer = new Indexer(topLimitSwitchTrigger);
  final Shooter shooter = new Shooter();
  //final PowerDistribution RevPDH = new PowerDistribution(1, ModuleType.kRev);
  final DoubleSubscriber dashboardIntakeSpeed = NetworkTableInstance.getDefault().getTable("Robot").getDoubleTopic("IntakeSpeed").subscribe(Constants.DefaultIntakeSpeed);
  final DoubleSubscriber dashboardReverseIntakeSpeed = NetworkTableInstance.getDefault().getTable("Robot").getDoubleTopic("ReverseIntakeSpeed").subscribe(Constants.DefaultReverseIntakeSpeed);
  

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
  final Orchestra THEOrchestra = new Orchestra();
  public final String[] orchestraPlayList = {"MarioMain.chrp", "MarioStar.chrp", "MarioUnder.chrp", "MarioWater.chrp" ,"Mario2.chrp", "Mario3.chrp","SNESMario.chrp", "SNESMarioYosh.chrp", "Zelda.chrp"};
  int currentSong = 0;
    

  
  public RobotContainer() {
    configureBindings();
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
    doubleSubPublisher(dashboardIntakeSpeed, Constants.DefaultIntakeSpeed);
    doubleSubPublisher(dashboardReverseIntakeSpeed, Constants.DefaultReverseIntakeSpeed);
    SmartDashboard.putData("Match Time", new Sendable() {
    @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Match Time");
            builder.addDoubleProperty("time left", ()->DriverStation.getMatchTime(), null);
        }
      });
    //SmartDashboard.putData(RevPDH);
    
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
          () -> requestFODrive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed)             // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)             // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(false).withName("DefaultDrive"));                   // Calculates requests during disabled
    intake.setDefaultCommand(intake.defaultRetractAndStop().withName("DefaultIntake"));
    indexer.setDefaultCommand(indexer.defaultSpinWhenNeeded().withName("DefaultIndexer"));
    
    shooter.setDefaultCommand(
      shooter.setShooterRPS(()->40).onlyWhile(topLimitSwitchTrigger).withName("DefaultShooter").repeatedly());

    //REVERSEINTAKE
    joystick.rightBumper().debounce(.1)
      .whileTrue(
      Commands.parallel(
        intake.cExtendAndSetSpeed(dashboardReverseIntakeSpeed),
        indexer.setTopBottomIndexer(.4, .4)).withName("RightBumperCommand")
      );

    //INTAKE
    joystick.rightTrigger(.5).debounce(.1)
      .whileTrue(
        Commands.parallel(
          intake.cExtendAndSetSpeed(dashboardIntakeSpeed),
          indexer.setBottomIndexer(-.35)
            .unless(topLimitSwitchTrigger)
            .until(topLimitSwitchTrigger)
          ).withName("RightTriggerCommand")
      );

    //SHOOT
    joystick.leftTrigger(.5).debounce(.1)
      .whileTrue(
        Commands.sequence(
          Commands.print("I'm shooting!"),
          shooter.setShooterRPS(()->60),

          Commands.either(
            indexer.setTopBottomIndexer(-.5, -.5),
            indexer.setTopBottomIndexer(0, 0),
            shooter::getShooterAtSpeed)).repeatedly().withName("LeftTriggerCommand")
          );
    
    //PLAY NEXT SONG
    joystick.a().debounce(.1)
      .onTrue(
        Commands.parallel(
          Commands.runOnce(()->nextTrack()),
          Commands.idle(
            drivetrain,
            shooter,
            indexer,
            intake
          ).until(joystick.b()).withName("A Button: Play Next Song").ignoringDisable(true)));
    
    //Pause/Play
    joystick.b().debounce(.1)
      .onTrue(Commands.either(
        Commands.runOnce(()->pauseOrchestra()),
        Commands.runOnce(()->playOrchestra()),
        ()->THEOrchestra.isPlaying()
      ).withName("B Command: Pause or play"));

    joystick.x().debounce(.1, DebounceType.kFalling).whileTrue(drivetrain
        .applyRequest(() -> requestPointWheelsAt.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))).withName("X Command:Point Wheels"));

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

  public void assembleOrchestra(){
    THEOrchestra.addInstrument(shooter.getHoodParent(), 0);
    THEOrchestra.addInstrument(shooter.getShooterParent(), 0);
    THEOrchestra.addInstrument(indexer.getBottomParent(), 1);
    THEOrchestra.addInstrument(indexer.getTopParent(), 1);
    THEOrchestra.addInstrument(intake.getIntakeParent(), 1);
    THEOrchestra.addInstrument(disabledClimber, 2);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(0, 0), 3);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(0, 1), 3);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(1, 0), 4);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(1, 1), 4);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(2, 0),5);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(2, 1), 6);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(3, 0), 9);
    THEOrchestra.addInstrument(drivetrain.getSwerveParent(3, 1), 9);
  }
  

  public void playOrchestra(){
    THEOrchestra.play();
  }

  public void pauseOrchestra(){
    THEOrchestra.pause();
  }

  public void nextTrack(){
    currentSong++;
    if (currentSong > orchestraPlayList.length-1) {
      currentSong = 0;
    }

    THEOrchestra.loadMusic(orchestraPlayList[currentSong]);
    playOrchestra();
  }

  public void doubleSubPublisher(DoubleSubscriber myDubSUb, double defValue){
    myDubSUb.getTopic().publish(PubSubOption.disableLocal(false)).setDefault(defValue);  
  }

}
