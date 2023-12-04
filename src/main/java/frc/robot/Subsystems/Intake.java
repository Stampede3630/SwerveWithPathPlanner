// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX intakeDrive = new TalonFX(Constants.IntakeMotorID);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeSolenoidForwardID, Constants.IntakeSolenoidReverseID);
  
  /** Creates a new Intake. */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command cDefault() {
    return run(()->{
      intakeDrive.set(0);
      intakeSolenoid.set(Value.kReverse);
    });
  }

  public Command cExtendAndSetSpeed(double inputSpeed){
    return Commands.sequence(cExtend(),
    cSetIntakeSpeed(inputSpeed));
  }

  public Command cSetIntakeSpeed(double inputSpeed){
    return run(()->{
      intakeDrive.set(inputSpeed);
    });
  }

  public Command cExtend(){
    return runOnce(()->{
      intakeSolenoid.set(Value.kForward);
    });
  }
  
  public Command cSetNone(){
    return Commands.none();
  }
}





