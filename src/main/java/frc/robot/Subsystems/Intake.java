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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public TalonFX intakeDrive;
  public TalonFX indexBottom;
  public TalonFX indexTop;
  public DoubleSolenoid intakeSolenoid;
  public DoubleSolenoid limelightSolenoid;

  public DigitalInput bottomLimitSwitch;
  public DigitalInput topLimitSwitch;

  TalonFXConfiguration intakeDriveConfig = new TalonFXConfiguration();
  
  /** Creates a new Intake. */
  public Intake() {
    indexBottom = new TalonFX(Constants.IndexBottomMotorID);
    indexTop = new TalonFX(Constants.IndexTopMotorID);
    indexBottom.setNeutralMode(NeutralModeValue.Brake);
    indexTop.setNeutralMode(NeutralModeValue.Brake);
    

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeSolenoidForwardID, Constants.IntakeSolenoidReverseID);

    bottomLimitSwitch = new DigitalInput(Constants.BottomIntakeSwitchID);
    topLimitSwitch = new DigitalInput(Constants.TopIntakeSwitchID);



    intakeDrive = new TalonFX(Constants.IntakeMotorID);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();

  }

  public void intakeDefaultOff() {
    intakeDrive.set(0);
    intakeSolenoid.set(Value.kReverse);
  }
}





