// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX hoodMotor;
  private TalonFX shooterMotor;
  private TalonFXConfiguration hoodConfiguration;
  private TalonFXConfiguration shooterConfiguration;

  private DigitalInput leftLimitSwitch;
  private DigitalInput rightLimitSwitch;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterConfiguration.
  }TalonFXConfiguration

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
