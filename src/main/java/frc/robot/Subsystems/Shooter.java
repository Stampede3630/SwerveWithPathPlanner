// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Monologue.LogNT;

public class Shooter extends SubsystemBase implements Logged {
  private final TalonFX hoodMotor = new TalonFX(Constants.HoodMotorID, "rio");
  private final TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive))
    .withSlot0(new Slot0Configs()
      .withKP(.1));
  private final PositionDutyCycle hoodPositionDrive = new PositionDutyCycle(0, 0, false, 0, 0, true);
    
  private final TalonFX shooterMotor = new TalonFX(Constants.ShooterMotorID, "rio");
  private TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive))
    .withSlot0(new Slot0Configs()
      .withKP(.11)     // An error of 1 rotation per second results in 5 amps output
      .withKI(.5)    // An error of 1 rotation per second increases output by 0.1 amps every second
      .withKD(.0001)
      .withKV(.12)); // A change of 1000 rotation per second squared results in 1 amp output
  /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
  private final VelocityVoltage shooterVelocityDrive = new VelocityVoltage(0, 0, false, 0, 0,false);
  private final DigitalInput leftLimitSwitch = new DigitalInput(Constants.LeftHoodSwitchID);
  private DigitalInput rightLimitSwitch = new DigitalInput(Constants.RightHoodSwitchID);
  /** Creates a new Shooter. */
  public Shooter() {
    applyTalonFXconfig(hoodMotor, hoodConfiguration);
    applyTalonFXconfig(shooterMotor, shooterConfiguration);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command reZeroHood(){
    return run(()->hoodMotor.set(-.1))
            .until(()->(hoodMotor.getTorqueCurrent().getValue() > 20))
            .andThen(runOnce(()->hoodMotor.setPosition(-3)));
  }

  public Command driveHoodToPosition(DoubleSupplier inputPosition){
    if (inputPosition.getAsDouble()>0) {
      return runOnce(()->hoodMotor.setControl(hoodPositionDrive.withPosition(inputPosition.getAsDouble())));
    }else{
      return runOnce(()->hoodMotor.setControl(hoodPositionDrive.withPosition(0)));
    }
  }

  public Command setShooterRPS(DoubleSupplier shooterRPS){
    return runOnce(()->shooterMotor.setControl(shooterVelocityDrive.withVelocity(shooterRPS.getAsDouble())));
  }

  public Command cCoastShooter(){
    return runOnce(()->shooterMotor.set(0));
  }

  @LogNT
  public boolean getShooterAtSpeed(){
    return shooterMotor.getClosedLoopReference().getValueAsDouble()*0.95 <= shooterMotor.getVelocity().getValueAsDouble();
  }

  public boolean checkHoodLimitSwitches(){
    return leftLimitSwitch.get() || rightLimitSwitch.get();
  }

  public ParentDevice getHoodParent(){
    return hoodMotor;
  }

  public ParentDevice getShooterParent(){
    return shooterMotor;
  }

  private void applyTalonFXconfig(TalonFX myTalonfx, TalonFXConfiguration myConfig){
          /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = myTalonfx.getConfigurator().apply(myConfig);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

}
