// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Monologue.LogNT;

public class Indexer extends SubsystemBase implements Logged{
  private final TalonFX bottomMotor = new TalonFX(Constants.IndexBottomMotorID);
  private final TalonFX topMotor = new TalonFX(Constants.IndexTopMotorID);
  private Trigger topLimitSwitchTrigger;  
  private final DigitalInput bottomLimitSwitch = new DigitalInput(Constants.BottomIntakeSwitchID);
  

  public Indexer(Trigger topLimitSwitchTrigger) {   
    bottomMotor.setNeutralMode(NeutralModeValue.Brake);
    topMotor.setNeutralMode(NeutralModeValue.Brake);  
    this.topLimitSwitchTrigger=topLimitSwitchTrigger;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command defaultSpinWhenNeeded(){
     return Commands.either(
      Commands.print("I'm default indexing").andThen(
        setTopBottomIndexer(-.7, -.8))
        .until(()->getTopLimitSwitch()),
        setTopBottomIndexer(0, 0),
        ()->getOnlyBottomSwitch()).repeatedly();
  }

  public Command setTopIndexer(double inputSpeed){
    return runOnce(()->topMotor.set(inputSpeed));
  }

  public Command setBottomIndexer(double inputSpeed){
    return runOnce(()->bottomMotor.set(inputSpeed));
  }

  public Command setTopBottomIndexer(double topSpeed, double bottomSpeed){
    return runOnce(()->{
      topMotor.set(topSpeed);
      bottomMotor.set(bottomSpeed);
    });
  }

  public Command cSetNone(){
    return Commands.none();
  }
  @LogNT
  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }
  @LogNT
  public boolean getTopLimitSwitch() {
    return topLimitSwitchTrigger.getAsBoolean();
  }
  @LogNT
  public boolean getBothSwitches() {
    return getTopLimitSwitch() && getBottomLimitSwitch();
  }
  @LogNT
  public boolean getOnlyBottomSwitch() {
    return !getTopLimitSwitch() && getBottomLimitSwitch();
  }

  public ParentDevice getTopParent(){
    return topMotor;
  }

  public ParentDevice getBottomParent(){
    return bottomMotor;
  }
}
