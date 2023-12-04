// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final TalonFX bottomMotor = new TalonFX(Constants.IndexBottomMotorID);
  private final TalonFX indexTop = new TalonFX(Constants.IndexTopMotorID);
  private Trigger topLimitSwitchTrigger;  
  private final DigitalInput bottomLimitSwitch = new DigitalInput(Constants.BottomIntakeSwitchID);

  public Indexer(Trigger topLimitSwitchTrigger) {   
    bottomMotor.setNeutralMode(NeutralModeValue.Brake);
    indexTop.setNeutralMode(NeutralModeValue.Brake);  
    this.topLimitSwitchTrigger=topLimitSwitchTrigger;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command cDefault(){
     return Commands.either(
        setTopBottomIndexer(-.7, -.8)
        .deadlineWith(Commands.waitSeconds(4))
        .until(()->getTopLimitSwitch()),
        setTopBottomIndexer(0, 0),
        ()->getOnlyBottomSwitch());
  }

  public Command setTopIndexer(double inputSpeed){
    return run(()->indexTop.set(inputSpeed));
  }

  public Command setBottomIndexer(double inputSpeed){
    return run(()->bottomMotor.set(inputSpeed));
  }

  public Command setTopBottomIndexer(double topSpeed, double bottomSpeed){
    return run(()->{
      indexTop.set(topSpeed);
      bottomMotor.set(bottomSpeed);
    });
  }

  public Command cSetNone(){
    return Commands.none();
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitchTrigger.getAsBoolean();
  }
  public boolean getBothSwitches() {
    return getTopLimitSwitch() && bottomLimitSwitch.get();
  }

  public boolean getOnlyBottomSwitch() {
    return !getTopLimitSwitch() && bottomLimitSwitch.get();
  }
}
