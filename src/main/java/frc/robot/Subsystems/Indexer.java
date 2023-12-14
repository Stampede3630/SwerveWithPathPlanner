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
  @LogNT
  boolean lockIndexer = false;
  @LogNT
  boolean indexNow = false;
  

  public Indexer(Trigger topLimitSwitchTrigger) {   
    bottomMotor.setNeutralMode(NeutralModeValue.Brake);
    topMotor.setNeutralMode(NeutralModeValue.Brake);  
    this.topLimitSwitchTrigger=topLimitSwitchTrigger;
  }

  @Override
  public void periodic() {
    if(lockIndexer && !getTopLimitSwitch()){
      lockIndexer = false;
    } else if(!lockIndexer && getTopLimitSwitch()) {
      lockIndexer = true;
      indexNow = false;
    }

    if(getOnlyBottomSwitch()){
      indexNow = true;
    }
  }

  public Command defaultSpinWhenNeeded(){
     return 
        setTopBottomIndexer(-.1, -.1)
        .onlyIf(()->!lockIndexer && indexNow)
        .until(()->lockIndexer);
  }

  public Command setTopIndexer(double inputSpeed){
    return startEnd(()->topMotor.set(inputSpeed),()->topMotor.set(0));
  }

  public Command setBottomIndexer(double inputSpeed){
    return startEnd(()->bottomMotor.set(inputSpeed),()->bottomMotor.set(0));
  }

  public Command setTopBottomIndexer(double topSpeed, double bottomSpeed){
    return startEnd(()->{bottomMotor.set(bottomSpeed); topMotor.set(topSpeed);},
    ()->{bottomMotor.set(0); topMotor.set(0);});
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
