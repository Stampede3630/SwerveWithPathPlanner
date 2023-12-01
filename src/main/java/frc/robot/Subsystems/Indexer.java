// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private TalonFX indexBottom;
  private TalonFX indexTop;
  
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;
  /** Creates a new Indexer. */
  public Indexer() {
    indexBottom = new TalonFX(Constants.IndexBottomMotorID);
    indexTop = new TalonFX(Constants.IndexTopMotorID);
    indexBottom.setNeutralMode(NeutralModeValue.Brake);
    indexTop.setNeutralMode(NeutralModeValue.Brake);   
    bottomLimitSwitch = new DigitalInput(Constants.BottomIntakeSwitchID);
    topLimitSwitch = new DigitalInput(Constants.TopIntakeSwitchID); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command indexersDefaultOff(){
    return run(()->{
      indexBottom.set(0);
      indexTop.set(0);
      });
  }

  public Command setTopIndexer(double inputSpeed){
    return run(()->indexTop.set(inputSpeed));
  }

  public Command setBottomIndexer(double inputSpeed){
    return run(()->indexBottom.set(inputSpeed));
  }

  public Command setTopBottomIndexer(double topSpeed, double bottomSpeed){
    return run(()->{
      indexTop.set(topSpeed);
      indexBottom.set(bottomSpeed);
    });
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }
    public boolean getBothSwitches() {
    return topLimitSwitch.get() && bottomLimitSwitch.get();
  }
}
