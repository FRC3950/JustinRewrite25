// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
  private final TalonFX indexer = new TalonFX(14, "CANivore");
  private final TalonFX intake = new TalonFX(41);
  private final TalonFX intake2 = new TalonFX(42);
  private DutyCycleOut intakeSpeed = new DutyCycleOut(Constants.intakeSpeed);
  private DutyCycleOut index = new DutyCycleOut(Constants.indexerSpeed);

  public Intake() {
    intake2.setControl(new Follower(intake.getDeviceID(), false));
  }

  public void startIntake(){
    intake.setControl(intakeSpeed);
    indexer.setControl(index);
  }

  public void reverse(){
    intake.setControl(intakeSpeed.withOutput(-Constants.intakeSpeed));
    indexer.setControl(index.withOutput(-Constants.indexerSpeed));
  }

  public void runIndexer(){
    indexer.setControl(index);
  }

  public void stopIntake(){
    intake.stopMotor();
    indexer.stopMotor();
  }

  public boolean hasNote() {
    return !indexer.getReverseLimit().getValue().equals(ReverseLimitValue.Open);
  }

  public Command intakeCommand() {
    return this.run(() -> startIntake())
    .finallyDo(interrupted -> stopIntake())
    .until(() -> hasNote());
  }

  public Command reverseCommand() {
    return this.run(() -> reverse())
    .finallyDo(interrupted -> stopIntake());
  }

  public Command indexer(){
    if (hasNote()){
      return this.run(() -> runIndexer())
      .finallyDo(interrupted -> stopIntake());
    } else {
      return Commands.none();
    }
  }
  
}
