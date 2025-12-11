// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Intake extends SubsystemBase {
  private final TalonFX indexer = new TalonFX(14, "CANivore");
  private final TalonFX intake = new TalonFX(41);
  private final TalonFX intake2 = new TalonFX(42);

  public Intake() {
    intake2.setControl(new Follower(intake.getDeviceID(), false));
  }

  public void startIntake() {
    intake.set(Constants.intakeSpeed);
    indexer.set(Constants.indexerSpeed);
  }

  public void reverse() {
    intake.set(-Constants.intakeSpeed);
    indexer.set(-Constants.indexerSpeed);
  }

  public void runIndexer() {
    indexer.set(Constants.indexerSpeedAct);
  }

  public void stopIntake() {
    intake.set(0);
    indexer.set(0);
  }

  public void runIndexerReverse() {
    indexer.set(-Constants.indexerSpeedAct);
  }

  public boolean hasNote() {
    return !indexer.getReverseLimit().getValue().equals(ReverseLimitValue.Open);
  }

  // Intakes until the condition of has a note.
  public Command intakeCommand() {
    return new FunctionalCommand(
        () -> {
        },
        () -> startIntake(),
        interrupted -> stopIntake(),
        () -> hasNote(),
        this);
  }

  public Command reverseCommand() {
    return Commands.startEnd(this::reverse, this::stopIntake, this);
  }

  public Command reverseIndexerCommand() {
    return Commands.startEnd(this::runIndexerReverse, this::stopIntake, this);
  }

  // Command instantly finishes if there isn't a note, as intended :)
  public Command indexer() {
    return new FunctionalCommand(
        () -> {
        },
        () -> runIndexer(),
        interrupted -> stopIntake(),
        () -> !hasNote(),
        this);
  }

  public TalonFX getIndexerMotor() {
    return indexer;
  }

  public TalonFX getIntakeMotor() {
    return intake;
  }

  public TalonFX getIntake2Motor() {
    return intake2;
  }
}