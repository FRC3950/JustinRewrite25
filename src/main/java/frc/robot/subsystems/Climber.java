// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class Climber extends SubsystemBase {

  private final TalonFX climberL = new TalonFX(19);
  private final TalonFX climberR = new TalonFX(49);

  private final DutyCycleOut climberOut = new DutyCycleOut(Constants.climberSpeed);

  private static final double offset = 5; // needs tuning

  public Climber() {
    climberL.setNeutralMode(NeutralModeValue.Brake);
    climberR.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getPositionL() {
    return climberL.getPosition().getValueAsDouble();
  }

  public double getPositionR() {
    return climberR.getPosition().getValueAsDouble();
  }

  public boolean isBottomL() {
    return climberL.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
  }

  public boolean isBottomR() {
    return climberR.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
  }

  public void stop() {
    climberL.stopMotor();
    climberR.stopMotor();
  }

  public void move(double speed) {
    SmartDashboard.putNumber("Climber L Pos", getPositionL());
    SmartDashboard.putNumber("Climber R Pos", getPositionR());
    SmartDashboard.putBoolean("Climber L Limit", isBottomL());
    SmartDashboard.putBoolean("Climber R Limit", isBottomR());

    // Left Climber Logic
    if (speed > 0) { // Going Up
      if (getPositionL() >= Constants.climberMaxHeight - offset) {
        climberL.stopMotor();
      } else {
        climberL.setControl(climberOut.withOutput(speed));
      }
    } else { // Going Down
      if (isBottomL()) {
        climberL.stopMotor();
      } else {
        climberL.setControl(climberOut.withOutput(speed));
      }
    }

    // Right Climber Logic
    if (speed > 0) { // Going Up
      if (getPositionR() >= Constants.climberMaxHeight - offset) {
        climberR.stopMotor();
      } else {
        climberR.setControl(climberOut.withOutput(speed));
      }
    } else { // Going Down
      if (isBottomR()) {
        climberR.stopMotor();
      } else {
        climberR.setControl(climberOut.withOutput(speed));
      }
    }
  }

  public Command Up() {
    return Commands.runEnd(() -> move(Constants.climberSpeed), this::stop, this);
  }

  public Command Down() {
    return Commands.runEnd(() -> move(-Constants.climberSpeed), this::stop, this);
  }
}