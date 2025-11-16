// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class Climber extends SubsystemBase {

  private final TalonFX climberL = new TalonFX(19);
  private final TalonFX climberR = new TalonFX(49);

  private final DutyCycleOut climberOut = new DutyCycleOut(Constants.climberSpeed);

  private static final double offset = 0.1; //needs tuning

  public Climber() {
    climberL.setNeutralMode(NeutralModeValue.Brake);
    climberR.setNeutralMode(NeutralModeValue.Brake);
    zeroIfAtBottom();
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

  private void zeroIfAtBottom() {
    if (isBottomL()) {
      climberL.setPosition(0.0);
    }
    if (isBottomR()) {
      climberR.setPosition(0.0);
    }
  }

  private void runClimbers(boolean goingUp) {

    double posL = getPositionL();
    double posR = getPositionR();

    boolean bottomL = isBottomL();
    boolean bottomR = isBottomR();

    if (!goingUp) {
      if (bottomL) {
        climberL.stopMotor();
      } else {
        climberL.setControl(climberOut.withOutput(-Constants.climberSpeed));
      }

      if (bottomR) {
        climberR.stopMotor();
      } else {
        climberR.setControl(climberOut.withOutput(-Constants.climberSpeed));
      }
      return;
    }

    if (posL >= Constants.climberMaxHeight - offset) {
      climberL.stopMotor();
    } else {
      climberL.setControl(climberOut);
    }

    if (posR >= Constants.climberMaxHeight - offset) {
      climberR.stopMotor();
    } else {
      climberR.setControl(climberOut);
    }
  }

  public Command Up() {
    return this.run(() -> runClimbers(true));
  }

  public Command Down() {
    return this.run(() -> runClimbers(false));
  }

  @Override
  public void periodic() {
    zeroIfAtBottom();
  }
}