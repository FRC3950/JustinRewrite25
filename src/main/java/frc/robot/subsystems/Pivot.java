// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import static edu.wpi.first.units.Units.Rotations;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private static final TalonFX pivot = new TalonFX(15, "CANivore");
  private final DynamicMotionMagicVoltage mm_request = new DynamicMotionMagicVoltage(0, 130, 260, 0);
  private final PositionVoltage pos = new PositionVoltage(0);

  public Pivot() {
  }

  private double getFeedForward() {
    double currentPos = pivot.getPosition().getValue().in(Rotations);
    double currentAngle = currentPos * Constants.pivotOffsetAngleThingy;
    return 0.2 * Math.sin(Math.toRadians(currentAngle));
  }

  public void pivotUp() {
    pivot.setControl(mm_request.withPosition(angleToPos(Constants.pivotShootAngle)).withFeedForward(getFeedForward()));
  }

  public void pivotDown() {
    pivot.setControl(mm_request.withPosition(0).withFeedForward(getFeedForward()));
  }

  public void pivotAmp() {
    pivot.setControl(mm_request.withPosition(Constants.pivotAmpPos).withFeedForward(getFeedForward()));
  }

  public double angleToPos(double angle) {
    return angle / Constants.pivotOffsetAngleThingy;
  }

  public void stow() {
    pivot.setControl(pos.withPosition(Constants.pivotStowPosition).withFeedForward(getFeedForward()));
  }

  public static BooleanSupplier pivotZero() {
    return () -> pivot.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
  }

  public double getPosition() {
    return pivot.getPosition().getValue().in(Rotations);
  }

  public static Command zeroEncoder() {
    return new InstantCommand(
        () -> pivot.setPosition(0));
  }

  public Command pivotStartEnd() {
    return Commands.run(this::pivotUp, this);
  }

  public Command stowDefault() {
    return Commands.run(() -> pivot.setControl(pos.withPosition(Constants.pivotStowPosition).withFeedForward(getFeedForward())), this)
        .until(pivotZero())
        .andThen(Commands.run(() -> pivot.setControl(new VoltageOut(Constants.pivotStowHoldVolts)), this));
  }

  public static TalonFX getMotor() {
    return pivot;
  }
}
