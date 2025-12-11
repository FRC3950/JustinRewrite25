// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Shooter extends SubsystemBase {
  /** Creates a new ShooterShoot. */
  private final TalonFX inner = new TalonFX(13, "CANivore");
  private final TalonFX outer = new TalonFX(17, "CANivore");

  private static final double kP = 0.2;
  private static final double kI = 0.0;
  private static final double kD = 0.01;

  private static final double kS = 0.1;
  private static final double kV = 0.12;

  public Shooter() {
    inner.getConfigurator().apply(new TalonFXConfiguration() {
      {
        Slot0.kP = kP;
        Slot0.kI = kI;
        Slot0.kD = kD;
        Slot0.kS = kS;
        Slot0.kV = kV;
        MotorOutput.NeutralMode = NeutralModeValue.Brake;
      }
    });

    outer.getConfigurator().apply(new TalonFXConfiguration() {
      {
        Slot0.kP = kP;
        Slot0.kI = kI;
        Slot0.kD = kD;
        Slot0.kS = kS;
        Slot0.kV = kV;
        MotorOutput.NeutralMode = NeutralModeValue.Brake;
      }
    });
  }

  VelocityVoltage outerVelocity = new VelocityVoltage(-Constants.shooterSpeed);
  VelocityVoltage innerVelocity = new VelocityVoltage(Constants.shooterSpeed);

  public void startWheels() {
    inner.setControl(innerVelocity.withVelocity(Constants.shooterSpeed));
    outer.setControl(outerVelocity.withVelocity(-Constants.shooterSpeed));
  }

  public void stopWheels() {
    inner.set(0);
    outer.set(0);
  }

  public Command shoot_StartStop() {
    return Commands.startEnd(this::startWheels, this::stopWheels, this);
  }

  public TalonFX getInnerMotor() {
    return inner;
  }

  public TalonFX getOuterMotor() {
    return outer;
  }
}