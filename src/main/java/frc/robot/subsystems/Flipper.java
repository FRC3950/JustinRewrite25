// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Flipper extends SubsystemBase {

  private final TalonFX flipper = new TalonFX(31, "CANivore");
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public Flipper() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = 2.5;
    cfg.Slot0.kI = 0;
    cfg.Slot0.kD = 0.01;
    cfg.Slot0.kS = 0;
    cfg.Slot0.kV = 0;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    flipper.getConfigurator().apply(cfg);

    // Optional: assume we start at stow when we boot
  }

  /** Hold at the stow position. */
  public void goToStow() {
    flipper.setControl(positionRequest.withPosition(Constants.flipperStowPos));
  }

  /** Move/hold at the amp position. */
  public void goToAmp() {
    flipper.setControl(positionRequest.withPosition(Constants.flipperAmpPos));
  }

  /** Command that continuously holds stow (for default command). */
  public Command holdStowCommand() {
    return this.run(this::goToStow);
  }

  /** Command to move/hold at amp position while scheduled. */
  public Command holdAmpCommand() {
    return this.run(this::goToAmp);
  }

  public TalonFX getMotor() {
    return flipper;
  }
}
