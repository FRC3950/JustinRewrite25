package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class Climber extends SubsystemBase {

  private final TalonFX climberL = new TalonFX(19);
  private final TalonFX climberR = new TalonFX(49);

  public Climber() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    cfg.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Constants.climberMaxHeight);
    cfg.HardwareLimitSwitch.withReverseLimitAutosetPositionEnable(true);
    cfg.HardwareLimitSwitch.withReverseLimitAutosetPositionValue(0);
    cfg.HardwareLimitSwitch.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);
    cfg.HardwareLimitSwitch.withReverseLimitEnable(true);
    cfg.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

    // Apply to Left (Inverted)
    cfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    climberL.getConfigurator().apply(cfg);

    // Apply to Right (Not Inverted)
    cfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    climberR.getConfigurator().apply(cfg);
  }

  public Command climbCommand(DoubleSupplier yAxisPercentage) {
    return Commands.runEnd(
        () -> setMotorVoltage(yAxisPercentage),
        () -> setMotorVoltage(() -> 0.0),
        this);
  }

  private double getTargetVoltage(DoubleSupplier yAxisPercentage, double motorPosition) {
    var percent = yAxisPercentage.getAsDouble();
    if (percent > 0.15) {
      return -yAxisPercentage.getAsDouble() * 6;
    } else if (percent < -0.15) {
      return yAxisPercentage.getAsDouble() * -8;
    }
    return 0;
  }

  private void setMotorVoltage(DoubleSupplier yAxisPercentage) {
    climberL.getPosition().refresh();
    climberR.getPosition().refresh();
    if (climberL.getReverseLimit().getValue() == ReverseLimitValue.Open) {
      climberL.setVoltage(-getTargetVoltage(yAxisPercentage, Math.abs(climberL.getPosition().getValueAsDouble())));
    }
    if (climberR.getReverseLimit().getValue() == ReverseLimitValue.Open) {
      climberR.setVoltage(-getTargetVoltage(yAxisPercentage, Math.abs(climberR.getPosition().getValueAsDouble())));
    }
  }

}