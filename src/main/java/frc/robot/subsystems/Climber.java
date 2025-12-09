package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

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
    cfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    climberL.getConfigurator().apply(cfg);

    // Apply to Right (Not Inverted)
    cfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    climberR.getConfigurator().apply(cfg);
  }

  public Command climbCommand(DoubleSupplier yAxisPercentage) {
    Command cmd = new Command() {
      private boolean stopLeft = false;
      private boolean stopRight = false;

      @Override
      public void initialize() {
        stopLeft = false;
        stopRight = false;
      }

      @Override
      public void execute() {
        climberL.getPosition().refresh();
        climberR.getPosition().refresh();
        climberL.getReverseLimit().refresh();
        climberR.getReverseLimit().refresh();

        // Check limits and latch stop state if hit
        if (climberL.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround) {
          stopLeft = true;
        }
        if (climberR.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround) {
          stopRight = true;
        }

        // Calculate target voltages (negative for down, positive for up)
        double targetVoltsL = -getTargetVoltage(yAxisPercentage, Math.abs(climberL.getPosition().getValueAsDouble()));
        double targetVoltsR = -getTargetVoltage(yAxisPercentage, Math.abs(climberR.getPosition().getValueAsDouble()));

        // Apply voltages with latching logic
        // If latched (limit hit) AND trying to go DOWN (negative voltage), force 0
        if (stopLeft && targetVoltsL < 0) {
          climberL.setVoltage(0);
        } else {
          climberL.setVoltage(targetVoltsL);
        }

        if (stopRight && targetVoltsR < 0) {
          climberR.setVoltage(0);
        } else {
          climberR.setVoltage(targetVoltsR);
        }
      }

      @Override
      public void end(boolean interrupted) {
        climberL.setVoltage(0);
        climberR.setVoltage(0);
      }
    };
    cmd.addRequirements(this);
    return cmd;
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

}