package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class Climber extends SubsystemBase {

  private final TalonFX climberL = new TalonFX(19);
  private final TalonFX climberR = new TalonFX(49);

  private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0).withSlot(0);
  private final PositionVoltage holdReq = new PositionVoltage(0).withSlot(0);

  public Climber() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // PID and Feedforward
    cfg.Slot0.kP = 1.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kS = 0.25; // Static friction compensation

    // Motion Magic
    cfg.MotionMagic.MotionMagicCruiseVelocity = 80; // rps
    cfg.MotionMagic.MotionMagicAcceleration = 160; // rps/s
    cfg.MotionMagic.MotionMagicJerk = 1600; // rps/s^2

    // Hardware Limit Switches
    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
    cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply configs
    climberL.getConfigurator().apply(cfg);
    climberR.getConfigurator().apply(cfg);

    // Zero encoders on boot if at bottom (optional, but good practice if hard stops
    // exist)
    // For now, we assume start at 0 or user manually zeros.
    // If we wanted to zero on hard stop, we'd need a routine.
    climberL.setPosition(0);
    climberR.setPosition(0);
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

  public void holdPosition() {
    // Hold current position
    climberL.setControl(holdReq.withPosition(getPositionL()));
    climberR.setControl(holdReq.withPosition(getPositionR()));
  }

  public void moveTo(double position) {
    // Clamp target to valid range
    double target = Math.max(0, Math.min(position, Constants.climberMaxHeight));

    climberL.setControl(mmReq.withPosition(target));
    climberR.setControl(mmReq.withPosition(target));

    SmartDashboard.putNumber("Climber Target", target);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber L Pos", getPositionL());
    SmartDashboard.putNumber("Climber R Pos", getPositionR());
    SmartDashboard.putBoolean("Climber L Limit", isBottomL());
    SmartDashboard.putBoolean("Climber R Limit", isBottomR());
  }

  public Command Up() {
    // Move to Max Height while held, hold position when released
    return Commands.runEnd(() -> moveTo(Constants.climberMaxHeight), this::holdPosition, this);
  }

  public Command Down() {
    // Move to 0 while held, hold position when released
    return Commands.runEnd(() -> moveTo(0), this::holdPosition, this);
  }

  public TalonFX getLeftMotor() {
    return climberL;
  }

  public TalonFX getRightMotor() {
    return climberR;
  }
}