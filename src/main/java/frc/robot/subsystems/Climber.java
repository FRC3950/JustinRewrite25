package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class Climber extends SubsystemBase {

  private final TalonFX climberL = new TalonFX(19);
  private final TalonFX climberR = new TalonFX(49);

  private final PositionVoltage posReqL = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage posReqR = new PositionVoltage(0).withSlot(0);

  private static final double offset = 5; // Safety offset

  // Trapezoidal Profile Constraints (Linear Speed, Linear Acceleration)
  // Velocity: 80 rotations/sec, Acceleration: 100 rotations/sec^2
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(80, 100);
  private TrapezoidProfile.State setpointL = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointR = new TrapezoidProfile.State();

  public Climber() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // PID and Feedforward
    cfg.Slot0.kP = 0.5;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kS = 0.25;

    // Hardware Limit Switches
    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
    cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply configs
    climberL.getConfigurator().apply(cfg);
    climberR.getConfigurator().apply(cfg);

    climberL.setPosition(0);
    climberR.setPosition(0);

    syncSetpoint();
  }

  public double getPositionL() {
    return climberL.getPosition().getValueAsDouble();
  }

  public double getPositionR() {
    return climberR.getPosition().getValueAsDouble();
  }

  public boolean isBottomL() {
    return climberL.getReverseLimit().getValue().equals(com.ctre.phoenix6.signals.ReverseLimitValue.ClosedToGround);
  }

  public boolean isBottomR() {
    return climberR.getReverseLimit().getValue().equals(com.ctre.phoenix6.signals.ReverseLimitValue.ClosedToGround);
  }

  public void stop() {
    climberL.stopMotor();
    climberR.stopMotor();
  }

  private void logTelemetry() {
    SmartDashboard.putNumber("Climber L Pos", getPositionL());
    SmartDashboard.putNumber("Climber R Pos", getPositionR());
    SmartDashboard.putBoolean("Climber L Limit", isBottomL());
    SmartDashboard.putBoolean("Climber R Limit", isBottomR());
  }

  public void holdPosition() {
    // Hold current position
    climberL.setControl(posReqL.withPosition(getPositionL()));
    climberR.setControl(posReqR.withPosition(getPositionR()));
    logTelemetry();
  }

  public void syncSetpoint() {
    // Initialize setpoints to current positions to avoid jumps
    setpointL = new TrapezoidProfile.State(getPositionL(), 0);
    setpointR = new TrapezoidProfile.State(getPositionR(), 0);
  }

  public void moveTo(double position) {
    // Clamp target to valid range with offset
    double safeMax = Constants.climberMaxHeight - offset;
    double target = Math.max(0, Math.min(position, safeMax));

    // Calculate next setpoint using TrapezoidProfile for each motor
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    setpointL = profile.calculate(0.02, setpointL, new TrapezoidProfile.State(target, 0));
    setpointR = profile.calculate(0.02, setpointR, new TrapezoidProfile.State(target, 0));

    climberL.setControl(posReqL.withPosition(setpointL.position));
    climberR.setControl(posReqR.withPosition(setpointR.position));

    SmartDashboard.putNumber("Climber Target L", setpointL.position);
    SmartDashboard.putNumber("Climber Target R", setpointR.position);
    logTelemetry();
  }

  @Override
  public void periodic() {
    // Telemetry handled in control methods
  }

  public Command Up() {
    // Move to Max Height while held, hold position when released
    return Commands.runEnd(() -> moveTo(Constants.climberMaxHeight), this::holdPosition, this)
        .beforeStarting(this::syncSetpoint);
  }

  public Command Down() {
    // Move to 0 while held, hold position when released
    return Commands.runEnd(() -> moveTo(0), this::holdPosition, this)
        .beforeStarting(this::syncSetpoint);
  }

  public TalonFX getLeftMotor() {
    return climberL;
  }

  public TalonFX getRightMotor() {
    return climberR;
  }
}