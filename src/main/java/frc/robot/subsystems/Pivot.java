// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private final TalonFX pivot = new TalonFX(15, "CANivore");
  private final DynamicMotionMagicVoltage mm_request = new DynamicMotionMagicVoltage(0, 130, 260, 0);
  public Pivot() {
  }
  public void pivotUp(){
    pivot.setControl(mm_request.withPosition(angleToPos(Constants.pivotShootAngle)));
  }
  public void pivotDown(){
    pivot.se
  }
  public double angleToPos(double angle){
    return angle/2.1;
  }
}
