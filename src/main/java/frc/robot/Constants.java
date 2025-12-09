// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static double shooterSpeed = 70; // percent 0-100
    public static double intakeSpeed = -0.50; // duty cycle 0-1
    public static double indexerSpeed = 0.40; // duty cycle 0-1
    public static double climberSpeed = 1; // duty cycle 0-1
    public final static double climberMaxHeight = 45; // rotations
    public final static double flipperAmpPos = -1.2; // rotations
    public final static double flipperStowPos = 0.65; // rotations
    public static double pivotShootAngle = 45; // 0-160ish
    public static double pivotStowPosition = -1.5; // position
    public static double pivotOffsetAngleThingy = 2.2949635; // calibration offset
    public static double drivetrainMaxSpeed = 4.49; // Meters per second

    // Presets
    public static final double shooterSpeedHigh = 90;
    public static final double shooterSpeedMedium = 65;
    public static final double shooterSpeedLow = 40;

    public static final double pivotAngleHigh = 20;
    public static final double pivotAngleMedium = 50;
    public static final double pivotAngleLow = 75;

    public static final double driveSpeedIndoor = 4.3;
    public static final double driveSpeedOutdoor = 3.5;
    public static final double driveSpeedKid = 1.5;
}
