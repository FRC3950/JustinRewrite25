// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static double shooterSpeed = 70; // percent 0-100
    public static double intakeSpeed = -0.50; // duty cycle 0-1
    public static double indexerSpeed = 0.3; // duty cycle 0-1
    public static double indexerSpeedAct = 0.65; // duty cycle 0-1
    public static double climberSpeed = 1; // duty cycle 0-1
    public final static double climberMaxHeight = 45; // rotations
    public final static double flipperAmpPos = -1.2; // rotations
    public final static double flipperStowPos = 0.65; // rotations
    public static double pivotShootAngle = 45; // 0-160ish
    public static double pivotStowPosition = -2; // position
    public static double pivotOffsetAngleThingy = 2.2949635; // calibration offset
    public static double drivetrainMaxSpeed = 4.49; // Meters per second
    public static double pivotAmpPos = 63; // position
    public static double pivotStowDescentVolts = -6.0;
    public static double pivotStowHoldVolts = -0.25;
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

    // Vision Constants
    public static final String limelightName = "limelight-notes";
    public static final double limelightMountHeight = Units.inchesToMeters(26.5); // Meters
    public static final double limelightMountAngle = -10.0; // Degrees (Angled down)
    public static final double limelightMountXOffset = Units.inchesToMeters(-3); // Meters (Forward/Back from center)
    public static final double limelightMountYOffset = Units.inchesToMeters(12); // Meters (Left/Right from center,
                                                                                   // -9.5 is Right)
    public static final double noteTargetHeight = Units.inchesToMeters(2); // Meters (on the floor)

    // DriveToNote PID Constants
    public static final double driveKP = 1.25;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.05;
    public static final double turnKP = 2.0;
    public static final double turnKI = 0.0;
    public static final double turnKD = 0.0;
    public static final double driveTolerance = 0.05; // Meters
    public static final double turnTolerance = Units.degreesToRadians(3); // Radians
}
