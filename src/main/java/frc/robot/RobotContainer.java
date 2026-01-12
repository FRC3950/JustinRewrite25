// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flipper;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.DriveToNote;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
        // MaxSpeed removed, using Constants.drivetrainMaxSpeed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.drivetrainMaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add
                                                                                                                       // a
                                                                                                                       // 10%
                                                                                                                       // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(Constants.drivetrainMaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Shooter shooter = new Shooter();
        public final Intake intake = new Intake();
        public final Climber climber = new Climber();
        public final Flipper flipper = new Flipper();
        public final Pivot pivot = new Pivot();

        /* Path follower */
        private final SendableChooser<Command> autoChooser;
        Trigger zeroPivot = new Trigger(Pivot.pivotZero());

        public RobotContainer() {
                zeroPivot.onTrue(Pivot.zeroEncoder());
                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();

                // Warmup PathPlanner to avoid Java pauses
                FollowPathCommand.warmupCommand().schedule();

                SmartDashboard.putNumber("Pivot Shoot Angle", Constants.pivotShootAngle);
                SmartDashboard.putNumber("Shooter Speed", Constants.shooterSpeed);
                SmartDashboard.putNumber("Drivetrain Max Speed", Constants.drivetrainMaxSpeed);

                new Trigger(() -> SmartDashboard.getNumber("Pivot Shoot Angle",
                                Constants.pivotShootAngle) != Constants.pivotShootAngle)
                                .onTrue(Commands.runOnce(() -> {
                                        Constants.pivotShootAngle = SmartDashboard.getNumber("Pivot Shoot Angle",
                                                        Constants.pivotShootAngle);
                                }).ignoringDisable(true));

                new Trigger(() -> SmartDashboard.getNumber("Shooter Speed",
                                Constants.shooterSpeed) != Constants.shooterSpeed)
                                .onTrue(Commands.runOnce(() -> {
                                        Constants.shooterSpeed = SmartDashboard.getNumber("Shooter Speed",
                                                        Constants.shooterSpeed);
                                }).ignoringDisable(true));

                new Trigger(() -> SmartDashboard.getNumber("Drivetrain Max Speed",
                                Constants.drivetrainMaxSpeed) != Constants.drivetrainMaxSpeed)
                                .onTrue(Commands.runOnce(() -> {
                                        Constants.drivetrainMaxSpeed = SmartDashboard.getNumber("Drivetrain Max Speed",
                                                        Constants.drivetrainMaxSpeed);
                                }).ignoringDisable(true));

                // Shooter Presets
                SendableChooser<Double> shooterChooser = new SendableChooser<>();
                shooterChooser.addOption("High (90%)", Constants.shooterSpeedHigh);
                shooterChooser.setDefaultOption("Medium (65%)", Constants.shooterSpeedMedium);
                shooterChooser.addOption("Low (40%)", Constants.shooterSpeedLow);
                SmartDashboard.putData("Shooter Presets", shooterChooser);

                // Pivot Presets
                SendableChooser<Double> pivotChooser = new SendableChooser<>();
                pivotChooser.addOption("High (20 deg)", Constants.pivotAngleHigh);
                pivotChooser.setDefaultOption("Medium (50 deg)", Constants.pivotAngleMedium);
                pivotChooser.addOption("Low (75 deg)", Constants.pivotAngleLow);
                SmartDashboard.putData("Pivot Presets", pivotChooser);

                // Drive Presets
                SendableChooser<Double> driveChooser = new SendableChooser<>();
                driveChooser.setDefaultOption("Indoor (4.3 m/s)", Constants.driveSpeedIndoor);
                driveChooser.addOption("Outdoor (3.5 m/s)", Constants.driveSpeedOutdoor);
                driveChooser.addOption("Kid Mode (1.5 m/s)", Constants.driveSpeedKid);
                SmartDashboard.putData("Drive Presets", driveChooser);

                // Monitor Choosers
                var listener = new Object() {
                        double lastShooter = shooterChooser.getSelected();
                        double lastPivot = pivotChooser.getSelected();
                        double lastDrive = driveChooser.getSelected();
                };

                new Trigger(() -> shooterChooser.getSelected() != listener.lastShooter)
                                .onTrue(Commands.runOnce(() -> {
                                        listener.lastShooter = shooterChooser.getSelected();
                                        SmartDashboard.putNumber("Shooter Speed", listener.lastShooter);
                                }).ignoringDisable(true));

                new Trigger(() -> pivotChooser.getSelected() != listener.lastPivot)
                                .onTrue(Commands.runOnce(() -> {
                                        listener.lastPivot = pivotChooser.getSelected();
                                        SmartDashboard.putNumber("Pivot Shoot Angle", listener.lastPivot);
                                }).ignoringDisable(true));

                new Trigger(() -> driveChooser.getSelected() != listener.lastDrive)
                                .onTrue(Commands.runOnce(() -> {
                                        listener.lastDrive = driveChooser.getSelected();
                                        SmartDashboard.putNumber("Drivetrain Max Speed", listener.lastDrive);
                                }).ignoringDisable(true));

                // Schedule startup music
        }

        public final VisionSubsystem vision = new VisionSubsystem();

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                flipper.setDefaultCommand(flipper.holdStowCommand());
                pivot.setDefaultCommand(pivot.stowDefault());
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-joystick.getLeftY() * Constants.drivetrainMaxSpeed) // Drive
                                                                                                                    // forward
                                                                                                                    // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(-joystick.getLeftX() * Constants.drivetrainMaxSpeed) // Drive
                                                                                                                    // left
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // X
                                                                                                                    // (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                AmpCommand ampCommand = new AmpCommand(flipper, pivot);
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                // flywheel control for now
                joystick.leftBumper().whileTrue(pivot.pivotStartEnd().alongWith(shooter.shoot_StartStop()));
                // intake
                joystick.a().whileTrue(intake.intakeCommand());
                // outake/reverse intake if note stuck
                joystick.b().whileTrue(intake.reverseCommand());
                // Command both Climber arms to go up
                joystick.pov(0).whileTrue(climber.climbCommand(() -> Constants.climberSpeed));
                // Command both Climber arms to go down
                joystick.pov(180).whileTrue(climber.climbCommand(() -> -Constants.climberSpeed));

                // Amp Command
                joystick.x().whileTrue(ampCommand);

                // Indexer control: Reverse if Amp command is running, otherwise normal
                joystick.rightTrigger().whileTrue(
                                Commands.either(
                                                intake.reverseIndexerCommand(),
                                                intake.indexer(),
                                                ampCommand::isScheduled));

                // Drive to Note
                joystick.leftTrigger().whileTrue(new DriveToNote(drivetrain, vision).alongWith(intake.intakeCommand()));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
        }
}
