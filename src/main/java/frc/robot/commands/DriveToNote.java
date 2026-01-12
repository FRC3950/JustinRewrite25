package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToNote extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    private final PIDController xController = new PIDController(Constants.driveKP, Constants.driveKI,
            Constants.driveKD);
    private final PIDController yController = new PIDController(Constants.driveKP, Constants.driveKI,
            Constants.driveKD);
    private final PIDController thetaController = new PIDController(Constants.turnKP, Constants.turnKI,
            Constants.turnKD);

    private Translation2d lockedTargetLocation = null;
    private Rotation2d lockedTargetRotation = null;

    private final Timer timer = new Timer();
    private static final double TIMEOUT = 2.5;

    public DriveToNote(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        timer.restart();
        Pose2d currentPose = drivetrain.getState().Pose;

        // CAPTURE ONCE
        lockedTargetLocation = vision.getNoteFieldPosition(currentPose);

        if (lockedTargetLocation != null) {
            // Calculate the angle from Note TO Robot (Backing in)
            // Note -> Robot vector is the opposite of Robot -> Note
            Translation2d robotToNote = lockedTargetLocation.minus(currentPose.getTranslation());

            // We want the back of the robot to face the note.
            // If the note is at 0 degrees, the robot should face 180.
            lockedTargetRotation = robotToNote.getAngle().plus(Rotation2d.fromDegrees(180));

            System.out.println("DriveToNote: Locked Target at " + lockedTargetLocation);
        }

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        // Update target if visible
        Translation2d newTargetLocation = vision.getNoteFieldPosition(currentPose);
        if (newTargetLocation != null) {
            lockedTargetLocation = newTargetLocation;

            // Recalculate rotation
            Translation2d robotToNote = lockedTargetLocation.minus(currentPose.getTranslation());
            lockedTargetRotation = robotToNote.getAngle().plus(Rotation2d.fromDegrees(180));
        }

        // If we still don't have a target (never saw one), stop
        if (lockedTargetLocation == null) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        // Drive to the static Field-Relative position
        double xSpeed = xController.calculate(currentPose.getX(), lockedTargetLocation.getX());
        double ySpeed = yController.calculate(currentPose.getY(), lockedTargetLocation.getY());

        // Turn to the static Field-Relative rotation
        double thetaSpeed = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                lockedTargetRotation.getRadians());

        // Use Field-Relative speeds directly
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        drivetrain.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(fieldSpeeds));
    }

    @Override
    public boolean isFinished() {
        // Stop if we never saw a note, if we timed out, or if we are at the spot
        return lockedTargetLocation == null ||
                timer.hasElapsed(TIMEOUT) ||
                (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}