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

    private final PIDController xController = new PIDController(Constants.driveKP, Constants.driveKI, Constants.driveKD);
    private final PIDController yController = new PIDController(Constants.driveKP, Constants.driveKI, Constants.driveKD);
    private final PIDController thetaController = new PIDController(Constants.turnKP, Constants.turnKI, Constants.turnKD);

    // We store the target as a Translation (Point) because we calculate Rotation dynamically
    private Translation2d lastKnownTargetLocation = null;

    private final Timer timer = new Timer();
    private static final double TIMEOUT = 3.0;

    public DriveToNote(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain); // Good practice to require the subsystem

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(Constants.driveTolerance);
        yController.setTolerance(Constants.driveTolerance);
        thetaController.setTolerance(Constants.turnTolerance);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d detection = vision.getNoteFieldPosition(currentPose);
        
        if (detection != null) {
            lastKnownTargetLocation = detection;
            System.out.println("DriveToNote: Initial target found at " + detection);
        } else {
            System.out.println("DriveToNote: No target found on init!");
            // We don't exit here; we hope to find it in execute, 
            // otherwise isFinished will handle the timeout.
        }

        xController.reset();
        yController.reset();
        thetaController.reset();
        timer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        // 1. Try to update target from Vision
        Translation2d freshDetection = vision.getNoteFieldPosition(currentPose);
        if (freshDetection != null) {
            lastKnownTargetLocation = freshDetection;
        }

        // 2. If we still haven't seen a target ever, just stop.
        if (lastKnownTargetLocation == null) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        // 3. Calculate dynamic rotation to face the note
        // Vector from Robot -> Note
        Translation2d robotToNote = lastKnownTargetLocation.minus(currentPose.getTranslation());
        Rotation2d desiredHeading = robotToNote.getAngle();

        // 4. Run PIDs
        double xSpeed = xController.calculate(currentPose.getX(), lastKnownTargetLocation.getX());
        double ySpeed = yController.calculate(currentPose.getY(), lastKnownTargetLocation.getY());
        double thetaSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(), 
            desiredHeading.getRadians()
        );

        // 5. Apply Control
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        System.out.println("DriveToNote: Ended. Interrupted=" + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(TIMEOUT)) return true;
        
        // If we never found a target, we can't be "at setpoint", but we might want to timeout/fail.
        if (lastKnownTargetLocation == null) return false;

        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}