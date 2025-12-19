package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
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

    // 1. Update target from Vision
    Translation2d freshDetection = vision.getNoteFieldPosition(currentPose);
    if (freshDetection != null) {
        lastKnownTargetLocation = freshDetection;
    }

    if (lastKnownTargetLocation == null) {
        drivetrain.setControl(new SwerveRequest.Idle());
        return;
    }

    // 2. CALCULATE ROBOT-RELATIVE ERROR
    // This transforms the Note's field position into "How many meters ahead/left of the robot"
    Translation2d relativeTranslation = lastKnownTargetLocation.minus(currentPose.getTranslation())
                                        .rotateBy(currentPose.getRotation().unaryMinus());

    // 3. Calculate Rotation to face the BACK to the note
    // We want the back of the robot (Angle PI) to face the relative translation
    double angleToNote = Math.atan2(relativeTranslation.getY(), relativeTranslation.getX());
    double rotationError = MathUtil.angleModulus(angleToNote - Math.PI);

    // 4. GENERATE SPEEDS
    // xSpeed: If relativeTranslation.getX() is positive, the note is in front. 
    // Since we want to back into it, a positive error should result in a NEGATIVE xSpeed.
    double xSpeed = xController.calculate(relativeTranslation.getX(), 0); 
    double ySpeed = yController.calculate(relativeTranslation.getY(), 0);
    double thetaSpeed = thetaController.calculate(rotationError, 0);

    // 5. APPLY ROBOT-RELATIVE SPEEDS
    // We use ApplyRobotSpeeds directly because our PIDs are now calculating robot-relative error
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed)));
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