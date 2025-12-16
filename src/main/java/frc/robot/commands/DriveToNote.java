package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private Pose2d targetPose;

    public DriveToNote(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(Constants.driveTolerance);
        yController.setTolerance(Constants.driveTolerance);
        thetaController.setTolerance(Constants.turnTolerance);
    }

    @Override
    public void initialize() {
        targetPose = vision.getTargetPose(drivetrain.getState().Pose);
        if (targetPose == null) {
            System.out.println("DriveToNote: No target found!");
            return;
        }
        System.out.println("DriveToNote: Target found at " + targetPose);

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        if (targetPose == null)
            return;

        Pose2d currentPose = drivetrain.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        // Transform field-relative speeds to robot-relative
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return targetPose != null && xController.atSetpoint() && yController.atSetpoint()
                && thetaController.atSetpoint();
    }
}
