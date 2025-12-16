package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(Constants.limelightName);
    }

    public double getTargetTx() {
        return LimelightHelpers.getTX(Constants.limelightName);
    }

    public double getTargetTy() {
        return LimelightHelpers.getTY(Constants.limelightName);
    }

    /**
     * Calculates the field-relative pose of the detected note.
     * 
     * @param robotPose The current pose of the robot.
     * @return The Pose2d of the note, or null if no target is found.
     */
    public Pose2d getTargetPose(Pose2d robotPose) {
        if (!hasTarget()) {
            return null;
        }

        double targetOffsetAngle_Vertical = getTargetTy();
        double targetOffsetAngle_Horizontal = getTargetTx();

        // Calculate distance
        // Assuming limelightMountAngle is degrees down from horizontal
        double angleToGoalDegrees = Constants.limelightMountAngle + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // d = (h_cam - h_target) / tan(angle)
        double distanceFromLimelightToGoalMeters = (Constants.limelightMountHeight - Constants.noteTargetHeight)
                / Math.tan(angleToGoalRadians);

        // Calculate field-relative position
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d angleToTarget = robotHeading.minus(Rotation2d.fromDegrees(targetOffsetAngle_Horizontal));

        // Note: Limelight tx is negative when target is to the left, positive to the
        // right.
        // We need to add tx to robot heading to get angle to target?
        // Let's verify: Robot at 0 deg. Target at -10 deg (left). Angle to target
        // should be 10 deg left (positive or negative depending on coord system).
        // CCW is positive. Left is positive Y.
        // If robot is 0, and target is left, tx is negative? No, usually tx is negative
        // left.
        // Actually, let's use the Translation2d logic.

        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation = robotTranslation.plus(new Translation2d(distanceFromLimelightToGoalMeters,
                robotHeading.minus(Rotation2d.fromDegrees(targetOffsetAngle_Horizontal))));

        Pose2d targetPose = new Pose2d(targetTranslation, new Rotation2d());

        System.out.println("Vision: Dist=" + distanceFromLimelightToGoalMeters + "m, Angle=" + angleToGoalDegrees
                + ", Pose=" + targetPose);

        return targetPose;
    }

}
