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

        // 3. Calculate Forward Distance (X)
        // Formula: d = (h_target - h_camera) / tan(mount_angle + ty)
        double targetHeightOffset = Constants.noteTargetHeight - Constants.limelightMountHeight;
        double totalPitchRadians = Math.toRadians(Constants.limelightMountAngle + targetOffsetAngle_Vertical);
        double distanceToGoalX = targetHeightOffset / Math.tan(totalPitchRadians);

        // 4. Calculate Horizontal Offset (Y)
        // Formula: y = x * tan(tx)
        double angleYawRadians = Math.toRadians(targetOffsetAngle_Horizontal);
        double distanceToGoalY = distanceToGoalX * Math.tan(angleYawRadians);

        // Calculate field-relative position
        // Robot-relative: X is forward, Y is left.
        // Limelight: +tx is right (negative Y), -tx is left (positive Y).
        // So robot-relative Y = -distanceToGoalY (if distanceToGoalY is calculated from
        // tx directly where +tx is right)

        Translation2d robotRelativeTranslation = new Translation2d(distanceToGoalX, -distanceToGoalY);

        // Transform to field-relative
        Pose2d currentRobotPose = robotPose; // Use the passed pose
        // Rotate the robot-relative translation by the robot's heading
        Translation2d fieldRelativeTranslation = currentRobotPose.getTranslation()
                .plus(robotRelativeTranslation.rotateBy(currentRobotPose.getRotation()));

        Pose2d targetPose = new Pose2d(fieldRelativeTranslation, new Rotation2d());

        System.out.println("Vision: X=" + distanceToGoalX + ", Y=" + -distanceToGoalY + ", Pose=" + targetPose);

        return targetPose;
    }

}
