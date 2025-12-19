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

    /**
     * Calculates the Field-Relative center point of the Note.
     * 
     * @param robotPose The current pose of the robot from Odometry.
     * @return Translation2d location of the note, or null if no target.
     */
    public Translation2d getNoteFieldPosition(Pose2d robotPose) {
        if (!hasTarget()) {
            return null;
        }

        double ty = LimelightHelpers.getTY(Constants.limelightName);
        double tx = LimelightHelpers.getTX(Constants.limelightName);

        // --- 1. Distance Calculation (Forward/X) ---
        // d = (h_target - h_camera) / tan(mount_angle + ty)
        double targetHeightOffset = Constants.noteTargetHeight - Constants.limelightMountHeight;

        // Ensure mountAngle is negative if pointing down, or handle signs appropriately
        double totalPitchRadians = Math.toRadians(Constants.limelightMountAngle + ty);

        // Calculate ground distance
        double distanceToGoalX = -Math.abs(targetHeightOffset / Math.tan(totalPitchRadians));

        // --- 2. Horizontal Offset (Y) ---
        // y = x * tan(tx)
        double distanceToGoalY = distanceToGoalX * Math.tan(Math.toRadians(tx));

        // --- 3. Robot-Relative Translation ---
        // WPILib: +X is Forward, +Y is Left
        // Limelight: +tx is Right. Therefore, Right = Negative Y
        Translation2d cameraToNote = new Translation2d(distanceToGoalX, -distanceToGoalY);
        Translation2d robotToCamera = new Translation2d(Constants.limelightMountXOffset,
                Constants.limelightMountYOffset);

        Translation2d robotRelativeTranslation = robotToCamera.plus(cameraToNote);

        // --- 4. Field-Relative Transformation ---
        // Rotate the robot-relative vector by the robot's heading, then add to robot's
        // position
        Translation2d fieldRelativeTranslation = robotPose.getTranslation()
                .plus(robotRelativeTranslation.rotateBy(robotPose.getRotation()));

        return fieldRelativeTranslation;
    }
}