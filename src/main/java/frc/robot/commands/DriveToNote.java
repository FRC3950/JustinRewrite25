package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToNote {

    public static Command create(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        return Commands.defer(() -> {
            Pose2d targetPose = vision.getTargetPose(drivetrain.getState().Pose);
            if (targetPose == null) {
                System.out.println("DriveToNote: No target found!");
                return Commands.none();
            }
            System.out.println("DriveToNote: Target found at " + targetPose);

            // Create a path to the target pose
            // We want to stop slightly before the note to intake it? Or drive through it?
            // Let's drive to it for now.

            PathConstraints constraints = new PathConstraints(
                    2.0, 3.0,
                    Units.degreesToRadians(360), Units.degreesToRadians(540));

            return AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0 // Goal end velocity
            ).andThen(Commands.runOnce(() -> System.out.println("DriveToNote: Path finished")))
                    .beforeStarting(Commands.runOnce(() -> System.out.println("DriveToNote: Path starting")));
        }, Set.of(drivetrain));
    }
}
