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

public class DriveToNote extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private Command pathCommand;

    public DriveToNote(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = vision.getTargetPose(drivetrain.getState().Pose);
        if (targetPose == null) {
            System.out.println("DriveToNote: No target found!");
            pathCommand = null;
            return;
        }
        System.out.println("DriveToNote: Target found at " + targetPose);

        PathConstraints constraints = new PathConstraints(
                2.0, 3.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540));

        pathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity
        );
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }
}
