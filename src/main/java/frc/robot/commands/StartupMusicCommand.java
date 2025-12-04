package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.List;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class StartupMusicCommand extends Command {
    private final Orchestra orchestra;
    private final List<TalonFX> motors = new ArrayList<>();
    private boolean isFinished = false;

    public StartupMusicCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter, Pivot pivot, Climber climber,
            Intake intake, Flipper flipper) {
        orchestra = new Orchestra();

        // Collect all motors
        // Swerve (0-7)
        for (int i = 0; i < 4; i++) {
            motors.add(drivetrain.getModule(i).getDriveMotor());
            motors.add(drivetrain.getModule(i).getSteerMotor());
        }

        // Shooter (13, 17)
        motors.add(shooter.getInnerMotor());
        motors.add(shooter.getOuterMotor());

        // Pivot (15)
        motors.add(Pivot.getMotor());

        // Climber (19, 49)
        motors.add(climber.getLeftMotor());
        motors.add(climber.getRightMotor());

        // Intake (14, 41, 42)
        motors.add(intake.getIndexerMotor());
        motors.add(intake.getIntakeMotor());
        motors.add(intake.getIntake2Motor());

        // Flipper (31)
        motors.add(flipper.getMotor());

        // Add instruments to orchestra
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }

        // Require subsystems to ensure exclusive access and proper initialization
        addRequirements(drivetrain, shooter, pivot, climber, intake, flipper);
    }

    @Override
    public void initialize() {
        System.out.println("StartupMusicCommand: Initializing...");

        // Disable boot beep for all motors
        AudioConfigs audioConfigs = new AudioConfigs();
        audioConfigs.BeepOnBoot = false;

        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(audioConfigs);
            System.out.println("Added motor ID " + motor.getDeviceID() + " on bus: " + motor.getNetwork());
        }

        // Load and play music
        // Expects file in deploy directory (usually /home/lvuser/deploy)
        File deployDir = Filesystem.getDeployDirectory();
        File musicFile = new File(deployDir, "clash.chrp");
        System.out.println("StartupMusicCommand: Loading music from " + musicFile.getAbsolutePath());

        var status = orchestra.loadMusic(musicFile.getAbsolutePath());
        if (!status.isOK()) {
            System.out.println("StartupMusicCommand: Failed to load music file. Status: " + status);
        } else {
            System.out.println("StartupMusicCommand: Music loaded successfully. Starting playback.");
            orchestra.play();
        }
    }

    @Override
    public void execute() {
        boolean playing = orchestra.isPlaying();
        if (!playing) {
            isFinished = true;
            System.out.println("StartupMusicCommand: Music finished.");
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("StartupMusicCommand: Ending. Interrupted: " + interrupted);
        orchestra.stop();
        // Clear instruments to release motors
        orchestra.clearInstruments();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
