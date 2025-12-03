package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.List;

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

        // We do not require subsystems because we are just using their motors for sound
        // and this runs when disabled/initializing.
        // However, if we wanted to be safe, we could require them, but that might block
        // other default commands.
        // Since this is for startup/disabled, it's safer to NOT require them to avoid
        // conflict
        // with default commands that might try to run (though usually they don't run in
        // disabled).
    }

    @Override
    public void initialize() {
        // Disable boot beep for all motors
        AudioConfigs audioConfigs = new AudioConfigs();
        audioConfigs.BeepOnBoot = false;

        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(audioConfigs);
        }

        // Load and play music
        String musicFile = "clash.chrp"; // Expects file in working directory (usually /home/lvuser)
        var status = orchestra.loadMusic(musicFile);
        if (!status.isOK()) {
            System.out.println("StartupMusicCommand: Failed to load music file '" + musicFile + "'. Status: " + status);
        } else {
            orchestra.play();
        }
    }

    @Override
    public void execute() {
        if (!orchestra.isPlaying()) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
        // Clear instruments to release motors
        orchestra.clearInstruments();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
