package frc.robot.commands;

import edu.wpi.first.math.trajectory.SwerveTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.units.Units.Seconds;


public class FollowTrajectoryCommand extends Command {
    private final DriveSubsystem drive;
    private final SwerveTrajectory trajectory;
    private final Timer timer = new Timer();


    public FollowTrajectoryCommand(DriveSubsystem drive, SwerveTrajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        var desiredState = trajectory.sampleAt(currentTime);
        drive.setModuleStates(desiredState.states);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.duration.in(Seconds);
    }
}
