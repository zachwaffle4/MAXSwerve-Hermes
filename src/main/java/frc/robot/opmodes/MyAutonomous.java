package frc.robot.opmodes;

import frc.robot.Constants;
import frc.robot.Robot;
import gay.zharel.hermes.wpitrajectories.SwerveDriveConstraint;
import gay.zharel.hermes.wpitrajectories.SwerveTrajectoryBuilder;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.SwerveTrajectory;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

@Autonomous(name="Peak Autonomous")
public class MyAutonomous extends PeriodicOpMode {
    private final Robot robot;
    private final Command autonomousCommand;

    public MyAutonomous(Robot robot) {
        this.robot = robot;
        autonomousCommand = createCommand();
    }

    @Override
    public void start() {
        Scheduler.getDefault().schedule(autonomousCommand);
    }

    @Override
    public void periodic() {
        Scheduler.getDefault().run();
    }

    public Command createCommand() {
        // Create constraints for trajectory
        SwerveDriveConstraint constraints = new SwerveDriveConstraint(
                Constants.DriveConstants.DRIVE_KINEMATICS.getModules(),
                Constants.AutoConstants.FEEDFORWARD,
                MetersPerSecond.of(Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND),
                Volts.of(Constants.DriveConstants.NOMINAL_VOLTAGE)
        );

        // An example trajectory to follow.  All units in meters.
        SwerveTrajectoryBuilder builder = new SwerveTrajectoryBuilder(
                Constants.DriveConstants.DRIVE_KINEMATICS,
                constraints,
                Pose2d.kZero
        ).bezierTo(new Translation2d(1, 1), new Translation2d(2, -1))
                .splineTo(new Translation2d(3, 0), new Rotation2d(0));

        SwerveTrajectory trajectory = builder.build();

        // Run trajectory following command
        return robot.drive.followTrajectoryCommand(trajectory);
    }
}
