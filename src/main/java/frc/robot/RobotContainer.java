// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.SwerveTrajectory;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DriveSubsystem;
import gay.zharel.hermes.profiles.ProfileParams;
import gay.zharel.hermes.trajectories.TrajectoryBuilder;
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams;
import gay.zharel.hermes.trajectory.HermesTrajectory;
import gay.zharel.hermes.trajectory.SwerveKinematicsConstraints;

import static gay.zharel.hermes.trajectory.ConversionsKt.hermes;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.DRIVE_DEADBAND),
                true),
                robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
                robotDrive::setX,
                robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create parameters for trajectory
    TrajectoryBuilderParams params = new TrajectoryBuilderParams(
            1e-6,
            new ProfileParams(0.25, Math.PI/8, 1e-6)
    );

    // Create constraints for trajectory
    SwerveKinematicsConstraints constraints = new SwerveKinematicsConstraints(
            DriveConstants.DRIVE_KINEMATICS,
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
    );

    TrajectoryBuilder builder = new TrajectoryBuilder(
            params,
            hermes(Pose2d.kZero),
            0.0,
            constraints.getVelConstraint(),
            constraints.getAccelConstraint()
    );

      // An example trajectory to follow. All units in meters.
    SwerveTrajectory trajectory = HermesTrajectory.fromTimeTrajectory(
            builder.bezierTo(hermes(new Translation2d(1, 1)), hermes(new Translation2d(2, -1)))
                    .splineTo(hermes(new Translation2d(3, 0)), hermes(new Rotation2d(0)))
                    .build().wrtTime()
    ).toSwerveTrajectory(DriveConstants.DRIVE_KINEMATICS);

    FollowTrajectoryCommand trajectoryCommand = new FollowTrajectoryCommand(robotDrive, trajectory);

    // Run path following command, then stop at the end.
    return trajectoryCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
  }
}
