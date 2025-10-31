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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import gay.zharel.hermes.wpitrajectories.SwerveDriveConstraint;
import gay.zharel.hermes.wpitrajectories.SwerveTrajectoryBuilder;
import org.wpilib.commands3.Command;
import org.wpilib.commands3.Scheduler;
import org.wpilib.commands3.button.JoystickButton;

import static edu.wpi.first.units.Units.*;

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

    Scheduler.getDefault().addPeriodic(robotDrive::periodic);

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        robotDrive.runRepeatedly(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.DRIVE_DEADBAND),
                true)
        ).named("Gamepad Drive")
    );
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
        .whileTrue(robotDrive.runRepeatedly(robotDrive::setX).named("X formation"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create constraints for trajectory
    SwerveDriveConstraint constraints = new SwerveDriveConstraint(
            DriveConstants.DRIVE_KINEMATICS.getModules(),
            AutoConstants.FEEDFORWARD,
            MetersPerSecond.of(DriveConstants.MAX_SPEED_METERS_PER_SECOND),
            Volts.of(DriveConstants.NOMINAL_VOLTAGE)
    );

    // An example trajectory to follow.  All units in meters.
    SwerveTrajectoryBuilder builder = new SwerveTrajectoryBuilder(
            DriveConstants.DRIVE_KINEMATICS,
            constraints,
            Pose2d.kZero
    ).bezierTo(new Translation2d(1, 1), new Translation2d(2, -1))
            .splineTo(new Translation2d(3, 0), new Rotation2d(0));

    SwerveTrajectory trajectory = builder.build();

    // Run trajectory following command
    return robotDrive.followTrajectoryCommand(trajectory);
  }
}
