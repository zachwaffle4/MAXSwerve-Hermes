// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.JoystickButton;
import org.wpilib.driverstation.PS5Controller;
import org.wpilib.driverstation.XboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.util.MathUtil;

/**
 * The VM is configured to automatically run this class;
 * if an OpMode constructor accepts it as a single argument,
 * it will be injected automatically.
 */
public class Robot extends OpModeRobot {
  // The robot's subsystems
  public final DriveSubsystem drive = new DriveSubsystem();

  // The driver's controller
  PS5Controller driverController = new PS5Controller(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
  public Robot() {
    // Configure the button bindings
    configureButtonBindings();

    Scheduler.getDefault().addPeriodic(drive::periodic);

    // Configure default commands
    drive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            drive.runRepeatedly(
                    () -> drive.drive(
                            -MathUtil.applyDeadband(driverController.getLeftY(), Constants.OIConstants.DRIVE_DEADBAND),
                            -MathUtil.applyDeadband(driverController.getLeftX(), Constants.OIConstants.DRIVE_DEADBAND),
                            -MathUtil.applyDeadband(driverController.getRightX(), Constants.OIConstants.DRIVE_DEADBAND),
                            true)
            ).named("Gamepad Drive")
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link org.wpilib.driverstation.GenericHID} or one of its
   * subclasses ({@link
   * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, PS5Controller.Button.kR1.value)
            .whileTrue(drive.runRepeatedly(drive::setX).named("X formation"));
  }
}
