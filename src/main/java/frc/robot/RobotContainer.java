// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.auto.Auto10Feet;
import frc.robot.commands.auto.AutoSegment;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Drivetrain m_drivetrainSubsystem;
  private Shooter shooter;
  private final Limelight m_limelightSubsystem = new Limelight();
  // private final Auto10Feet autoCommand = new Auto10Feet(m_drivetrainSubsystem);

  // private final XboxController m_controller = new XboxController(0);
  private Joystick joystick1;
  private Joystick joystick2;

  //joystick2 buttons
  private Button resetGyro;
  private JoystickButton move;
  private JoystickButton startShootin;
  private JoystickButton stopShootin;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem = new Drivetrain();
    //shooter = new Shooter();
    defineButtons();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(joystick1.getY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.TRAINING_WHEELS,
        () -> -modifyAxis(joystick1.getX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.TRAINING_WHEELS,
        () -> -modifyAxis(joystick2.getX()) * Constants.MAX_ANGULAR_VELOCITY * Constants.TRAINING_WHEELS));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void defineButtons() {
    //joystick declaration
    joystick1 = new Joystick(0);
    joystick2 = new Joystick(1);

    //joystick2 button declaration
    resetGyro = new Button(joystick2::getTrigger);
    move = new JoystickButton(joystick2, 2);

    startShootin = new JoystickButton(joystick2, 4);
    stopShootin = new JoystickButton(joystick2, 5);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    resetGyro
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    AutoSegment autoSegment = new Auto10Feet(m_drivetrainSubsystem, "Auto 3 Meters with S curve");
    Command combinedCommand = autoSegment.getCommand()
        .andThen(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))).andThen(new WaitCommand(5));

    move.whileHeld(combinedCommand);

    //startShootin.whenPressed(new StartShooter(shooter));
    //stopShootin.whenPressed(new StopShooter(shooter));
    new JoystickButton(joystick2, 2).whileHeld(combinedCommand);

    new JoystickButton(joystick1, 3).whenPressed(new InstantCommand(() -> m_limelightSubsystem.on()))
        .whenReleased(new InstantCommand(() -> m_limelightSubsystem.off()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // AutoSegment autoSegment = new Auto10Feet(m_drivetrainSubsystem, "Auto 3
    // Meters with S curve");
    AutoSegment autoSegment = new Auto10Feet(m_drivetrainSubsystem, "Auto 3 Meters with S curve");
    return autoSegment.getCommand().andThen(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
