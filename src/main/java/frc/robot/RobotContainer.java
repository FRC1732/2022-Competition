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
import frc.robot.commands.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems and commands are defined but not instantiated here...
  private Drivetrain drivetrainSubsystem;
  private Shooter shooter;
  private Intake intakeSubsystem;
  private Indexer indexerSubsystem;
  private Limelight limelightSubsystem;
  private AutoSegment autoCommand;
  private Feeder feederSubsystem;
  private Centerer centererSubsystem;

  // private final XboxController m_controller = new XboxController(0);
  private Joystick joystick1;
  private Joystick joystick2;

  // joystick2 buttons
  private Button resetGyro;
  private JoystickButton autoMove;
  private JoystickButton startShootin;
  private JoystickButton stopShootin;
  private JoystickButton intakeButton;
  private JoystickButton feedButton;
  private JoystickButton ejectButton;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (Constants.HARDWARE_CONFIG_HAS_DRIVETRAIN) {
      drivetrainSubsystem = new Drivetrain();
    }
    if (Constants.HARDWARE_CONFIG_HAS_SHOOTER) {
      shooter = new Shooter();
    }
    if (Constants.HARDWARE_CONFIG_HAS_INDEX) {
      indexerSubsystem = new Indexer();
    }
    if (Constants.HARDWARE_CONFIG_HAS_INTAKE) {
      intakeSubsystem = new Intake();
    }
    if (Constants.HARDWARE_CONFIG_HAS_LIMELIGHT) {
      limelightSubsystem = new Limelight();
    }
    if(Constants.HARDWARE_CONFIG_HAS_FEEDER){
      feederSubsystem = new Feeder();
    }
    if (Constants.HARDWARE_CONFIG_HAS_CENTERER) {
      centererSubsystem = new Centerer();
    }

    defineButtons();

    if (drivetrainSubsystem != null) {
      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
          drivetrainSubsystem,
          () -> -modifyAxis(joystick1.getY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.TRAINING_WHEELS,
          () -> -modifyAxis(joystick1.getX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.TRAINING_WHEELS,
          () -> -modifyAxis(joystick2.getX()) * Constants.MAX_ANGULAR_VELOCITY * Constants.TRAINING_WHEELS));
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  private void defineButtons() {
    // joystick declaration
    joystick1 = new Joystick(0);
    joystick2 = new Joystick(1);

    //joystick1 button declaration
    intakeButton = new JoystickButton(joystick1, 1);
    ejectButton = new JoystickButton(joystick1, 2);
    
    // joystick2 button declaration
    resetGyro = new Button(joystick2::getTrigger);
    autoMove = new JoystickButton(joystick2, 2);    
    startShootin = new JoystickButton(joystick2, 4);
    stopShootin = new JoystickButton(joystick2, 5);
    feedButton = new JoystickButton(joystick2, 3);
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
    if (drivetrainSubsystem != null) {
      // Back button zeros the gyroscope
      resetGyro.whenPressed(() -> drivetrainSubsystem.zeroGyroscope());

      // @todo is try/catch needed here?
      autoCommand = new Auto10Feet(drivetrainSubsystem, "Auto 3 Meters");
      Command combinedCommand = autoCommand.getCommand()
          .andThen(() -> drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))).andThen(new WaitCommand(5));

      autoMove.whileHeld(combinedCommand);

      new JoystickButton(joystick2, 11).whileHeld(combinedCommand);
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null){
      intakeButton.whileHeld(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem));
    }
    
    if (feederSubsystem != null && centererSubsystem != null && indexerSubsystem != null){
      feedButton.whileHeld(new FeedCommand(feederSubsystem, centererSubsystem, indexerSubsystem));
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null && feederSubsystem != null){
      ejectButton.whileHeld(new EjectCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, feederSubsystem));
    }

    if (shooter != null) {
      startShootin.whenPressed(new StartShooter(shooter));
      stopShootin.whenPressed(new StopShooter(shooter));
    }

    if (limelightSubsystem != null) {
      new JoystickButton(joystick2, 10).whenPressed(new InstantCommand(() -> limelightSubsystem.on()))
          .whenReleased(new InstantCommand(() -> limelightSubsystem.off()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (drivetrainSubsystem != null) {
      // @todo is try/catch needed here?
      autoCommand = new Auto10Feet(drivetrainSubsystem, "Auto 3 Meters");
      return autoCommand.getCommand().andThen(() -> drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
    } else {
      return new InstantCommand();
    }
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
