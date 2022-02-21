// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private RobotConfig rc;
  // The robot's subsystems and commands are defined but not instantiated here...
  private Drivetrain drivetrainSubsystem;
  private Shooter shooter;
  private Intake intakeSubsystem;
  private Indexer indexerSubsystem;
  private Limelight limelightSubsystem;
  private Servos servosSubsystem;
  private Feeder feederSubsystem;
  private Centerer centererSubsystem;

  private SendableChooser<DriveSegmentBaseCommand> autonomousModeOption;
  private Drive10Feet drive10Feet;
  private DriveSCurve driveSCurve;

  // private final XboxController m_controller = new XboxController(0);
  private Joystick joystick1;
  private Joystick joystick2;

  // joystick2 buttons
  private Button resetGyro;
  private JoystickButton startShootin;
  private JoystickButton stopShootin;

  private JoystickButton intakeDeploy;
  private JoystickButton intakeRetract;

  private JoystickButton intakeButton;
  private JoystickButton feedButton;
  private JoystickButton ejectButton;
  private JoystickButton alignTarget;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private boolean limelightRotation;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(RobotConfig rc) {
    this.rc = rc;
    defineSubsystems();
    defineButtons();
    configureButtonBindings();
    setDefaultDriveCommand();
    setupShuffleboard();

    limelightSubsystem.off(); // turn the light off upon startup
  }

  DoubleSupplier m_translationXSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return (-modifyAxis(m_xspeedLimiter.calculate(joystick1.getY())) * Constants.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.TRAINING_WHEELS);
    }
  };

  DoubleSupplier m_translationYSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return (-modifyAxis(m_yspeedLimiter.calculate(joystick1.getX())) * Constants.MAX_VELOCITY_METERS_PER_SECOND
          * Constants.TRAINING_WHEELS);
    }
  };

  DoubleSupplier m_rotationSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      if (limelightSubsystem != null && limelightRotation) {
        return limelightSubsystem.rotation.getAsDouble();
      } else {
        return (-modifyAxis(joystick2.getX())
            * Constants.MAX_ANGULAR_VELOCITY * Constants.TRAINING_WHEELS);
      }
    }
  };

  private void setDefaultDriveCommand() {
    if (drivetrainSubsystem != null) {
      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
          drivetrainSubsystem,
          m_translationXSupplier,
          m_translationYSupplier,
          m_rotationSupplier));
    }
  }

  private void defineSubsystems() {
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

    if (Constants.HARDWARE_CONFIG_HAS_SERVOS) {
      servosSubsystem = new Servos();
    }

    if (Constants.HARDWARE_CONFIG_HAS_FEEDER) {
      feederSubsystem = new Feeder();
    }

    if (Constants.HARDWARE_CONFIG_HAS_CENTERER) {
      centererSubsystem = new Centerer();
    }

  }

  private void defineButtons() {
    // joystick declaration
    joystick1 = new Joystick(0);
    joystick2 = new Joystick(1);

    // joystick1 button declaration
    intakeButton = new JoystickButton(joystick1, 1);
    ejectButton = new JoystickButton(joystick1, 2);
    intakeDeploy = new JoystickButton(joystick1, 6);
    intakeRetract = new JoystickButton(joystick1, 7);

    // joystick2 button declaration
    resetGyro = new Button(this::tempGetButton2);
    startShootin = new JoystickButton(joystick2, 6);
    stopShootin = new JoystickButton(joystick2, 7);
    feedButton = new JoystickButton(joystick2, 1);
    alignTarget = new JoystickButton(joystick2, 10);
  }

  private boolean tempGetButton2()
  {
    return joystick2.getRawButton(2);
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
      resetGyro.whenPressed(drivetrainSubsystem::zeroGyroscope);
      // new JoystickButton(joystick2, 11).whileHeld(combinedCommand);
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null) {
      intakeButton.whileHeld(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem));
      intakeDeploy.whenPressed(new InstantCommand(() -> intakeSubsystem.deploy()));
      intakeRetract.whenPressed(new InstantCommand(() -> intakeSubsystem.retract()));
    }

    if (feederSubsystem != null && centererSubsystem != null && indexerSubsystem != null) {
      feedButton.whileHeld(new FeedCommand(feederSubsystem, centererSubsystem, indexerSubsystem));
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null && feederSubsystem != null) {
      ejectButton.whileHeld(new EjectCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, feederSubsystem));
    }

    if (shooter != null) {
      startShootin.whenPressed(new RunShooterCommand(shooter));
      stopShootin.whenPressed(new StopShooterCommand(shooter));
    }

    if (limelightSubsystem != null && drivetrainSubsystem != null) {
      alignTarget.whenPressed(new InstantCommand(() -> limelightRotationOn()))
          .whenReleased(new InstantCommand(() -> limelightRotationOff()));
    }

    if (limelightSubsystem != null) {
      new JoystickButton(joystick2, 10).whenPressed(new InstantCommand(() -> limelightSubsystem.on()))
          .whenReleased(new InstantCommand(() -> limelightSubsystem.off()));
    }

    if (Constants.HARDWARE_CONFIG_HAS_SERVOS) {
      new JoystickButton(joystick2, 6).whenPressed(new InstantCommand(() -> servosSubsystem.incrementSetServoX()));
      new JoystickButton(joystick2, 7).whenPressed(new InstantCommand(() -> servosSubsystem.decrementSetServoX()));
      new JoystickButton(joystick2, 11).whenPressed(new InstantCommand(() -> servosSubsystem.incrementSetServoY()));
      new JoystickButton(joystick2, 10).whenPressed(new InstantCommand(() -> servosSubsystem.decrementSetServoY()));
      new JoystickButton(joystick1, 10)
          .whileHeld(new AlignToTargetAndShoot(servosSubsystem, limelightSubsystem, shooter, servosSubsystem));
    }
  }

  private void limelightRotationOn() {
    limelightSubsystem.on();
    // m_rotationSupplier = limelightSubsystem.rotation;
    System.out.println("limelight rotation on");
    limelightRotation = true;

    // drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // drivetrainSubsystem,
    // m_translationXSupplier,
    // m_translationYSupplier,
    // limelightSubsystem.rotation));
  }

  private void limelightRotationOff() {
    limelightSubsystem.off();
    // m_rotationSupplier = () -> -modifyAxis(joystick2.getX()) *
    // Constants.MAX_ANGULAR_VELOCITY * Constants.TRAINING_WHEELS;
    System.out.println("limelight rotation off");
    limelightRotation = false;

    // drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // drivetrainSubsystem,
    // m_translationXSupplier,
    // m_translationYSupplier,
    // m_rotationSupplier));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return (Command) autonomousModeOption.getSelected();
    // if (drivetrainSubsystem == null) {
    // return new InstantCommand();
    // }
    // return autoCommand.getCommand(true).andThen(() ->
    // drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
    // return new Drive10Feet(drivetrainSubsystem, "Auto 3 Meters").getCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) < deadband)
      return 0;
    return value;
    // if (value > 0.0) {
    //   return (value - deadband) / (1.0 - deadband);
    // }
    // return (value + deadband) / (1.0 - deadband);
  }

  private static double minInput(double value, double minInput) {
    return 0;
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.05); // Deadband
    // value = minInput(value, 0.1);
    value = Math.copySign(value * value, value); // Square the axisF
    return value;
  }

  private void setupShuffleboard() {
    if (Constants.HARDWARE_CONFIG_HAS_AUTOS) {

      // Create the commands
      drive10Feet = new Drive10Feet(drivetrainSubsystem);
      driveSCurve = new DriveSCurve(drivetrainSubsystem);

      // Create the sendable chooser (dropdown menu) for Shuffleboard
      autonomousModeOption = new SendableChooser<>();
      autonomousModeOption.setDefaultOption("Drive 10 Feet", drive10Feet);
      autonomousModeOption.addOption("Drive S Curve", driveSCurve);

    }
    ShuffleboardTab tab = Shuffleboard.getTab("COMPETITION");
    tab.add("Auto selection", autonomousModeOption).withSize(4, 1).withPosition(0, 0);

    tab = Shuffleboard.getTab("Drivetrain");
    tab.addNumber("DSupp_Rotation", m_rotationSupplier);
    tab.addNumber("DSupp_X", m_translationXSupplier);
    tab.addNumber("DSupp_Y", m_translationYSupplier);
  }

  public Command getTestCommand() {
    Command testCommand = new InstantCommand();

    if (Constants.HARDWARE_CONFIG_HAS_INTAKE && Constants.HARDWARE_CONFIG_HAS_CENTERER
        && Constants.HARDWARE_CONFIG_HAS_INDEX && Constants.HARDWARE_CONFIG_HAS_FEEDER) {
      testCommand = testCommand
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> intakeSubsystem.forward()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> intakeSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> centererSubsystem.forward()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> centererSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> indexerSubsystem.forward()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> indexerSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> feederSubsystem.forward()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> feederSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> feederSubsystem.reverse()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> feederSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> indexerSubsystem.reverse()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> indexerSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> centererSubsystem.reverse()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> centererSubsystem.stop()))

          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> intakeSubsystem.reverse()))
          .andThen(new WaitCommand(1.0)).andThen(new InstantCommand(() -> intakeSubsystem.stop()));
    }

    if (Constants.HARDWARE_CONFIG_HAS_SHOOTER) {
      testCommand = testCommand
          .andThen(new WaitCommand(1.0)).andThen(new RunShooterCommand(shooter))
          .andThen(new WaitCommand(2.0)).andThen(new StopShooterCommand(shooter));
    }

    return testCommand;
  }

  public boolean returnLimelightRotation() {
    return limelightRotation;
  }
}
