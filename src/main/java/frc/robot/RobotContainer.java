// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
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
import frc.TestCommand;
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

  private SendableChooser<Command> _autoChooser;

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
      var input = -modifyAxis(m_xspeedLimiter.calculate(joystick1.getY())) * Constants.TRAINING_WHEELS;
      var speed = input * Constants.MAX_VELOCITY_METERS_PER_SECOND;
      // speed = highPassFilter(speed, Constants.MIN_VELOCITY_METERS_PER_SECOND);
      return speed;
    }
  };

  DoubleSupplier m_translationYSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      var input = -modifyAxis(m_yspeedLimiter.calculate(joystick1.getX())) * Constants.TRAINING_WHEELS;
      var speed = input * Constants.MAX_VELOCITY_METERS_PER_SECOND;
      // speed = highPassFilter(speed, Constants.MIN_VELOCITY_METERS_PER_SECOND);
      return speed;
    }
  };

  DoubleSupplier m_rotationSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      var input = 0.0;
      if (limelightSubsystem != null && limelightRotation) {
        input = limelightSubsystem.rotation.getAsDouble() * 0.15;
      } else {
        input = (-modifyAxis(joystick2.getX())) * Constants.TRAINING_WHEELS;
      }
      var speed = input * Constants.MAX_ANGULAR_VELOCITY;
      // speed = highPassFilter(speed, Constants.MIN_ANGULAR_VELOCITY);
      return speed;
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
    resetGyro = new Button(() -> joystick2.getRawButton(2));
    startShootin = new JoystickButton(joystick2, 6);
    stopShootin = new JoystickButton(joystick2, 7);
    feedButton = new JoystickButton(joystick2, 1);
    alignTarget = new JoystickButton(joystick2, 10);
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
    if (!Constants.HARDWARE_CONFIG_HAS_AUTOS)
      return new InstantCommand();

    Command autoCommand = _autoChooser.getSelected();
    if (autoCommand == null)
      return new InstantCommand();
    return autoCommand;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) < deadband)
      return 0;
    if (value > 0.0) {
      return (value - deadband) / (1.0 - deadband);
    }
    return (value + deadband) / (1.0 - deadband);
  }

  private static double highPassFilter(double value, double minValue) {
    return (Math.abs(value) < Math.abs(minValue)) ? 0 : value;
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.05); // Deadband
    value = Math.copySign(value * value, value); // Square the axisF
    return value;
  }

  private void setupAutChooser() {
    // Create the commands
    Command AutoLayup1Shoot4 = new DriveAB(drivetrainSubsystem)
        .andThen(new DriveBC(drivetrainSubsystem))
        .andThen(new DriveCD(drivetrainSubsystem))
        .andThen(new DriveDE(drivetrainSubsystem))
        .andThen(new DriveED(drivetrainSubsystem));

    // Auto Commands
    Command AutoLayup1 = new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem)
        .andThen(new DriveAB(drivetrainSubsystem));

    // Command AutoLayup1Shoot2;
    // Command AutoShoot1;
    // Command AutoShoot2;
    // Command AutoShoot3;
    // Command AutoShoot4;
    // Command AutoShoot5;
    // Command AutoLayup2;
    // Command AutoLayup3;

    // Create the sendable chooser (dropdown menu) for Shuffleboard
    _autoChooser = new SendableChooser<>();
    _autoChooser.setDefaultOption("AutoLayup1Shoot4", AutoLayup1Shoot4);
    _autoChooser.addOption("AutoLayup1", AutoLayup1);
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("COMPETITION");

    if (Constants.HARDWARE_CONFIG_HAS_AUTOS) {
      setupAutChooser();
      tab.add("Auto selection", _autoChooser).withSize(4, 1).withPosition(0, 0);
    }
    tab = Shuffleboard.getTab("Drivetrain");
    tab.addNumber("DSupp_Rotation", m_rotationSupplier);
    tab.addNumber("DSupp_X", m_translationXSupplier);
    tab.addNumber("DSupp_Y", m_translationYSupplier);    
  }

  public Command getTestCommand() {
    return new TestCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, feederSubsystem, shooter);
  }

  public boolean returnLimelightRotation() {
    return limelightRotation;
  }
}
