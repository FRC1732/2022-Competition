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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private Climber climberSubsystem;

  private SendableChooser<Command> _autoChooser;

  // private final XboxController m_controller = new XboxController(0);
  private Joystick joystick1;
  private Joystick joystick2;
  private Joystick joystick3;
  private Joystick joystick4;

  // joystick1 buttons
  private JoystickButton driverIntakeButton;
  private JoystickButton driverFeedButton;
  private JoystickButton driverEjectButton;
  private JoystickButton driverStartShootin;

  // joystick2 buttons
  private Button resetGyro;
  private JoystickButton alignTarget;

  // joystick3/4 buttons
  private JoystickButton climberArmTwoUpButton;
  private JoystickButton climberArmOneUpButton;
  private JoystickButton climberArmTwoDownButton;
  private JoystickButton climberArmOneDownButton;
  private JoystickButton climberUpButton;
  private JoystickButton climberDownButton;
  private JoystickButton climberArmTwoOut;
  private JoystickButton climberFinishClimbing;

  private JoystickButton operatorIntakeButton;
  private JoystickButton operatorEjectButton;
  private JoystickButton operatorFeedButton;
  private JoystickButton operatorShooterOnButton;
  private JoystickButton operatorHoodButton;

  private JoystickButton autoClimb;

  private Trigger testButton;

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

    // limelightSubsystem.off(); // turn the light off upon startup
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

    if (Constants.HARDWARE_CONFIG_HAS_CLIMBER) {
      climberSubsystem = new Climber();
    }
  }

  private void defineButtons() {
    // joystick declaration
    joystick1 = new Joystick(0);
    joystick2 = new Joystick(1);
    joystick3 = new Joystick(2);
    joystick4 = new Joystick(3);

    // joystick1 button declaration
    driverIntakeButton = new JoystickButton(joystick1, 1);
    driverEjectButton = new JoystickButton(joystick1, 3);
    driverFeedButton = new JoystickButton(joystick1, 2);

    // must press and hold buttons 8 and 9 to run test commands.
    testButton = new JoystickButton(joystick1, 8).and(new JoystickButton(joystick1, 9));

    // joystick2 button declaration
    resetGyro = new Button(() -> joystick2.getRawButton(2));
    driverStartShootin = new JoystickButton(joystick2, 1);
    alignTarget = new JoystickButton(joystick2, 10);
    // stopShootin = new JoystickButton(joystick2, 7);

    // joystick3 button declaration
    operatorIntakeButton = new JoystickButton(joystick3, 1);
    operatorEjectButton = new JoystickButton(joystick3, 2);
    operatorFeedButton = new JoystickButton(joystick3, 3);
    climberArmTwoOut = new JoystickButton(joystick3, 6);
    autoClimb = new JoystickButton(joystick3, 7);
    operatorHoodButton = new JoystickButton(joystick3, 4);
    operatorShooterOnButton = new JoystickButton(joystick3, 5);
    climberFinishClimbing = new JoystickButton(joystick3, 8);

    // joystick4 button declaration
    climberArmOneDownButton = new JoystickButton(joystick4, 4);
    climberArmOneUpButton = new JoystickButton(joystick4, 3);
    climberArmTwoDownButton = new JoystickButton(joystick4, 6);
    climberArmTwoUpButton = new JoystickButton(joystick4, 5);
    climberDownButton = new JoystickButton(joystick4, 2);
    climberUpButton = new JoystickButton(joystick4, 1);
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
      // new JoystickButton(joystick2, 11).whileHeld(combinedCommand);
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null) {
      driverIntakeButton.whileHeld(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem));
      operatorIntakeButton.whenHeld(new IntakeCommand(intakeSubsystem,
          centererSubsystem, indexerSubsystem));
    }

    if (feederSubsystem != null && centererSubsystem != null && indexerSubsystem != null) {
      driverFeedButton.whileHeld(new FeedCommand(feederSubsystem, centererSubsystem, indexerSubsystem));
      operatorFeedButton.whileHeld(new FeedCommand(feederSubsystem,
          centererSubsystem, indexerSubsystem));
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null && feederSubsystem != null) {
      driverEjectButton
          .whileHeld(new EjectCommand(centererSubsystem, indexerSubsystem, feederSubsystem));
      operatorEjectButton.whileHeld(new EjectCommand(centererSubsystem, indexerSubsystem, feederSubsystem));
    }

    if (shooter != null) {
      driverStartShootin.whenPressed(new RunShooterCommand(shooter), true);
      driverStartShootin.whenReleased(new StopShooterCommand(shooter));
      operatorShooterOnButton.whenActive(new InstantCommand(() -> shooter.startFlywheel()));
      operatorShooterOnButton.whenInactive(new InstantCommand(() -> shooter.stopFlywheel()));
      operatorHoodButton.whenActive(new InstantCommand(() -> shooter.extendHood()));
      operatorHoodButton.whenInactive(new InstantCommand(() -> shooter.retractHood()));
      // stopShootin.whenPressed(new StopShooterCommand(shooter));
    }

    if (climberSubsystem != null) {
      climberUpButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberUp()));
      climberDownButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberDown()));
      climberArmOneUpButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmOneUp()));
      climberArmOneDownButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmTwoDown()));
      climberArmTwoUpButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmTwoUp()));
      climberArmTwoDownButton.whenPressed(new InstantCommand(() -> climberSubsystem.climberArmTwoDown()));
      climberFinishClimbing.whenPressed(new InstantCommand(() -> climberSubsystem.climberBrakeAllOn()));
      climberArmTwoOut.whenActive(new InstantCommand(() -> climberSubsystem.ArmTwoOut()));
      climberArmTwoOut.whenInactive(new InstantCommand(() -> climberSubsystem.ArmTwoIn()));
      climberFinishClimbing.whenPressed(new InstantCommand(() -> climberSubsystem.finishClimb()));
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

    // testButton
    // .whenActive(
    // new RunShooterCommand(shooter).andThen(new PrintCommand("Test Button
    // Pressed")))
    // .whenInactive(
    // new StopShooterCommand(shooter).andThen(new PrintCommand("Test Button
    // Released")));

    testButton.whenActive(new PrintCommand("Test Button Pressed").andThen(provideTestCommand()))
        .whenInactive(new PrintCommand("Test Button Released").andThen(provideAllStopCommand()));
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
    ShuffleboardTab tab;

    // Always add the autos dropdown
    tab = Shuffleboard.getTab("COMPETITION");
    if (Constants.HARDWARE_CONFIG_HAS_AUTOS) {
      setupAutChooser();
      tab.add("Auto selection", _autoChooser).withSize(4, 1).withPosition(0, 0);
    }

    switch (RobotConfig.SB_LOGGING) {
      case COMPETITION:
        break;
      case DEBUG:
        tab = Shuffleboard.getTab("Drivetrain");
        tab.addNumber("DSupp_Rotation", m_rotationSupplier);
        tab.addNumber("DSupp_X", m_translationXSupplier);
        tab.addNumber("DSupp_Y", m_translationYSupplier);
        break;
      case NONE:
      default:
        break;
    }

  }

  public Command provideAllStopCommand() {
    Command allStopCommand = new InstantCommand();

    if (intakeSubsystem != null) {
      allStopCommand = allStopCommand.andThen(new InstantCommand(() -> intakeSubsystem.stop()));
    }

    if (centererSubsystem != null) {
      allStopCommand = allStopCommand.andThen(new InstantCommand(() -> centererSubsystem.stop()));
    }

    if (indexerSubsystem != null) {
      allStopCommand = allStopCommand.andThen(new InstantCommand(() -> indexerSubsystem.stop()));
    }

    if (feederSubsystem != null) {
      allStopCommand = allStopCommand.andThen(new InstantCommand(() -> feederSubsystem.stop()));
    }

    if (shooter != null) {
      allStopCommand = allStopCommand.andThen(new StopShooterCommand(shooter));
    }

    if (drivetrainSubsystem != null) {
      allStopCommand = allStopCommand.andThen(new InstantCommand(() -> drivetrainSubsystem.stop()));
    }

    return allStopCommand;
  }

  public Command provideTestCommand() {
    Command testCommand = new InstantCommand();

    if (intakeSubsystem != null && centererSubsystem != null
        && indexerSubsystem != null && feederSubsystem != null) {
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

    if (shooter != null) {
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
