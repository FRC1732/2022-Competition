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
import frc.robot.commands.AutoClimb.*;
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
  private Joystick joystick0;
  private Joystick joystick1;
  private Joystick joystick2;
  private Joystick joystick3;

  // joystick1 buttons
  private JoystickButton driverIntakeButton;
  private JoystickButton driverFeedButton;
  private JoystickButton driverEjectButton;

  // joystick2 buttons
  private Button resetGyro;
  private JoystickButton alignTarget;
  private JoystickButton driverStartShootin;
  private JoystickButton driverStartShooter;
  private JoystickButton driverStopShooter;

  // joystick3/4 buttons
  private JoystickButton climberArmTwoUpButton;
  private JoystickButton climberArmOneUpButton;
  private JoystickButton climberArmTwoDownButton;
  private JoystickButton climberArmOneDownButton;
  private JoystickButton climberBothUpButton;
  private JoystickButton climberBothDownButton;
  private JoystickButton climberArmTwoSwitch;
  private JoystickButton autoClimb_Phase1Button;
  private JoystickButton autoClimb_Phase2Button;
  private JoystickButton climberAutoClimb;

  private JoystickButton operatorIntakeButton;
  private JoystickButton operatorEjectButton;
  private JoystickButton operatorFeedButton;
  private JoystickButton operatorShooterOnButton;
  private JoystickButton operatorHoodSwitch;

  private JoystickButton brakeOverrideSwitch;

  private Trigger testButton;

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
      var input = -modifyAxis(joystick0.getY()) * Constants.TRAINING_WHEELS;
      var speed = input * Constants.MAX_VELOCITY_METERS_PER_SECOND;
      // speed = highPassFilter(speed, Constants.MIN_VELOCITY_METERS_PER_SECOND);
      return speed;
    }
  };

  DoubleSupplier m_translationYSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      var input = -modifyAxis(joystick0.getX()) * Constants.TRAINING_WHEELS; //speed not limited by slewrate limiter
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
        input = (-modifyAxis(joystick1.getX())) * Constants.TRAINING_WHEELS;
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
    joystick0 = new Joystick(0);
    joystick1 = new Joystick(1);
    joystick2 = new Joystick(2);
    joystick3 = new Joystick(3);

    // joystick0 button declaration
    driverIntakeButton = new JoystickButton(joystick0, 1);
    driverEjectButton = new JoystickButton(joystick0, 2);
    driverFeedButton = new JoystickButton(joystick0, 3);
    resetGyro = new Button(() -> joystick0.getRawButton(4));

    // must press and hold buttons 8 and 9 to run test commands.
    testButton = new JoystickButton(joystick0, 8).and(new JoystickButton(joystick0, 9));

    // joystick1 button declaration
    driverStartShootin = new JoystickButton(joystick1, 1);
    driverStartShooter = new JoystickButton(joystick1, 3);
    driverStopShooter = new JoystickButton(joystick1, 2);
    alignTarget = new JoystickButton(joystick1, 10);
    // stopShootin = new JoystickButton(joystick2, 7);

    // joystick2 button declaration
    operatorIntakeButton = new JoystickButton(joystick2, 7);
    operatorEjectButton = new JoystickButton(joystick2, 2);
    operatorFeedButton = new JoystickButton(joystick2, 3);
    climberAutoClimb = new JoystickButton(joystick2, 4);
    operatorShooterOnButton = new JoystickButton(joystick2, 5);
    climberArmTwoSwitch = new JoystickButton(joystick2, 6);
    autoClimb_Phase1Button = new JoystickButton(joystick2, 1);
    autoClimb_Phase2Button = new JoystickButton(joystick2, 8);

    // joystick3 button declaration
    climberArmOneDownButton = new JoystickButton(joystick3, 4);
    climberArmOneUpButton = new JoystickButton(joystick3, 3);
    climberArmTwoDownButton = new JoystickButton(joystick3, 6);
    climberArmTwoUpButton = new JoystickButton(joystick3, 5);
    climberBothDownButton = new JoystickButton(joystick3, 2);
    climberBothUpButton = new JoystickButton(joystick3, 1);
    operatorHoodSwitch = new JoystickButton(joystick3, 8);
    brakeOverrideSwitch = new JoystickButton(joystick3, 7);
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
      resetGyro.whenPressed(() -> drivetrainSubsystem.zeroGyroscope());
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
      driverStartShootin.whenPressed(new ShootFromAnywhereCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem, limelightSubsystem)
          .deadlineWith(new InstantCommand(() -> limelightRotationOn())));
      driverStartShootin
          .whenReleased(new StopShooterCommand(shooter).alongWith(new InstantCommand(() -> limelightRotationOff())));
      driverStopShooter.whenPressed(new StopShooterCommand(shooter));
      operatorShooterOnButton.whenHeld(new RunShooterCommand(shooter))
          .whenReleased(new StopShooterCommand(shooter));
      operatorHoodSwitch.whenActive(new InstantCommand(() -> shooter.extendHood()));
      operatorHoodSwitch.whenInactive(new InstantCommand(() -> shooter.retractHood()));
    }

    if (climberSubsystem != null) {
      // Both Climbers
      climberBothUpButton.whenHeld(new InstantCommand(() -> intakeSubsystem.deploy())
          .andThen(new InstantCommand(() -> climberSubsystem.ArmTwoOut()))
          .andThen(new InstantCommand(() -> climberSubsystem.armsAllUp())));
      climberBothUpButton.whenReleased(new InstantCommand(() -> climberSubsystem.armsAllStop()));
      climberBothDownButton.whenHeld(new InstantCommand(() -> climberSubsystem.armsAllDown()));
      climberBothDownButton.whenReleased(new InstantCommand(() -> climberSubsystem.armsAllStop()));

      // Climber One Arm (Stationary)
      climberArmOneUpButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmOneUp()));
      climberArmOneUpButton.whenReleased(new InstantCommand(() -> climberSubsystem.climberArmOneStop()));
      climberArmOneDownButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmOneDown()));
      climberArmOneDownButton.whenReleased(new InstantCommand(() -> climberSubsystem.climberArmOneStop()));

      // Climber Two Arm (Moving)
      climberArmTwoUpButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmTwoUp()));
      climberArmTwoUpButton.whenReleased(new InstantCommand(() -> climberSubsystem.climberArmTwoStop()));
      climberArmTwoDownButton.whenHeld(new InstantCommand(() -> climberSubsystem.climberArmTwoDown()));
      climberArmTwoDownButton.whenReleased(new InstantCommand(() -> climberSubsystem.climberArmTwoStop()));

      // Climber Two Arm (Moving) - Tilt
      climberArmTwoSwitch.whenActive(new InstantCommand(() -> intakeSubsystem.deploy())
          .andThen(new WaitCommand(0.2))
          .andThen(new InstantCommand(() -> climberSubsystem.ArmTwoOut())));
      climberArmTwoSwitch.whenInactive(new InstantCommand(() -> climberSubsystem.ArmTwoIn())
          .andThen(new WaitCommand(0.2))
          .andThen(new InstantCommand(() -> intakeSubsystem.retract())));

      // Manual Brake Override (to unjam brakes if necessary)
      brakeOverrideSwitch.whenPressed(new InstantCommand(() -> climberSubsystem.enableBrakeOverride()));
      brakeOverrideSwitch.whenReleased(new InstantCommand(() -> climberSubsystem.disableBrakeOverride()));

      climberAutoClimb.whenActive(new InstantCommand(() -> climberSubsystem.enableAutoClimb()));
      climberAutoClimb.whenInactive(new InstantCommand(() -> climberSubsystem.disableAutoClimb()));

      autoClimb_Phase1Button.whenHeld(new AutoClimb_Phase1(intakeSubsystem, climberSubsystem));
      autoClimb_Phase2Button.whenHeld(new AutoClimb_Phase2(intakeSubsystem, climberSubsystem));

    }

    if (limelightSubsystem != null) {
      if (drivetrainSubsystem != null) {
        alignTarget.whenPressed(new InstantCommand(() -> limelightRotationOn()))
            .whenReleased(new InstantCommand(() -> limelightRotationOff()));
      } else {
        alignTarget.whenPressed(new InstantCommand(() -> limelightSubsystem.on()))
            .whenReleased(new InstantCommand(() -> limelightSubsystem.off()));
      }
    }

    if (Constants.HARDWARE_CONFIG_HAS_SERVOS) {
      new JoystickButton(joystick1, 6).whenPressed(new InstantCommand(() -> servosSubsystem.incrementSetServoX()));
      new JoystickButton(joystick1, 7).whenPressed(new InstantCommand(() -> servosSubsystem.decrementSetServoX()));
      new JoystickButton(joystick1, 11).whenPressed(new InstantCommand(() -> servosSubsystem.incrementSetServoY()));
      new JoystickButton(joystick1, 10).whenPressed(new InstantCommand(() -> servosSubsystem.decrementSetServoY()));
      new JoystickButton(joystick0, 10)
          .whileHeld(new AlignToTargetAndShoot(servosSubsystem, limelightSubsystem, shooter, servosSubsystem));
    }

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

  private void setupAutChooser() {
    // Create the commands
    // Command AutoLayup1Shoot4 = new DriveAB(drivetrainSubsystem)
    // .andThen(new DriveBC(drivetrainSubsystem))
    // .andThen(new DriveCD(drivetrainSubsystem))
    // .andThen(new DriveDE(drivetrainSubsystem))
    // .andThen(new DriveED(drivetrainSubsystem));

    Command AutoShoot5 = new InstantCommand(() -> shooter.setSlowShot(true))
        .andThen(new ShootFromAnywhereCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem, limelightSubsystem))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new InstantCommand(() -> shooter.setSlowShot(false)))
        .andThen(new DriveHB(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new DriveBC(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new DriveCD(drivetrainSubsystem))
        .andThen(new ShootFromAnywhereCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem, limelightSubsystem))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new DriveDE(drivetrainSubsystem)
            .andThen(new WaitCommand(0.5))
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new DriveED(drivetrainSubsystem))
        .andThen(new ShootFromAnywhereCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem, limelightSubsystem)
            .deadlineWith(new InstantCommand(() -> limelightRotationOn())))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new InstantCommand(() -> limelightRotationOff()));

    Command ExperimentalAutoShoot5 = new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem)
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new DriveHB(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new DriveBC(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new InstantCommand(() -> shooter.startFlywheel(), shooter))
        .andThen(new DriveCD(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new DriveDE(drivetrainSubsystem)
            .andThen(new WaitCommand(1))
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new InstantCommand(() -> shooter.startFlywheel(), shooter))
        .andThen(new DriveED(drivetrainSubsystem))
        .andThen(new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem)
            .deadlineWith(new InstantCommand(() -> limelightRotationOn())))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new InstantCommand(() -> limelightRotationOff()));

    Command AutoShoot3 = new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem)
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new DriveHB(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new DriveBC(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem)))
        .andThen(new DriveCD(drivetrainSubsystem))
        .andThen(new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter));

    Command AutoShoot2 = new DriveFG(drivetrainSubsystem)
        .andThen(new WaitCommand(0.5))
        .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem))
        .andThen(new DriveGF(drivetrainSubsystem))
        .andThen(new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem))
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter));

    Command AutoShoot1 = new ShootCommand(shooter, feederSubsystem, centererSubsystem, indexerSubsystem)
        .andThen(new InstantCommand(() -> shooter.stopFlywheel(), shooter))
        .andThen(new DriveHL(drivetrainSubsystem));

    // Command AutoLayup1Shoot2;
    // Command AutoShoot4;
    // Command AutoLayup2;
    // Command AutoLayup3;

    // Create the sendable chooser (dropdown menu) for Shuffleboard
    _autoChooser = new SendableChooser<>();
    _autoChooser.setDefaultOption("AutoShoot5", AutoShoot5);
    _autoChooser.addOption("AutoShoot3", AutoShoot3);
    _autoChooser.addOption("AutoShoot2", AutoShoot2);
    _autoChooser.addOption("AutoShoot1", AutoShoot1);
    _autoChooser.addOption("ExperimentalAutoShoot5", ExperimentalAutoShoot5);
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
