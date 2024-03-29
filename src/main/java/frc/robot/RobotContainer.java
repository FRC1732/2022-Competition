// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.AlignToTargetAndShoot;
import frc.robot.commands.AutoClimb.AutoClimb_Phase1;
import frc.robot.commands.AutoClimb.AutoClimb_Phase2;
import frc.robot.commands.AutoClimb.AutoClimb_Phase2_v3;
import frc.robot.commands.DefaultCommands.*;
import frc.robot.commands.DefaultCommands.DefaultDriveCommand;
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
  private Shooter shooterSubsystem;
  private Intake intakeSubsystem;
  private Indexer indexerSubsystem;
  private Limelight limelightSubsystem;
  private Servos servosSubsystem;
  private Feeder feederSubsystem;
  private Centerer centererSubsystem;
  private Climber climberSubsystem;
  private ColorSensor colorSensorSubsystem;
  private LedStatus ledStatusSubsystem ;

  private SendableChooser<Command> _autoChooser;

  // private final XboxController m_controller = new XboxController(0);
  private Joystick joystick0;
  private Joystick joystick1;
  private Joystick joystick2;
  private Joystick joystick3;
  private Joystick joystick4;

  // joystick0 buttons
  private JoystickButton driverIntakeButton;
  private JoystickButton driverFeedButton;
  private JoystickButton driverEjectButton;

  // joystick1 buttons
  private Button resetGyro;
  private JoystickButton alignTarget;
  private JoystickButton driverStartShootin;
  private JoystickButton driverStartShooter;
  private JoystickButton driverStopShooter;
  private JoystickButton alignWithHangar;

  // joystick2/3 buttons
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
  private JoystickButton operatorYeetEjectButton;
  private JoystickButton operatorFeedButton;
  private JoystickButton operatorShooterOnButton;
  private JoystickButton operatorToggleReject;

  private JoystickButton brakeOverrideSwitch;

  private Trigger testButton;

  private boolean _hangarAlign;
  private boolean _limelightRotation;
  private boolean intakeDown;
  private boolean _stoppedTimerRunning = false;
  private Timer _stoppedTimer = new Timer();
  private ProfiledPIDController _thetaController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(RobotConfig rc) {
    this.rc = rc;
    defineSubsystems();
    defineButtons();
    configureButtonBindings();
    setDefaultCommands();
    setupShuffleboard();

    // limelightSubsystem.off(); // turn the light off upon startup
  }

  BooleanSupplier m_stoppedSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      double deadband = 0.05;
      boolean translationStopped = Math.abs(joystick0.getX()) < deadband && Math.abs(joystick0.getY()) < deadband;
      if (!translationStopped){
        if (_stoppedTimerRunning){
          _stoppedTimerRunning = false;
          _stoppedTimer.stop();
          _stoppedTimer.reset();
        }
        return false;
      }
      if (!_stoppedTimerRunning){
        _stoppedTimerRunning = true;
        _stoppedTimer.start();
      }
      return _stoppedTimer.get() > 0.25;
    }
  };

  private static Translation2d rotateCartesian(Rotation2d angleOffset, double x, double y)
  {
    double inputAngle = Math.atan2(y, x);
    double inputMag = Math.sqrt(x * x + y * y);
    double offsetRad = angleOffset.times(-1).plus(Rotation2d.fromDegrees(90)).getRadians();
    double newX = Math.cos(inputAngle + offsetRad) * inputMag;
    double newY = Math.sin(inputAngle + offsetRad) * inputMag;
    return new Translation2d(newX, newY);
  }

  DoubleSupplier m_translationXSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      var input = 0.0;
      // if (intakeDown) {
      //   var auxInput = rotateCartesian(drivetrainSubsystem.getGyroscopeRotation(), joystick4.getX(), joystick4.getY());
      //   input = -modifyAxis(joystick0.getY() + auxInput.getY() * Constants.OWEN_WHEELZ) * Constants.TRAINING_WHEELS;
      // } else {
      input = -modifyAxis(joystick0.getY()) * Constants.TRAINING_WHEELS;
      // }

      var speed = input * Constants.MAX_VELOCITY_METERS_PER_SECOND;
      return speed;
    }
  };

  DoubleSupplier m_translationYSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      var input = 0.0;
      // if (intakeDown) {
      //   var auxInput = rotateCartesian(drivetrainSubsystem.getGyroscopeRotation(), joystick4.getX(), joystick4.getY());
      //   input = -modifyAxis(joystick0.getX() + auxInput.getX() * Constants.OWEN_WHEELZ)* Constants.TRAINING_WHEELS;
      // } else {
      input = -modifyAxis(joystick0.getX()) * Constants.TRAINING_WHEELS;
      // }
      var speed = input * Constants.MAX_VELOCITY_METERS_PER_SECOND;
      return speed;
    }
  };

  DoubleSupplier m_rotationSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      if (_hangarAlign) {
        return hangarRotation.getAsDouble();
      }
      var input = 0.0;
      if (limelightSubsystem != null && _limelightRotation && limelightSubsystem.hasTarget()) {
        if (!limelightSubsystem.isAligned())
        {
          input = limelightSubsystem.rotation.getAsDouble();
        }
        // input = highPassFilter(input, Constants.MIN_ANGULAR_VELOCITY);
      } else {
        input = (-modifyAxis(joystick1.getX())) * Constants.TRAINING_WHEELS * Constants.MAX_ANGULAR_VELOCITY;
      }
      //var speed = input * Constants.MAX_ANGULAR_VELOCITY;
      // speed = highPassFilter(speed, Constants.MIN_ANGULAR_VELOCITY);
      return input;
    }
  };

  DoubleSupplier hangarRotation = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      var robotState = drivetrainSubsystem.getGyroscopeRotation();
      double targetRad = robotState.getRadians() + Math.PI / 2;
      if (_thetaController == null)
      {
        var profileConstraints = new TrapezoidProfile.Constraints(
                Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                Constants.MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
        _thetaController = new ProfiledPIDController(5, .1, 0, profileConstraints);
        _thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
        _thetaController.reset(targetRad);
      }

      return _thetaController.calculate(targetRad, 0);
    }
  };

  BooleanSupplier m_rejectSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return operatorToggleReject.getAsBoolean();
    }
  };

  private void setDefaultCommands() {
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

    if (centererSubsystem != null) {
      centererSubsystem.setDefaultCommand(new DefaultCentererCommand(centererSubsystem));
    }

    if (feederSubsystem != null) {
      feederSubsystem.setDefaultCommand(new DefaultFeederCommand(feederSubsystem));
    }

    if (intakeSubsystem != null) {
      intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(intakeSubsystem));
    }

    if (indexerSubsystem != null) {
      indexerSubsystem.setDefaultCommand(new DefaultIndexerCommand(indexerSubsystem));
    }

    if (shooterSubsystem != null) {
      shooterSubsystem.setDefaultCommand(new DefaultShooterCommand(shooterSubsystem));
    }

    if (climberSubsystem != null) {
      climberSubsystem.setDefaultCommand(new DefaultClimberCommand(climberSubsystem));
    }

    if (colorSensorSubsystem != null) {
      colorSensorSubsystem.setDefaultCommand(new DefaultColorSensorCommand(colorSensorSubsystem));
    }
  }

  private void setDefaultColorSensorCommand() {

  }
  
  private void setDefaultLedStatusCommand() {
    ledStatusSubsystem.setDefaultCommand(new DefaultLedStatusCommand(ledStatusSubsystem));
  }

  private void defineSubsystems() {
    if (Constants.HARDWARE_CONFIG_HAS_DRIVETRAIN) {
      drivetrainSubsystem = new Drivetrain();
    }

    if (Constants.HARDWARE_CONFIG_HAS_SHOOTER) {
      shooterSubsystem = new Shooter();
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

    if (Constants.HARDWARE_CONFIG_HAS_COLORSENSOR) {
      colorSensorSubsystem = new ColorSensor();
    }

    if(Constants.HARDWARE_CONFIG_HAS_LED) {
      ledStatusSubsystem = new LedStatus();

      if(colorSensorSubsystem != null){
        ledStatusSubsystem.setHasOneBallSupplier(() -> colorSensorSubsystem.hasOneBall());
        ledStatusSubsystem.setHasTwoBallSupplier(() -> colorSensorSubsystem.hasTwoBalls());
        ledStatusSubsystem.setHasWrongBottomBallSupplier(() -> colorSensorSubsystem.hasWrongBottomBall());
      }

      if(driverStartShootin != null){
        ledStatusSubsystem.setIsAligningSupplier(() -> driverStartShootin.getAsBoolean() );
      }

      if(shooterSubsystem != null && limelightSubsystem != null) {
        ledStatusSubsystem.setIsShootingSupplier(() -> shooterSubsystem.isFlywheelAtTargetVelocity() && limelightSubsystem.hasTarget() && limelightSubsystem.isAligned());
      }
    }
  }

  private void defineButtons() {
    // joystick declaration
    joystick0 = new Joystick(0);
    joystick1 = new Joystick(1);
    joystick2 = new Joystick(2);
    joystick3 = new Joystick(3);
    joystick4 = new Joystick(4);

    // joystick0 button declaration
    driverIntakeButton = new JoystickButton(joystick0, 1);
    driverEjectButton = new JoystickButton(joystick0, 2);
    driverFeedButton = new JoystickButton(joystick0, 11);
    resetGyro = new Button(() -> joystick0.getRawButton(4));

    // must press and hold buttons 8 and 9 to run test commands.
    testButton = new JoystickButton(joystick0, 8).and(new JoystickButton(joystick0, 9));

    // joystick1 button declaration
    driverStartShootin = new JoystickButton(joystick1, 1);
    driverStartShooter = new JoystickButton(joystick1, 3);
    driverStopShooter = new JoystickButton(joystick1, 2);
    alignTarget = new JoystickButton(joystick1, 10);
    alignWithHangar = new JoystickButton(joystick1, 4);
    // stopShootin = new JoystickButton(joystick2, 7);

    // joystick2 button declaration
    operatorIntakeButton = new JoystickButton(joystick2, 3);
    operatorEjectButton = new JoystickButton(joystick2, 2);
    operatorFeedButton = new JoystickButton(joystick2, 1);
    operatorShooterOnButton = new JoystickButton(joystick2, 5);
    climberArmTwoSwitch = new JoystickButton(joystick2, 7);
    brakeOverrideSwitch = new JoystickButton(joystick2, 6);

    // joystick3 button declaration
    climberAutoClimb = new JoystickButton(joystick3, 10);
    autoClimb_Phase1Button = new JoystickButton(joystick3, 11);
    autoClimb_Phase2Button = new JoystickButton(joystick3, 12);
    climberArmOneDownButton = new JoystickButton(joystick3, 9);
    climberArmOneUpButton = new JoystickButton(joystick3, 8);
    climberArmTwoDownButton = new JoystickButton(joystick3, 7);
    climberArmTwoUpButton = new JoystickButton(joystick3, 6);
    climberBothDownButton = new JoystickButton(joystick3, 5);
    climberBothUpButton = new JoystickButton(joystick3, 4);
    operatorYeetEjectButton = new JoystickButton(joystick3, 1);

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
      alignWithHangar.whenPressed(new InstantCommand(() -> hangarAlignOn()))
        .whenReleased(new InstantCommand(() -> hangarAlignOff()));
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null) {
      driverIntakeButton.whenHeld(new InstantCommand(() -> intakeDown()).andThen(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem, m_rejectSupplier)));
      driverIntakeButton.whenReleased(new InstantCommand(() -> intakeUp()));
      operatorIntakeButton.whenHeld(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem, m_rejectSupplier));
    }

    if (feederSubsystem != null && centererSubsystem != null && indexerSubsystem != null) {
      driverFeedButton.whileHeld(new FeedCommand(feederSubsystem, centererSubsystem, indexerSubsystem));
      operatorFeedButton.whileHeld(new FeedCommand(feederSubsystem, centererSubsystem, indexerSubsystem));
    }

    if (intakeSubsystem != null && centererSubsystem != null && indexerSubsystem != null && feederSubsystem != null) {
      driverEjectButton
          .whenHeld(new EjectCommand(centererSubsystem, indexerSubsystem, feederSubsystem, intakeSubsystem)
          .andThen(new InstantCommand(() -> colorSensorSubsystem.oneEjected())));
      operatorEjectButton
          .whileHeld(new EjectCommand(centererSubsystem, indexerSubsystem, feederSubsystem, intakeSubsystem)
          .andThen(new InstantCommand(() -> colorSensorSubsystem.makeEmpty())));
      operatorYeetEjectButton
          .whileHeld(new YeetEjectCommand(centererSubsystem, indexerSubsystem, feederSubsystem));
          
    }
    
    if (shooterSubsystem != null) {
      driverStartShootin.whenPressed(new InstantCommand(() -> limelightSubsystem.nullifyPID())
              .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem, limelightSubsystem, m_stoppedSupplier))
              .deadlineWith(new InstantCommand(() -> limelightRotationOn())
              .andThen(new InstantCommand(() -> colorSensorSubsystem.makeEmpty()))));

      driverStartShootin.whenReleased(new StopShooterCommand(shooterSubsystem).alongWith(new InstantCommand(() -> limelightRotationOff())));
      driverStopShooter.whenPressed(new StopShooterCommand(shooterSubsystem));
      operatorShooterOnButton.whenHeld(new RunShooterCommand(shooterSubsystem))
          .whenReleased(new StopShooterCommand(shooterSubsystem));
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
      autoClimb_Phase2Button.whenHeld(new AutoClimb_Phase2_v3(intakeSubsystem, climberSubsystem));
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
      new JoystickButton(joystick0, 10).whenHeld(new AlignToTargetAndShoot(servosSubsystem, limelightSubsystem, shooterSubsystem, servosSubsystem));
    }

    // testButton.whenActive(new PrintCommand("Test Button
    // Pressed").andThen(provideTestCommand()))
    // .whenInactive(new PrintCommand("Test Button
    // Released").andThen(provideAllStopCommand()));
  }

  private void limelightRotationOn() {
    limelightSubsystem.on();
    // m_rotationSupplier = limelightSubsystem.rotation;
    System.out.println("limelight rotation on");
    _limelightRotation = true;

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
    _limelightRotation = false;

    // drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // drivetrainSubsystem,
    // m_translationXSupplier,
    // m_translationYSupplier,
    // m_rotationSupplier));
  }

  private void hangarAlignOn() {
    _hangarAlign = true;
  }

  private void hangarAlignOff() {
    _hangarAlign = false;
  }
  
  private void intakeDown() {
    intakeDown = true;
  }

  private void intakeUp() {
    intakeDown = false;
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

  private static double normalize(double value, double limit)
  {
    return (value > limit) ? limit : value;
  }

  private static double modifyAxis(double value) {
    value = normalize(value, 1);
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

    Command AutoShoot5 = new ShootFromAnywhereCommand(shooterSubsystem, feederSubsystem, centererSubsystem,
        indexerSubsystem, limelightSubsystem)
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new DriveHB(drivetrainSubsystem)
                .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem,
                    colorSensorSubsystem, m_rejectSupplier)))
            .andThen(new DriveBC(drivetrainSubsystem)
                .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem,
                    colorSensorSubsystem, m_rejectSupplier)))
            .andThen(new DriveCD(drivetrainSubsystem))
            .andThen(new ShootFromAnywhereCommand(shooterSubsystem, feederSubsystem, centererSubsystem,
                indexerSubsystem, limelightSubsystem))
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new DriveDE(drivetrainSubsystem)
                .andThen(new WaitCommand(0.5))
                .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem,
                    colorSensorSubsystem, m_rejectSupplier)))
            .andThen(new DriveED(drivetrainSubsystem))
            // shoot
            .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
                limelightSubsystem, () -> true)
                    .deadlineWith(new InstantCommand(() -> limelightRotationOn())
                        .andThen(new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> 0, m_rotationSupplier))))
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new InstantCommand(() -> limelightRotationOff()));

    Command ExperimentalAutoShoot5 = new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem,
        colorSensorSubsystem, m_rejectSupplier)
            .raceWith(
                new DriveNB(drivetrainSubsystem)
                    .andThen(new InstantCommand(() -> shooterSubsystem.startFlywheel(), shooterSubsystem))
                    .andThen(new DriveBM(drivetrainSubsystem)))
            // Shoot
            .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
                limelightSubsystem, () -> true)
                    .deadlineWith(new InstantCommand(() -> limelightRotationOn())
                        .andThen(new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> 0, m_rotationSupplier))))
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new InstantCommand(() -> limelightRotationOff()))
            // Collect 2nd ball
            .andThen(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem,
                m_rejectSupplier)
                    .raceWith(
                        new DriveMC(drivetrainSubsystem)
                            .andThen(new InstantCommand(() -> shooterSubsystem.startFlywheel(), shooterSubsystem))
                            .andThen(new DriveCO(drivetrainSubsystem))))
            // Shoot
            .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
                limelightSubsystem, () -> true)
                    .deadlineWith(new InstantCommand(() -> limelightRotationOn())
                        .andThen(new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> 0, m_rotationSupplier))))
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new InstantCommand(() -> limelightRotationOff()))
            // Collect HP balls
            .andThen(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem, m_rejectSupplier)
                    .raceWith(
                      new DriveOE(drivetrainSubsystem)
                          .andThen(new WaitCommand(0.25))
                          .andThen(new DriveET(drivetrainSubsystem))
                          .andThen(new WaitCommand(0.50))))
            // Drive to main shoot location
            .andThen(new DriveTO(drivetrainSubsystem)
                .alongWith(new WaitCommand(0.25)
                    .andThen(new InstantCommand(() -> shooterSubsystem.startFlywheel(), shooterSubsystem))))
            // Shoot
            .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
                limelightSubsystem, () -> true)
                    .deadlineWith(new InstantCommand(() -> limelightRotationOn())
                        .andThen(new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> 0, m_rotationSupplier))))
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new InstantCommand(() -> limelightRotationOff()));

    // @todo update with first part of 5 ball we like most
    Command AutoShoot3 = new ShootCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem)
        .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
        .andThen(new DriveHB(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem,
                m_rejectSupplier)))
        .andThen(new DriveBC(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem,
                m_rejectSupplier)))
        .andThen(new DriveCD(drivetrainSubsystem))
        .andThen(new ShootCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem));

    Command AutoShoot2 = new DriveFG(drivetrainSubsystem)
        // .andThen(new WaitCommand(0.5))
        // .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem,
        // indexerSubsystem, colorSensorSubsystem, m_rejectSupplier))
        .andThen(new DriveGF(drivetrainSubsystem))
        .andThen(new ShootFromAnywhereCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
            limelightSubsystem));

    Command ExperimentalAutoShoot2 =
        // Collect first ball
        new DriveFG(drivetrainSubsystem)
            .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem, colorSensorSubsystem,
                m_rejectSupplier))
            .andThen(new InstantCommand(() -> shooterSubsystem.startFlywheel(), shooterSubsystem))
            // Drive to first shoot location
            .andThen(new DriveGP(drivetrainSubsystem)
                .deadlineWith(new IntakeCommand(intakeSubsystem, centererSubsystem, indexerSubsystem,
                    colorSensorSubsystem, m_rejectSupplier)))
            // Shoot
            .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
                limelightSubsystem, () -> true)
                    .deadlineWith(new InstantCommand(() -> limelightRotationOn())
                        .andThen(new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> 0, m_rotationSupplier))))
            .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
            .andThen(new InstantCommand(() -> limelightRotationOff()))
            .andThen(new DriveGS(drivetrainSubsystem));

    Command AutoShoot1 = new WaitCommand(9)
    .andThen(new DriveQR(drivetrainSubsystem))
        .andThen(new AimLockCommand(shooterSubsystem, feederSubsystem, centererSubsystem, indexerSubsystem,
            limelightSubsystem, () -> true)
                .deadlineWith(new InstantCommand(() -> limelightRotationOn())
                    .andThen(new DefaultDriveCommand(drivetrainSubsystem, () -> 0, () -> 0, m_rotationSupplier))))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopFlywheel(), shooterSubsystem))
        .andThen(new InstantCommand(() -> limelightRotationOff()));


    // Create the sendable chooser (dropdown menu) for Shuffleboard
    _autoChooser = new SendableChooser<>();
    _autoChooser.addOption("AutoShoot5", AutoShoot5);
    _autoChooser.addOption("AutoShoot3", AutoShoot3);
    _autoChooser.addOption("AutoShoot2", AutoShoot2);
    _autoChooser.addOption("ExperimentalAutoShoot2", ExperimentalAutoShoot2);
    _autoChooser.addOption("AutoShoot1", AutoShoot1);
    _autoChooser.setDefaultOption("ExperimentalAutoShoot5", ExperimentalAutoShoot5);
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

  public boolean returnLimelightRotation() {
    return _limelightRotation;
  }
}
