// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import static frc.robot.RobotConfig.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterLeft = new TalonFX(CAN_SHOOTER_MOTOR_LEFT);
  private final TalonFX shooterRight = new TalonFX(CAN_SHOOTER_MOTOR_RIGHT);
  private Solenoid hoodPneumatic = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.CTREPCM,
      Constants.SHOOTER_SOLENOID_CHANNEL_HOOD);
  private TalonFXConfiguration flywheelConfiguration;
  private double r_fwVelocity, r_fwTargetVelocity, r_fwPosition;
  private boolean r_fw_IsAtTargetVelocity;
  private boolean _hoodPosition;
  private boolean _slowShot;
  private double targetFarRpm = TARGET_RPM_FAR;
  private double targetNearRpm = TARGET_RPM_NEAR;

  public Shooter() {
    configureComponents();
    configureShuffleboard();
  }

  private void configureShuffleboard() {

    ShuffleboardTab tab;

    switch (RobotConfig.SB_LOGGING) {
      case COMPETITION:
        tab = Shuffleboard.getTab("COMPETITION");
        tab.addBoolean("AT SPEED", bs_FlyWheelAtSpeed)
            .withPosition(4, 0)
            .withSize(1, 2);
        break;
      case DEBUG:
        tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Flywheel Target", ds_FlywheelTargetVelocity);
        tab.addNumber("Flywheel Speed", ds_FlywheelVelocity);
        tab.addBoolean("FLYWHEEL AT SPEED", bs_FlyWheelAtSpeed).withSize(2, 1);
        // ==== FOR DEVELOPMENT PURPOSES ONLY ====
        // shooterSpeed = tab.add("Shooter Speed", 1)
        // .withWidget(BuiltInWidgets.kNumberSlider)
        // .withPosition(0, 0)
        // .withSize(2, 1)
        // .getEntry();
        break;
      case NONE:
      default:
        break;
    }
  }

  private void configureComponents() {
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterRight.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    shooterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    if (RobotConfig.ROBOT_DESIGNATION.equals(RobotDesignation.COMPETITION)) {
      shooterRight.setInverted(true); // Comm bot has motors mounted facing opposite of each other.
    }

    flywheelConfiguration = new TalonFXConfiguration();
    flywheelConfiguration.slot0.kP = FLYWHEEL_P;
    flywheelConfiguration.slot0.kI = FLYWHEEL_I;
    flywheelConfiguration.slot0.kD = FLYWHEEL_D;
    flywheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    flywheelConfiguration.supplyCurrLimit.currentLimit = FLYWHEEL_CURRENT_LIMIT;
    flywheelConfiguration.supplyCurrLimit.enable = true;
    flywheelConfiguration.voltageCompSaturation = 11.5;

    shooterLeft.configAllSettings(flywheelConfiguration);
    shooterRight.configAllSettings(flywheelConfiguration);

    shooterLeft.setNeutralMode(NeutralMode.Coast);
    shooterRight.setNeutralMode(NeutralMode.Coast);

    shooterLeft.enableVoltageCompensation(false);
    shooterRight.enableVoltageCompensation(false);

    shooterRight.follow(shooterLeft);
  }

  public void setSlowShot(boolean isSlow){
    _slowShot = isSlow;
  }

  public void startFlywheel() {
    double speed = _hoodPosition ? targetFarRpm : targetNearRpm;
    speed = speed - (_slowShot ? 50 : 0);
    shootFlywheel(speed);// * shooterSpeed.getDouble(1));
  }

  public void stopFlywheel() {
    shooterLeft.set(ControlMode.Disabled, 0);
  }

  public void extendHood() {
    _hoodPosition = true;
    hoodPneumatic.set(true);
  }

  public void retractHood() {
    _hoodPosition = false;
    hoodPneumatic.set(false);
  }

  private void shootFlywheel(double speed) {
    double feedforward = (FLYWHEEL_FEEDFORWARD_COEFFICIENT * speed + FLYWHEEL_STATIC_FRICTION_CONSTANT)
        / RobotController.getBatteryVoltage();
    shooterLeft.set(ControlMode.Velocity, speed / FLYWHEEL_TICKS_TO_RPM_COEFFICIENT, DemandType.ArbitraryFeedForward,
        feedforward);
  }

  public double getFlywheelPosition() {
    return r_fwPosition;
  }

  public double getFlywheelVelocity() {
    return r_fwVelocity;
  }

  public double getFlywheelTargetVelocity() {
    return r_fwTargetVelocity;
  }

  public boolean isFlywheelAtTargetVelocity() {
    return r_fw_IsAtTargetVelocity;
  }

  public void resetFlywheelPosition() {
    shooterLeft.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
  }

  public void setTargetFarRpm(double target) {
    targetFarRpm = target;
  }

  DoubleSupplier ds_FlywheelVelocity = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return r_fwVelocity;
    }
  };

  DoubleSupplier ds_FlywheelTargetVelocity = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return r_fwTargetVelocity;
    }
  };

  BooleanSupplier bs_FlyWheelAtSpeed = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_fw_IsAtTargetVelocity;
    }
  };

  @Override
  public void periodic() {
    r_fwVelocity = shooterLeft.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_TICKS_TO_RPM_COEFFICIENT;

    // Need to check if the controller is in Velocity mode - otherwise CTR Error
    // appears in console
    // if (shooterLeft.getControlMode().equals(ControlMode.Velocity))
    //   r_fwTargetVelocity = shooterLeft.getClosedLoopTarget() * FLYWHEEL_TICKS_TO_RPM_COEFFICIENT;
    r_fwTargetVelocity = _hoodPosition ? targetFarRpm : targetNearRpm;

    r_fwPosition = shooterLeft.getSensorCollection().getIntegratedSensorPosition()
        * FLYWHEEL_TICKS_TO_ROTATIONS_COEFFICIENT;
    r_fw_IsAtTargetVelocity = (Math.abs(r_fwVelocity - r_fwTargetVelocity) < FLYWHEEL_ALLOWABLE_ERROR) ? true : false;
  }
}
