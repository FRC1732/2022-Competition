// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import static frc.robot.RobotConfig.*;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterLeft = new TalonFX(CAN_SHOOTER_MOTOR_LEFT);
  private final TalonFX shooterRight = new TalonFX(CAN_SHOOTER_MOTOR_RIGHT);
  private NetworkTableEntry shooterSpeed;
  
  /** Creates a new Shooter. */
  public Shooter() {
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterRight.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    shooterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
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
    // shooterRight.setInverted(TalonFXInvertType.Clockwise);

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    shooterSpeed = tab.add("Shooter Speed", 1)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withPosition(0, 0)
          .withSize(2, 1)
          .getEntry();
    // FIXME: adding these to shuffleboard causes issues, figure out why that is/fix it
    // tab.addBoolean("Is Flywheel at Target", this::isFlywheelAtTargetVelocity)
    //       .withPosition(0, 1)
    //       .withSize(2, 1);
    // tab.addNumber("Flywheel Target", this::getFlywheelTargetVelocity)
    //       .withPosition(0, 2)
    //       .withSize(2, 1);
    tab.addNumber("Flywheel Speed", this::getFlywheelVelocity)
          .withPosition(0, 3)
          .withSize(2, 1);
  }

  public void setFlywheelCurrentLimitEnabled(boolean enabled) {
    SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
    config.currentLimit = FLYWHEEL_CURRENT_LIMIT;
    config.enable = enabled;
    shooterLeft.configSupplyCurrentLimit(config, 0);
    shooterRight.configSupplyCurrentLimit(config, 0);
  }

  public void shootFlywheel() {
    shootFlywheel(shooterSpeed.getDouble(1)*TARGET_RPM);
  }

  public void shootFlywheel(double speed) {
    double feedforward = (FLYWHEEL_FEEDFORWARD_COEFFICIENT * speed + FLYWHEEL_STATIC_FRICTION_CONSTANT) / RobotController.getBatteryVoltage();

    shooterLeft.set(ControlMode.Velocity, speed / FLYWHEEL_TICKS_TO_RPM_COEFFICIENT, DemandType.ArbitraryFeedForward, feedforward);
  }

  public void setFlywheelOutput(double percentage) {
    shooterLeft.set(ControlMode.PercentOutput, percentage);
  }

  public void stopFlywheel() {
    shooterLeft.set(ControlMode.Disabled, 0);
  }

  public double getFlywheelPosition() {
    return shooterLeft.getSensorCollection().getIntegratedSensorPosition() * FLYWHEEL_TICKS_TO_ROTATIONS_COEFFICIENT;
  }

  public double getFlywheelVelocity() {
    return shooterLeft.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_TICKS_TO_RPM_COEFFICIENT;
  }

  public double getFlywheelTargetVelocity() {
    return shooterLeft.getClosedLoopTarget() * FLYWHEEL_TICKS_TO_RPM_COEFFICIENT;
  }

  public void resetFlywheelPosition() {
    shooterLeft.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
  }

  public boolean isFlywheelAtTargetVelocity() {
    return Math.abs(getFlywheelVelocity() - getFlywheelTargetVelocity()) < FLYWHEEL_ALLOWABLE_ERROR;
  }
}
