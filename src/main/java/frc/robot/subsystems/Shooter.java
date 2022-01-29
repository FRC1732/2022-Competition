// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private final double TARGET_SPEED = 0.7;
  private NetworkTableEntry shooterSpeed;
  private double speed;
  private final TalonFX shooterLeft = new TalonFX(SHOOTER_LEFT);
  private final TalonFX shooterRight = new TalonFX(SHOOTER_RIGHT);

  /** Creates a new Shooter. */
  public Shooter() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    shooterSpeed = tab.add("Shooter Speed", 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterLeft.setNeutralMode(NeutralMode.Coast);
    shooterRight.setNeutralMode(NeutralMode.Coast);

    shooterLeft.configClosedloopRamp(.1);
    shooterRight.configClosedloopRamp(.1);
  }

  public void start(double speed) {
    shooterSpeed.setDouble(speed);
    this.speed = speed;
    // shooterLeft.set(ControlMode.PercentOutput, speed);
    // shooterRight.set(ControlMode.PercentOutput, speed);
  }

  public void start() {
    speed = shooterSpeed.getDouble(TARGET_SPEED);
    // shooterLeft.set(ControlMode.PercentOutput, shooterSpeed.getDouble(TARGET_SPEED));
    // shooterRight.set(ControlMode.PercentOutput, shooterSpeed.getDouble(TARGET_SPEED));
  }

  public void stop() {
    speed = 0;
    shooterLeft.set(ControlMode.PercentOutput, 0);
    shooterRight.set(ControlMode.PercentOutput, 0);
  }

  public void increaseSpeed() {
    double tempSpeed = shooterSpeed.getDouble(TARGET_SPEED) + 0.05;
    if (tempSpeed > 1.0) {
      speed = 1.0;
    } else {
      speed = tempSpeed;
    }
    shooterSpeed.setDouble(speed);
  }

  public void decreaseSpeed() {
    double tempSpeed = shooterSpeed.getDouble(TARGET_SPEED) - 0.05;
    if (tempSpeed < -1.0) {
      speed = -1.0;
    } else {
      speed = tempSpeed;
    }
    shooterSpeed.setDouble(speed);
  }

  @Override
  public void periodic() {
    shooterLeft.set(ControlMode.PercentOutput, speed);
    shooterRight.set(ControlMode.PercentOutput, speed);
  }
}
