// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterLeft = new TalonFX(SHOOTER_LEFT);
  private final TalonFX shooterRight = new TalonFX(SHOOTER_RIGHT);
  /** Creates a new Shooter. */
  public Shooter() {
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterLeft.setNeutralMode(NeutralMode.Coast);
    shooterRight.setNeutralMode(NeutralMode.Coast);

    shooterLeft.configClosedloopRamp(.1);
    shooterRight.configClosedloopRamp(.1);
  }

  public void start() {
    shooterLeft.set(ControlMode.PercentOutput, SHOOTER_SPEED);
    shooterRight.set(ControlMode.PercentOutput, SHOOTER_SPEED);
  }

  public void stop() {
    shooterLeft.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
