// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Centerer extends SubsystemBase {
  private CANSparkMax centererMotor;

  /** Creates a new Centerer. */
  public Centerer() {
    centererMotor = new CANSparkMax(Constants.CAN_CENTERER_MOTOR, MotorType.kBrushless);

    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);
  }

  public void forward() {
    centererMotor.set(Constants.CENTERER_FORWARD_SPEED);
  }

  public void reverse() {
    centererMotor.set(Constants.CENTERER_REVERSE_SPEED);
  }

  public void max_reverse() {
    centererMotor.set(Constants.CENTERER_MAX_REVERSE_SPEED);
  }
  

  public void stop() {
    centererMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
