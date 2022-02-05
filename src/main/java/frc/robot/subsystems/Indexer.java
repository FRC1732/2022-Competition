// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
private CANSparkMax indexerMotor;
  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new CANSparkMax (Constants.INDEXER_MOTOR, MotorType.kBrushless);
  }

  public void forward(){
    indexerMotor.set(Constants.FOWARDS_INDEX_SPEED);
  }

  public void reverse(){
    indexerMotor.set(Constants.REVERSE_INDEX_SPEED);
  }

  public void stop(){
    indexerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
