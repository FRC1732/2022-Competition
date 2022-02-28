// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.Constants.RobotDesignation;

public class Indexer extends SubsystemBase {
private CANSparkMax indexerMotor;
  /** Creates a new Indexer. */
  public Indexer() {
    if (RobotConfig.ROBOT_DESIGNATION.equals(RobotDesignation.COMPETITION)) {
      indexerMotor = new CANSparkMax (Constants.CAN_INDEXER_MOTOR, MotorType.kBrushless);
      // TODO need to check firmware config.  This may be invented in firmware
    } else {
      indexerMotor = new CANSparkMax (Constants.CAN_INDEXER_MOTOR, MotorType.kBrushed);
      indexerMotor.setInverted(false);
    }

    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);    
    
  }

  public void forward(){
    indexerMotor.set(Constants.FOWARDS_INDEX_SPEED);
  }

  public void forwardSlow(){
    indexerMotor.set(Constants.FOWARDS_INDEX_SPEED_SLOW);
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
