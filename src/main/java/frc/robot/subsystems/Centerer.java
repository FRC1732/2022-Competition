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

public class Centerer extends SubsystemBase {
  private CANSparkMax centererMotor;
  /** Creates a new Centerer. */
  public Centerer() {
    if (RobotConfig.ROBOT_DESIGNATION.equals(RobotDesignation.COMPETITION)) {
      centererMotor = new CANSparkMax(Constants.CAN_CENTERER_MOTOR, MotorType.kBrushless);
      centererMotor.setInverted(true);  // 'cause of differnt motor placement
    } else {
      centererMotor = new CANSparkMax(Constants.CAN_CENTERER_MOTOR, MotorType.kBrushed);
    }

    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    centererMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);
  }

  public void forward(){
    centererMotor.set(Constants.CENTERER_FORWARD_SPEED);
  }

  public void reverse(){
    centererMotor.set(Constants.CENTERER_REVERSE_SPEED);
  }

  public void stop(){
    centererMotor.set(0);
  }

  /*public void beastFwd(){
    intakeMotor.set(1);
  }

  public void beastReverse(){
    intakeMotor.set(-1);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
