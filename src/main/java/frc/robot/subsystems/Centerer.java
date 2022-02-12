// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Centerer extends SubsystemBase {
  private CANSparkMax centererMotor;
  /** Creates a new Centerer. */
  public Centerer() {
    centererMotor = new CANSparkMax(Constants.CENTERER, MotorType.kBrushed);
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
