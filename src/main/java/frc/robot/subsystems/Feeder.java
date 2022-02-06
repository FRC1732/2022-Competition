// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;
  /** Creates a new Intake. */
  public Feeder() {
    feederMotor = new CANSparkMax(Constants.FEEDER, MotorType.kBrushed);
    feederMotor.setInverted(true);
  }

  public void forward(){
    feederMotor.set(Constants.FEEDER_FWD_SPEED);
  }

  public void backward(){
    feederMotor.set(Constants.FEEDER_BACKWARD_SPEED);
  }

  public void stop(){
    feederMotor.set(0);
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
