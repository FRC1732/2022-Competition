// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
  }

  public void forward(){
    intakeMotor.set(Constants.INTAKE_FWD_SPEED);
  }

  public void reverse(){
    intakeMotor.set(Constants.INTAKE_REVERSE_SPEED);
  }

  public void stop(){
    intakeMotor.set(0);
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
