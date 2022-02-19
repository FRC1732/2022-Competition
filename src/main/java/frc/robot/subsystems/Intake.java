// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private Solenoid intakeSolenoidLeft;
  private Solenoid intakeSolenoidRight;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.CAN_INTAKE_MOTOR, MotorType.kBrushed);
    intakeSolenoidLeft = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_CHANNEL_LEFT);
    intakeSolenoidRight = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_CHANNEL_RIGHT);
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

  public void deploy(){
    intakeSolenoidLeft.set(true);
    intakeSolenoidRight.set(true);
  }

  public void retract(){
    intakeSolenoidLeft.set(false);
    intakeSolenoidRight.set(false);
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
