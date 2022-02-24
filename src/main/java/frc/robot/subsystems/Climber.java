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

public class Climber extends SubsystemBase {
  private CANSparkMax climberLeftLowerMotor;
  private CANSparkMax climberRightLowerMotor;
  private CANSparkMax climberLeftUpperMotor;
  private CANSparkMax climberRightUpperMotor;
  private Solenoid climberSolenoidRight;
  private Solenoid climberSolenoidLeft;
  private Solenoid climberSolenoidBreakOne;
  private Solenoid climberSolenoidBreakTwo;
  private Solenoid climberSolenoidBreakThree;
  private Solenoid climberSolenoidBreakFour;

  
  /** Creates a new Climber. */
  public Climber() {
    climberLeftLowerMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_ONE_MOTOR_ID, MotorType.kBrushed);
    climberRightLowerMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_ONE_MOTOR_ID, MotorType.kBrushed);
    climberLeftUpperMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_TWO_MOTOR_ID, MotorType.kBrushed);
    climberRightUpperMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_TWO_MOTOR_ID, MotorType.kBrushed);
    climberSolenoidLeft = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_LEFT);
    climberSolenoidRight = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_RIGHT);
    climberSolenoidBreakOne = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_BREAK_ONE);
    climberSolenoidBreakTwo = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_BREAK_TWO);
    climberSolenoidBreakThree = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_BREAK_THREE);
    climberSolenoidBreakFour = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_BREAK_FOUR);
  }

  public void climberLeftUp(){
    climberLeftLowerMotor.set(Constants.CLIMBER_UP_SPEED);
    climberLeftUpperMotor.set(Constants.CLIMBER_UP_SPEED);
    climberSolenoidLeft.set(true);
  }

  public void climberRightUp(){
      climberRightLowerMotor.set(Constants.CLIMBER_UP_SPEED);
      climberRightUpperMotor.set(Constants.CLIMBER_UP_SPEED);
      climberSolenoidRight.set(true);
  }

  public void climberUp(){
      climberRightUpperMotor.set(Constants.CLIMBER_UP_SPEED);
      climberLeftUpperMotor.set(Constants.CLIMBER_UP_SPEED);
      climberLeftLowerMotor.set(Constants.CLIMBER_UP_SPEED);
      climberRightLowerMotor.set(Constants.CLIMBER_UP_SPEED);
      climberSolenoidRight.set(true);
      climberSolenoidLeft.set(true);
  }

  public void climberLeftDown(){
    climberLeftLowerMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberLeftUpperMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberSolenoidLeft.set(false);
  }

  public void climberRightDown(){
      climberRightLowerMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberRightUpperMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberSolenoidRight.set(false);
  }

  public void climberDown(){
      climberRightUpperMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberLeftUpperMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberLeftLowerMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberRightLowerMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberSolenoidRight.set(false);
      climberSolenoidLeft.set(false);
  }

  public void climberBreakOn(){
    climberSolenoidBreakOne.set(true);
    climberSolenoidBreakTwo.set(true);
    climberSolenoidBreakThree.set(true);
    climberSolenoidBreakFour.set(true);
  }
  
  public void climberBreakOff(){
    climberSolenoidBreakOne.set(false);
    climberSolenoidBreakTwo.set(false);
    climberSolenoidBreakThree.set(false);
    climberSolenoidBreakFour.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
