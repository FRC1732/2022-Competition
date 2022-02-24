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
  private CANSparkMax climberLeftArmTwoMotor;
  private CANSparkMax climberRightArmOneMotor;
  private CANSparkMax climberLeftArmOneMotor;
  private CANSparkMax climberRightArmTwoMotor;
  private Solenoid climberSolenoidRightTilter;
  private Solenoid climberSolenoidLeftTilter;
  private Solenoid climberSolenoidLeftBreakOne;
  private Solenoid climberSolenoidRightBreakOne;
  private Solenoid climberSolenoidLeftBreakTwo;
  private Solenoid climberSolenoidRightBreakTwo;

  
  /** Creates a new Climber. */
  public Climber() {
    climberLeftArmOneMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_ONE_MOTOR_ID, MotorType.kBrushed);
    climberRightArmOneMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_ONE_MOTOR_ID, MotorType.kBrushed);
    climberLeftArmTwoMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_TWO_MOTOR_ID, MotorType.kBrushed);
    climberRightArmTwoMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_TWO_MOTOR_ID, MotorType.kBrushed);
    climberSolenoidLeftTilter = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_LEFT_TILTER);
    climberSolenoidRightTilter = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_RIGHT_TILTER);
    climberSolenoidLeftBreakOne = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_LEFT_BREAK_ONE);
    climberSolenoidRightBreakOne = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_RIGHT_BREAK_ONE);
    climberSolenoidLeftBreakTwo = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_LEFT_BREAK_TWO);
    climberSolenoidRightBreakTwo = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_RIGHT_BREAK_TWO);
  }

  public void climberLeftUp(){
    climberLeftArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
    climberLeftArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberRightUp(){
      climberRightArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
      climberRightArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberUp(){
      climberRightArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
      climberLeftArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
      climberLeftArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
      climberRightArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberLeftDown(){
    climberLeftArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberLeftArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberRightDown(){
      climberRightArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberRightArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberDown(){
      climberRightArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberLeftArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberLeftArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberRightArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void tiltLeftArmTwo(){
    climberSolenoidLeftTilter.set(true);
  }

  public void untiltLeftArmTwo(){
    climberSolenoidLeftTilter.set(false);
  }

  public void tiltRightArmTwo(){
    climberSolenoidRightTilter.set(true);
  }

  public void untiltRightArmTwo(){
    climberSolenoidRightTilter.set(false);
  }

  public void climberBreakArmOneOn(){
    climberSolenoidLeftBreakOne.set(true);
    climberSolenoidRightBreakOne.set(true);
  }
  
  public void climberBreakOneOff(){
    climberSolenoidLeftBreakOne.set(false);
    climberSolenoidRightBreakOne.set(false);
  }

  public void climberBreakArmTwoOn(){
    climberSolenoidLeftBreakTwo.set(true);
    climberSolenoidRightBreakTwo.set(true);
  }

  public void climberBreakTwoOff(){
    climberSolenoidLeftBreakTwo.set(false);
    climberSolenoidRightBreakTwo.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
