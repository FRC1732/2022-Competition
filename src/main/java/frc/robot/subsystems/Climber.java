// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberLeftArmTwoMotor;
  private CANSparkMax climberRightArmOneMotor;
  private CANSparkMax climberLeftArmOneMotor;
  private CANSparkMax climberRightArmTwoMotor;
  private Solenoid climberSolenoidTilter;
  private Solenoid climberSolenoidStationaryBrakeOne;
  private Solenoid climberSolenoidMovingBrakeTwo;
  public Boolean brakeOverride;

  
  /** Creates a new Climber. */
  public Climber() {
    climberLeftArmOneMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_ONE_MOTOR_ID, MotorType.kBrushed);
    climberRightArmOneMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_ONE_MOTOR_ID, MotorType.kBrushed);
    climberLeftArmTwoMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_TWO_MOTOR_ID, MotorType.kBrushed);
    climberRightArmTwoMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_TWO_MOTOR_ID, MotorType.kBrushed);
    climberSolenoidTilter = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_BOTH_TILTER);
    climberSolenoidStationaryBrakeOne = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_STATIONARY_BRAKE_ONE);
    climberSolenoidMovingBrakeTwo = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_CHANNEL_MOVING_BRAKE_TWO);
    
    climberLeftArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberLeftArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    climberLeftArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberLeftArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    brakeOverride = false;
  }

  public void climberArmTwoUp(){
    climberLeftArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
    climberRightArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberArmOneUp(){
      climberRightArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
      climberLeftArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberUp(){
      climberArmOneUp();
      climberArmTwoUp();
  }

  public void climberArmOneDown(){
    climberLeftArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberRightArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberArmTwoDown(){
      climberLeftArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
      climberRightArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberDown(){
    climberArmTwoDown();
    climberArmOneDown();
  }

  public void climberArmOneStop(){
    climberLeftArmOneMotor.set(0);
    climberRightArmOneMotor.set(0);
  }

  public void climberArmTwoStop(){
    climberLeftArmTwoMotor.set(0);
    climberRightArmTwoMotor.set(0);
  }

  public void climberAllStop(){
    climberArmTwoStop();
    climberArmOneStop();
  }

  public void bothArmTwoOut(){
    climberSolenoidTilter.set(true);
  }

  public void bothArmTwoIn(){
    climberSolenoidTilter.set(false);
  }

  public void ArmTwoOut(){
    bothArmTwoOut();
  }

  public void ArmTwoIn(){
    bothArmTwoIn();
  }

  public void climberBrakeArmOneOn(){
    climberSolenoidStationaryBrakeOne.set(true);
  }
  
  public void climberBrakeOneOff(){
    climberSolenoidStationaryBrakeOne.set(false);
  }

  public void climberBrakeArmTwoOn(){
    climberSolenoidMovingBrakeTwo.set(true);
  }

  public void climberBrakeTwoOff(){
    climberSolenoidMovingBrakeTwo.set(false);
  }

  public void climberBrakeAllOn(){
    climberBrakeArmOneOn();
    climberBrakeArmTwoOn();
  }

  public void climberBrakeAllOff(){
    climberBrakeOneOff();
    climberBrakeTwoOff();
  }

  public void finishClimb(){
    climberArmOneUp();
    climberArmTwoDown();
  }

  public void enableBrakeOverride(){
    brakeOverride = true;
  }

  public void disableBrakeOverride(){
    brakeOverride = false;
  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
