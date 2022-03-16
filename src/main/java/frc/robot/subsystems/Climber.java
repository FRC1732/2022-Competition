// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberLeftArmTwoMotor;
  private CANSparkMax climberRightArmOneMotor;
  private CANSparkMax climberLeftArmOneMotor;
  private CANSparkMax climberRightArmTwoMotor;
  private RelativeEncoder climberLeftArmOneEncoder;
  private RelativeEncoder climberLeftArmTwoEncoder;
  private RelativeEncoder climberRightArmOneEncoder;
  private RelativeEncoder climberRightArmTwoEncoder;
  private Solenoid climberSolenoidTilter;
  private Solenoid climberSolenoidStationaryBrakeOne;
  private Solenoid climberSolenoidMovingBrakeTwo;
  private Double climberLeftArmOneMotorPosition;
  private Double climberLeftArmTwoMotorPosition;
  private Double climberRightArmOneMotorPosition;
  private Double climberRightArmTwoMotorPosition;

  public Boolean brakeOverride;

  private enum Mode {
    autoExtend, autoRetract, owenMode
  };

  private Mode mode;

  /** Creates a new Climber. */
  public Climber() {
    configureShuffleBoard();

    climberLeftArmOneMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_ONE_MOTOR_ID, MotorType.kBrushless);
    climberRightArmOneMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_ONE_MOTOR_ID, MotorType.kBrushless);
    climberLeftArmTwoMotor = new CANSparkMax(Constants.CAN_CLIMBER_LEFT_ARM_TWO_MOTOR_ID, MotorType.kBrushless);
    climberRightArmTwoMotor = new CANSparkMax(Constants.CAN_CLIMBER_RIGHT_ARM_TWO_MOTOR_ID, MotorType.kBrushless);

    climberRightArmOneEncoder = climberRightArmOneMotor.getEncoder();
    climberRightArmTwoEncoder = climberRightArmTwoMotor.getEncoder();
    climberLeftArmOneEncoder = climberLeftArmOneMotor.getEncoder();
    climberLeftArmTwoEncoder = climberLeftArmTwoMotor.getEncoder();

    climberRightArmOneEncoder.setPosition(0);
    climberRightArmTwoEncoder.setPosition(0);
    climberLeftArmOneEncoder.setPosition(0);
    climberLeftArmTwoEncoder.setPosition(0);

    climberLeftArmOneMotorPosition = 0.0;
    climberLeftArmTwoMotorPosition = 0.0;
    climberRightArmOneMotorPosition = 0.0;
    climberRightArmTwoMotorPosition = 0.0;

    climberSolenoidTilter = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.CTREPCM,
        Constants.CLIMBER_SOLENOID_CHANNEL_BOTH_TILTER);
    climberSolenoidStationaryBrakeOne = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.CTREPCM,
        Constants.CLIMBER_SOLENOID_CHANNEL_STATIONARY_BRAKE_ONE);
    climberSolenoidMovingBrakeTwo = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.CTREPCM,
        Constants.CLIMBER_SOLENOID_CHANNEL_MOVING_BRAKE_TWO);

    climberLeftArmOneMotor.restoreFactoryDefaults();
    climberLeftArmTwoMotor.restoreFactoryDefaults();
    climberRightArmOneMotor.restoreFactoryDefaults();
    climberRightArmTwoMotor.restoreFactoryDefaults();

    climberLeftArmOneMotor.setInverted(true);
    climberLeftArmTwoMotor.setInverted(true);

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

    climberLeftArmOneMotor.setIdleMode(IdleMode.kBrake);
    climberRightArmOneMotor.setIdleMode(IdleMode.kBrake);
    climberLeftArmTwoMotor.setIdleMode(IdleMode.kBrake);
    climberRightArmTwoMotor.setIdleMode(IdleMode.kBrake);

    brakeOverride = false;
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("climber");
    tab.addNumber("left arm 1", leftArmOneSupplier);
    tab.addNumber("left arm 2", leftArmTwoSupplier);
    tab.addNumber("right arm 1", rightArmOneSupplier);
    tab.addNumber("right arm 2", rightArmTwoSupplier);
  }

  DoubleSupplier leftArmOneSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return climberLeftArmOneMotorPosition;
    }
  };

  DoubleSupplier leftArmTwoSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return climberLeftArmTwoMotorPosition;
    }
  };

  DoubleSupplier rightArmOneSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return climberRightArmOneMotorPosition;
    }
  };

  DoubleSupplier rightArmTwoSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return climberRightArmTwoMotorPosition;
    }
  };

  public void climberArmOneUp() {
    retractBrakes();
    // climberRightArmOneMotor.getEncoder().getPosition();
    climberRightArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
    climberLeftArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberArmOneDown() {
    retractBrakes();
    climberLeftArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberRightArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberArmOneStop() {
    climberLeftArmOneMotor.set(0);
    climberRightArmOneMotor.set(0);
  }

  public void climberArmTwoUp() {
    retractBrakes();
    climberLeftArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
    climberRightArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberArmTwoDown() {
    retractBrakes();
    climberLeftArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberRightArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberArmTwoStop() {
    climberLeftArmTwoMotor.set(0);
    climberRightArmTwoMotor.set(0);
  }

  public void armsAllUp() {
    climberArmOneUp();
    climberArmTwoUp();
  }

  public void armsAllDown() {
    climberArmTwoDown();
    climberArmOneDown();
  }

  public void armsAllStop() {
    climberArmTwoStop();
    climberArmOneStop();
  }

  /**
   * Environment: Pneumatic piston is plumbed so the default state is RETRACTED
   * (VERTICAL)
   * Component: Climber 2 (Moving)
   * Solenoid: 11
   * Execution: Sets solenoid state to FALSE
   * Result: EXTENDS brake piston (Tilts arm)
   */
  public void ArmTwoOut() {
    climberSolenoidTilter.set(true);
  }

  /**
   * Environment: Pneumatic piston is plumbed so the default state is RETRACTED
   * (VERTICAL)
   * Component: Climber 2 (Moving)
   * Solenoid: 11
   * Execution: Sets solenoid state to FALSE
   * Result: RETRACTS brake piston (Returns arm to vertical)
   */
  public void ArmTwoIn() {
    climberSolenoidTilter.set(false);
  }

  /**
   * Environment: Pneumatic piston is plumbed so the default state is EXTENDED
   * Component: Climber 1 (Stationary)
   * Solenoid: 12
   * Execution: Sets solenoid state to FALSE
   * Result: EXTENDS brake piston
   */
  public void armOneBrakeExtend() {
    climberSolenoidStationaryBrakeOne.set(false);
  }

  /**
   * Environment: Pneumatic piston is plumbed so the default state is EXTENDED
   * Component: Climber 1 (Stationary)
   * Solenoid: 12
   * Execution: Sets solenoid state to TRUE
   * Result: RETRACTS brake piston
   */
  public void armOneBrakeRetract() {
    climberSolenoidStationaryBrakeOne.set(true);
  }

  /**
   * Environment: Pneumatic piston is plumbed so the default state is EXTENDED
   * Component: Climber 2 (Moving)
   * Solenoid: 13
   * Execution: Sets solenoid state to FALSE
   * Result: EXTENDS brake piston
   */
  public void armTwoBrakeExtend() {
    climberSolenoidMovingBrakeTwo.set(false);
  }

  /**
   * Environment: Pneumatic piston is plumbed so the default state is EXTENDED
   * Component: Climber 2 (Moving)
   * Solenoid: 13
   * Execution: Sets solenoid state to TRUE
   * Result: RETRACTS brake piston
   */
  public void armTwoBrakeRetract() {
    climberSolenoidMovingBrakeTwo.set(true);
  }

  public void retractBrakes() {
    armOneBrakeRetract();
    armTwoBrakeRetract();
  }

  public void extendBrakes() {
    armOneBrakeExtend();
    armTwoBrakeExtend();
  }

  public void finishClimb() {
    climberArmOneUp();
    climberArmTwoDown();
  }

  public void enableBrakeOverride() {
    brakeOverride = true;
    retractBrakes();
  }

  public void disableBrakeOverride() {
    brakeOverride = false;
    extendBrakes();
  }

  @Override
  public void periodic() {
    climberLeftArmOneMotorPosition = climberLeftArmOneEncoder.getPosition() < 0 ? 0
        : climberLeftArmOneEncoder.getPosition();
    climberLeftArmTwoMotorPosition = climberLeftArmTwoEncoder.getPosition() < 0 ? 0
        : climberLeftArmTwoEncoder.getPosition();
    climberRightArmOneMotorPosition = climberRightArmOneEncoder.getPosition() < 0 ? 0
        : climberRightArmOneEncoder.getPosition();
    climberRightArmTwoMotorPosition = climberRightArmTwoEncoder.getPosition() < 0 ? 0
        : climberRightArmTwoEncoder.getPosition();
  }
}
