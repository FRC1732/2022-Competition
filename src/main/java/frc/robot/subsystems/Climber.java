// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  public Boolean autoClimbEnable;

  private ProfiledPIDController _thetaControllerLeftOne;
  private ProfiledPIDController _thetaControllerLeftTwo;
  private ProfiledPIDController _thetaControllerRightOne;
  private ProfiledPIDController _thetaControllerRightTwo;  

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

    //Set default values here in constructor to avoid NullPointerException when Shuffleboard tries to read these values 
    climberLeftArmOneMotorPosition = 0.0;
    climberLeftArmTwoMotorPosition = 0.0;
    climberRightArmOneMotorPosition = 0.0;
    climberRightArmTwoMotorPosition = 0.0;
    zeroEncoderPositions();

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
    //climberLeftArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberLeftArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    //climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberRightArmOneMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    //climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberLeftArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    //climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    climberRightArmTwoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    climberLeftArmOneMotor.setIdleMode(IdleMode.kBrake);
    climberRightArmOneMotor.setIdleMode(IdleMode.kBrake);
    climberLeftArmTwoMotor.setIdleMode(IdleMode.kBrake);
    climberRightArmTwoMotor.setIdleMode(IdleMode.kBrake);

    brakeOverride = false;
    autoClimbEnable = false;
  }

  public void zeroEncoderPositions() {
    climberRightArmOneEncoder.setPosition(0);
    climberRightArmTwoEncoder.setPosition(0);
    climberLeftArmOneEncoder.setPosition(0);
    climberLeftArmTwoEncoder.setPosition(0);
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("climber");
    tab.addNumber("front left arm", leftArmOneSupplier).withPosition(0, 0);
    ;
    tab.addNumber("back left arm", leftArmTwoSupplier).withPosition(0, 1);
    tab.addNumber("front right arm", rightArmOneSupplier).withPosition(5, 0);
    tab.addNumber("back right arm", rightArmTwoSupplier).withPosition(5, 1);

    tab.addBoolean("front left arm - at extend target", leftArmOneAtExtendTarget).withPosition(1, 0).withSize(2, 1);
    tab.addBoolean("back left arm - at extend target", leftArmTwoAtExtendTarget).withPosition(1, 1).withSize(2, 1);
    tab.addBoolean("front right arm - at extend target", rightArmOneAtExtendTarget).withPosition(3, 0).withSize(2, 1);
    tab.addBoolean("back right arm - at extend target", rightArmTwoAtExtendTarget).withPosition(3, 1).withSize(2, 1);

    tab.addBoolean("front left arm - at retract target", leftArmOneAtRetractTarget).withPosition(5, 0).withSize(2, 1);
    tab.addBoolean("back left arm - at retract target", leftArmTwoAtRetractTarget).withPosition(5, 1).withSize(2, 1);
    tab.addBoolean("front right arm - at retract target", rightArmOneAtRetractTarget).withPosition(7, 0).withSize(2, 1);
    tab.addBoolean("back right arm - at retract target", rightArmTwoAtRetractTarget).withPosition(7, 1).withSize(2, 1);
  }

  public void nullifyPID() {
    _thetaControllerLeftOne = null;
    _thetaControllerLeftTwo = null;
    _thetaControllerRightOne = null;
    _thetaControllerRightTwo = null;
  }

  DoubleSupplier leftArmOneExtendSpeedSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      if (_thetaControllerLeftOne == null)
      {
        var profileConstraints = new TrapezoidProfile.Constraints(
                Constants.CLIMBER_MOTOR_MAX_RPM,
                100);
        _thetaControllerLeftOne = new ProfiledPIDController(50, 1, 0, profileConstraints);
        _thetaControllerLeftOne.enableContinuousInput(Math.PI * -1, Math.PI);
        _thetaControllerLeftOne.reset(Constants.CLIMBER_FRONT_EXTEND_TARGET_POSITION);
      }

      return _thetaControllerLeftOne.calculate(Constants.CLIMBER_FRONT_EXTEND_TARGET_POSITION, 0);
    }
  };

  DoubleSupplier leftArmTwoExtendSpeedSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      if (_thetaControllerLeftTwo == null)
      {
        var profileConstraints = new TrapezoidProfile.Constraints(
                Constants.CLIMBER_MOTOR_MAX_RPM,
                100);
        _thetaControllerLeftTwo = new ProfiledPIDController(50, 1, 0, profileConstraints);
        _thetaControllerLeftTwo.enableContinuousInput(Math.PI * -1, Math.PI);
        _thetaControllerLeftTwo.reset(Constants.CLIMBER_BACK_EXTEND_TARGET_POSITION);
      }

      return _thetaControllerLeftTwo.calculate(Constants.CLIMBER_BACK_EXTEND_TARGET_POSITION, 0);
    }
  };

  DoubleSupplier rightArmOneExtendSpeedSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      if (_thetaControllerRightOne == null)
      {
        var profileConstraints = new TrapezoidProfile.Constraints(
                Constants.CLIMBER_MOTOR_MAX_RPM,
                100);
        _thetaControllerRightOne = new ProfiledPIDController(50, 1, 0, profileConstraints);
        _thetaControllerRightOne.enableContinuousInput(Math.PI * -1, Math.PI);
        _thetaControllerRightOne.reset(Constants.CLIMBER_FRONT_EXTEND_TARGET_POSITION);
      }

      return _thetaControllerRightOne.calculate(Constants.CLIMBER_FRONT_EXTEND_TARGET_POSITION, 0);
    }
  };

  DoubleSupplier rightArmTwoExtendSpeedSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      if (_thetaControllerRightTwo == null)
      {
        var profileConstraints = new TrapezoidProfile.Constraints(
                Constants.CLIMBER_MOTOR_MAX_RPM,
                100);
        _thetaControllerRightTwo = new ProfiledPIDController(50, 1, 0, profileConstraints);
        _thetaControllerRightTwo.enableContinuousInput(Math.PI * -1, Math.PI);
        _thetaControllerRightTwo.reset(Constants.CLIMBER_BACK_EXTEND_TARGET_POSITION);
      }

      return _thetaControllerRightTwo.calculate(Constants.CLIMBER_BACK_EXTEND_TARGET_POSITION, 0);
    }
  };

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

  public BooleanSupplier leftArmOneAtExtendTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberLeftArmOneAtExtendTarget();
    }
  };

  public BooleanSupplier leftArmTwoAtExtendTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberLeftArmTwoAtExtendTarget();
    }
  };

  public BooleanSupplier rightArmOneAtExtendTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberRightArmOneAtExtendTarget();
    }
  };

  public BooleanSupplier rightArmTwoAtExtendTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberRightArmTwoAtExtendTarget();
    }
  };

  public BooleanSupplier leftArmOneAtRetractTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberLeftArmOneAtRetractTarget();
    }
  };

  public BooleanSupplier leftArmTwoAtRetractTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberLeftArmTwoAtRetractTarget();
    }
  };

  public BooleanSupplier rightArmOneAtRetractTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberRightArmOneAtRetractTarget();
    }
  };

  public BooleanSupplier rightArmTwoAtRetractTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberRightArmTwoAtRetractTarget();
    }
  };

  public BooleanSupplier leftArmTwoAtRetractRelease = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberLeftArmTwoAtRetractRelease();
    }
  };
  public BooleanSupplier rightArmTwoAtRetractRelease = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberRightArmTwoAtRetractRelease();
    }
  };


  public BooleanSupplier leftArmOneAtExtendRelease = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberLeftArmOneAtExtendRelease();
    }
  };
  public BooleanSupplier rightArmOneAtExtendRelease = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return r_ClimberRightArmOneAtExtendRelease();
    }
  };

  public boolean r_ClimberLeftArmOneAtExtendTarget() {
    return climberLeftArmOneMotorPosition >= Constants.CLIMBER_FRONT_EXTEND_TARGET_POSITION;
  }

  public boolean r_ClimberLeftArmTwoAtExtendTarget() {
    return climberLeftArmTwoMotorPosition >= Constants.CLIMBER_BACK_EXTEND_TARGET_POSITION;
  }

  public boolean r_ClimberRightArmOneAtExtendTarget() {
    return climberRightArmOneMotorPosition >= Constants.CLIMBER_FRONT_EXTEND_TARGET_POSITION;
  }

  public boolean r_ClimberRightArmTwoAtExtendTarget() {
    return climberRightArmTwoMotorPosition >= Constants.CLIMBER_BACK_EXTEND_TARGET_POSITION;
  }

  public boolean r_ClimberArmOneAtExtendTarget() {
    return r_ClimberLeftArmOneAtExtendTarget() && r_ClimberRightArmOneAtExtendTarget();
  }

  public boolean r_ClimberArmTwoAtExtendTarget() {
    return r_ClimberLeftArmTwoAtExtendTarget() && r_ClimberRightArmTwoAtExtendTarget();
  }

  public boolean r_ClimberLeftArmOneAtRetractTarget() {
    return climberLeftArmOneMotorPosition <= Constants.CLIMBER_FRONT_RETRACT_TARGET_POSITION;
  }

  public boolean r_ClimberLeftArmTwoAtRetractTarget() {
    return climberLeftArmTwoMotorPosition <= Constants.CLIMBER_BACK_RETRACT_TARGET_POSITION;
  }

  public boolean r_ClimberRightArmOneAtRetractTarget() {
    return climberRightArmOneMotorPosition <= Constants.CLIMBER_FRONT_RETRACT_TARGET_POSITION;
  }

  public boolean r_ClimberRightArmTwoAtRetractTarget() {
    return climberRightArmTwoMotorPosition <= Constants.CLIMBER_BACK_RETRACT_TARGET_POSITION;
  }

  public boolean r_ClimberArmOneAtRetractTarget() {
    return r_ClimberLeftArmOneAtRetractTarget() && r_ClimberRightArmOneAtRetractTarget();
  }

  public boolean r_ClimberArmTwoAtRetractTarget() {
    return r_ClimberLeftArmTwoAtRetractTarget() && r_ClimberRightArmTwoAtRetractTarget();
  }

  public boolean r_ClimberLeftArmTwoAtRetractRelease() {
    return climberLeftArmTwoMotorPosition <= Constants.CLIMBER_BACK_RETRACT_RELEASE_POSITION;
  }

  public boolean r_ClimberRightArmTwoAtRetractRelease() {
    return climberRightArmTwoMotorPosition <= Constants.CLIMBER_BACK_RETRACT_RELEASE_POSITION;
  }

  public boolean r_ClimberLeftArmOneAtExtendRelease() {
    return climberLeftArmOneMotorPosition >= Constants.CLIMBER_FRONT_EXTEND_RELEASE_POSITION;
  }

  public boolean r_ClimberRightArmOneAtExtendRelease() {
    return climberRightArmOneMotorPosition >= Constants.CLIMBER_FRONT_EXTEND_RELEASE_POSITION;
  }

  public void climberArmOneUp() {
    armOneBrakeRetract();
    climberRightArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
    climberLeftArmOneMotor.set(Constants.CLIMBER_UP_SPEED);
  }
  
  public void climberRightArmOneUp() {
    armOneBrakeRetract();
    climberRightArmOneMotor.set(rightArmOneExtendSpeedSupplier.getAsDouble());
  }

  public void climberLeftArmOneUp() {
    armOneBrakeRetract();
    climberLeftArmOneMotor.set(leftArmOneExtendSpeedSupplier.getAsDouble());
  }

  public void climberRightArmOneUpSlow() {
    armOneBrakeRetract();
    climberRightArmOneMotor.set(Constants.CLIMBER_UP_SLOW_SPEED);
  }

  public void climberLeftArmOneUpSlow() {
    armOneBrakeRetract();
    climberLeftArmOneMotor.set(Constants.CLIMBER_UP_SLOW_SPEED);
  }

  public void climberRightArmTwoUp() {
    armOneBrakeRetract();
    climberRightArmTwoMotor.set(rightArmTwoExtendSpeedSupplier.getAsDouble());
  }

  public void climberLeftArmTwoUp() {
    armOneBrakeRetract();
    climberLeftArmTwoMotor.set(leftArmTwoExtendSpeedSupplier.getAsDouble());
  }

  public void climberRightArmOneDown() {
    armOneBrakeRetract();
    climberRightArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberLeftArmOneDown() {
    armOneBrakeRetract();
    climberLeftArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberRightArmTwoDown() {
    armOneBrakeRetract();
    climberRightArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberLeftArmTwoDown() {
    armOneBrakeRetract();
    climberLeftArmTwoMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberArmOneDown() {
    armOneBrakeRetract();
    climberLeftArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
    climberRightArmOneMotor.set(Constants.CLIMBER_DOWN_SPEED);
  }

  public void climberArmOneStop() {
    climberLeftArmOneMotor.set(0);
    climberRightArmOneMotor.set(0);
  }

  public void climberRightArmOneStop() {
    climberRightArmOneMotor.set(0);
  }

  public void climberLeftArmOneStop() {
    climberLeftArmOneMotor.set(0);
  }
   
  public void climberRightArmTwoStop() {
    climberRightArmTwoMotor.set(0);
  }

  public void climberLeftArmTwoStop() {
    climberLeftArmTwoMotor.set(0);
  }

  public void climberArmTwoUp() {
    armTwoBrakeRetract();
    climberLeftArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
    climberRightArmTwoMotor.set(Constants.CLIMBER_UP_SPEED);
  }

  public void climberArmTwoDown() {
    armTwoBrakeRetract();
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

  public void enableBrakeOverride() {
    brakeOverride = true;
    retractBrakes();
  }

  public void disableBrakeOverride() {
    brakeOverride = false;
    extendBrakes();
  }

  public void enableAutoClimb() {
    autoClimbEnable = true;
  }

  public void disableAutoClimb() {
    autoClimbEnable = false;
  }

  @Override
  public void periodic() {
    climberLeftArmOneMotorPosition = climberLeftArmOneEncoder.getPosition();
    climberLeftArmTwoMotorPosition = climberLeftArmTwoEncoder.getPosition();
    climberRightArmOneMotorPosition = climberRightArmOneEncoder.getPosition();
    climberRightArmTwoMotorPosition = climberRightArmTwoEncoder.getPosition();
  }
}
