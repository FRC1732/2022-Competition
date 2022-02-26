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

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;

  /** Creates a new Intake. */
  public Feeder() {
   if (RobotConfig.ROBOT_IS_COMPETITION) {
    feederMotor = new CANSparkMax(Constants.CAN_FEEDER_MOTOR, MotorType.kBrushless);
   } else {
    feederMotor = new CANSparkMax(Constants.CAN_FEEDER_MOTOR, MotorType.kBrushed);
   }
   feederMotor.setInverted(false);
   
   feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
   feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
   feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
   feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);
  }

  public void forward() {
    feederMotor.set(Constants.FEEDER_FWD_SPEED);
  }

  public void reverse() {
    feederMotor.set(Constants.FEEDER_BACKWARD_SPEED);
  }

  public void stop() {
    feederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
