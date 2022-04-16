// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStatus extends SubsystemBase {
  private BooleanSupplier hasOneBallSupplier;
  private BooleanSupplier hasTwoBallSupplier;
  private BooleanSupplier isAligningSupplier;
  private BooleanSupplier isShootingSupplier;
  private int count = 0;

  private boolean isTeleop;
  private boolean isAuto;
  private boolean haveDoneTeleop;

  private DigitalOutput out0 = new DigitalOutput(0);
  private DigitalOutput out1 = new DigitalOutput(1);
  private DigitalOutput out2 = new DigitalOutput(2);

  /** Creates a new LedStatus. */
  public LedStatus() {
    haveDoneTeleop = false;

    // seting defaults to avoid Null Pointer Exceptions
    hasOneBallSupplier = () -> false;
    hasTwoBallSupplier = () -> false;
    isAligningSupplier = () -> false;
    isShootingSupplier = () -> false;
  }

  public void setHasOneBallSupplier(BooleanSupplier hasOneBallSupplier) {
    this.hasOneBallSupplier = hasOneBallSupplier;
  }

  public void setHasTwoBallSupplier(BooleanSupplier hasTwoBallSupplier) {
    this.hasTwoBallSupplier = hasTwoBallSupplier;
  }

  public void setIsAligningSupplier(BooleanSupplier isAligningSupplier) {
    this.isAligningSupplier = isAligningSupplier;
  }

  public void setIsShootingSupplier(BooleanSupplier isShootingSupplier) {
    this.isShootingSupplier = isShootingSupplier;
  }

  @Override
  public void periodic() {
    isTeleop = DriverStation.isTeleop();
    isAuto = DriverStation.isAutonomous();

    int status = 0;

    if (isAuto) {
      if (hasOneBallSupplier.getAsBoolean()) {
        status = 1;
      }
      if (hasTwoBallSupplier.getAsBoolean()) {
        status = 2;
      }
    }

    if (isTeleop) {
      haveDoneTeleop = true;
      if (hasOneBallSupplier.getAsBoolean()) {
        status = 3;
      }
      if (hasTwoBallSupplier.getAsBoolean()) {
        status = 4;
      }
      if (isAligningSupplier.getAsBoolean()) {
        status = 5;
      }
      if (isShootingSupplier.getAsBoolean()) {
        status = 6;
      }
    }

    if (haveDoneTeleop && DriverStation.isDisabled()) {
      status = 7;
    }
    
    count++;
    if (count % 50 == 0)
      System.out.println("LED STATUS: " + status);

    out0.set((status & 0b0001) == 1);
    out1.set((status & 0b0010) == 1);
    out2.set((status & 0b0100) == 1);
  }

  public void setPartyMode() {
    out0.set(true);
    out1.set(true);
    out2.set(true);
  }
}
