// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  private double tv, tx, ty, ta;

  /** Creates a new Limelight. */
  public Limelight() {
  }

  public void on() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(Constants.LEDSTATE_ON);
  }

  public void off() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(Constants.LEDSTATE_OFF);
  }

  @Override
  public void periodic() {
    // read values periodically
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }
}
