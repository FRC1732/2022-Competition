// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private double tv;
  private double tx;
  private double ty;
  private double ta;

  private static final int LEDSTATE_USE = 0;
  private static final int LEDSTATE_OFF = 1;
  private static final int LEDSTATE_BLINK = 2;
  private static final int LEDSTATE_ON = 3;

  private static final int CAMMODE_VISION = 0;
  private static final int CAMMODE_DRIVER = 1;

  /** Creates a new Limelight. */
  public Limelight() {
  }

  public void on() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LEDSTATE_ON);
  }

  public void off() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LEDSTATE_OFF);
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
