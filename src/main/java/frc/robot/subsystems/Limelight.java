// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Servo;

public class Limelight extends SubsystemBase {
  private NetworkTable table; 
  private NetworkTableEntry llData_camerastream; 
  private NetworkTableEntry tv; 
  private NetworkTableEntry tx; 
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry ledMode; 
  private NetworkTableEntry camMode; 

  private static final int LEDSTATE_USE = 0;
  private static final int LEDSTATE_OFF = 1;
  private static final int LEDSTATE_BLINK = 2;
  private static final int LEDSTATE_ON = 3;

  private static final int CAMMODE_VISION = 0;
  private static final int CAMMODE_DRIVER = 1;

  private Servo servoX;
  private Servo servoY;
  private double servoPositionX;
  private double servoPositionY;

  /** Creates a new Limelight. */
  public Limelight() {
    servoX = new Servo(0);         // Servo connected to roboRIO PWM 0, x-axis
    servoY = new Servo(1);         // Servo connected to roboRIO PWM 1, y-axis
    servoPositionX = 0.5;
    servoPositionY = 0.5;

    configureNetworkTableEntries();
    configureShuffleBoard();
  }

  public double getServoX() {
    return servoX.getPosition();
  }

  public double getServoY() {
    return servoY.getPosition();
  }

  public void incrementSetServoX() {
    servoPositionX += 0.05;
    servoX.set(servoPositionX);
  }

  public void incrementSetServoY() {
    servoPositionY += 0.05;
    servoY.set(servoPositionY);
  }

  public void decrementSetServoX() {
    servoPositionX -= 0.05;
    servoX.set(servoPositionX);
  }

  public void decrementSetServoY() {
    servoPositionY -= 0.05;
    servoY.set(servoPositionY);
  }

  public void on() {
    ledMode.setNumber(LEDSTATE_ON);
  }

  public void off() {
    ledMode.setNumber(LEDSTATE_OFF);
  }

  private void configureNetworkTableEntries()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
  }

  private void configureShuffleBoard()
  {
    Shuffleboard.getTab("limelight").add("limelight", 1).withWidget(BuiltInWidgets.kCameraStream).withPosition(5,0).withSize(4, 4).getEntry();
    //Shuffleboard.getTab("limelight").add("ll_ledMode", ll_ledModeSupplier).withWidget(BuiltInWidgets.kTextView).getEntry();
  }



  DoubleSupplier ll_ledModeSupplier =  new DoubleSupplier(){
    @Override
    public double getAsDouble() {
      return ledMode.getDouble(-1);
    }
  };

  @Override
  public void periodic() {
    // read values periodically
  }
}
