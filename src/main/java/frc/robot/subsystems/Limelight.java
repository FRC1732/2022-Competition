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

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
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

  private MjpegServer server;
  private HttpCamera LLFeed;
  private int cameraStream = 0;

  /** Creates a new Limelight. */
  public Limelight() {
    configureNetworkTableEntries();
    configureShuffleBoard();
  }

  public void on() {
    ledMode.setNumber(LEDSTATE_ON);
  }

  public void off() {
    ledMode.setNumber(LEDSTATE_OFF);
  }

  private void configureNetworkTableEntries() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab = Shuffleboard.getTab("limelight");
    tab.addNumber("LED Mode", ll_ledModeSupplier);
    tab.addNumber("tv - Valid Targets", ll_tvSupplier);
    tab.addNumber("tx - Horiz Offset", ll_txSupplier);
    tab.addNumber("ty - Vert Offset", ll_tySupplier);
    tab.addNumber("ta - Target Area", ll_taSupplier);

    LLFeed = new HttpCamera("limelight", "http://10.17.32.11:5800/stream.mjpg");
    server = CameraServer.addSwitchedCamera("Toggle Cam");
    server.setSource(LLFeed);
    tab.add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withPosition(1, 1).withSize(5, 4)
        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here

  }

  DoubleSupplier ll_ledModeSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return ledMode.getDouble(-1);
    }
  };

  DoubleSupplier ll_tvSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return tv.getDouble(-1);
    }
  };

  DoubleSupplier ll_txSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return tx.getDouble(-1);
    }
  };

  DoubleSupplier ll_tySupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return ty.getDouble(-1);
    }
  };

  DoubleSupplier ll_taSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return ta.getDouble(-1);
    }
  };

  public DoubleSupplier rotation = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      double retVal = 0;
      if (hasTarget()) {
        if (getTx() < -5) {
          retVal = 0.15;
        } else if (getTx() > 5) {
          retVal = -0.15;
        }
      }
      return retVal;
    }
  };

  @Override
  public void periodic() {
    // read values periodically
  }

  public boolean hasTarget() {
    return tv.getDouble(0) > 0;
  }

  public Double getTx() {
    return tx.getDouble(0); // FIXME; what should default be
  }

  public Double getTy() {
    return ty.getDouble(0); // FIXME; what should default be
  }
}
