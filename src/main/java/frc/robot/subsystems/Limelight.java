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
import java.util.ResourceBundle.Control;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import static frc.robot.Constants.*;

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

  private double r_tv;
  private double r_tx;
  private double r_ty;
  private double r_ta;

  private UsbCamera usbCamera;
  private MjpegServer server;
  private HttpCamera LLFeed;
  private int cameraStream = 0;

  /** Creates a new Limelight. */
  public Limelight() {
    configureNetworkTableEntries();
    configureShuffleBoard();

  }

  public void on() {
    ledMode.setNumber(LL_LEDSTATE_ON);
  }

  public void off() {
    // ledMode.setNumber(Constants.LEDSTATE_OFF);
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
    ShuffleboardTab tab;
    LLFeed = new HttpCamera("limelight", "http://10.17.32.11:5800/stream.mjpg");
    usbCamera = CameraServer.startAutomaticCapture(0);
    usbCamera.setConnectVerbose(0);
    server = CameraServer.addSwitchedCamera("Toggle Cam");
    server.setSource(LLFeed);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181); {

        switch (RobotConfig.SB_LOGGING) {
          case COMPETITION:
            tab = Shuffleboard.getTab("COMPETITION");
            tab.addBoolean("ACQUIRED", ll_hasTarget).withPosition(4, 2).withSize(1, 2);
            tab.add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withPosition(5, 0).withSize(5, 5)
                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here
            break;
          case DEBUG:
            tab = Shuffleboard.getTab("limelight");
            tab.addNumber("LED Mode", ll_ledModeSupplier);
            tab.addNumber("tv - Valid Targets", ll_tvSupplier);
            tab.addNumber("tx - Horiz Offsets", ll_txSupplier);
            tab.addNumber("ty - Vert Offset", ll_tySupplier);
            tab.addNumber("ta - Target Area", ll_taSupplier);
            tab.addNumber("theta - degrees", thetaDegrees);
            tab.addNumber("distance to target", distToTarget);
            // tab.addNumber("projected distance to target", projectedDistToTarget);
            tab.addBoolean("Target Acquired", ll_hasTarget);
            tab.add(mjpegServer1.getSource()).withWidget(BuiltInWidgets.kCameraStream);
            tab.add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withPosition(5, 0).withSize(5, 5)
                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here
            break;
          case NONE:
          default:
            // No shuffleboard configuration
            break;

        }
      }
    }

  DoubleSupplier distToTarget = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return (8.5 - LIMELIGHT_HEIGHT) / Math.sin(ty.getDouble(-1) * 0.0214 + 0.781);
    }
  };

  // DoubleSupplier projectedDistToTarget = new DoubleSupplier() {
  // @Override
  // public double getAsDouble() {
  // return Math.sqrt(Math.pow((8.5 - Constants.LIMELIGHT_HEIGHT) /
  // Math.sin(ty.getDouble(-1) * 0.0214 + 0.781),2) - Math.pow(8.5 -
  // Constants.LIMELIGHT_HEIGHT,2));
  // }
  // };

  DoubleSupplier thetaDegrees = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return Math.toDegrees(ty.getDouble(-1) * 0.0214 + 0.781); // angle from limelight to target with respect to the
                                                                // ground
    }
  };

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
      if (!hasTarget())
        return 0;
      double setpoint = 0;
      double error = setpoint - getTx();
      double tolerance = 2;
      double kp = 0.04;
      if (Math.abs(error) < tolerance)
        return 0;
      double minSpeed = Constants.MIN_ANGULAR_VELOCITY / 1.1; // @todo no minimum if robot is moving
      double maxSpeed = Constants.MAX_ANGULAR_VELOCITY / 9;
      double output = Math.signum(error) * Math.pow(Math.min((Math.abs(error) * kp), 1), 2);
      return ((maxSpeed - minSpeed) * output) + Math.signum(error) * minSpeed;
    }
  };

  BooleanSupplier ll_hasTarget = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return hasTarget();
    }
  };

  @Override
  public void periodic() {
    // read and store values periodically
    r_tx = tx.getDouble(0);
    r_ty = ty.getDouble(0);
    r_ta = ta.getDouble(0);
    r_tv = tv.getDouble(0);
  }

  public boolean hasTarget() {
    return r_tv > 0;
  }

  public Double getTx() {
    // FIXME; what should default be
    return r_tx;
  }

  public Double getTy() {
    // FIXME; what should default be
    return r_ty;
  }

  public void setDriverCamMode(){
    table.getEntry("camMode").setNumber(Constants.LL_CAMMODE_DRIVER);
  }

  public void setVisionMode(){
    table.getEntry("camMode").setNumber(Constants.LL_CAMMODE_VISION);
  }


}
