// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.plaf.TreeUI;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */

  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  private SuppliedValueWidget allianceColor;
  private SuppliedValueWidget upperBallColor;
  private SuppliedValueWidget lowerBallColor;

  public ColorSensor() {
    configureShuffleBoard();

    i2cPort = I2C.Port.kMXP;
    colorSensor = new ColorSensorV3(i2cPort);
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("color sensor");
    tab.addNumber("proximity", proximitySupplier);
    allianceColor = tab.addBoolean("Alliance Color", () -> true);
    upperBallColor = tab.addBoolean("Upper Ball", () -> true);
    lowerBallColor = tab.addBoolean("Lower Ball", () -> true);
  }

  public Color getColor() {
    Color color = colorSensor.getColor();
    return color;
  }

  public Color getAllianceColor() {
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Red) {
      return Color.kRed;
    } else {
      return Color.kBlue;
    }
  }

  DoubleSupplier redSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return getRed();
    }
  };

  public int getRed() {
    int color = colorSensor.getRed();
    return color;
  }

  DoubleSupplier blueSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return getBlue();
    }
  };

  public int getBlue() {
    int color = colorSensor.getBlue();
    return color;
  }

  private Color determineBallColor() {
    // TODO determine values for if statements
    int difference = colorSensor.getRed() - colorSensor.getBlue();

    if (difference > 0 && Math.abs(difference) > 100 && hasBall()) {
      return Color.kRed;
    } else if (difference < 0 && Math.abs(difference) > 100 && hasBall()) {
      return Color.kBlue;
    } else {
      return Color.kKhaki;
    }
  }

  private String colorToString(Color color) {
    if (Color.kRed.equals(color)) {
      return "red";
    } else if (Color.kBlue.equals(color)) {
      return "blue";
    } else if (Color.kKhaki.equals(color)) {
      return "yellow";
    } else {
      return "black";
    }
  }

  // if (colorSensor.getRed() > 50) {
  // return Color.kRed;
  // } else if (colorSensor.getBlue() > 50) {
  // return Color.kBlue;
  // } else {
  // return Color.kKhaki; // I LOVE KHAKI #nojeansever
  // }

  BooleanSupplier ballSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return hasBall();
    }
  };

  DoubleSupplier proximitySupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return colorSensor.getProximity();
    }
  };

  BooleanSupplier wrongBallSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return isWrongBall();
    }
  };

  public boolean hasBall() {
    int proximity = colorSensor.getProximity();
    if (proximity >= 100) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isWrongBall() {
    if (hasBall() && getAllianceColor() != determineBallColor()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    allianceColor.withProperties(Map.of("colorWhenTrue", colorToString(getAllianceColor())));
    upperBallColor.withProperties(Map.of("colorWhenTrue", "black"));
    lowerBallColor.withProperties(Map.of("colorWhenTrue", colorToString(determineBallColor())));
  }
}
