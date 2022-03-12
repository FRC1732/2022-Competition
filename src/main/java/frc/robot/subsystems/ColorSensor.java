// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ColorSensor {
  /** Creates a new ColorSensor. */

  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;

  public ColorSensor() {
    configureShuffleBoard();

    i2cPort = I2C.Port.kMXP;
    colorSensor = new ColorSensorV3(i2cPort);
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("color sensor");
    tab.addNumber("get red", redSupplier);
    tab.addNumber("get blue", blueSupplier);
    tab.addBoolean("has ball", ballSupplier);
    tab.addNumber("proximity", proximitySupplier);
    tab.addBoolean("is wrong ball", wrongBallSupplier);
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

    // if (colorSensor.getRed() > 50) {
    //   return Color.kRed;
    // } else if (colorSensor.getBlue() > 50) {
    //   return Color.kBlue;
    // } else {
    //   return Color.kKhaki; // I LOVE KHAKI #nojeansever
    // }

  }

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

}
