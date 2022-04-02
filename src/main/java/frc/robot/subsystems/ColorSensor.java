// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */
  private final ColorSensorV3 colorSensor;
  private SuppliedValueWidget<Boolean> allianceColor;
  private SuppliedValueWidget<Boolean> upperBallColor;
  private SuppliedValueWidget<Boolean> lowerBallColor;

  private Color upperBall;
  private Color lowerBall;

  private boolean previousHasBall;
  private boolean lookingForColor;
  private int ballCount;

  public ColorSensor() {
    configureShuffleBoard();

    colorSensor = new ColorSensorV3(I2C.Port.kMXP);

    // Khaki is the empty state
    upperBall = Color.kKhaki;
    lowerBall = Color.kKhaki;
    previousHasBall = false;
    lookingForColor = false;
    ballCount = 0;
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    switch (RobotConfig.SB_LOGGING) {
      case COMPETITION:
        tab = Shuffleboard.getTab("COMPETITON");
        tab.addNumber("proximity", proximitySupplier);
        allianceColor = tab.addBoolean("Alliance Color", () -> true);
        upperBallColor = tab.addBoolean("Upper Ball", () -> true);
        lowerBallColor = tab.addBoolean("Lower Ball", () -> true);
        break;
      case DEBUG:
        tab = Shuffleboard.getTab("color sensor");
        tab.addNumber("proximity", proximitySupplier);
        allianceColor = tab.addBoolean("Alliance Color", () -> true);
        upperBallColor = tab.addBoolean("Upper Ball", () -> true);
        lowerBallColor = tab.addBoolean("Lower Ball", () -> true);
        break;
      case NONE:
      default:
        break;
    }
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
    //System.out.println(String.format("Determine Color -- Red: (%d) - Blue: (%d)", colorSensor.getRed(), colorSensor.getBlue()));

    if (difference > 0 && Math.abs(difference) > 100 && hasBall()) {
      return Color.kRed;
    } else if (difference < 0 && Math.abs(difference) > 100 && hasBall()) {
      return Color.kBlue;
    } else {
      return Color.kKhaki; // I LOVE KHAKI #nojeansever
    }
  }

  private String colorToString(Color color) {
    if (Color.kRed.equals(color)) {
      return "red";
    } else if (Color.kBlue.equals(color)) {
      return "blue";
    } else if (Color.kKhaki.equals(color)) {
      return "black";
    } else {
      return "yellow";
    }
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

  public boolean hasColor() {
    return determineBallColor() != Color.kKhaki;
  }

  public boolean isWrongBall() {
    if (hasBall() && getAllianceColor() != determineBallColor()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (hasBall() != previousHasBall) {
      // ball state change
      if (previousHasBall == false) { // new ball in
        lookingForColor = true;
        // TODO - find out which way intake is running to detemine if advancing forward.
        // But for now, assume all is forward
        ballCount++;
        System.out.println("Color Sensor Ball Count: " + ballCount);
      } else { // ball going out
        upperBall = lowerBall;
        lowerBall = Color.kKhaki; // No Ball
        logStateChange(previousHasBall == false);
      }      

      previousHasBall = hasBall();
    }

    if (lookingForColor && hasColor()) {
      lowerBall = determineBallColor();
      lookingForColor = false;
      logStateChange(previousHasBall == true);
    }

    // This method will be called once per scheduler run
    allianceColor.withProperties(Map.of("colorWhenTrue", colorToString(getAllianceColor())));
    upperBallColor.withProperties(Map.of("colorWhenTrue", colorToString(upperBall)));
    lowerBallColor.withProperties(Map.of("colorWhenTrue", colorToString(lowerBall)));
  }

  private void logStateChange(boolean withBall) {
    String mode = DriverStation.isAutonomous() ? "Auto" : DriverStation.isTeleop() ? "Tele" : "Unkn";
    int secondsRemaining = (int) DriverStation.getMatchTime();
    String colorStr = colorToString(determineBallColor());

    if (withBall) {
      System.out.println(String.format("Ball Change: %s(%d s) - Has Ball (%s)", mode, secondsRemaining, colorStr));
    } else {
      System.out.println(String.format("Ball Change: %s(%d s) - No Ball Detected", mode, secondsRemaining));
    }
  }
}
