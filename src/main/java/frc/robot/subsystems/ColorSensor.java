// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.CIEColor;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */
  private ColorSensorV3 colorSensor;

  private SuppliedValueWidget<Boolean> allianceColor,
      currentUpperBallColor,
      currentLowerBallColor,
      previousUpperBallColor,
      previousLowerBallColor;

  private Color currentUpperBall,
      previousUpperBall,
      currentLowerBall,
      previousLowerBall;

  private DriverStation.Alliance alliance;

  private boolean previousHasBall;
  private boolean lookingForColor;

  private int m_totalBallCount,
      m_proximity,
      m_redColor,
      m_blueColor,
      m_greenColor,
      m_irValue;

  public ColorSensor() {
    configureComponents();
    configureShuffleBoard();
  }

  private void configureComponents() {
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    // Khaki is the empty state
    previousUpperBall = currentUpperBall = Color.kKhaki;
    previousLowerBall = currentLowerBall = Color.kKhaki;
    previousHasBall = false;
    lookingForColor = false;
    m_totalBallCount = 0;
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    switch (RobotConfig.SB_LOGGING) {
      case COMPETITION:
        tab = Shuffleboard.getTab("COMPETITION");
        allianceColor = tab.addBoolean("Alliance Color", () -> true);
        currentUpperBallColor = tab.addBoolean("Upper Ball", () -> true);
        currentLowerBallColor = tab.addBoolean("Lower Ball", () -> true);
        break;
      case DEBUG:
        tab = Shuffleboard.getTab("color sensor");
        tab.addNumber("R", (DoubleSupplier) redSupplier);
        tab.addNumber("G", (DoubleSupplier) greenSupplier);
        tab.addNumber("B", (DoubleSupplier) blueSupplier);
        tab.addNumber("IR", (DoubleSupplier) irSupplier);
        tab.addNumber("Prox", (DoubleSupplier) proximitySupplier);
        allianceColor = tab.addBoolean("Alliance Color", () -> true);
        currentUpperBallColor = tab.addBoolean("Upper Ball", () -> true);
        currentLowerBallColor = tab.addBoolean("Lower Ball", () -> true);
        break;
      case NONE:
      default:
        break;
    }
  }

  IntSupplier redSupplier = new IntSupplier() {
    @Override
    public int getAsInt() {
      return m_redColor;
    }
  };

  IntSupplier greenSupplier = new IntSupplier() {
    @Override
    public int getAsInt() {
      return m_greenColor;
    }
  };

  IntSupplier blueSupplier = new IntSupplier() {
    @Override
    public int getAsInt() {
      return m_blueColor;
    }
  };

  IntSupplier proximitySupplier = new IntSupplier() {
    @Override
    public int getAsInt() {
      return m_proximity;
    }
  };

  IntSupplier irSupplier = new IntSupplier() {
    @Override
    public int getAsInt() {
      return m_irValue;
    }
  };

  public Color getAllianceColor() {
    return alliance.equals(Alliance.Red) ? Color.kRed : Color.kBlue;
  }

  private Color determineBallColor() {
    // TODO determine values for if statements
    int difference = colorSensor.getRed() - colorSensor.getBlue();
    // System.out.println(String.format("Determine Color -- Red: (%d) - Blue: (%d)",
    // colorSensor.getRed(), colorSensor.getBlue()));

    if (difference > 0 && Math.abs(difference) > 100 && hasBall()) {
      return Color.kRed;
    } else if (difference < 0 && Math.abs(difference) > 100 && hasBall()) {
      return Color.kBlue;
    } else {
      return Color.kKhaki; // I LOVE KHAKI #nojeansever
    }
  }

  private String colorToString(Color color) {
    return color.equals(Color.kRed) ? "red"
        : color.equals(Color.kBlue) ? "blue"
            : color.equals(Color.kKhaki) ? "khaki"
                : "yellow";
  }

  BooleanSupplier ballSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return hasBall();
    }
  };

  BooleanSupplier wrongBallSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return isWrongBall();
    }
  };

  public boolean hasBall() {
    return m_proximity > Constants.COLOR_SENSOR_PROX_HAS_BALL_THRESHOLD;
  }

  public boolean hasColor() {
    return determineBallColor() != Color.kKhaki;
  }

  public boolean isWrongBall() {
    return (hasBall() && getAllianceColor() != determineBallColor());
  }

  /**
   * Provides state of 2 tracked balls in the robot
   * 
   * @return boolean if two balls are tracked within the robot.
   */
  public boolean hasTwoBalls() {
    return currentUpperBall != Color.kKhaki && currentLowerBall != Color.kKhaki;
  }

  /**
   * This allow for Shooter or Reject commands to make ball state Empty
   */
  public void makeEmpty() {
    currentUpperBall = currentLowerBall = Color.kKhaki;
  }

  @Override
  public void periodic() {
    m_proximity = colorSensor.getProximity();
    m_redColor = colorSensor.getRed();
    m_blueColor = colorSensor.getBlue();
    m_greenColor = colorSensor.getGreen();
    m_irValue = colorSensor.getIR();
    alliance = DriverStation.getAlliance();
    System.out.println(String.format("Prox: [%d] Red: [%d] Green: [%d] Blue: [$d] IR: [%d]", m_proximity, m_redColor,
        m_greenColor, m_blueColor, m_irValue));

    if (hasBall()) {
      // we have
    }

    if (hasBall() != previousHasBall) {
      // ball state change
      if (previousHasBall == false) { // new ball in
        lookingForColor = true;
        // TODO - find out which way intake is running to detemine if advancing forward.
        // But for now, assume all is forward
        m_totalBallCount++;
        System.out.println(String.format("Color Sensor Ball Count: [%d]", m_totalBallCount));
      } else { // ball going out
        currentUpperBall = currentLowerBall;
        currentLowerBall = Color.kKhaki; // No Ball
        logStateChange(previousHasBall == false);
      }

      previousHasBall = hasBall();
    }

    if (lookingForColor && hasColor()) {
      currentLowerBall = determineBallColor();
      lookingForColor = false;
      logStateChange(previousHasBall == true);
    }

    // This method will be called once per scheduler run
    allianceColor.withProperties(Map.of("colorWhenTrue", colorToString(getAllianceColor())));
    currentUpperBallColor.withProperties(Map.of("colorWhenTrue", colorToString(currentUpperBall)));
    currentLowerBallColor.withProperties(Map.of("colorWhenTrue", colorToString(currentLowerBall)));

    previousUpperBallColor.withProperties(Map.of("colorWhenTrue", colorToString(previousUpperBall)));
    previousLowerBallColor.withProperties(Map.of("colorWhenTrue", colorToString(previousLowerBall)));
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
