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
      currentLowerBallColor;

  private Color currentUpperBall,
      currentLowerBall;

  private DriverStation.Alliance alliance;

  private boolean previousHasBall;
  private boolean lookingForColor;

  private int m_totalBallCount,
      m_redBallCount,
      m_blueBallCount,
      m_unknownBallCount,
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
    currentUpperBall = currentLowerBall = Color.kKhaki;
    previousHasBall = false;
    lookingForColor = false;
    m_totalBallCount = 0;
    m_redBallCount = 0;
    m_blueBallCount = 0;
    m_unknownBallCount = 0;
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    switch (RobotConfig.SB_LOGGING) {
      case COMPETITION:
        tab = Shuffleboard.getTab("COMPETITION");
        allianceColor = tab.addBoolean("Alliance Color", () -> true);
        currentUpperBallColor = tab.addBoolean("Current Upper", () -> true).withPosition(3, 0);
        currentLowerBallColor = tab.addBoolean("Current Lower", () -> true).withPosition(3, 1);
        break;
      case DEBUG:
        tab = Shuffleboard.getTab("color sensor");
        tab.addNumber("R", redSupplier).withPosition(0, 0);
        tab.addNumber("G", greenSupplier).withPosition(1, 0);
        tab.addNumber("B", blueSupplier).withPosition(2, 0);
        tab.addNumber("Prox", proximitySupplier).withPosition(0, 1);
        tab.addNumber("IR", irSupplier).withPosition(1, 1);
        tab.addNumber("Total Ball Count", ballCountSupplier).withPosition(4, 0);
        tab.addNumber("Red Ball Count", redBallCountSupplier).withPosition(4, 1);
        tab.addNumber("Blue Ball Count", blueBallCountSupplier).withPosition(4, 2);
        tab.addNumber("Unknown Ball Count", unknownBallCountSupplier).withPosition(4, 3);
        allianceColor = tab.addBoolean("Alliance Color", () -> true).withPosition(2, 1);
        currentUpperBallColor = tab.addBoolean("Current Upper", () -> true).withPosition(5, 0);
        currentLowerBallColor = tab.addBoolean("Current Lower", () -> true).withPosition(6, 0);
        break;
      case NONE:
      default:
        break;
    }
  }

  DoubleSupplier redSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_redColor;
    }
  };

  DoubleSupplier greenSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_greenColor;
    }
  };

  DoubleSupplier blueSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_blueColor;
    }
  };

  DoubleSupplier proximitySupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_proximity;
    }
  };

  DoubleSupplier irSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_irValue;
    }
  };

  DoubleSupplier ballCountSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_totalBallCount;
    }
  };

  DoubleSupplier redBallCountSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_redBallCount;
    }
  };

  DoubleSupplier blueBallCountSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_blueBallCount;
    }
  };

  DoubleSupplier unknownBallCountSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return m_unknownBallCount;
    }
  };

  public Color getAllianceColor() {
    return alliance.equals(Alliance.Red) ? Color.kRed : Color.kBlue;
  }

  private Color determineBallColor() {

    // TODO determine values for COLOR THRESHOLDS
    // Is Red AND is NOT Blue
    // if (hasBall() && isRedBall() && isBlueBall() == false) {
    // m_redBallCount++;
    // return Color.kRed;
    // } else if (hasBall() && isRedBall() == false && isBlueBall()) {
    // m_blueBallCount++;
    // return Color.kBlue;
    // } else if (hasBall() && isRedBall() == false && isBlueBall() == false)
    // m_unknownBallCount++;
    // return Color.kKhaki; // I LOVE KHAKI #nojeansever

    int difference = m_redColor - m_blueColor;
    System.out.println(String.format("Determine Color -- Red: (%d) - Blue: (%d)", m_redColor, m_blueColor));

    if (difference > 0 && Math.abs(difference) > Constants.COLOR_RED_BLUE_DIFFERENCE_THRESHOLD && hasBall()) {
      m_redBallCount++;
      return Color.kRed;
    } else if (difference < 0 && Math.abs(difference) > Constants.COLOR_RED_BLUE_DIFFERENCE_THRESHOLD && hasBall()) {
      m_blueBallCount++;
      return Color.kBlue;
    } else {
      m_unknownBallCount++;
      return Color.kKhaki;
    }
  }

  private String colorToString(Color color) {
    return color.equals(Color.kRed) ? "red"
        : color.equals(Color.kBlue) ? "blue"
            : color.equals(Color.kKhaki) ? "black"
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
    return m_proximity >= Constants.COLOR_SENSOR_PROX_HAS_BALL_THRESHOLD;
  }

  public boolean hasColor() {
    return determineBallColor() != Color.kKhaki;
  }

  public boolean isWrongBall() {
    return (hasBall() && getAllianceColor() != determineBallColor());
  }

  // public boolean isRedBall() {
  // return (isWithinTolerance(m_redColor,
  // Constants.COLOR_SENSOR_RED_BALL_R_THRESHOLD, 0.95, 1.05)
  // && isWithinTolerance(m_greenColor,
  // Constants.COLOR_SENSOR_RED_BALL_G_THRESHOLD, 0.95, 1.05)
  // && isWithinTolerance(m_blueColor,
  // Constants.COLOR_SENSOR_RED_BALL_B_THRESHOLD, 0.95, 1.05));
  // }

  // public boolean isBlueBall() {
  // return (isWithinTolerance(m_redColor,
  // Constants.COLOR_SENSOR_BLUE_BALL_R_THRESHOLD, 0.95, 1.05)
  // && isWithinTolerance(m_greenColor,
  // Constants.COLOR_SENSOR_BLUE_BALL_G_THRESHOLD, 0.95, 1.05)
  // && isWithinTolerance(m_blueColor,
  // Constants.COLOR_SENSOR_BLUE_BALL_B_THRESHOLD, 0.95, 1.05));
  // }

  // public boolean isWithinTolerance(int value, int limit, double lowTol, double
  // highTol) {
  // return (value >= (limit * lowTol) && value <= (limit * highTol));
  // }

  /**
   * Provides state of 2 tracked balls in the robot
   * 
   * @return boolean if two balls are tracked within the robot.
   */
  public boolean hasTwoBalls() {
    return currentUpperBall != Color.kKhaki && currentLowerBall != Color.kKhaki;
  }

  public boolean hasOneBall() {
    return currentUpperBall != Color.kKhaki;
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
    // System.out.println(String.format("Prox: [%d] Red: [%d] Green: [%d] Blue: [%d]
    // IR: [%d]", m_proximity, m_redColor, m_greenColor, m_blueColor, m_irValue));

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
        logStateChange(false);
      }

      previousHasBall = hasBall();
    }

    if (lookingForColor && hasColor()) {
      currentLowerBall = determineBallColor();
      lookingForColor = false;
      logStateChange(true);
    }

    // This method will be called once per scheduler run
    allianceColor.withProperties(Map.of("colorWhenTrue", colorToString(getAllianceColor())));
    currentUpperBallColor.withProperties(Map.of("colorWhenTrue", colorToString(currentUpperBall)));
    currentLowerBallColor.withProperties(Map.of("colorWhenTrue", colorToString(currentLowerBall)));
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
