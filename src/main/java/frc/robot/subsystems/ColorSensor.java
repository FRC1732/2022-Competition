// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */

  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  private Color allianceColor;

  public ColorSensor() {
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);

    //get color of ball preloaded before auto;
    //make this work
    allianceColor = colorSensor.getColor();
  }

  public Color getColor(){
    Color color = colorSensor.getColor();
    return color;
  }

  public int getRed(){
    int color = colorSensor.getRed();
    return color;
  }

  public int getBlue(){
    int color = colorSensor.getBlue();
    return color;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
