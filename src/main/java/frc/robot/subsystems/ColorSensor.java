// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class ColorSensor {
  /** Creates a new ColorSensor. */

  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  private DriverStation.Alliance alliance;
  private Color allianceColor;

  public ColorSensor() {
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    alliance = DriverStation.getAlliance();
    if(alliance == Alliance.Red){
      allianceColor = Color.kRed;
    }else{
      allianceColor = Color.kBlue;
    }
    
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

  public Color getAllianceColor(){
    return allianceColor;
  }

  private Color determineBallColor(){
    //TODO determine values for if statements
    if(colorSensor.getRed() > 50){
      return Color.kRed;
    }else if (colorSensor.getBlue() > 50){
      return Color.kBlue;
    }else{
      return Color.kKhaki; //I LOVE KHAKI #nojeansever
    }

  }

  public boolean hasBall(){
    int proximity = colorSensor.getProximity();
    //TODO get proximity from sensor to ball
    if(proximity <= 100 ){
      return true;
    }else{
      return false;
    }
  }

  public boolean isWrongBall(){
    if(hasBall() && allianceColor != determineBallColor()){
      return true;
    }
    return false;
  }




}
