// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveHL extends DriveSegmentBaseCommand {
  public DriveHL(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_H.getTranslation(), WAYPOINT_L.getTranslation()),
        WAYPOINT_H.getRotation(),
        WAYPOINT_L.getRotation(),
        false,
        true);
  }
}
