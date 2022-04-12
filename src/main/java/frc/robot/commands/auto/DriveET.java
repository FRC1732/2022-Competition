// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;
//back up for Adit to feed
public class DriveET extends DriveSegmentBaseCommand {
  public DriveET(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_E.getTranslation(), WAYPOINT_T.getTranslation()),
        WAYPOINT_E.getRotation(),
        WAYPOINT_T.getRotation(),
        true,
        false);
  }
}
