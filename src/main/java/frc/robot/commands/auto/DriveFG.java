// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveFG extends DriveSegmentBaseCommand {
  public DriveFG(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_F.getTranslation(), WAYPOINT_G.getTranslation()),
        WAYPOINT_F.getRotation(),
        WAYPOINT_G.getRotation(),
        false,
        true);
  }
}
