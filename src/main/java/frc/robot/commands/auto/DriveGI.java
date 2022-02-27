// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveGI extends DriveSegmentBaseCommand {
  public DriveGI(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_G.getTranslation(), WAYPOINT_I.getTranslation()),
        WAYPOINT_G.getRotation(),
        WAYPOINT_I.getRotation(),
        true,
        false);
  }
}
