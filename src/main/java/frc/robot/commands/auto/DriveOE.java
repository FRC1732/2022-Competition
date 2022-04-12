// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveOE extends DriveSegmentBaseCommand {
  public DriveOE(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_O.getTranslation(), WAYPOINT_E.getTranslation()),
        WAYPOINT_O.getRotation(),
        WAYPOINT_E.getRotation(),
        true,
        false);
  }
}
