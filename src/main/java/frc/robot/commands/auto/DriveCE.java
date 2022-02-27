// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveCE extends DriveSegmentBaseCommand {
  public DriveCE(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_C.getTranslation(), WAYPOINT_E.getTranslation()),
        WAYPOINT_C.getRotation(),
        WAYPOINT_E.getRotation(),
        false,
        false);
  }
}
