// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveBJ extends DriveSegmentBaseCommand {
  public DriveBJ(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_B.getTranslation(), WAYPOINT_J.getTranslation()),
        WAYPOINT_B.getRotation(),
        WAYPOINT_J.getRotation(),
        true,
        false);
  }
}
