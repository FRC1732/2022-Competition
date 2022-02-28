// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveCA extends DriveSegmentBaseCommand {
  public DriveCA(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_C.getTranslation(), WAYPOINT_A.getTranslation()),
        WAYPOINT_C.getRotation(),
        WAYPOINT_A.getRotation(),
        true,
        false);
  }
}
