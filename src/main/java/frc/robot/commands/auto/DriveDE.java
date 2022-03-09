// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveDE extends DriveSegmentBaseCommand {
  public DriveDE(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_D.getTranslation(), WAYPOINT_E.getTranslation()),
        WAYPOINT_D.getRotation(),
        WAYPOINT_E.getRotation(),
        false,
        false);
  }
}
