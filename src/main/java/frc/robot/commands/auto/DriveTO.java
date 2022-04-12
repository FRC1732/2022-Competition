// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveTO extends DriveSegmentBaseCommand {
  public DriveTO(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_T.getTranslation(), WAYPOINT_O.getTranslation()),
        WAYPOINT_T.getRotation(),
        WAYPOINT_O.getRotation(),
        true,
        false);
  }
}
