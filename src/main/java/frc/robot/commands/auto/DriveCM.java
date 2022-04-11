// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveCM extends DriveSegmentBaseCommand {
  public DriveCM(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(WAYPOINT_C.getTranslation(), WAYPOINT_M.getTranslation()),
        WAYPOINT_C.getRotation(),
        WAYPOINT_M.getRotation(),
        true,
        false);
  }
}
