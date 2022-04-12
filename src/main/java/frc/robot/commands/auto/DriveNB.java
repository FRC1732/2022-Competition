// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveNB extends DriveSegmentBaseCommand {
  public DriveNB(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_N.getTranslation(), WAYPOINT_B.getTranslation()),
        WAYPOINT_N.getRotation(),
        WAYPOINT_B.getRotation(),
        false,
        true);
  }
}
