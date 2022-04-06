// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import frc.robot.subsystems.Drivetrain;

public class DriveQR extends DriveSegmentBaseCommand {
  public DriveQR(Drivetrain drivetrain) {
    super(drivetrain, 
    List.of(WAYPOINT_Q.getTranslation(), WAYPOINT_R.getTranslation()),
        WAYPOINT_Q.getRotation(),
        WAYPOINT_R.getRotation(),
        true,
        true);
  }
}
