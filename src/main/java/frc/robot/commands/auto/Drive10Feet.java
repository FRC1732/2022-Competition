// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class Drive10Feet extends DriveSegmentBaseCommand {
  public Drive10Feet(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(new Translation2d(0,0),
            new Translation2d(1.5, 0),
            new Translation2d(3, 0)),
        Rotation2d.fromDegrees(0),
        true);
  }
}
