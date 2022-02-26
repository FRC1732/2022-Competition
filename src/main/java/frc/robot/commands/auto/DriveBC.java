// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class DriveBC extends DriveSegmentBaseCommand {
  public DriveBC(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(new Translation2d(0,0),
            new Translation2d(0.762 / 1.1, 1.9558 / 1.1)),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(-112),
        false);
  }
}
