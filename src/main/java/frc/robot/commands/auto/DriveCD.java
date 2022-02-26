// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class DriveCD extends DriveSegmentBaseCommand {
  public DriveCD(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(new Translation2d(0,0),
            new Translation2d(0.734 / 1.5, -0.9982 / 1.5)),
        Rotation2d.fromDegrees(-112),
        Rotation2d.fromDegrees(-43),
        true);
  }
}
