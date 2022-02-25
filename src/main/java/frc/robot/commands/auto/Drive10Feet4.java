// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class Drive10Feet4 extends DriveSegmentBaseCommand {
  public Drive10Feet4(Drivetrain drivetrain) {
    super(drivetrain, 
        List.of(new Translation2d(0,0),
            new Translation2d(-1.07 / 1.1, 6.5 / 1.1)),
        Rotation2d.fromDegrees(-43),
        Rotation2d.fromDegrees(-45),
        false);
  }
}
