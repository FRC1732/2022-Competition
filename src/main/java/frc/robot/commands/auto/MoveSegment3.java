// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class MoveSegment3 extends DriveSegmentBaseCommand {
  public MoveSegment3(Drivetrain drivetrain) {
    super(drivetrain, 
    //to ball 4
        List.of(new Translation2d(0,0),
            new Translation2d(-28.9* .0254/2, 39.3* .0254/2),
            new Translation2d(-28.9* .0254, 39.3* .0254)),
        Rotation2d.fromDegrees(158),
        true);
  }
}