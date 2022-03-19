// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterCommand extends CommandBase {
  private final Shooter shooter;
  DoubleSupplier distance;

  public RunShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    this.distance = null;
    addRequirements(shooter);
  }

  public RunShooterCommand(Shooter shooter, DoubleSupplier distance) {
    this.shooter = shooter;
    this.distance = distance;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if (distance != null) {
      shooter.setRpmTargetUsingDistance(distance.getAsDouble());
    }
    shooter.startFlywheel(); //TODO: create proper shooting command
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      shooter.stopFlywheel();
  }
}
