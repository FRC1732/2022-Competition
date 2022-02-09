// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterCommand extends CommandBase {
  private final Shooter shooter;

  public RunShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    //shooter.shootFlywheel();
    shooter.setFlywheelOutput(0.25);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();
  }
}
