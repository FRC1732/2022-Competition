// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Centerer;

public class DefaultCentererCommand extends CommandBase {
  private final Centerer m_centererSubsystem;

  public DefaultCentererCommand(Centerer centererSubsystem) {
    m_centererSubsystem = centererSubsystem;
    addRequirements(centererSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DefaultCentererCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    if (!interrupted) {
      m_centererSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
