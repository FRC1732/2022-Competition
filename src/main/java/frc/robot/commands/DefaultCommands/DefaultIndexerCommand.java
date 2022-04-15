// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class DefaultIndexerCommand extends CommandBase {
  Indexer m_indexerSubsystem;

  public DefaultIndexerCommand(Indexer indexerSubsystem) {
    addRequirements(indexerSubsystem);
    m_indexerSubsystem = indexerSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DefaultIndexerCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    if (!interrupted)
      m_indexerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
