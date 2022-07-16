// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class DefaultFeederCommand extends CommandBase {
  private final Feeder m_feederSubsystem;

  public DefaultFeederCommand(Feeder feederSubsystem) {
    m_feederSubsystem = feederSubsystem;
    addRequirements(feederSubsystem);
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
    System.out.println("DefaultFeederCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    if (!interrupted) {
      m_feederSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
