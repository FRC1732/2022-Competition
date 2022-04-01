// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class DefaultClimberCommand extends CommandBase {
  Climber m_climberSubsystem;

  public DefaultClimberCommand(Climber climberSubsystem) {
    addRequirements(climberSubsystem);
    m_climberSubsystem = climberSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_climberSubsystem.armsAllStop();
      m_climberSubsystem.extendBrakes();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
