// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase {
  Intake m_intakeSubsystem;

  public DefaultIntakeCommand(Intake intakeSubsystem) {
    addRequirements(intakeSubsystem);
    m_intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DefaultIntakeCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    if (!interrupted) {
      m_intakeSubsystem.retract();
      m_intakeSubsystem.stop();
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
