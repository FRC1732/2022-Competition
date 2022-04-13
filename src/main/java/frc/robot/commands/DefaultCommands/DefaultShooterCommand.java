// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class DefaultShooterCommand extends CommandBase {

  Shooter m_shooterSubsystem;

  public DefaultShooterCommand(Shooter shooterSubsystem) {
    addRequirements(shooterSubsystem);
    m_shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DefaultShooterCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    if (!interrupted) {
      m_shooterSubsystem.stopFlywheel();
      m_shooterSubsystem.retractHood();
      m_shooterSubsystem.setTargetNearRpm(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
