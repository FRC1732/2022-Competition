// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class ExtendClimberArms extends CommandBase {
  private final Intake intake;
  private final Climber climber;

  public ExtendClimberArms(Intake in, Climber climb) {
    this.intake = in;
    this.climber = climb;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    intake.deploy();
    climber.ArmTwoOut();
    climber.armsAllUp();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      climber.armsAllStop();
  }
}
