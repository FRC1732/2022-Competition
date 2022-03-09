// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private Indexer mIndexer;
  private Centerer mCenterer;
  private Intake mIntake;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Centerer centerer, Indexer indexer) {
    addRequirements(intake);
    addRequirements(centerer);
    addRequirements(indexer);
    mIntake = intake;
    mCenterer = centerer;
    mIndexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.deploy();
    mIntake.forward();
    mCenterer.forward();
    mIndexer.forwardSlow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.stop();
    mCenterer.stop();
    mIndexer.stop();
    mIntake.retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
