// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;

public class FeedCommand extends CommandBase {
  private Indexer mIndexer;
  private Centerer mCenterer;
  private Feeder mFeeder;

  /** Creates a new IntakeCommand. */
  public FeedCommand(Feeder feeder, Centerer centerer, Indexer indexer) {
    addRequirements(feeder);
    addRequirements(centerer);
    addRequirements(indexer);
    mFeeder = feeder;
    mCenterer = centerer;
    mIndexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mFeeder.forward();
    mCenterer.forward();
    mIndexer.forward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFeeder.stop();
    mCenterer.stop();
    mIndexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
