// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;

public class YeetEjectCommand extends WaitCommand {
  private Indexer mIndexer;
  private Centerer mCenterer;
  private Feeder mFeeder;

  /** Creates a new IntakeCommand. */
  public YeetEjectCommand(Centerer centerer, Indexer indexer, Feeder feeder) {
    super(0.1);
    addRequirements(centerer);
    addRequirements(indexer);
    addRequirements(feeder);
    mCenterer = centerer;
    mIndexer = indexer;
    mFeeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    mCenterer.max_reverse();
    mIndexer.max_reverse();
    mFeeder.max_reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    mCenterer.stop();
    mIndexer.stop();
    mFeeder.stop();
  }
}
