// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class EjectCommand extends CommandBase {
  private Indexer mIndexer;
  private Centerer mCenterer;
  private Intake mIntake;
  private Feeder mFeeder;

  /** Creates a new IntakeCommand. */
  public EjectCommand(Intake intake, Centerer centerer, Indexer indexer, Feeder feeder) {
    addRequirements(intake);
    addRequirements(centerer);
    addRequirements(indexer);
    addRequirements(feeder);
    mIntake = intake;
    mCenterer = centerer;
    mIndexer = indexer;
    mFeeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.reverse();
    mCenterer.reverse();
    mIndexer.reverse();
    mFeeder.reverse();
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
    mFeeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
