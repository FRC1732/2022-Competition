// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class EjectSingleBallCommand extends WaitCommand {
  private Shooter mShooter;
  private Feeder mFeeder;

  /** Creates a new IntakeCommand. */
  public EjectSingleBallCommand(Feeder feeder, Shooter shooter) {
    super(0.75);
    addRequirements(feeder);
    addRequirements(shooter);
    mShooter = shooter;
    mFeeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mFeeder.forward();
    mShooter.startDribble();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      return;
    mShooter.stopFlywheel();
    mFeeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
