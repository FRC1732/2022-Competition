// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeWithRejectCommand extends CommandBase {
  private Indexer mIndexer;
  private Centerer mCenterer;
  private Intake mIntake;
  private Shooter mShooter;
  private Feeder mFeeder;
  private ColorSensor mColorSensor;


  /** Creates a new IntakeCommand. */
  public IntakeWithRejectCommand(Intake intake, Centerer centerer, Indexer indexer, Feeder feeder, Shooter shooter, ColorSensor colorSensor) {
    addRequirements(intake);
    addRequirements(centerer);
    addRequirements(indexer);
    addRequirements(feeder);
    addRequirements(shooter);
    mShooter = shooter;
    mFeeder = feeder;
    mIntake = intake;
    mCenterer = centerer;
    mIndexer = indexer;
    mColorSensor = colorSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.deploy();
    mIntake.forward();
    mCenterer.forward();
    mIndexer.forwardSlow();
    mFeeder.stop();
    mShooter.stopFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //TODO: workout the reject commands
    /*if(mIsRejectEnabled.getAsBoolean()){
      if(mColorSensor.isWrongBall() && !isTimerRunning){
        mTimer.start();
        isTimerRunning = true;
        mIntake.reverse();
        mIndexer.reverse();
        mCenterer.reverse();
      }
      if(mTimer.hasElapsed(1)){
        mTimer.stop();
        isTimerRunning = false;
        mIntake.forward();
        mCenterer.forward();
        mIndexer.forward();
      }
    } */
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      return;
    mIntake.stop();
    mCenterer.stop();
    mIndexer.stop();
    mIntake.retract();
    mFeeder.stop();
    mShooter.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
