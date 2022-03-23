// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private Indexer mIndexer;
  private Centerer mCenterer;
  private Intake mIntake;
  private ColorSensor mColorSensor;
  private BooleanSupplier mIsRejectEnabled;
  private Timer mTimer;
  private boolean isTimerRunning;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Centerer centerer, Indexer indexer, ColorSensor colorSensor, BooleanSupplier isRejectEnabled) {
    addRequirements(intake);
    addRequirements(centerer);
    addRequirements(indexer);
    mIntake = intake;
    mCenterer = centerer;
    mIndexer = indexer;
    mColorSensor = colorSensor;
    mIsRejectEnabled = isRejectEnabled;
    mTimer = new Timer();
    isTimerRunning = false;
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
