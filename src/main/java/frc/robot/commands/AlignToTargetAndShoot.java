// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.alignment.MoveToAlign;
import frc.robot.commands.alignment.MoveToAlign.Direction;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AlignToTargetAndShoot extends CommandBase {
  private Limelight limelight;
  private Shooter shooter;
  private MoveToAlign moveToAlign;

  /** Creates a new AlignToTargetAndShoot. */
  public AlignToTargetAndShoot(Subsystem drivetrain, Limelight limelight, Shooter shooter, MoveToAlign moveToAlign) {
    addRequirements(drivetrain, limelight);
    if (shooter != null) {
      addRequirements(shooter);
    }
    this.limelight = limelight;
    this.shooter = shooter;
    this.moveToAlign = moveToAlign;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveToAlign.stop();
    if (shooter != null) {
      shooter.shootFlywheel();
    }
    limelight.on();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.getTx() != 0 && limelight.getTy() != 0) {
      if (limelight.getTx() < -5) {
        moveToAlign.move(Direction.Right);
      } else if (limelight.getTx() > 5) {
        moveToAlign.move(Direction.Left);
      } else {
        moveToAlign.move(Direction.None);
      }
      if (limelight.getTy() < -5) {
        // move up
      } else if (limelight.getTy() > 5) {
        // move down
      } else {
        // dont turn
      }
    } else {
      if (shooter != null) {
        // shoot
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.off();
    if (shooter != null) {
      shooter.stopFlywheel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !limelight.hasTarget();
  }
}
