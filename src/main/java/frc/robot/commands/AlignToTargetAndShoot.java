// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AlignToTargetAndShoot extends CommandBase {
  private Drivetrain drivetrain;
  private Limelight limelight;
  private Shooter shooter;

  /** Creates a new AlignToTargetAndShoot. */
  public AlignToTargetAndShoot(Drivetrain drivetrain, Limelight limelight, Shooter shooter) {
    addRequirements(drivetrain, limelight, shooter);
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stop();
    shooter.start();
    limelight.on();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.getTx() != 0 && limelight.getTy() != 0) {
      if (limelight.getTx() < -5) {
        // move right
      } else if (limelight.getTx() > 5) {
        // move left
      } else {
        // dont turn
      }
      if (limelight.getTy() < -5) {
        // move up
      } else if (limelight.getTy() > 5) {
        // move down
      } else {
        // dont turn
      }
    } else {
      // shoot
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.off();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !limelight.hasTarget();
  }
}
