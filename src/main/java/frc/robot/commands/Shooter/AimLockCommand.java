// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimLockCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public AimLockCommand(Shooter shooter, Feeder feeder, Centerer centerer, Indexer indexer, Limelight limelight, BooleanSupplier robotStopped, DoubleSupplier feedSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new RunShooterCommand(shooter, limelight.projectedDistToTarget)
      .raceWith(
        new WaitCommand(50)
        .withInterrupt(() -> shooter.isFlywheelAtTargetVelocity() && limelight.hasTarget() && limelight.isAligned() && robotStopped.getAsBoolean())
        .andThen(new FeedCommand(feeder, centerer, indexer, feedSpeed)
        .withTimeout(0.75))
      )
    );

    shooter.setTargetNearRpm(TARGET_RPM_NEAR);
  }
}
