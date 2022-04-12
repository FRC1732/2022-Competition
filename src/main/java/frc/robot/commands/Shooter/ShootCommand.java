// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooter, Feeder feeder, Centerer centerer, Indexer indexer, DoubleSupplier feedSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunShooterCommand(shooter)
      .raceWith(
        new WaitCommand(5)
        .withInterrupt(() -> shooter.isFlywheelAtTargetVelocity())
        .andThen(new FeedCommand(feeder, centerer, indexer, feedSpeed)
        .withTimeout(0.75))
      )
    );
  }
}
