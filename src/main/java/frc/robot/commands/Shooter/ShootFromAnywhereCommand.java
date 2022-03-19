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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromAnywhereCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public ShootFromAnywhereCommand(Shooter shooter, Feeder feeder, Centerer centerer, Indexer indexer, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double distance = limelight.getProjectedDistance();
    double speed = 9.375 * distance * distance - 100 * distance + 2190.625; //TODO: find equation to translate distance to shooter speed
    shooter.setTargetNearRpm(speed);

    addCommands(new RunShooterCommand(shooter)
      .raceWith(
        new WaitCommand(5)
        .withInterrupt(() -> shooter.isFlywheelAtTargetVelocity())
        .andThen(new FeedCommand(feeder, centerer, indexer)
        .withTimeout(0.75))
      )
    );

    shooter.setTargetNearRpm(TARGET_RPM_NEAR);
  }
}
