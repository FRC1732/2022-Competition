// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Centerer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoRejectHigh extends SequentialCommandGroup {
    public AutoRejectHigh(Intake intakeSubsystem, ColorSensor colorsensorSubsystem, Shooter shooterSubsystem, Feeder feederSubsystem, Centerer centererSubsystem) {
        addCommands(
            new SequentialCommandGroup (
                new ParallelCommandGroup(
                    new InstantCommand()
                    // run feeder
                    // run indexer
                    // EXTRA - stop intake
                ),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                    // stop flywheel
                    // stop feeder
                    // EXTRA - resume intake
                )
            )
        );
    }
}
