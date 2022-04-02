// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.AutoClimb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Climber.*;

public class AutoClimb_Phase2 extends SequentialCommandGroup {
  public AutoClimb_Phase2(Intake intakeSubsystem, Climber climberSubsystem) {
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RightArmOneDown(climberSubsystem)
                    .withInterrupt(climberSubsystem.rightArmOneAtRetractTarget),
                new LeftArmOneDown(climberSubsystem)
                    .withInterrupt(climberSubsystem.leftArmOneAtRetractTarget)),
            new InstantCommand(() -> climberSubsystem.ArmTwoIn()),
            new WaitCommand(.7),
            new ParallelCommandGroup(
                new InstantCommand(() -> intakeSubsystem.retract()),
                new RightArmTwoDown(climberSubsystem)
                    .withInterrupt(climberSubsystem.rightArmTwoAtRetractTarget),
                new LeftArmTwoDown(climberSubsystem)
                    .withInterrupt(climberSubsystem.leftArmTwoAtRetractTarget))));
  }
}
