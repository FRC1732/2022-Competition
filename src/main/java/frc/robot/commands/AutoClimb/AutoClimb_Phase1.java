// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Climber.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class AutoClimb_Phase1 extends SequentialCommandGroup {
  /** Creates a new ExtendClimberArms. */
  public AutoClimb_Phase1(Intake intakeSubsystem, Climber climberSubsystem) {
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(() -> climberSubsystem.zeroEncoderPositions()),
            new InstantCommand(() -> climberSubsystem.nullifyPID()),
            new InstantCommand(() -> intakeSubsystem.deploy()),
            new WaitCommand(0.2),
            new InstantCommand(() -> climberSubsystem.ArmTwoOut()),
            new ParallelCommandGroup(
                new RightArmOneUp(climberSubsystem)
                    .withInterrupt(climberSubsystem.rightArmOneAtExtendTarget),
                new LeftArmOneUp(climberSubsystem)
                    .withInterrupt(climberSubsystem.leftArmOneAtExtendTarget),
                new RightArmTwoUp(climberSubsystem)
                    .withInterrupt(climberSubsystem.rightArmTwoAtExtendTarget),
                new LeftArmTwoUp(climberSubsystem)
                    .withInterrupt(climberSubsystem.leftArmTwoAtExtendTarget))));
  }
}
