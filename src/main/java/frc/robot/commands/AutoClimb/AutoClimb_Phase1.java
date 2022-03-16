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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb_Phase1 extends SequentialCommandGroup {
  /** Creates a new ExtendClimberArms. */
  public AutoClimb_Phase1(Intake intakeSubsystem, Climber climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(() -> intakeSubsystem.deploy()),
            new WaitCommand(0.2),
            new InstantCommand(() -> climberSubsystem.ArmTwoOut()),
            new ParallelCommandGroup(
                new InstantCommand(() -> climberSubsystem.climberArmOneUp())
                    .withInterrupt(() -> climberSubsystem.r_ClimberArmOneAtExtendTarget()),
                new InstantCommand(() -> climberSubsystem.climberArmTwoUp())
                    .withInterrupt(() -> climberSubsystem.r_ClimberArmTwoAtExtendTarget()))));

  }
}
